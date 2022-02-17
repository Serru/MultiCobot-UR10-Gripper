#!/usr/bin/env python
import sys
import copy

import rospy
import geometry_msgs.msg

import moveit_commander
import moveit_msgs.msg

import Tkinter as tk
from Tkinter import Checkbutton
import tkMessageBox

from leap_motion.msg import leapcobotright

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi


    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))

# r, p and y in rads
def plan_cartesian_path_orientation(group, r, p, y, scale=1):
    joint_target = group.get_current_joint_values()
    joint_target[3] += scale * r
    joint_target[4] += scale * p
    joint_target[5] += scale * y
    succeeded = group.go(joint_target)

def plan_cartesian_path_pose(group, desiredPos):
    # append current pose to the waypoints array
    waypoints = []
    waypoints.append(group.get_current_pose().pose)
    # set the pose for x, y and z
    wpose = geometry_msgs.msg.Pose()
    wpose.position.x = desiredPos.x
    wpose.position.y = desiredPos.z # we switch z and y because the leap motion is faced upwards
    wpose.position.z = desiredPos.y
    waypoints.append(copy.deepcopy(wpose))
    # plan the movement
    (plan, fraction) = group.compute_cartesian_path(
                            waypoints,   # waypoints to follow
                            0.01,        # eef_step
                            0.0)         # jump_threshold
    # execute the plan
    #group.execute(plan, wait=True)
    group.execute(plan)

# class for moving the robot
class MoveIt(object):
    def __init__(self, robot_initialPos):
        print("MoveIt: Start")
        self.subscriber = None # we need to have the subscriber as an object so we can unregister/register when paused/resumed
        self.subscribeToTopic() # Subscribe to the topic to receive the data from Leap Motion
        print("MoveIt: 1")
        #self.resumedRun = False # flag to determine if it is the first run after pressing 'resume'
        #self.paused = True # flag to determine if program is paused
        self.executing = False # flag to determine if we are currently executing a plan
        
        self.cobot_right_pos = geometry_msgs.msg.Pose().position # keep track of the robot zero position
        self.cobot_left_pos = geometry_msgs.msg.Pose().position
        
        self.lm_reference_pos = geometry_msgs.msg.Pose().position # keep track of the hand zero position
        self.left_hand_pos = geometry_msgs.msg.Pose().position

        self.right_hand_reference_pos = geometry_msgs.msg.Pose().position
        self.left_hand_reference_pos = geometry_msgs.msg.Pose().position
        
        self.cobot_right_initial_pos = robot_initialPos # keep track of the robot starting position
        self.cobot_left_initial_pos = None

        self.set_origin = True
        self.execute_trayectory = False

        # Death zone limits
        self.death_zone_limit = 30.0

        #self.enableDiffCheck = True # If enabled will check for difference against a set max before allowing a move
        #self.enableBoundaryCheck = True # If enabled will check if robot is exceeding a boundary

        # boundaries | initial coords: x: 0.603 y: 0.124 z: 0.566
        #self.maxLeft = self.robot_initialPos.x - 0.200 # x left
        #self.maxRight = self.robot_initialPos.x + 0.200 # x right
        #self.maxHeight = self.robot_initialPos.y + 0.300 # y height
        #self.maxUp = self.robot_initialPos.z + 0.100 # z up
        #self.maxDown = self.robot_initialPos.z - 0.100 # z down

        # maximum step, user cannot move more than this in one step
        self.maxStep = 0.150

        # this is used to round co-ordinates when checking if previous and current coords are very similar
        self.dp = 3 # decimal points

        '''
        -----------------------------------------
            gui_mainWindow
        -----------------------------------------
        '''
        print("MoveIt: 2")
        # main GUI window
        self.gui_mainWindow = tk.Tk()
        self.gui_mainWindow.title("Leap Motion Controller Status")
        self.gui_mainWindow.geometry("1065x175")
        self.gui_mainWindow.config(bg='#8f14b8')
        #self.gui_mainWindow.grid_rowconfigure(0, weight=1)
        #self.gui_mainWindow.grid_columnconfigure(0, weight=1)
        self.gui_mainWindow.resizable(True, True)
        print("MoveIt: 3")
        #================================ first row =================================
        self.cobot_right_position("UR10 ARM Controller A")
        self.lm_right_position("Leap Controller A")
        self.lm_right_gestures("Leap Gestures A")
        self.lm_right_values("Gesture's Value A")
        #================================ second row =================================
        #self.cobot_left_position("UR10 ARM Controller B")
        #self.lm_left_position("Leap Controller B")
        #self.lm_left_gestures("Leap Gestures B")
        #self.lm_left_values("Gesture's Value B")
        #================================ end window =================================
        print("MoveIt: 4")
        # subscribe to data
        # self.gui_mainWindow.after(1, self.subscribeToTopic)
        # handle closing of GUI
        self.gui_mainWindow.protocol("WM_DELETE_WINDOW", self.onCloseQuit)
        # keep the GUI running
        print("MoveIt: 5")
        self.gui_mainWindow.mainloop()
        print("MoveIt: end")

    # method to begin planning and executing movement of the robot
    def beginPlan(self, hand_pos):
        print("beginPlan: Start")
        if self.executing:
            print("beginPlan: 1")
            #self.cobot_right_pos.x = group.get_current_pose().pose.position.x
            #self.cobot_right_pos.y = group.get_current_pose().pose.position.z
            #self.cobot_right_pos.z = group.get_current_pose().pose.position.y
            #if self.set_origin:
            #    print("beginPlan: 2")
            #    self.lm_reference_pos.x = hand_pos.x
            #    self.lm_reference_pos.y = hand_pos.y
            #    self.lm_reference_pos.z = hand_pos.z
            #    self.set_origin=False
            print("beginPlan: 3")

            #self.cobot_right_position_update(self.cobot_right_pos.x, self.cobot_right_pos.y, self.cobot_right_pos.z)
            print("beginPlan: 4")
            #self.lm_right_position_update(hand_pos.x, hand_pos.y, hand_pos.z)
            
            print("beginPlan: 5")
            # need to transform leap motion input
            desiredPos = self.getDesiredPos(hand_pos)
    
            # we round the positions so it is much easier to carry out checks
            desiredPos.x = round(desiredPos.x, self.dp)
            desiredPos.y = round(desiredPos.y, self.dp)
            desiredPos.z = round(desiredPos.z, self.dp)
            print("beginPlan: 6")
            
            if self.execute_trayectory:
                plan_cartesian_path_pose(arm, desiredPos)
                ## append current pose to the waypoints array
                #waypoints = []
                #waypoints.append(group.get_current_pose().pose)
                ## set the pose for x, y and z
                #wpose = geometry_msgs.msg.Pose()
                #wpose.position.x = desiredPos.x
                #wpose.position.y = desiredPos.z # we switch z and y because the leap motion is faced upwards
                #wpose.position.z = desiredPos.y
                #waypoints.append(copy.deepcopy(wpose))
                ## plan the movement
                #print("beginPlan: 7")
                #(plan, fraction) = group.compute_cartesian_path(
                #                        waypoints,   # waypoints to follow
                #                        0.01,        # eef_step
                #                        0.0)         # jump_threshold
                ## execute the plan
                #print("beginPlan: 8")
                ##group.execute(plan, wait=True)
                #group.execute(plan)
                self.execute_trayectory = False
            
            print("beginPlan: 9")

            # we are no longer executing
            self.executing = False


            #self.cobot_right_pos.x = group.get_current_pose().pose.position.x
            #self.cobot_right_pos.y = group.get_current_pose().pose.position.z
            #self.cobot_right_pos.z = group.get_current_pose().pose.position.y
            #self.cobot_right_position_update(self.cobot_right_pos.x, self.cobot_right_pos.y, self.cobot_right_pos.z)
            #print("beginPlan: 10")
            #self.lm_right_position_update(hand_pos.x, hand_pos.y, hand_pos.z)


            print("beginPlan: End")


    # method to begin sending information to the planner
    def beginExecution(self, leap_msg):
        print("beginExecution: Start")
        print(self.executing)
        if not self.executing:
            print("beginExecution: 1")
            # store xyz information in a variable
            palmPos = leap_msg.right_hand_palmpos
            print("beginExecution: 2")
            
            # create an array of the xyz
            posArray = [palmPos.x, palmPos.y, palmPos.z]
            print("beginExecution: 3")
            # check if x,y,z is > than 0.0 (avoid passing 0.0 coordinates)
            if any(value > 0.0 for value in posArray):
                self.executing = True
                print("beginExecution: 4")
                self.beginPlan(palmPos)
            print("beginExecution: End")

    # method to get the desired position of the robot
    def getDesiredPos(self, hand_pos):
        print("getDesiredPos: Start")
        # lower leap motion values by this much
        conversion_value = 0.001
        #
        desired_pos = geometry_msgs.msg.Pose().position
        print(desired_pos)

        # check death zone limits
        axis_x = abs(hand_pos.x - self.lm_reference_pos.x)
        axis_y = abs(hand_pos.y - self.lm_reference_pos.y)
        axis_z = abs(hand_pos.z - self.lm_reference_pos.z)

        #death_zone_array = [axis_x, axis_y, axis_z]
        # check if axis_x, axis_y, axis_z is > than 0.0 (avoid death zone)
        #if any(axis > 20.0 for axis in death_zone_array):
        if axis_x > self.death_zone_limit:
            dist_walked = hand_pos.x - self.lm_reference_pos.x
            if dist_walked > self.death_zone_limit:
                dist_walked = dist_walked - self.death_zone_limit
            elif dist_walked < self.death_zone_limit:
                dist_walked = dist_walked + self.death_zone_limit

            # adjust the offset and pass back the new coordinates
            desired_pos.x = self.cobot_right_pos.x - (dist_walked * conversion_value) # inverted
            print("cobot_right_pos.x: " + str(self.cobot_right_pos.x))
            print("hand_pos.x: " + str(hand_pos.x))
            print("lm_reference_pos.x: " + str(self.lm_reference_pos.x))
            print("conversion_value: " + str(conversion_value))
            print("(hand_pos.x - self.lm_reference_pos.x): " + str(hand_pos.x - self.lm_reference_pos.x))
            print("((hand_pos.x - self.lm_reference_pos.x) * conversion_value): " + str(((hand_pos.x - self.lm_reference_pos.x) * conversion_value)))
            print("\033[92mgetDesiredPos.x: "+str(desired_pos.x)+"\033[0m")
            self.execute_trayectory = True
        else:
            desired_pos.x = self.cobot_right_pos.x

        if axis_y > self.death_zone_limit:
            dist_walked = hand_pos.y - self.lm_reference_pos.y
            if dist_walked > self.death_zone_limit:
                dist_walked = dist_walked - self.death_zone_limit
            elif dist_walked < self.death_zone_limit:
                dist_walked = dist_walked + self.death_zone_limit

            desired_pos.y = self.cobot_right_pos.y + (dist_walked * conversion_value)
            print("\033[92mgetDesiredPos.y: "+str(desired_pos.y)+"\033[0m")
            self.execute_trayectory = True
        else:
            desired_pos.y = self.cobot_right_pos.y
        if axis_z > self.death_zone_limit:
            dist_walked = hand_pos.z - self.lm_reference_pos.z
            if dist_walked > self.death_zone_limit:
                dist_walked = dist_walked - self.death_zone_limit
            elif dist_walked < self.death_zone_limit:
                dist_walked = dist_walked + self.death_zone_limit
            desired_pos.z = self.cobot_right_pos.z + (dist_walked * conversion_value)
            print("\033[92mgetDesiredPos.z: "+str(desired_pos.z)+"\033[0m")
            self.execute_trayectory = True
        else:
            desired_pos.z = self.cobot_right_pos.z

        print("getDesiredPos: End")
        return desired_pos



    def frame_management(self, frame):
        #print(frame.is_right_hand)
        #if frame.is_right_hand: # There is a hand and not fist gesture    
        #    print("there is a right hand")
        #    print(frame.right_hand_fist)
        #    if not frame.right_hand_fist: # Le cuesta diferenciarlo del thumb up => meter el dedo pulgar dentro de la palma de la mano
        if self.lm_status_update(frame):
            #if frame.right_hand_thumb_up: # Thumb up gesture to start the work
                #print("Thumb up gesture done")
                #print("Checking for movements to the cobot")
                #if frame.right_hand_pinch:
                    # Controla el gripper
                #    print("Gripper Close")
                #    print(frame.right_hand_pinch)
            self.cobot_gripper_update(frame.right_hand_pinch)
                #else:
                #    print("Gripper Open")
                #    self.cobot_gripper_update(frame.right_hand_pinch)



            # Before executing
            self.cobot_right_position_update()
            
            self.lm_right_gestures_update(frame)
            self.beginExecution(frame)
                        
            # After executing
            self.cobot_right_position_update()

#            if frame.right_hand_origin_frame:
#                # Set palm coords as reference
#                print("frame_management: set coords as reference")
#                self.lm_reference_pos.x = frame.right_hand_palmpos.x
#                self.lm_reference_pos.y = frame.right_hand_palmpos.x
#                self.lm_reference_pos.z = frame.right_hand_palmpos.x
#    
#                # update rotate, turn and swing of the gripper eef
#                print("Update Gripper rotation")
#                print(frame.right_hand_rotate_value)
#                print("Update Gripper turning")
#                print(frame.right_hand_turn_value)
#                print("Update Gripper swinging")
#                print(frame.right_hand_swing_value)
#            else:
#                print("Not reference frame set")
            


        #        else:
        #            print("Not thumb gesture done")
        #    else:
        #        print("Fist gesture done")
        #else:
        #    print("Not right hand available")
                

    # method to subscribe to the ros topic
    def subscribeToTopic(self):
        # register to the topic
        print("subscribeToTopic: Start")
        self.subscriber = rospy.Subscriber("/leapmotion/data", leapcobotright, self.frame_management, queue_size=1)
        print("subscribeToTopic: End")

    # action method for quitting the program upon clicking [x]
    def onCloseQuit(self, *ignore):
        if tkMessageBox.askokcancel("Quit", "Do you want to quit?"):
            self.gui_mainWindow.destroy()

    def cobot_right_position(self, title):
        # Create the root label
        self.cobot_right_label = tk.Label(self.gui_mainWindow, bg="#8f14b8", borderwidth=3, relief="groove")     
        self.cobot_right_label.grid(padx=(5, 5), pady=(5, 5))
        self.cobot_right_label.grid(column=0, row=0)
    
        # Title Label
        self.cobot_right_title = tk.Label(
            self.cobot_right_label, 
            text=title, 
            bg="#8f14b8", 
            fg="white", 
            font=("Helvetica", 16, 'underline')
            ) 
        self.cobot_right_title.grid(padx=5, pady=5)
        self.cobot_right_title.grid(column=0, row=0)
        
        # X label
        self.cobot_right_x = tk.Label(
            self.cobot_right_label, 
            text="X: 0.0", 
            bg="#8f14b8", 
            fg="white", 
            font=("Arial Bold", 12)
            )
        self.cobot_right_x.grid(column=0, row=1)
        
        # Y label
        self.cobot_right_y = tk.Label(
            self.cobot_right_label, 
            text="Y: 0.0", 
            bg="#8f14b8", 
            fg="white", 
            font=("Arial Bold", 12)) 
        self.cobot_right_y.grid(column=0, row=2)    
            
        # Z label
        self.cobot_right_z = tk.Label(
            self.cobot_right_label, 
            text="Z: 0.0", 
            bg="#8f14b8", 
            fg="white", 
            font=("Arial Bold", 12)) 
        self.cobot_right_z.grid(column=0, row=3)
            
        # Gripper label
        self.cobot_right_gripper = tk.Label(
            self.cobot_right_label, 
            text=" Gripper: ---- ", 
            bg="blue", 
            fg="white", 
            font=("Arial Bold", 12),
            borderwidth=3, 
            relief="groove", width=24) 
        self.cobot_right_gripper.grid(column=0, row=4)
        self.cobot_right_gripper.grid(pady=(0, 5))

    def lm_right_position(self, title):
        # Create the root label
        self.lm_right_label = tk.Label(self.gui_mainWindow, bg="#8f14b8", borderwidth=3, relief="groove") 
        self.lm_right_label.grid(padx=(5, 5), pady=(5, 5))
        self.lm_right_label.grid(column=1, row=0)
    
        # Title label
        self.lm_right_title = tk.Label(
            self.lm_right_label, 
            text=title, 
            bg="#8f14b8", 
            fg="white", 
            font=("Helvetica", 16, 'underline')
            ) 
        self.lm_right_title.grid(padx=5, pady=5)
        self.lm_right_title.grid(column=0, row=0)

        # X label
        self.lm_right_x = tk.Label(
            self.lm_right_label, 
            text="X: 0.0", 
            bg="#8f14b8", 
            fg="white", 
            font=("Arial Bold", 12)
            )
        self.lm_right_x.grid(column=0, row=1)
            
        # Y label
        self.lm_right_y = tk.Label(
            self.lm_right_label, 
            text="Y: 0.0", 
            bg="#8f14b8", 
            fg="white", 
            font=("Arial Bold", 12)) 
        self.lm_right_y.grid(column=0, row=2)
            
        # Z label
        self.lm_right_z = tk.Label(
            self.lm_right_label, 
            text="Z: 0.0", 
            bg="#8f14b8", 
            fg="white", 
            font=("Arial Bold", 12)) 
        self.lm_right_z.grid(column=0, row=3)
            
        # Leap motion state label
        self.lm_right_status = tk.Label(
            self.lm_right_label, 
            text=" Waiting Leap Motion ", 
            bg="blue", 
            fg="white", 
            font=("Arial Bold", 12),
            borderwidth=3, 
            relief="groove", width=24)
        self.lm_right_status.grid(column=0, row=4)
        self.lm_right_status.grid(pady=(0, 5))

    def lm_right_gestures(self, title):
        # Create the root label
        self.lm_right_label_gesture = tk.Label(self.gui_mainWindow, bg="#8f14b8", borderwidth=3, relief="groove") 
        self.lm_right_label_gesture.grid(padx=(5, 5), pady=(5, 5))
        self.lm_right_label_gesture.grid(column=3, row=0)
    
        # Title label
        self.lm_right_title_gesture = tk.Label(
            self.lm_right_label_gesture, 
            text=title, 
            bg="#8f14b8", 
            fg="white", 
            font=("Helvetica", 16, 'underline')
            ) 
        self.lm_right_title_gesture.grid(padx=5, pady=5)
        self.lm_right_title_gesture.grid(column=0, row=0)
        
            
        # Leap motion state label
        self.lm_right_origin_gesture = tk.Label(
            self.lm_right_label_gesture, 
            text=" Set Reference Coords: None ", 
            bg="blue", 
            fg="white", 
            font=("Arial Bold", 12),
            borderwidth=3, 
            relief="groove", width=24)
        self.lm_right_origin_gesture.grid(column=0, row=1)
        self.lm_right_origin_gesture.grid(pady=(0, 2), padx=(2,2))

                        # Leap motion state label
        self.lm_right_swing_gesture = tk.Label(
            self.lm_right_label_gesture, 
            text=" Swing Gripper: Inactive ", 
            bg="blue", 
            fg="white", 
            font=("Arial Bold", 12),
            borderwidth=3, 
            relief="groove", width=24)
        self.lm_right_swing_gesture.grid(column=0, row=2)
        self.lm_right_swing_gesture.grid(pady=(0, 2), padx=(2,2))

                # Leap motion state label
        self.lm_right_rotate_gesture = tk.Label(
            self.lm_right_label_gesture, 
            text=" Rotate Gripper: Inactive ", 
            bg="blue", 
            fg="white", 
            font=("Arial Bold", 12),
            borderwidth=3, 
            relief="groove", width=24)
        self.lm_right_rotate_gesture.grid(column=0, row=3)
        self.lm_right_rotate_gesture.grid(pady=(0, 2), padx=(2,2))

                # Leap motion state label
        self.lm_right_turn_gesture = tk.Label(
            self.lm_right_label_gesture, 
            text=" Turn Gripper: Inactive ", 
            bg="blue", 
            fg="white", 
            font=("Arial Bold", 12),
            borderwidth=3, 
            relief="groove", width=24)
        self.lm_right_turn_gesture.grid(column=0, row=4)
        self.lm_right_turn_gesture.grid(pady=(0, 5), padx=(2,2))

    def lm_right_values(self, title):
        # Create the root label
        self.lm_right_label_value = tk.Label(self.gui_mainWindow, bg="#8f14b8", borderwidth=3, relief="groove") 
        self.lm_right_label_value.grid(padx=(5, 5), pady=(5, 5))
        self.lm_right_label_value.grid(column=4, row=0)
    
        # Title label
        self.lm_right_title_value = tk.Label(
            self.lm_right_label_value, 
            text=title, 
            bg="#8f14b8", 
            fg="white", 
            font=("Helvetica", 16, 'underline')
            ) 
        self.lm_right_title_value.grid(padx=5, pady=5)
        self.lm_right_title_value.grid(column=0, row=0)
        
            
        # Leap motion state label
        self.lm_right_origin_value = tk.Label(
            self.lm_right_label_value, 
            text=" Coords: x: 0.0, y: 0.0, z: 0.0 ", 
            bg="blue", 
            fg="white", 
            font=("Arial Bold", 12),
            borderwidth=3, 
            relief="groove", width=24)
        self.lm_right_origin_value.grid(column=0, row=1)
        self.lm_right_origin_value.grid(pady=(0, 2), padx=(2,2))

                        # Leap motion state label
        self.lm_right_swing_value = tk.Label(
            self.lm_right_label_value, 
            text=" Swing Value: 0.0 ", 
            bg="blue", 
            fg="white", 
            font=("Arial Bold", 12),
            borderwidth=3, 
            relief="groove", width=24)
        self.lm_right_swing_value.grid(column=0, row=2)
        self.lm_right_swing_value.grid(pady=(0, 2), padx=(2,2))

                # Leap motion state label
        self.lm_right_rotate_value = tk.Label(
            self.lm_right_label_value, 
            text=" Rotate Value: 0.0 ", 
            bg="blue", 
            fg="white", 
            font=("Arial Bold", 12),
            borderwidth=3, 
            relief="groove", width=24)
        self.lm_right_rotate_value.grid(column=0, row=3)
        self.lm_right_rotate_value.grid(pady=(0, 2), padx=(2,2))

                # Leap motion state label
        self.lm_right_turn_value = tk.Label(
            self.lm_right_label_value, 
            text=" Turn Value: 0.0 ", 
            bg="blue", 
            fg="white", 
            font=("Arial Bold", 12),
            borderwidth=3, 
            relief="groove", width=24)
        self.lm_right_turn_value.grid(column=0, row=4)
        self.lm_right_turn_value.grid(pady=(0, 5), padx=(2,2))
    
    def cobot_left_position(self, title):
        # Create the root label
        self.cobot_left_label = tk.Label(self.gui_mainWindow, bg="#8f14b8", borderwidth=3, relief="groove") 
        self.cobot_left_label.grid(padx=(5, 5), pady=(5, 5))
        self.cobot_left_label.grid(column=0, row=1)
    
        # Title label
        self.cobot_left_title = tk.Label(
            self.cobot_left_label, 
            text=title, 
            bg="#8f14b8", 
            fg="white", 
            font=("Helvetica", 16, 'underline')
            ) 
        self.cobot_left_title.grid(padx=5, pady=5)
        self.cobot_left_title.grid(column=0, row=0)
        
        # X label
        self.cobot_left_x = tk.Label(
            self.cobot_left_label, 
            text="X: 0", 
            bg="#8f14b8", 
            fg="white", 
            font=("Arial Bold", 12)
            )
        self.cobot_left_x.grid(column=0, row=1)    
        
        # Y label
        self.cobot_left_y = tk.Label(
            self.cobot_left_label, 
            text="Y: 0", 
            bg="#8f14b8", 
            fg="white", 
            font=("Arial Bold", 12)) 
        self.cobot_left_y.grid(column=0, row=2)
        
        # Z label
        self.cobot_left_z = tk.Label(
            self.cobot_left_label, 
            text="Z: 0", 
            bg="#8f14b8", 
            fg="white", 
            font=("Arial Bold", 12)) 
        self.cobot_left_z.grid(column=0, row=3)
        
        # Gripper label
        self.cobot_left_gripper = tk.Label(
            self.cobot_left_label, 
            text=" Gripper: Open ", 
            bg="#0c9132", 
            fg="white", 
            font=("Arial Bold", 12),
            borderwidth=3, 
            relief="groove")
        self.cobot_left_gripper.grid(column=0, row=4)
        self.cobot_left_gripper.grid(pady=(0, 5))
    
    def lm_left_position(self, title):
        # Create the root label
        self.lm_left_label = tk.Label(self.gui_mainWindow, bg="#8f14b8", borderwidth=3, relief="groove") 
        self.lm_left_label.grid(padx=(5, 5), pady=(5, 5))
        self.lm_left_label.grid(column=1, row=1)
    
        # Title label
        self.lm_left_title = tk.Label(
            self.lm_left_label, 
            text=title, 
            bg="#8f14b8", 
            fg="white", 
            font=("Helvetica", 16, 'underline')
            ) 
        self.lm_left_title.grid(padx=5, pady=5)
        self.lm_left_title.grid(column=0, row=0)
        
        # X label
        self.lm_left_x = tk.Label(
            self.lm_left_label, 
            text="X: 0", 
            bg="#8f14b8", 
            fg="white", 
            font=("Arial Bold", 12)
            )
        self.lm_left_x.grid(column=0, row=1)
            
        # Y label
        self.lm_left_y = tk.Label(
            self.lm_left_label, 
            text="Y: 0", 
            bg="#8f14b8", 
            fg="white", 
            font=("Arial Bold", 12)) 
        self.lm_left_y.grid(column=0, row=2)
            
        # Z label
        self.lm_left_z = tk.Label(
            self.lm_left_label, 
            text="Z: 0", 
            bg="#8f14b8", 
            fg="white", 
            font=("Arial Bold", 12)) 
        self.lm_left_z.grid(column=0, row=3)
            
        # Leap motion status label
        self.lm_left_status = tk.Label(
            self.lm_left_label, 
            text=" Status: Ready ", 
            bg="#0c9132", 
            fg="white", 
            font=("Arial Bold", 12),
            borderwidth=3, 
            relief="groove")
        self.lm_left_status.grid(column=0, row=4)
        self.lm_left_status.grid(pady=(0, 5))

    def lm_left_gestures(self, title):
        # Create the root label
        self.lm_left_label_gesture = tk.Label(self.gui_mainWindow, bg="#8f14b8", borderwidth=3, relief="groove") 
        self.lm_left_label_gesture.grid(padx=(5, 5), pady=(5, 5))
        self.lm_left_label_gesture.grid(column=3, row=1)
    
        # Title label
        self.lm_left_title_gesture = tk.Label(
            self.lm_left_label_gesture, 
            text=title, 
            bg="#8f14b8", 
            fg="white", 
            font=("Helvetica", 16, 'underline')
            ) 
        self.lm_left_title_gesture.grid(padx=5, pady=5)
        self.lm_left_title_gesture.grid(column=0, row=0)
        
            
        # Leap motion state label
        self.lm_left_origin_gesture = tk.Label(
            self.lm_left_label_gesture, 
            text=" Set Reference Coords: None ", 
            bg="#0c9132", 
            fg="white", 
            font=("Arial Bold", 12),
            borderwidth=3, 
            relief="groove", width=24)
        self.lm_left_origin_gesture.grid(column=0, row=1)
        self.lm_left_origin_gesture.grid(pady=(0, 2), padx=(2,2))

                        # Leap motion state label
        self.lm_left_swing_gesture = tk.Label(
            self.lm_left_label_gesture, 
            text=" Swing Gpipper: Ready ", 
            bg="#0c9132", 
            fg="white", 
            font=("Arial Bold", 12),
            borderwidth=3, 
            relief="groove", width=24)
        self.lm_left_swing_gesture.grid(column=0, row=2)
        self.lm_left_swing_gesture.grid(pady=(0, 2), padx=(2,2))

                # Leap motion state label
        self.lm_left_rotate_gesture = tk.Label(
            self.lm_left_label_gesture, 
            text=" Rotate Gripper: Ready ", 
            bg="#0c9132", 
            fg="white", 
            font=("Arial Bold", 12),
            borderwidth=3, 
            relief="groove", width=24)
        self.lm_left_rotate_gesture.grid(column=0, row=3)
        self.lm_left_rotate_gesture.grid(pady=(0, 2), padx=(2,2))

                # Leap motion state label
        self.lm_left_turn_gesture = tk.Label(
            self.lm_left_label_gesture, 
            text=" Turn Gripper: Ready ", 
            bg="#0c9132", 
            fg="white", 
            font=("Arial Bold", 12),
            borderwidth=3, 
            relief="groove", width=24)
        self.lm_left_turn_gesture.grid(column=0, row=4)
        self.lm_left_turn_gesture.grid(pady=(0, 5), padx=(2,2))

    def lm_left_values(self, title):
        # Create the root label
        self.lm_left_label_value = tk.Label(self.gui_mainWindow, bg="#8f14b8", borderwidth=3, relief="groove") 
        self.lm_left_label_value.grid(padx=(5, 5), pady=(5, 5))
        self.lm_left_label_value.grid(column=4, row=1)
    
        # Title label
        self.lm_left_title_value = tk.Label(
            self.lm_left_label_value, 
            text=title, 
            bg="#8f14b8", 
            fg="white", 
            font=("Helvetica", 16, 'underline')
            ) 
        self.lm_left_title_value.grid(padx=5, pady=5)
        self.lm_left_title_value.grid(column=0, row=0)
        
            
        # Leap motion state label
        self.lm_left_origin_value = tk.Label(
            self.lm_left_label_value, 
            text=" Coords: x:0, y:0, z:0 ", 
            bg="#0c9132", 
            fg="white", 
            font=("Arial Bold", 12),
            borderwidth=3, 
            relief="groove", width=24)
        self.lm_left_origin_value.grid(column=0, row=1)
        self.lm_left_origin_value.grid(pady=(0, 2), padx=(2,2))

                        # Leap motion state label
        self.lm_left_swing_value = tk.Label(
            self.lm_left_label_value, 
            text=" Swing Value: 0 ", 
            bg="#0c9132", 
            fg="white", 
            font=("Arial Bold", 12),
            borderwidth=3, 
            relief="groove", width=24)
        self.lm_left_swing_value.grid(column=0, row=2)
        self.lm_left_swing_value.grid(pady=(0, 2), padx=(2,2))

                # Leap motion state label
        self.lm_left_rotate_value = tk.Label(
            self.lm_left_label_value, 
            text=" Rotate Value: 0 ", 
            bg="#0c9132", 
            fg="white", 
            font=("Arial Bold", 12),
            borderwidth=3, 
            relief="groove", width=24)
        self.lm_left_rotate_value.grid(column=0, row=3)
        self.lm_left_rotate_value.grid(pady=(0, 2), padx=(2,2))

                # Leap motion state label
        self.lm_left_turn_value = tk.Label(
            self.lm_left_label_value, 
            text=" Turn Value: 0 ", 
            bg="#0c9132", 
            fg="white", 
            font=("Arial Bold", 12),
            borderwidth=3, 
            relief="groove", width=24)
        self.lm_left_turn_value.grid(column=0, row=4)
        self.lm_left_turn_value.grid(pady=(0, 5), padx=(2,2))
    
    def cobot_right_position_update(self):
        
        self.cobot_right_pos.x = group.get_current_pose().pose.position.x
        self.cobot_right_pos.y = group.get_current_pose().pose.position.z
        self.cobot_right_pos.z = group.get_current_pose().pose.position.y

        # X label
        #print("cobot_rightupdate: Start")
        self.cobot_right_x["text"] = "X: "+ str(round(self.cobot_right_pos.x, 2))
        
        # Y label
        self.cobot_right_y["text"] = "Y: "+ str(round(self.cobot_right_pos.y, 2))
        #print("cobot_rightupdate: Y: "+ str(yvalue))    
        
        # Z label
        self.cobot_right_z["text"] = "Z: "+ str(round(self.cobot_right_pos.z, 2))
        #print("cobot_rightupdate: Z: "+ str(zvalue))

    def cobot_gripper_update(self, gripper_state):
        # Gripper label
        if gripper_state:
            # Add Close Gripper Code
            gripper.set_named_target("close")

            # Label update content
            self.cobot_right_gripper["text"] = " Gripper: Close "
            self.cobot_right_gripper["bg"] = "blue"
        else:
            # Add Open Gripper Code
            gripper.set_named_target("open")

            # Label update content
            self.cobot_right_gripper["text"] = " Gripper: Open "
            self.cobot_right_gripper["bg"] = "#0c9132"

        #self.cobot_right_gripper = tk.Label(self.cobot_right_label, text=" Gripper: Open ") 
        #print("cobot_rightupdate: End")
    
    def lm_right_position_update(self, frame):    
        
        xvalue = frame.right_hand_palmpos.x
        yvalue = frame.right_hand_palmpos.y
        zvalue = frame.right_hand_palmpos.z

        if abs(xvalue - self.lm_reference_pos.x) > self.death_zone_limit:
            self.lm_right_x["bg"] = "green"
        else:
            self.lm_right_x["bg"] = "#8f14b8"

        if abs(yvalue - self.lm_reference_pos.y) > self.death_zone_limit:
            self.lm_right_y["bg"] = "green"
        else:
            self.lm_right_y["bg"] = "#8f14b8"

        if abs(zvalue - self.lm_reference_pos.z) > self.death_zone_limit:
            self.lm_right_z["bg"] = "green"
        else:
            self.lm_right_z["bg"] = "#8f14b8"

        # X label
        self.lm_right_x["text"] = "X: "+ str(round(xvalue, 2))
            
        # Y label
        self.lm_right_y["text"] = "Y: "+ str(round(yvalue, 2))
            
        # Z label
        self.lm_right_z["text"] = "Z: "+ str(round(zvalue, 2))
        
    def lm_status_update(self, frame):
        if frame.is_right_hand: # There is a hand and not fist gesture    
            if not frame.right_hand_fist: # Le cuesta diferenciarlo del thumb up => meter el dedo pulgar dentro de la palma de la mano
                if frame.right_hand_thumb_up: # Thumb up gesture to start the work
                    self.lm_right_status["text"] = " Status: Ready "
                    self.lm_right_status["bg"] = "#0c9132"
                    return True
                else:
                    #print("Not thumb gesture done")
                    self.lm_right_status["text"] = " Status: THUMB UP to start "
                    self.lm_right_status["bg"] = "blue"
                    return False
            else:
                #print("Fist gesture done")
                self.lm_right_status["text"] = " Status: Stopped "
                self.lm_right_status["bg"] = "blue"
                return False
        else:
            self.lm_right_status["text"] = " Status: Hand out of bounds "
            self.lm_right_status["bg"] = "blue"
            return False
            #print("Not right hand available")
    
        # Leap motion state label
        #self.lm_right_status = tk.Label(self.lm_right_label, text=" Status: Ready ")
        self.lm_right_status["text"] = " Status: Ready "
        self.lm_right_status["bg"] = "blue"

    def lm_right_gestures_update(self, frame):        
        if frame.right_hand_set_origin_frame_detected: # Set a new coords as reference
            # Set palm coords as reference
            print("frame_management: set coords as reference")
            self.lm_reference_pos.x = frame.right_hand_palmpos.x
            self.lm_reference_pos.y = frame.right_hand_palmpos.y
            self.lm_reference_pos.z = frame.right_hand_palmpos.z

            # Show ref coords
            ref_coord_x = str(round(self.lm_reference_pos.x, 2))
            ref_coord_y = str(round(self.lm_reference_pos.y, 2))
            ref_coord_z = str(round(self.lm_reference_pos.z, 2))

            self.lm_right_origin_value["text"] = "X: " + ref_coord_x + ", Y: "+ ref_coord_y +", Z:" + ref_coord_z
            self.lm_right_origin_value["bg"] = "#0c9132"

            # Leap motion state label
            self.lm_right_origin_gesture["text"] = "Reference Coords: OK"
            self.lm_right_origin_gesture["bg"] = "#0c9132"
            self.lm_right_swing_gesture["text"] = "Swing Gripper: OK"
            self.lm_right_swing_gesture["bg"] = "#0c9132"
            self.lm_right_rotate_gesture["text"] = "Rotate Gripper: OK"
            self.lm_right_rotate_gesture["bg"] = "#0c9132"
            self.lm_right_turn_gesture["text"] = "Turn Gripper: OK"
            self.lm_right_turn_gesture["bg"] = "#0c9132"

        if frame.right_hand_origin_frame: # there is a reference frame
            self.lm_right_position_update(frame)
            self.lm_right_values_update(frame)
        else:
            #print("Not reference frame set")

            # Leap motion state label
            self.lm_right_origin_gesture["text"] = "Rock gesture to start"
            self.lm_right_origin_gesture["bg"] = "blue"
            self.lm_right_swing_gesture["text"] = "Swing Gripper: Inactive"
            self.lm_right_swing_gesture["bg"] = "blue"
            self.lm_right_rotate_gesture["text"] = "Rotate Gripper: Inactive"
            self.lm_right_rotate_gesture["bg"] = "blue"
            self.lm_right_turn_gesture["text"] = "Turn Gripper: Inactive"
            self.lm_right_turn_gesture["bg"] = "blue"

    def lm_right_values_update(self, frame):

        if frame.right_hand_origin_frame:
            # update rotate, turn and swing of the gripper eef
            #print("Update Gripper rotation")
            #print(frame.right_hand_rotate_value)
            #print("Update Gripper turning")
            #print(frame.right_hand_turn_value)
            #print("Update Gripper swinging")
            #print(frame.right_hand_swing_value)

            rotate = str(round(frame.right_hand_rotate_value, 3))
            turn = str(round(frame.right_hand_turn_value, 3))
            swing= str(round(frame.right_hand_swing_value, 3))

            if swing != 0.0:
                self.lm_right_swing_value["text"] = "Swing Value: " + swing
                self.lm_right_swing_value["bg"] = "#0c9132"
            
            else:
                self.lm_right_swing_value["text"] = "Swing Value: 0.0"
                self.lm_right_swing_value["bg"] = "blue"

            if rotate != 0.0:
                self.lm_right_rotate_value["text"] = "Rotate Value: " + rotate
                self.lm_right_rotate_value["bg"] = "#0c9132"
            
            else:
                self.lm_right_rotate_value["text"] = "Rotate Value: 0.0"
                self.lm_right_rotate_value["bg"] = "blue"

            if turn != 0.0:
                self.lm_right_turn_value["text"] = "Turn Value: " + turn
                self.lm_right_turn_value["bg"] = "#0c9132"
            
            else:
                self.lm_right_turn_value["text"] = "Turn Value: 0.0"
                self.lm_right_turn_value["bg"] = "blue"

            plan_cartesian_path_orientation(arm, swing, rotate, swing, 1.0)
            

        else:
            self.lm_right_origin_value["text"] = "X: 0.0, Y: 0.0, Z: 0.0"
            self.lm_right_origin_value["fg"] = "blue"
            self.lm_right_swing_value["text"] = "Swing Value: 0.0"
            self.lm_right_swing_value["fg"] = "blue"
            self.lm_right_rotate_value["text"] = "Rotate Value: 0.0"
            self.lm_right_rotate_value["fg"] = "blue"
            self.lm_right_turn_value["text"] = "Turn Value: 0.0"
            self.lm_right_turn_value["fg"] = "blue"






#moveit_commander.roscpp_initialize(sys.argv) # initialise moveit
#rospy.init_node('lm_move', anonymous=True) # create a node
#display_trajectory_publisher = rospy.Publisher(
#                                    '/move_group/display_planned_path',
#                                    moveit_msgs.msg.DisplayTrajectory,
#                                    queue_size=1) # we don't want to buffer any messages
#rospy.sleep(10) # Wait for rviz to initialise
#rospy.loginfo("\n=[ INFO: Waiting for RVIZ: DONE! ]=\n")

# This object is an interface to one group of joints.
#group = moveit_commander.MoveGroupCommander("manipulator")

if __name__ == '__main__':
    try:
        # move the robot to an initial comfortable position
        #initialJointValues = [4.111435176058169, 
        #          2.715653621728593, 
        #          0.7256647920137681, 
        #          5.983459446005512, 
        #          -5.682231515319553, 
        #          -6.283185179581844]
        #group.set_joint_value_target(initialJointValues)
        #plan = group.plan()
        #group.execute(plan)
        #rospy.sleep(3) # wait for robot to move to initial position

        # save these positions on run
        #robot_initialPos = geometry_msgs.msg.Pose().position
        #robot_initialPos.x = group.get_current_pose().pose.position.x
        #robot_initialPos.y = group.get_current_pose().pose.position.z
        #robot_initialPos.z = group.get_current_pose().pose.position.y

        ## First initialize moveit_commander and rospy.
        print("============ Starting dual arms moveit")
        #moveit_commander.roscpp_initialize(sys.argv)
        moveit_commander.roscpp_initializer.roscpp_initialize(sys.argv)
        rospy.init_node('ur10_1_dual_moveit',
                      anonymous=True)

        PLANNING_GROUP_GRIPPER = "gripper"
        PLANNING_GROUP_ARM = "manipulator"
        PLANNING_NS = "/ur10_1/"

        ## Instantiate a RobotCommander object.  This object is an interface to
        ## the robot as a whole.
        robot = moveit_commander.RobotCommander("%srobot_description"%PLANNING_NS, ns="/ur10_1")

        ## Instantiate the MoveGroupCommander objects.  This object is an interface
        ## to one group of joints.  In this case the group is the joints in the ur10
        ## arm and gripper. This interface can be used to plan and execute motions on the ur10
        ## arm and gripper.
    
        arm = moveit_commander.move_group.MoveGroupCommander(PLANNING_GROUP_ARM,"%srobot_description"%PLANNING_NS, ns="/ur10_1")
        gripper = moveit_commander.move_group.MoveGroupCommander(PLANNING_GROUP_GRIPPER, "%srobot_description"%PLANNING_NS, ns="/ur10_1")

        ## Instantiate a PlanningSceneInterface object.  This object is an interface
        ## to the world surrounding the robot.
        scene = moveit_commander.PlanningSceneInterface()

        ## We create this DisplayTrajectory publisher which is used below to publish
        ## trajectories for RVIZ to visualize.
        display_trajectory_publisher = rospy.Publisher(
                                          '/move_group/display_planned_path',
                                          moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        rospy.sleep(2)
    
        arm.set_planner_id("RRT")
        arm.set_num_planning_attempts(15)
        arm.allow_looking(True)
        arm.allow_replanning(True)

        gripper.set_planner_id("RRTConnect")
        gripper.set_num_planning_attempts(15)
        gripper.allow_replanning(True)
        gripper.allow_looking(True)

        arm.set_named_target("home")
        arm.go(wait=True)

        print("Main: OK")

        # instance of MoveIt() class
        mi = MoveIt(robot_initialPos)
    except rospy.ROSInterruptException:
        pass