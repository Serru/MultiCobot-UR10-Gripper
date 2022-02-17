#!/usr/bin/env python

import sys
import copy
import rospy

from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import geometry_msgs.msg
import moveit_msgs.msg
from std_msgs.msg import String

import moveit_commander

import tf.transformations as tf
from geometry_msgs.msg import Pose, Quaternion
#from kinematics_utils import *

from leap_motion.msg import leapcobotleft

class CmdTrajectory():
    def __init__(self):
        self.robot_pose_updated = False
        
        ## LEAP MOTION CONFIG
        self.leap_motion_left_hand_sub = rospy.Subscriber('/leapmotion/data_left', leapcobotleft, self.send_leap_motion_trajectory, queue_size=1)

        self.set_leap_motion_reference_position = False
        self.leap_motion_reference_position = geometry_msgs.msg.Pose().position
        self.robot_reference = geometry_msgs.msg.Pose()
        self.start_leap_motion_control = False

        self.PLANNING_GROUP_GRIPPER = "gripper"
        self.PLANNING_GROUP_ARM = "manipulator"
        self.PLANNING_NS = "/ur10_2/"
        self.executing = False

        ## Instantiate a RobotCommander object.  This object is an interface to
        ## the robot as a whole.
        self.robot = moveit_commander.RobotCommander("%srobot_description"%self.PLANNING_NS, ns="/ur10_2/")

        self.arm = moveit_commander.move_group.MoveGroupCommander(self.PLANNING_GROUP_ARM,"%srobot_description"%self.PLANNING_NS, ns="/ur10_2/")
        self.gripper = moveit_commander.move_group.MoveGroupCommander(self.PLANNING_GROUP_GRIPPER, "%srobot_description"%self.PLANNING_NS, ns="/ur10_2/")

        self.display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory, queue_size=10)

    def send_leap_motion_trajectory(self, frame):
        # Gestos:
        # Punyo: Para de enviar nuevas instrucciones
        # Ok: Comienza a enviar instrucciones
        # Pinza: Cierra o abre el gripper
        # Movimiento de munyeca: Rota el Gripper [para la simulacion no se activan]
        # Gesto de ROCK: Configura el frame de referencia de Leap Motion
        # Antes de comenzar, es bueno situar el robot en una posicion en donde trabajara


        # Obtiene la posicion de la palma de la mano
        palm_pos = frame.left_hand_palmpos

        if frame.left_hand_fist:
            self.start_leap_motion_control = False

        if frame.left_hand_thumb_up:
            self.start_leap_motion_control = True

        if self.start_leap_motion_control:
            # Configura la posicion de referencia en Leap Motion,
            # cada vez que reconozca el gesto de ROCK&ROLL
            if frame.left_hand_set_origin_frame_detected:
                self.set_leap_motion_reference_position = True
                self.leap_motion_reference_position.x = palm_pos.x
                self.leap_motion_reference_position.y = palm_pos.y
                self.leap_motion_reference_position.z = palm_pos.z

            # Solamente si la referencia de lm esta configurada
            if self.set_leap_motion_reference_position:
    
                if frame.left_hand_pinch:
                    #self.send_gripper_cmd(frame.left_hand_pinch_value)
                    self.send_gripper_cmd(self.gripper, frame.left_hand_pinch_value)
                else:
                    self.send_gripper_cmd(self.gripper, 0.0)
    
                if not self.executing:
                    self.executing = True
                    desired_pos = self.get_transformed_position(palm_pos)
                    #print("desired_pos (xyz): "+ str(desired_pos.x) + ", " + str(desired_pos.y) + ", " + str(desired_pos.z))
                    pos_x = self.robot_reference.position.x + desired_pos.x * 0.001
                    pos_y = self.robot_reference.position.y + desired_pos.z * 0.001
                    pos_z = self.robot_reference.position.z + desired_pos.y * 0.001
                    print("desired_pos (xyz) * 0.001: "+ str(desired_pos.x*0.001) + ", " + str(desired_pos.y*0.001) + ", " + str(desired_pos.z*0.001))
                    print("robot_reference (xyz): "+ str(self.robot_reference.position.x) + ", " + str(self.robot_reference.position.y) + ", " + str(self.robot_reference.position.z))
                    current_pose = self.arm.get_current_pose().pose.position
                    #print("robot_current_pose (xyz): "+ str(current_pose.x) + ", " + str(current_pose.y) + ", " + str(current_pose.z))
                    #print pos_x
                    #print pos_y
                    #print pos_z
                    
                    self.send_trajectory(self.arm, round(pos_x, 3), round(pos_y, 3), round(pos_z, 3))
                    self.executing = False

    def get_transformed_position(self, palm_pos):
        
        desired_pos = geometry_msgs.msg.Pose().position
        pos_x = abs(self.leap_motion_reference_position.x - palm_pos.x)
        pos_y = abs(self.leap_motion_reference_position.y - palm_pos.y)
        pos_z = abs(self.leap_motion_reference_position.z - palm_pos.z)

        if palm_pos.x > self.leap_motion_reference_position.x:
            desired_pos.x = pos_x
        if palm_pos.x < self.leap_motion_reference_position.x:
            desired_pos.x = -pos_x
        if palm_pos.x == self.leap_motion_reference_position.x:
            desired_pos.x = 0

        if palm_pos.y > self.leap_motion_reference_position.y:
            desired_pos.y = pos_y
        if palm_pos.y < self.leap_motion_reference_position.y:
            desired_pos.y = -pos_y
        if palm_pos.y == self.leap_motion_reference_position.y:
            desired_pos.y = 0

        if palm_pos.z > self.leap_motion_reference_position.z:
            desired_pos.z = -pos_z
        if palm_pos.z < self.leap_motion_reference_position.z:
            desired_pos.z = pos_z
        if palm_pos.z == self.leap_motion_reference_position.z:
            desired_pos.z = 0

        return desired_pos
    def send_gripper_cmd(self, group, gripper_distance):
        self.gripper.set_joint_value_target("robotiq_85_left_knuckle_joint", round(gripper_distance, 2))
        self.gripper.go(wait = True)
        print('\033[93m[' + str(gripper_distance) + ']\033[0m')


    def set_robot_reference(self):
        ## Puede que se anyada las orientaciones... primero posiciones
        self.robot_reference.position = self.arm.get_current_pose().pose.position

    # Set init pose with articular values
    def set_init_pose(self):
        self.arm.set_named_target("home")
        self.arm.go(wait = True)
        self.gripper.set_named_target("gripper_open")
        self.gripper.go(wait = True)
        #print(self.gripper.get_current_joint_values())
        #print(self.gripper.get_joints())
        print(self.arm.get_current_joint_values())
        print(self.arm.get_current_pose())

#    def send_trajectory(self,group, pos_x, pos_y, pos_z, rot_x, rot_y, rot_z, rot_w):
    def send_trajectory(self, group, pos_x, pos_y, pos_z):
        group.set_pose_target([pos_x, pos_y, pos_z,  0.714702085642, 0.0169188757643, -0.69900917270, 0.0173452269234], "ee_link")
        group.go(wait=True)
        print('\033[93m[' + str(pos_x) + ', ' + str(pos_y) + ', ' + str(pos_z) + ']\033[0m')
        
if __name__ == '__main__':
    ## First initialize moveit_commander and rospy.
    print("============ Starting dual arms moveit")
    moveit_commander.roscpp_initializer.roscpp_initialize(sys.argv)
    rospy.init_node('ur10_1_two_arm_moveit_leap_motion',
                  anonymous=True)
    rospy.sleep(2)
    
    cmd = CmdTrajectory()
    cmd.set_init_pose()
    cmd.set_robot_reference()    
    while not rospy.is_shutdown():
        rospy.sleep(0.1)