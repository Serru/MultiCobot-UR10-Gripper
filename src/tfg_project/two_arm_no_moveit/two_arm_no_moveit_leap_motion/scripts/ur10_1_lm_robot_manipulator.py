#!/usr/bin/env python

import sys
import copy
import rospy

from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import geometry_msgs.msg

import tf.transformations as tf
from geometry_msgs.msg import Pose, Quaternion
from kinematics_utils import *

from leap_motion.msg import leapcobotright

class CmdTrajectory():
    def __init__(self):
        self.send_trajectory_pub = rospy.Publisher('/ur10_1_pub_ik_trajectory', JointTrajectory, queue_size=10)
        self.send_gripper_cmd_pub = rospy.Publisher('/ur10_1_pub_gripper_control', JointTrajectory, queue_size=10)
        self.current_robot_pose = Pose()
        self.robot_pose_sub = rospy.Subscriber('/ur10_1_robot_pose', Pose, self.update_current_pose)
        self.robot_pose_updated = False
        
        ## LEAP MOTION CONFIG
        self.leap_motion_right_hand_sub = rospy.Subscriber('/leapmotion/data_right', leapcobotright, self.send_leap_motion_trajectory, queue_size=10)

        self.set_leap_motion_reference_position = False
        self.leap_motion_reference_position = geometry_msgs.msg.Pose().position
        self.robot_reference = geometry_msgs.msg.Pose()
        self.start_leap_motion_control = False

    def send_leap_motion_trajectory(self, frame):
        # Gestos:
        # Punyo: Para de enviar nuevas instrucciones
        # Ok: Comienza a enviar instrucciones
        # Pinza: Cierra o abre el gripper
        # Movimiento de munyeca: Rota el Gripper [para la simulacion no se activan]
        # Gesto de ROCK: Configura el frame de referencia de Leap Motion
        # Antes de comenzar, es bueno situar el robot en una posicion en donde trabajara


        # Obtiene la posicion de la palma de la mano
        palm_pos = frame.right_hand_palmpos

        if frame.right_hand_fist:
            self.start_leap_motion_control = False

        if frame.right_hand_thumb_up:
            self.start_leap_motion_control = True

        if self.start_leap_motion_control:
            # Configura la posicion de referencia en Leap Motion,
            # cada vez que reconozca el gesto de ROCK&ROLL
            if frame.right_hand_set_origin_frame_detected:
                self.set_leap_motion_reference_position = True
                self.leap_motion_reference_position.x = palm_pos.x
                self.leap_motion_reference_position.y = palm_pos.y
                self.leap_motion_reference_position.z = palm_pos.z

            # Solamente si la referencia de lm esta configurada
            if self.set_leap_motion_reference_position:
    
                if frame.right_hand_pinch:
                    #self.send_gripper_cmd(frame.right_hand_pinch_value)
                    self.send_gripper_cmd(0.43)
                else:
                    self.send_gripper_cmd(0.0)
    
                ## Se obvia las rotaciones de momento
                #r = frame.right_hand_rotate_value
                #p = frame.right_hand_turn_value
                #y = frame.right_hand_swing_value
                #rpy = tf.quaternion_from_euler(r, p, y)
            
                #print str(self.robot_reference.position.x)
                #print str(palm_pos.x*0.001)
                desired_pos = self.get_transformed_position(palm_pos)
                print("desired_pos (xyz): "+ str(desired_pos.x) + ", " + str(desired_pos.y) + ", " + str(desired_pos.z))
                pos_x = self.robot_reference.position.x + desired_pos.x * 0.001
                pos_y = self.robot_reference.position.y + desired_pos.z * 0.001
                pos_z = self.robot_reference.position.z + desired_pos.y * 0.001
                print("desired_pos (xyz) * 0.001: "+ str(desired_pos.x*0.001) + ", " + str(desired_pos.y*0.001) + ", " + str(desired_pos.z*0.001))
                print("robot_reference (xyz): "+ str(self.robot_reference.position.x) + ", " + str(self.robot_reference.position.y) + ", " + str(self.robot_reference.position.z))            
                print pos_x
                print pos_y
                print pos_z
                
                #self.send_trajectory(palm_pos.x, palm_pos.y, palm_pos.z, rpy[0], rpy[1], rpy[2], rpy[3])
                self.send_trajectory(round(pos_x, 3), round(pos_y, 3), round(pos_z, 3), -0.68945825, -0.72424496, 0.00781949, 0.00744391)

    def get_transformed_position(self, palm_pos):
        
        desired_pos = geometry_msgs.msg.Pose().position
        pos_x = abs(self.leap_motion_reference_position.x - palm_pos.x)
        pos_y = abs(self.leap_motion_reference_position.y - palm_pos.y)
        pos_z = abs(self.leap_motion_reference_position.z - palm_pos.z)

        if palm_pos.x > self.leap_motion_reference_position.x:
            desired_pos.x = -pos_x
        if palm_pos.x < self.leap_motion_reference_position.x:
            desired_pos.x = pos_x
        if palm_pos.x == self.leap_motion_reference_position.x:
            desired_pos.x = 0

        if palm_pos.y > self.leap_motion_reference_position.y:
            desired_pos.y = pos_y
        if palm_pos.y < self.leap_motion_reference_position.y:
            desired_pos.y = -pos_y
        if palm_pos.y == self.leap_motion_reference_position.y:
            desired_pos.y = 0

        if palm_pos.z > self.leap_motion_reference_position.z:
            desired_pos.z = pos_z
        if palm_pos.z < self.leap_motion_reference_position.z:
            desired_pos.z = -pos_z
        if palm_pos.z == self.leap_motion_reference_position.z:
            desired_pos.z = 0

        return desired_pos
    def send_gripper_cmd(self, gripper_distance):
        gripper = JointTrajectory()
        gripper.header.stamp=rospy.Time.now()
        gripper.header.frame_id = "/ur10_1_ee_link"    
        gripper.joint_names = ['ur10_1_robotiq_85_left_knuckle_joint']
        
        points = JointTrajectoryPoint()
        points.positions = [gripper_distance]
        points.time_from_start = rospy.Duration.from_sec(0.4)
        gripper.points.append(points)
        self.send_gripper_cmd_pub.publish(gripper)
        print('\033[93m[' + str(gripper_distance) + ']\033[0m')

    def pick_place(self):
        # Obtener el primer cubo
        self.send_trajectory(0.29, 0.632, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(0.29, 0.775, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(0.29, 0.775, 0.15, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(0.29, 0.775, 0.08, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
    
        # Primer cubo
        self.send_gripper_cmd(0.43)
        rospy.sleep(4)
    
        # Enviar el cubo a la caja
        self.send_trajectory(0.28, 0.775, 0.15, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(-0.70, 0.775, 0.15, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(-0.70, 0.6, 0.15, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(10)
        self.send_gripper_cmd(0.0)
        rospy.sleep(4)
    
        # Obtener el segundo cubo
        self.send_trajectory(0.29, 0.632, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(0.9, 0.632, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(4)
        self.send_trajectory(0.9, 0.632, 0.2, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(0.9, 0.5, 0.2, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(3)
        self.send_trajectory(0.9, 0.5, 0.08, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(5)
    
        # Segundo cubo
        self.send_gripper_cmd(0.43)
        rospy.sleep(4)
    
        # Enviar el cubo a la caja
        self.send_trajectory(0.28, 0.775, 0.15, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(-0.70, 0.775, 0.15, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(-0.70, 0.6, 0.15, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(10)
        self.send_gripper_cmd(0.0)
        rospy.sleep(4)
    
        # Obtener el tercer cubo
        self.send_trajectory(0.29, 0.632, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(0.675, 0.632, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(4)
        self.send_trajectory(0.675, 0.632, 0.2, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(0.675, 0.88, 0.2, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(3)
        self.send_trajectory(0.675, 0.88, 0.1, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(3)
        self.send_trajectory(0.675, 0.88, 0.08, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(5)
    
        # Tercer cubo
        self.send_gripper_cmd(0.43)
        rospy.sleep(4)
    
        # Enviar el cubo a la caja
        self.send_trajectory(0.28, 0.775, 0.15, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(-0.70, 0.775, 0.15, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(-0.70, 0.6, 0.15, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(10)
        self.send_gripper_cmd(0.0)
        rospy.sleep(4)

        # Posicion inicial del brazo
        cmd.send_gripper_cmd(0.0)
        cmd.send_trajectory(-0.24, 0.632, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(7)

    def set_robot_reference(self):
        ## Puede que se anyada las orientaciones... primero posiciones
        self.robot_reference.position.x = self.current_robot_pose.position.x
        self.robot_reference.position.y = self.current_robot_pose.position.y
        self.robot_reference.position.z = self.current_robot_pose.position.z


    def update_current_pose(self, pose):
        self.current_robot_pose.position.x = (-1 * pose.position.x)
        self.current_robot_pose.position.y = (-1 * pose.position.y)
        self.current_robot_pose.position.z = pose.position.z
        self.current_robot_pose.orientation.x = pose.orientation.x
        self.current_robot_pose.orientation.y = pose.orientation.y
        self.current_robot_pose.orientation.z = pose.orientation.z
        self.current_robot_pose.orientation.w = pose.orientation.w
        self.robot_pose_updated = True

    # Set init pose with articular values
    def set_init_pose(self, pos_x, pos_y, pos_z, rot_x, rot_y, rot_z):
        position = JointTrajectory()
        position.header.stamp=rospy.Time.now()
        position.header.frame_id = "/ur10_1_base_link"    
        position.joint_names = ['ur10_1_shoulder_pan_joint','ur10_1_shoulder_lift_joint','ur10_1_elbow_joint',
                          'ur10_1_wrist_1_joint','ur10_1_wrist_2_joint','ur10_1_wrist_3_joint']
        
        rcs = [pos_x, pos_y, pos_z, rot_x, rot_y, rot_z]
        
        points = JointTrajectoryPoint()
        points.positions = rcs
        points.time_from_start = rospy.Duration.from_sec(5)
        position.points.append(points)
        self.send_trajectory_pub.publish(position)

    def send_trajectory(self, pos_x, pos_y, pos_z, rot_x, rot_y, rot_z, rot_w):
        position = JointTrajectory()
        position.header.stamp=rospy.Time.now()
        position.header.frame_id = "/ur10_1_base_link"    
        position.joint_names = ['ur10_1_shoulder_pan_joint','ur10_1_shoulder_lift_joint','ur10_1_elbow_joint',
                          'ur10_1_wrist_1_joint','ur10_1_wrist_2_joint','ur10_1_wrist_3_joint']
        
        rate = rospy.Rate(10)
        while not self.robot_pose_updated:
            rate.sleep()

        #print self.current_robot_pose

        (roll, pitch, yaw) = tf.euler_from_quaternion([
                                            self.current_robot_pose.orientation.x,
                                            self.current_robot_pose.orientation.y,
                                            self.current_robot_pose.orientation.z,
                                            self.current_robot_pose.orientation.w])

        rcs = [ self.current_robot_pose.position.x,
                self.current_robot_pose.position.y,
                self.current_robot_pose.position.z,
                roll, pitch, yaw]

        #print rcs
        #array_pos = fwd_kin(self.current_robot_pose, 'r', 'n')
        #print(cartesian_pos)

        ps = Pose()
        ps.position.x = pos_x
        ps.position.y = pos_y
        ps.position.z = pos_z
        ps.orientation.x = rot_x
        ps.orientation.y = rot_y
        ps.orientation.z = rot_z
        ps.orientation.w = rot_w
    
        #state = []
    
        #sol = inv_kin(ps, array_pos)
        #print(sol)

        
        points = JointTrajectoryPoint()
        try:
            points.positions = inv_kin(ps, rcs)
        except Exception:
            print('\033[91m[ Singularidad, valores:' + str(ps.position.x) + ', ' + str(ps.position.y) + ', ' + str(ps.position.z) + ']\033[0m')
        
        duration = max([abs(points.positions[0])-abs(rcs[0]), abs(points.positions[1])-abs(rcs[1]), abs(points.positions[2])-abs(rcs[2])])
        print duration

        points.time_from_start = rospy.Duration.from_sec(duration*0.1)
        position.points.append(points)
        self.send_trajectory_pub.publish(position)
        #state = sol
        #rospy.sleep(0.1)
        self.robot_pose_updated = False
        print points.positions
        print('\033[93m[' + str(ps.position.x) + ', ' + str(ps.position.y) + ', ' + str(ps.position.z) + ']\033[0m')
        
if __name__ == '__main__':
    rospy.init_node('ur10_1_robot_manipulator_lm', anonymous=True)
    cmd = CmdTrajectory()
    rpy = tf.quaternion_from_euler(-3.12, 0.0, 1.62)
    print rpy
    #[-0.68945825 -0.72424496  0.00781949  0.00744391]
    #cmd.send_trajectory(-0.6, -0.16, 0.62, rpy[0], rpy[1], rpy[2], rpy[3])
    
    # Posicion inicial del brazo
    cmd.set_init_pose(2.176, -1.518, -1.671, -1.511, 1.589, -1.014)
    cmd.send_gripper_cmd(0.0)
    cmd.send_trajectory(-0.24, 0.632, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
    rospy.sleep(4)
    cmd.send_trajectory(0.24, 0.632, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
    rospy.sleep(4)
    cmd.send_trajectory(0.24, 0.8, 0.15, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
    rospy.sleep(4)


    cmd.set_robot_reference()

    # Ejemplo de movmiento no deseado
    #cmd.send_trajectory(-0.30, 0.300, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
    #rospy.sleep(4)
    #cmd.send_trajectory(-0.40, 0.0, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
    #rospy.sleep(4)
    #cmd.send_trajectory(-0.80, -0.3, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
    #rospy.sleep(4)
    
    #cmd.pick_place()
    while not rospy.is_shutdown():
        rospy.sleep(0.1)