##Instalación y configuración para un único robot UR10 sin MoveIt! y Leap Motion

Esta sección la continuación del apartado anterior, pero en vez de tener un script que envié las trayectorias deseadas, será mediante Leap Motion el cual es controlado por una persona.

Para ello se va a modificar parte del repositorio de Leap Motion directamente y adaptarlo para que controle adecuadamente el robot simulado.

La idea es identica a lo presentado anteriormente, pero tiene como entrada de datos la información que Leap Motion provee.

Hay que tratar esta informaócin adecuadamente con la API que provee y adecuarlo para su uso.

### Leap Motion GUI
Se proveerá de un GUI para el usuario en el futuro si se decide que esta es la versión que se elegirá para realizar las pruebas en el robot real.

### Implementación del manipulador con Leap Motion
Para realizar la implementaócin hay que tener varias cosas en cuenta:
* Las referencias que de la posición del robot que se obtiene de script *ur10_robot_pose.py*
* El entorno de trabajo del robot
* El entorno de trabajo de Leap Motion

Como se desea que el robot siga los movimientos que realiza Leap Motion, se va a implementar un controlador que su espacio de trabajo viene contenido dentro del espacio de trabajo del robot.

Hay otra posibilidad y es tratar leap motion como si fuese un Joystick, que se implementará si hay tiempo en el futuro.

#### Creación del paquete
```{bash}
cd ~/tfg_multirobot/src/tfg_project/one_arm_no_moveit
catkin_create_pkg one_arm_no_moveit_leap_motion rospy
cd one_arm_no_moveit_leap_motion
mkdir scripts
cd scripts
cp ~/tfg_multirobot/src/tfg_project/one_arm_no_moveit/one_arm_no_moveit_manipulator/scripts/kinematics_utils.py .
touch lm_robot_manipulator.py
touch sender.py
touch leap_interface.py
```

Cotenido del fichero *lm_robot_manipulator.py*
```{C}
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
        self.send_trajectory_pub = rospy.Publisher('/pub_ik_trajectory', JointTrajectory, queue_size=10)
        self.send_gripper_cmd_pub = rospy.Publisher('/pub_gripper_control', JointTrajectory, queue_size=10)
        self.current_robot_pose = Pose()
        self.robot_pose_sub = rospy.Subscriber('/robot_pose', Pose, self.update_current_pose)
        self.robot_pose_updated = False
        
        ## LEAP MOTION CONFIG
        self.leap_motion_right_hand_sub = rospy.Subscriber('/leapmotion/data', leapcobotright, self.send_leap_motion_trajectory, queue_size=10)

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
        gripper.header.frame_id = "/ee_link"    
        gripper.joint_names = ['robotiq_85_left_knuckle_joint']
        
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
        position.header.frame_id = "/base_link"    
        position.joint_names = ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint',
                          'wrist_1_joint','wrist_2_joint','wrist_3_joint']
        
        rcs = [pos_x, pos_y, pos_z, rot_x, rot_y, rot_z]
        
        points = JointTrajectoryPoint()
        points.positions = rcs
        points.time_from_start = rospy.Duration.from_sec(5)
        position.points.append(points)
        self.send_trajectory_pub.publish(position)

    def send_trajectory(self, pos_x, pos_y, pos_z, rot_x, rot_y, rot_z, rot_w):
        position = JointTrajectory()
        position.header.stamp=rospy.Time.now()
        position.header.frame_id = "/base_link"    
        position.joint_names = ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint',
                          'wrist_1_joint','wrist_2_joint','wrist_3_joint']
        
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
    rospy.init_node('robot_manipulator', anonymous=True)
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
```

Cotenido del fichero *sender.py*
```{C}
#!/usr/bin/env python

""" For backwards compatibility with the old driver files
                Will be DELETED in the future               """

__author__ = 'Miguel Burgh Olivan'

import argparse

import rospy
import leap_interface

from leap_motion.msg import leap_motion
from leap_motion.msg import leapcobotright, leapcobotleft

FREQUENCY_ROSTOPIC_DEFAULT = 0.1
NODENAME = 'one_arm_no_moveit_lm_pub'
PARAMNAME_FREQ = 'freq'
PARAMNAME_FREQ_ENTIRE = '/' + NODENAME + '/' + PARAMNAME_FREQ

def sender():
    '''
    This method publishes the data defined in leapcobotright to /leapmotion/data
    '''
    rospy.loginfo("Parameter set on server: PARAMNAME_FREQ={}".format(rospy.get_param(PARAMNAME_FREQ_ENTIRE, FREQUENCY_ROSTOPIC_DEFAULT)))

    li = leap_interface.Runner()
    li.setDaemon(True)
    li.start()

    pub_ros_right   = rospy.Publisher('leapmotion/data', leapcobotright, queue_size=1)
    #pub_ros_left   = rospy.Publisher('leapmotion/data', leapcobotleft, queue_size=1)
    rospy.init_node(NODENAME)

    while not rospy.is_shutdown():
        right_hand_palm_pos_    = li.get_right_hand_palmpos()   # Palm's position
        left_hand_palm_pos_    = li.get_left_hand_palmpos()     # Palm's position

        # Right hand information
        msg_right = leapcobotright()
        msg_right.is_right_hand = li.get_is_right_hand()                     # Right hand detected
        msg_right.right_hand_palmpos.x = right_hand_palm_pos_[0]
        msg_right.right_hand_palmpos.y = right_hand_palm_pos_[1]
        msg_right.right_hand_palmpos.z = right_hand_palm_pos_[2]

        msg_right.right_hand_fist = li.get_right_hand_fist()                 # Fist gesture recognize
        msg_right.right_hand_thumb_up = li.get_right_hand_thumb_up()         # Thumb up gesture recognize
        msg_right.right_hand_pinch = li.get_right_hand_pinch()               # Pinch gesture recognize
        msg_right.right_hand_pinch_value = li.get_right_hand_pinch_value()   # Pinch gesture value
        msg_right.right_hand_origin_frame = li.get_right_hand_origin_frame() # Reference frame set
        msg_right.right_hand_set_origin_frame_detected = li.get_right_hand_set_origin_frame_detected()
        msg_right.right_hand_rotate_value = li.get_right_hand_rotate_value() # Values between [-1..0..1]
        msg_right.right_hand_turn_value = li.get_right_hand_turn_value()     # Values between [-1..0..1]
        msg_right.right_hand_swing_value = li.get_right_hand_swing_value()   # Values between [-1..0..1]


        # Left hand information
        msg_left = leapcobotleft()
        msg_left.is_left_hand = li.get_is_left_hand()                       # Left hand detected
        msg_left.left_hand_palmpos.x = left_hand_palm_pos_[0]
        msg_left.left_hand_palmpos.y = left_hand_palm_pos_[1]
        msg_left.left_hand_palmpos.z = left_hand_palm_pos_[2]

        msg_left.left_hand_fist = li.get_left_hand_fist()                    # Fist gesture recognize
        msg_left.left_hand_thumb_up = li.get_left_hand_thumb_up()            # Thumb up gesture recognize
        msg_left.left_hand_pinch = li.get_left_hand_pinch()                  # Pinch gesture recognize
        msg_left.left_hand_pinch_value = li.get_left_hand_pinch_value()   # Pinch gesture value
        msg_left.left_hand_origin_frame = li.get_left_hand_origin_frame()   # Reference frame set
        msg_left.left_hand_set_origin_frame_detected = li.get_left_hand_set_origin_frame_detected()
        msg_left.left_hand_rotate_value = li.get_left_hand_rotate_value()    # Values between [-1..0..1]
        msg_left.left_hand_turn_value = li.get_left_hand_turn_value()       # Values between [-1..0..1]
        msg_left.left_hand_swing_value = li.get_left_hand_swing_value()      # Values between [-1..0..1]

        print(msg_right)
        print("\n")
        pub_ros_right.publish(msg_right)
        #pub_ros_left.publish(msg_left)
        rospy.sleep(rospy.get_param(PARAMNAME_FREQ_ENTIRE, FREQUENCY_ROSTOPIC_DEFAULT))


if __name__ == '__main__':
    try:
        sender()
    except rospy.ROSInterruptException:
        pass

```

Cotenido del fichero *leap_interface.py*
```{C}
#################################################################################
# Copyright (C) 2012-2013 Leap Motion, Inc. All rights reserved.                #
# Leap Motion proprietary and confidential. Not for distribution.               #
# Use subject to the terms of the Leap Motion SDK Agreement available at        #
# https://developer.leapmotion.com/sdk_agreement, or another agreement          #
# between Leap Motion and you, your company or other organization.              #
#################################################################################

#################################################################################
# Altered LEAP example by Florian Lier, you need to have the LEAP SDK installed #
# for this to work properly ;)                                                  #
# This interface provides access to the LEAP MOTION hardware, you will need to  #
# have the official LEAP MOTION SDK installed in order to load the shared       #
# provided with the SDK.                                                        #
#################################################################################

""" For backwards compatibility with the old driver files
                Will be DELETED in the future               """

#################################################################################
# Altered LEAP leap_interface by Miguel Burgh, you need to have the LEAP SDK    #
# installed for this to work properly ;)                                        #
# This interface provides access to the LEAP MOTION hardware, you will need to  #
# have the official LEAP MOTION SDK installed in order to load the shared       #
# provided with the SDK.                                                        #
#################################################################################


# sys.path.append("/home/YOUR_NAME/path/to/Leap_Developer/LeapSDK/lib")
# sys.path.append("/home/YOUR_NAME/path/to/Leap_Developer/Leap_Developer/LeapSDK/lib/x64")
import threading
import time

import Leap
from Leap import Vector


# Set (append) your PYTHONPATH properly, or just fill in the location of your LEAP
# SDK folder, e.g., $HOME/LeapSDK/lib where the Leap.py lives and /LeapSDK/lib/x64 or
# x86 where the *.so files reside.
# Below, you can see the "dirty" version - NOT RECOMMENDED!

class LeapInterface(Leap.Listener):
    def on_init(self, controller):
        self.is_right_hand = False
        self.is_left_hand = False
        self.right_hand_palm_pos = [0, 0, 0]
        self.left_hand_palm_pos = [0, 0, 0]
        print "Initialized Leap Motion Device"

    def on_connect(self, controller):
        print "Connected to Leap Motion Controller"

        # Enable gestures
        controller.enable_gesture(Leap.Gesture.TYPE_CIRCLE);
        controller.enable_gesture(Leap.Gesture.TYPE_KEY_TAP);
        controller.enable_gesture(Leap.Gesture.TYPE_SCREEN_TAP);
        controller.enable_gesture(Leap.Gesture.TYPE_SWIPE);

    def on_disconnect(self, controller):
        # Note: not dispatched when running in a debugger.
        print "Disconnected Leap Motion"

    def on_exit(self, controller):
        print "Exited Leap Motion Controller"

    def on_frame(self, controller):
        # Get the most recent frame and report some basic information
        frame = controller.frame()

        print "Frame id: %d, timestamp: %d, hands: %d, fingers: %d, tools: %d, gestures: %d" % (
            frame.id, frame.timestamp, len(frame.hands), len(frame.fingers), len(frame.tools), len(frame.gestures()))

        self.is_left_hand = False
        self.is_right_hand = False
        if not frame.hands.is_empty:  # recently changed in API
            # Get the first hand
            for hand in frame.hands:
                if hand.is_right:
                    self.is_right_hand = True
                    pos = hand.palm_position
                    self.right_hand_palm_pos[0] = pos.x
                    self.right_hand_palm_pos[1] = pos.y
                    self.right_hand_palm_pos[2] = pos.z
                elif hand.is_left:
                    self.is_left_hand = True
                    pos = hand.palm_position
                    self.left_hand_palm_pos[0] = pos.x
                    self.left_hand_palm_pos[1] = pos.y
                    self.left_hand_palm_pos[2] = pos.z

    def get_is_right_hand(self):  # added
        return self.is_right_hand

    def get_is_left_hand(self):  # added
        return self.is_left_hand

    def get_right_hand_palmpos(self):
        return self.right_hand_palm_pos

    def get_left_hand_palmpos(self):
        return self.left_hand_palm_pos

class Fist_Thumb_up_Listener(Leap.Listener):
    '''
    detects when the right or left hand is curled into a fist
    '''

    def on_init(self, controller):
        self.GRAB_STRENGTH_THRESHOLD = 0.5
        self.right_hand_fist = False
        self.left_hand_fist = False
        self.right_hand_thumb_up = False
        self.left_hand_thumb_up = False

    def on_frame(self, controller):
        frame = controller.frame()

        for hand in frame.hands:
            handType = "Left hand" if hand.is_left else "Right hand"

            if handType == "Right hand":
                if hand.grab_strength > self.GRAB_STRENGTH_THRESHOLD:
                    thumb_finger = hand.fingers.finger_type(0)
                    for _ in thumb_finger:
                        if len(thumb_finger.extended()) == 0:
                            # print("fist in right hand: Gesture to stop controling the cobot A")
                            self.right_hand_fist = True
                            self.right_hand_thumb_up = False
                        elif len(thumb_finger.extended()) == 1:
                            # print("Pulgar extendido en la mano derecha")
                            self.right_hand_thumb_up = True
                            self.right_hand_fist = False
            else:
                if hand.grab_strength > self.GRAB_STRENGTH_THRESHOLD:
                    thumb_finger = hand.fingers.finger_type(0)
                    for _ in thumb_finger:
                        if len(thumb_finger.extended()) == 0:
                            # print("fist in left hand: Gesture to stop controling the cobot B")
                            self.left_hand_fist = True
                            self.left_hand_thumb_up = False
                        elif len(thumb_finger.extended()) == 1:
                            # print("Pulgar extendido en la mano izquierda")
                            self.left_hand_fist = False
                            self.left_hand_thumb_up = True

    def get_right_hand_fist(self):
        return self.right_hand_fist

    def get_right_hand_thumb_up(self):
        return self.right_hand_thumb_up

    def get_left_hand_fist(self):
        return self.left_hand_fist

    def get_left_hand_thumb_up(self):
        return self.left_hand_thumb_up


class GripperListener(Leap.Listener):
    '''
    detects when the right or left hand is making a gripper gesture
    '''

    def on_init(self, controller):
        self.PINCH_STRENGTH_THRESHOLD = 0.2
        self.right_hand_pinch = False
        self.left_hand_pinch = False
        self.right_hand_pinch_value = 0.0
        self.left_hand_pinch_value = 0.0

    def on_frame(self, controller):
        frame = controller.frame()

        for hand in frame.hands:
            handType = "Left hand" if hand.is_left else "Right hand"
            if handType == "Right hand":
                if hand.pinch_strength > self.PINCH_STRENGTH_THRESHOLD:
                    # print("Gripper Close A")
                    self.right_hand_pinch = True
                    self.right_hand_pinch_value = round(hand.pinch_strength - self.PINCH_STRENGTH_THRESHOLD, 2)
                else:
                    # print("Gripper Open A")
                    self.right_hand_pinch = False
                    self.right_hand_pinch_value = 0.0
            else:
                if hand.pinch_strength > self.PINCH_STRENGTH_THRESHOLD:
                    # print("Gripper Close B")
                    self.left_hand_pinch = True
                    self.left_hand_pinch_value = round(hand.pinch_strength - self.PINCH_STRENGTH_THRESHOLD, 2)
                else:
                    # print("Gripper Open B")
                    self.left_hand_pinch = False
                    self.left_hand_pinch_value = 0.0

    def get_right_hand_pinch(self):
        return self.right_hand_pinch

    def get_left_hand_pinch(self):
        return self.left_hand_pinch

    def get_right_hand_pinch_value(self):
        return self.right_hand_pinch_value

    def get_left_hand_pinch_value(self):
        return self.left_hand_pinch_value



class Gripper_Control_Gestures_Listener(Leap.Listener):
    '''
    Detects when the right or left hand are making gestures to control the Gripper:
    Rock hand gesture: To set the current frame as the one for reference
    Swinging up and down: To turn the gripper up or down
    Rotate the wrist to the right/left: To Rotate the gripper right/left
    Turn the hand to the right/left: To turn the gripper to the right/left
    '''

    def on_init(self, controller):
        self.ORIGIN_FRAME_RIGHT_HAND = None
        self.ORIGIN_FRAME_LEFT_HAND = None

        self.RIGHT_FRAME_EMPTY = True
        self.LEFT_FRAME_EMPTY = True

        self.RIGHT_GESTURE_DETECTED = False
        self.LEFT_GESTURE_DETECTED = False

        self.right_hand_rotate_value = 0.0
        self.right_hand_turn_value = 0.0
        self.right_hand_swing_value = 0.0

        self.left_hand_rotate_value = 0.0
        self.left_hand_turn_value = 0.0
        self.left_hand_swing_value = 0.0

    def on_frame(self, controller):
        frame = controller.frame()

        self.RIGHT_GESTURE_DETECTED = False
        self.LEFT_GESTURE_DETECTED = False

        for hand in frame.hands:
            handType = "Left hand" if hand.is_left else "Right hand"

            thumb_finger = hand.fingers.finger_type(0)
            index_finger = hand.fingers.finger_type(1)
            middle_finger = hand.fingers.finger_type(2)
            ring_finger = hand.fingers.finger_type(3)
            pinky_finger = hand.fingers.finger_type(4)

            if len(thumb_finger.extended()) == 1 and len(index_finger.extended()) == 1 and len(
                    pinky_finger.extended()) == 1 and len(middle_finger.extended()) == 0 and len(
                    ring_finger.extended()) == 0:
                if handType == "Right hand":
                    print("Set origin frame for right hand")
                    self.ORIGIN_FRAME_RIGHT_HAND = frame
                    self.RIGHT_FRAME_EMPTY = False
                    self.RIGHT_GESTURE_DETECTED = True
                    print(self.ORIGIN_FRAME_RIGHT_HAND)
                else:
                    print("Set origin frame for left hand")
                    self.ORIGIN_FRAME_LEFT_HAND = frame
                    self.LEFT_FRAME_EMPTY = False
                    self.LEFT_GESTURE_DETECTED = True
                    print(self.ORIGIN_FRAME_LEFT_HAND)

            if handType == "Right hand" and not self.RIGHT_FRAME_EMPTY:
                # print("[RIGHT]")
                self.right_hand_rotate_value = self.rotate_wrist_gesture(self.ORIGIN_FRAME_RIGHT_HAND, frame)
                self.right_hand_turn_value = self.turn_hand_gesture(self.ORIGIN_FRAME_RIGHT_HAND, frame)
                self.right_hand_swing_value = self.swing_hand_gesture(self.ORIGIN_FRAME_RIGHT_HAND, frame)
            elif handType == "Left hand" and not self.LEFT_FRAME_EMPTY:
                # print("[LEFT]")
                self.left_hand_rotate_value = self.rotate_wrist_gesture(self.ORIGIN_FRAME_LEFT_HAND, frame)
                self.left_hand_turn_value = self.turn_hand_gesture(self.ORIGIN_FRAME_LEFT_HAND, frame)
                self.left_hand_swing_value = self.swing_hand_gesture(self.ORIGIN_FRAME_LEFT_HAND, frame)

    def rotate_wrist_gesture(self, since_frame, current_frame):  # rotate the z axis
        rotation_around_z_axis = current_frame.rotation_angle(since_frame, Vector.z_axis)
        z_axis_threshold_left = -0.40
        z_axis_threshold_right = 0.40
        if rotation_around_z_axis > z_axis_threshold_right:
            # print("Rotating wrist to the right")
            return rotation_around_z_axis - z_axis_threshold_right

        elif rotation_around_z_axis < z_axis_threshold_left:
            # print("Rotating wrist to the left")
            return rotation_around_z_axis - z_axis_threshold_left
        return 0.0

    def turn_hand_gesture(self, since_frame, current_frame):  # rotate the y axis
        rotation_around_y_axis = current_frame.rotation_angle(since_frame, Vector.y_axis)
        y_axis_threshold_left = -0.15
        y_axis_threshold_right = 0.35
        if rotation_around_y_axis > y_axis_threshold_right:
            # print("Turning hand to the right")
            return (rotation_around_y_axis - y_axis_threshold_right)
        elif rotation_around_y_axis < y_axis_threshold_left:
            # print("Turning hand to the left")
            return rotation_around_y_axis - y_axis_threshold_left
        return 0.0

    def swing_hand_gesture(self, since_frame, current_frame):  # rotate the x axis
        rotation_around_x_axis = current_frame.rotation_angle(since_frame, Vector.x_axis)
        x_axis_threshold_down = 0.45
        x_axis_threshold_up = -0.05
        if rotation_around_x_axis < x_axis_threshold_up:
            # print("Swinging up the wrist")
            return rotation_around_x_axis - x_axis_threshold_up
        elif rotation_around_x_axis > x_axis_threshold_down:
            # print("Swinging down the wrist")
            return rotation_around_x_axis - x_axis_threshold_down
        return 0.0

    def get_right_hand_origin_frame(self):
        return not self.RIGHT_FRAME_EMPTY

    def get_left_hand_origin_frame(self):
        return not self.LEFT_FRAME_EMPTY

    def get_right_hand_set_origin_frame_detected(self):
        return not self.RIGHT_GESTURE_DETECTED

    def get_left_hand_set_origin_frame_detected(self):
        return not self.LEFT_GESTURE_DETECTED

    def get_right_hand_rotate_value(self):
        return self.right_hand_rotate_value

    def get_right_hand_turn_value(self):
        return self.right_hand_turn_value

    def get_right_hand_swing_value(self):
        return self.right_hand_swing_value

    def get_left_hand_rotate_value(self):
        return self.left_hand_rotate_value

    def get_left_hand_turn_value(self):
        return self.left_hand_turn_value

    def get_left_hand_swing_value(self):
        return self.left_hand_swing_value

class Runner(threading.Thread):

    def __init__(self, arg=None):
        threading.Thread.__init__(self)
        self.arg = arg
        self.listener = LeapInterface()
        self.fist_thumb_up_listener = Fist_Thumb_up_Listener()
        self.gripper_listener = GripperListener()
        self.gripper_control_gestures_listener = Gripper_Control_Gestures_Listener()
        self.controller = Leap.Controller()

        self.controller.add_listener(self.listener)
        self.controller.add_listener(self.fist_thumb_up_listener)
        self.controller.add_listener(self.gripper_listener)
        self.controller.add_listener(self.gripper_control_gestures_listener)

    def __del__(self):
        self.controller.remove_listener(self.listener)
        self.controller.remove_listener(self.listener)
        self.controller.remove_listener(self.fist_thumb_up_listener)
        self.controller.remove_listener(self.gripper_listener)
        self.controller.remove_listener(self.gripper_control_gestures_listener)

    def get_is_right_hand(self):  # added
        return self.listener.get_is_right_hand()

    def get_is_left_hand(self):  # added
        return self.listener.get_is_left_hand()

    def get_right_hand_fist(self):
        return self.fist_thumb_up_listener.get_right_hand_fist()

    def get_right_hand_thumb_up(self):
        return self.fist_thumb_up_listener.get_right_hand_thumb_up()

    def get_left_hand_fist(self):
        return self.fist_thumb_up_listener.get_left_hand_fist()

    def get_left_hand_thumb_up(self):
        return self.fist_thumb_up_listener.get_left_hand_thumb_up()

    def get_right_hand_pinch(self):
        return self.gripper_listener.get_right_hand_pinch()

    def get_left_hand_pinch(self):
        return self.gripper_listener.get_left_hand_pinch()

    def get_right_hand_pinch_value(self):
        return self.gripper_listener.get_right_hand_pinch_value()

    def get_left_hand_pinch_value(self):
        return self.gripper_listener.get_left_hand_pinch_value()

    def get_right_hand_origin_frame(self):
        return self.gripper_control_gestures_listener.get_right_hand_origin_frame()

    def get_left_hand_origin_frame(self):
        return self.gripper_control_gestures_listener.get_left_hand_origin_frame()

    def get_right_hand_set_origin_frame_detected(self):
        return not self.gripper_control_gestures_listener.get_right_hand_set_origin_frame_detected()

    def get_left_hand_set_origin_frame_detected(self):
        return not self.gripper_control_gestures_listener.get_left_hand_set_origin_frame_detected()

    def get_right_hand_rotate_value(self):
        return self.gripper_control_gestures_listener.get_right_hand_rotate_value()

    def get_right_hand_turn_value(self):
        return self.gripper_control_gestures_listener.get_right_hand_turn_value()

    def get_right_hand_swing_value(self):
        return self.gripper_control_gestures_listener.get_right_hand_swing_value()

    def get_left_hand_rotate_value(self):
        return self.gripper_control_gestures_listener.get_left_hand_rotate_value()

    def get_left_hand_turn_value(self):
        return self.gripper_control_gestures_listener.get_left_hand_turn_value()

    def get_left_hand_swing_value(self):
        return self.gripper_control_gestures_listener.get_left_hand_swing_value()

    def get_right_hand_palmpos(self):
        return self.listener.right_hand_palm_pos

    def get_left_hand_palmpos(self):
        return self.listener.left_hand_palm_pos

    def run(self):
        while True:
            # Save some CPU time
            # time.sleep(0.001)
            time.sleep(0.01)

```

Se ha modificado del repositiorio original:
Creado los ficheros: *leapcobotright.msg* y *leapcobotleft.msg* en el directorio *msg*:

Cotenido del fichero *leapcobotright.msg*
```{C}
######################################################
## Custom Message to communicate with app developed ##
######################################################

Header header

# Right hand information
bool is_right_hand                      # Right hand detected
geometry_msgs/Point right_hand_palmpos  # Palm's position
bool right_hand_fist                    # Fist gesture recognize
bool right_hand_thumb_up                # Thumb up gesture recognize
bool right_hand_pinch                   # Pinch gesture recognize
float32 right_hand_pinch_value          # Pinch gesture value
bool right_hand_origin_frame            # Reference frame set
bool right_hand_set_origin_frame_detected # Detect gesture
float32 right_hand_rotate_value         # Values between [-1..0..1] rads
float32 right_hand_turn_value           # Values between [-1..0..1] rads
float32 right_hand_swing_value          # Values between [-1..0..1] rads
```

Cotenido del fichero *leapcobotleft.msg*
```{C}
######################################################
## Custom Message to communicate with app developed ##
######################################################

Header header

# Left hand information
bool is_left_hand                      # Left hand detected
geometry_msgs/Point left_hand_palmpos  # Palm's position
bool left_hand_fist                    # Fist gesture recognize
bool left_hand_thumb_up                # Thumb up gesture recognize
bool left_hand_pinch                   # Pinch gesture recognize
float32 left_hand_pinch_value          # Pinch gesture value
bool left_hand_origin_frame            # Reference frame set
bool left_hand_set_origin_frame_detected # Detect gesture
float32 left_hand_rotate_value         # Values between [-1..0..1]
float32 left_hand_turn_value           # Values between [-1..0..1]
float32 left_hand_swing_value          # Values between [-1..0..1]
```

Y el fichero *CMakeLists.txt* añadiendo los ficheors de mensajes recién creados para su compilación:
```{C}
## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Arm.msg
  Bone.msg
  Finger.msg
  Gesture.msg
  Hand.msg
  Human.msg

  # For backwards compatibility with the old driver files
  leap.msg
  leapros.msg
  leapcobotright.msg
  leapcobotleft.msg
)
```
#### Pruebas en el simulador Gazebo
Para realizar las pruebas, se necesitarán al menos 3 terminales, aunque se pueden reducir para lanzarlos automáticamente en los ficheros launch, pero visualizar la información que síe envía se va a dejaír así de momento.

Terminal 1:
```{bash}
cd ~/tfg_mutilrobot
source devel/setup.bash
roslaunch one_arm_no_moveit_gazebo ur10_joint_limited.launch
```

Terminal 2:
```{bash}
cd ~/tfg_mutilrobot
source devel/setup.bash
rosrun one_arm_no_moveit_leap_motion sender.py
```

Terminal 3:
```{bash}
cd ~/tfg_mutilrobot
source devel/setup.bash
rosrun one_arm_no_moveit_leap_motion lm_robot_manipulator.py 
```

Terminal 4 (en caso de fallo en leap motion):
```{bash}
sudo service leapd restart
```
