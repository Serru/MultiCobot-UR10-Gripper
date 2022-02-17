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
