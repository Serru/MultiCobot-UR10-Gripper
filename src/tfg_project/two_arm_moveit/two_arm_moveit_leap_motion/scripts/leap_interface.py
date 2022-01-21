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
