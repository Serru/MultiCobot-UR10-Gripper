#!/usr/bin/env python  
import roslib
roslib.load_manifest('four_arm_no_moveit_gazebo')
import rospy, sys
import math
import tf
from geometry_msgs.msg import Point, Pose, Quaternion

class RobotPose():
    def __init__(self):
        self.namenode = "ur10_1_robot_pose"
        self.robot_pose_pub = rospy.Publisher('/ur10_1_robot_pose', Pose, queue_size=10)

    def callRobotPoseService(self):
        # wait for model to exist
        rospy.init_node(self.namenode)
        listener = tf.TransformListener()
    
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                (trans,rot) = listener.lookupTransform('/ur10_1_base_link', '/ur10_1_ee_link', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            #print trans
            #print rot
            rate.sleep()

            # setting initial pose
            robot_pose = Pose()
            robot_pose.position.x = trans[0]
            robot_pose.position.y = trans[1]
            robot_pose.position.z = trans[2]
            # convert rpy to quaternion for Pose message
            #tmpq = tft.quaternion_from_euler(self.initial_rpy[0],self.initial_rpy[1],self.initial_rpy[2])
            q = Quaternion(rot[0],rot[1],rot[2],rot[3])
            robot_pose.orientation = q
            self.robot_pose_pub.publish(robot_pose)
            print robot_pose
            

if __name__ == "__main__":
    sm = RobotPose()
    sm.callRobotPoseService()
