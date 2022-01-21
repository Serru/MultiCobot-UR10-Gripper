#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory
import roslib
roslib.load_manifest('two_arm_no_moveit_gazebo')
import sys

class PubIkTrajectory():
    def __init__(self):
        self.ik_trajectory = JointTrajectory()
        self.namenode = "ur10_2_pub_ik_trajectory"
        self.cmd_pose_pub = rospy.Publisher('/ur10_2_arm_controller/command', JointTrajectory, queue_size=10)
        self.trajectory_sub = rospy.Subscriber('/ur10_2_pub_ik_trajectory', JointTrajectory, self.update_trajectory)

    # Se puede optimizar dejando esta tarea a unos wokers y solo se procesaria el resultado final.
    # Deben estar ordenados y desechar resultados viejos con respecto a un resultado mas reciente.
    def update_trajectory(self, data):
        #global ik_trajectory
        self.ik_trajectory = data
        #print("update")
        #print(data)

    def publisher_trajectory(self):
        rate = rospy.Rate(10) # 10hz  
        while not rospy.is_shutdown():
            #print("publishing")
            self.ik_trajectory.header.stamp = rospy.Time.now()
            self.cmd_pose_pub.publish(self.ik_trajectory)
            #print(ik_trajectory)
            rate.sleep()

    def callIkTrajectoryService(self):
        rospy.init_node(self.namenode, anonymous=True)
        try:
            self.publisher_trajectory()
        except rospy.ROSInterruptException:
            pass

if __name__ == "__main__":
    sm = PubIkTrajectory()
    sm.callIkTrajectoryService()
