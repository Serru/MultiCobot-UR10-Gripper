#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory
import roslib
roslib.load_manifest('four_arm_no_moveit_gazebo')
import sys

class PubGripperCmd():
    def __init__(self):
        self.trajectory_gripper_cmd = JointTrajectory()
        self.namenode = "ur10_3_pub_gripper_control"
        self.cmd_gripper_pub = rospy.Publisher('/ur10_3_gripper/command', JointTrajectory, queue_size=10)
        self.gripper_control_sub = rospy.Subscriber('/ur10_3_pub_gripper_control', JointTrajectory, self.update_cmd)

    # Se puede optimizar dejando esta tarea a unos wokers y solo se procesaria el resultado final.
    # Deben estar ordenados y desechar resultados viejos con respecto a un resultado mas reciente.
    def update_cmd(self, data):
        #global trajectory_gripper_cmd
        self.trajectory_gripper_cmd = data
        #print("update")
        #print(data)


    def publisher_gripper_cmd(self):
        rate = rospy.Rate(10) # 10hz  
        while not rospy.is_shutdown():
            #print("publishing")
            self.trajectory_gripper_cmd.header.stamp = rospy.Time.now()
            self.cmd_gripper_pub.publish(self.trajectory_gripper_cmd)
            #print(trajectory_gripper_cmd)
            rate.sleep()

    def callGripperCmdService(self):
        rospy.init_node(self.namenode, anonymous=True)
        try:
            self.publisher_gripper_cmd()
        except rospy.ROSInterruptException:
            pass

if __name__ == "__main__":
    sm = PubGripperCmd()
    sm.callGripperCmdService()
