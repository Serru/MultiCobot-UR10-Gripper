#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory
import roslib
roslib.load_manifest('one_arm_no_moveit_gazebo')
import sys

def usage():
    print('''Commands:
    -namenode <namenode> - Set the name of the node,can't exists two nodes with the same name. Default value: ur10_robot_pose
    -namespace <namespace> - Let it empty to no add any namespacing.
    ''')
    sys.exit(1)

class PubGripperCmd():
    def __init__(self):
        self.trajectory_gripper_cmd = JointTrajectory()
        self.namespace               = ""
        self.namenode               = "pub_gripper_control"
        self.cmd_gripper_pub = rospy.Publisher(self.namespace + '/gripper/command', JointTrajectory, queue_size=10)
        self.gripper_control_sub =         rospy.Subscriber(self.namespace + '/pub_gripper_control', JointTrajectory, self.update_cmd)

    def parseUserInputs(self):
        # get goal from commandline
        for i in range(0,len(sys.argv)):
          if sys.argv[i] == '-namespace':
            if len(sys.argv) > i+1:
              self.namespace = sys.argv[i+1]
              self.cmd_gripper_pub = rospy.Publisher(self.namespace + '/gripper/command', JointTrajectory, queue_size=10)
          if sys.argv[i] == '-namenode':
            if len(sys.argv) > i+1:
              self.namenode = sys.argv[i+1]

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
    if len(sys.argv) < 2:
        print(usage())
    else:
        print("PubGripperCmd script started") # make this a print incase roscore has not been started
        sm = PubGripperCmd()
        sm.parseUserInputs()
        sm.callGripperCmdService()
