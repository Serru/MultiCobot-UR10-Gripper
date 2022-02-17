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

class PubIkTrajectory():
    def __init__(self):
        self.ik_trajectory = JointTrajectory()
        self.namespace               = ""
        self.namenode               = "pub_ik_trajectory"
        self.cmd_pose_pub = rospy.Publisher(self.namespace + '/arm_controller/command', JointTrajectory, queue_size=10)
        self.trajectory_sub =         rospy.Subscriber(self.namespace + '/pub_ik_trajectory', JointTrajectory, self.update_trajectory)

    def parseUserInputs(self):
        # get goal from commandline
        for i in range(0,len(sys.argv)):
          if sys.argv[i] == '-namespace':
            if len(sys.argv) > i+1:
              self.namespace = sys.argv[i+1]
              self.cmd_pose_pub = rospy.Publisher(self.namespace + '/arm_controller/command', JointTrajectory, queue_size=10)
          if sys.argv[i] == '-namenode':
            if len(sys.argv) > i+1:
              self.namenode = sys.argv[i+1]

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
    if len(sys.argv) < 2:
        print(usage())
    else:
        print("PubIkTrajectory script started") # make this a print incase roscore has not been started
        sm = PubIkTrajectory()
        sm.parseUserInputs()
        sm.callIkTrajectoryService()
