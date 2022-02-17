#!/usr/bin/env python  
import roslib
roslib.load_manifest('one_arm_no_moveit_gazebo')
import rospy, sys
import math
import tf
from geometry_msgs.msg import Point, Pose, Quaternion

def usage():
    print('''Commands:
    -namenode <namenode> - Set the name of the node,can't exists two nodes with the same name. Default value: ur10_robot_pose
    -namespace <namespace> - Let it empty to no add any namespacing.
    ''')
    sys.exit(1)

class RobotPose():
    def __init__(self):
        self.namespace               = ""
        self.namenode               = "ur10_robot_pose"
        self.robot_pose_pub = rospy.Publisher(self.namespace + '/robot_pose', Pose, queue_size=10)

    def parseUserInputs(self):
        # get goal from commandline
        for i in range(0,len(sys.argv)):
          if sys.argv[i] == '-namespace':
            if len(sys.argv) > i+1:
              self.namespace = sys.argv[i+1]
              self.robot_pose_pub = rospy.Publisher(self.namespace + '/robot_pose', Pose, queue_size=10)
          if sys.argv[i] == '-namenode':
            if len(sys.argv) > i+1:
              self.namenode = sys.argv[i+1]


    def callRobotPoseService(self):

        # wait for model to exist
        print self.namespace
        print self.namenode
        rospy.init_node(self.namenode)
        listener = tf.TransformListener()
    

        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                (trans,rot) = listener.lookupTransform(self.namespace + '/base_link', self.namespace + '/ee_link', rospy.Time(0))
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
    if len(sys.argv) < 2:
        print(usage())
    else:
        print("RobotPose script started") # make this a print incase roscore has not been started
        sm = RobotPose()
        sm.parseUserInputs()
        sm.callRobotPoseService()
