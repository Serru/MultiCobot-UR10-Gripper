#!/usr/bin/env python
from __future__ import print_function
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi


    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))

def cartesian_pose_target(group, x, y, z, rot_x, rot_y, rot_z, rot_w):
    success = False
    group.set_start_state_to_current_state()
    while not success and not rospy.is_shutdown():
        target_pose = geometry_msgs.msg.PoseStamped()
        target_pose.header.frame_id = "/ur10_3/world"
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z
        target_pose.pose.orientation.x = rot_x
        target_pose.pose.orientation.y = rot_y
        target_pose.pose.orientation.z = rot_z
        target_pose.pose.orientation.w = rot_w
        
        group.set_pose_target(target_pose, "ee_link")
        #group.set_pose_target([x, y, z, rot_x, rot_y, rot_z, rot_w], "ee_link")
        success = group.go(wait = True)
        if not success:
            group.clear_pose_target("ee_link")
            rospy.sleep(1)
        group.stop()
    print(success)
   # group.stop()

def pick_place(arm, gripper):
    ## Planning to a Pose goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the 
    ## end-effector

    # Primer Cubo
    success = False
    arm.set_start_state_to_current_state()
    while not success and not rospy.is_shutdown():
        arm.set_named_target("home")        
        success = arm.go(wait = True)
        if not success:
            rospy.sleep(1)
        arm.stop()
    print(success)

    success = False
    gripper.set_start_state_to_current_state()
    while not success and not rospy.is_shutdown():
        gripper.set_named_target("gripper_open")
        success = gripper.go(wait = True)
        if not success:
            rospy.sleep(1)
        gripper.stop()
    print(success)
        
    cartesian_pose_target(arm, -0.29, -0.632, 0.62, -0.0173102007701, 0.699047127267, 0.0169648143179, 0.714664722709)
    cartesian_pose_target(arm, -0.29, -0.775, 0.62, -0.0173102007701, 0.699047127267, 0.0169648143179, 0.714664722709)
    cartesian_pose_target(arm, -0.29, -0.775, 0.15, -0.0173102007701, 0.699047127267, 0.0169648143179, 0.714664722709)
    cartesian_pose_target(arm, -0.29, -0.775, 0.08, -0.0173102007701, 0.699047127267, 0.0169648143179, 0.714664722709)
    
    success = False
    gripper.set_start_state_to_current_state()
    while not success and not rospy.is_shutdown():
        gripper.set_named_target("gripper_close")
        success = gripper.go(wait = True)
        if not success:
            rospy.sleep(1)
        gripper.stop()
    print(success)

    cartesian_pose_target(arm, -0.10, -0.775, 0.15, -0.0173102007701, 0.699047127267, 0.0169648143179, 0.714664722709)
    cartesian_pose_target(arm, 0.0, -0.775, 0.15, -0.0173102007701, 0.699047127267, 0.0169648143179, 0.714664722709)
    cartesian_pose_target(arm, 0.28, -0.775, 0.15, -0.0173102007701, 0.699047127267, 0.0169648143179, 0.714664722709)
    cartesian_pose_target(arm, 0.40, -0.775, 0.15, -0.0173102007701, 0.699047127267, 0.0169648143179, 0.714664722709)
    cartesian_pose_target(arm, 0.50, -0.775, 0.15, -0.0173102007701, 0.699047127267, 0.0169648143179, 0.714664722709)
    cartesian_pose_target(arm, 0.60, -0.775, 0.15, -0.0173102007701, 0.699047127267, 0.0169648143179, 0.714664722709)
    cartesian_pose_target(arm, 0.70, -0.6, 0.15, -0.0173102007701, 0.699047127267, 0.0169648143179, 0.714664722709)
    
    success = False
    gripper.set_start_state_to_current_state()
    while not success and not rospy.is_shutdown():
        gripper.set_named_target("gripper_open")
        success = gripper.go(wait = True)
        if not success:
            rospy.sleep(1)
        gripper.stop()
    print(success)    

    # Segundo Cubo
    success = False
    arm.set_start_state_to_current_state()
    while not success and not rospy.is_shutdown():
        arm.set_named_target("home")        
        success = arm.go(wait = True)
        if not success:
            rospy.sleep(1)
        arm.stop()
    print(success)

    cartesian_pose_target(arm, -0.29, -0.632, 0.62, -0.0173102007701, 0.699047127267, 0.0169648143179, 0.714664722709)
    cartesian_pose_target(arm, -0.5, -0.632, 0.62, -0.0173102007701, 0.699047127267, 0.0169648143179, 0.714664722709)
    cartesian_pose_target(arm, -0.7, -0.632, 0.2, -0.0173102007701, 0.699047127267, 0.0169648143179, 0.714664722709)
    cartesian_pose_target(arm, -0.8, -0.5, 0.2, -0.0173102007701, 0.699047127267, 0.0169648143179, 0.714664722709)
    cartesian_pose_target(arm, -0.9, -0.5, 0.08, -0.0173102007701, 0.699047127267, 0.0169648143179, 0.714664722709)
                        
    success = False
    gripper.set_start_state_to_current_state()
    while not success and not rospy.is_shutdown():
        gripper.set_named_target("gripper_close")
        success = gripper.go(wait = True)
        if not success:
            rospy.sleep(1)
        gripper.stop()
    print(success)

    cartesian_pose_target(arm, -0.7, -0.775, 0.15, -0.0173102007701, 0.699047127267, 0.0169648143179, 0.714664722709)
    cartesian_pose_target(arm, -0.6, -0.775, 0.15, -0.0173102007701, 0.699047127267, 0.0169648143179, 0.714664722709)
    cartesian_pose_target(arm, -0.5, -0.775, 0.15, -0.0173102007701, 0.699047127267, 0.0169648143179, 0.714664722709)
    cartesian_pose_target(arm, -0.4, -0.775, 0.15, -0.0173102007701, 0.699047127267, 0.0169648143179, 0.714664722709)
    cartesian_pose_target(arm, -0.3, -0.775, 0.15, -0.0173102007701, 0.699047127267, 0.0169648143179, 0.714664722709)
    cartesian_pose_target(arm, -0.2, -0.775, 0.15, -0.0173102007701, 0.699047127267, 0.0169648143179, 0.714664722709)
    cartesian_pose_target(arm, -0.1, -0.775, 0.15, -0.0173102007701, 0.699047127267, 0.0169648143179, 0.714664722709)
    cartesian_pose_target(arm, 0.28, -0.775, 0.15, -0.0173102007701, 0.699047127267, 0.0169648143179, 0.714664722709)
    cartesian_pose_target(arm, 0.40, -0.775, 0.15, -0.0173102007701, 0.699047127267, 0.0169648143179, 0.714664722709)
    cartesian_pose_target(arm, 0.50, -0.775, 0.15, -0.0173102007701, 0.699047127267, 0.0169648143179, 0.714664722709)
    cartesian_pose_target(arm, 0.60, -0.775, 0.15, -0.0173102007701, 0.699047127267, 0.0169648143179, 0.714664722709)
    cartesian_pose_target(arm, 0.70, -0.6, 0.15, -0.0173102007701, 0.699047127267, 0.0169648143179, 0.714664722709)
    
    success = False
    gripper.set_start_state_to_current_state()
    while not success and not rospy.is_shutdown():
        gripper.set_named_target("gripper_open")
        success = gripper.go(wait = True)
        if not success:
            rospy.sleep(1)
        gripper.stop()
    print(success)            

    # Tercer Cubo
    success = False
    arm.set_start_state_to_current_state()
    while not success and not rospy.is_shutdown():
        arm.set_named_target("home")        
        success = arm.go(wait = True)
        if not success:
            rospy.sleep(1)
        arm.stop()
    print(success)
    
    cartesian_pose_target(arm, -0.29, -0.632, 0.62, -0.0173102007701, 0.699047127267, 0.0169648143179, 0.714664722709)
    cartesian_pose_target(arm, -0.475, -0.632, 0.62, -0.0173102007701, 0.699047127267, 0.0169648143179, 0.714664722709)
    cartesian_pose_target(arm, -0.575, -0.632, 0.2, -0.0173102007701, 0.699047127267, 0.0169648143179, 0.714664722709)
    cartesian_pose_target(arm, -0.675, -0.88, 0.2, -0.0173102007701, 0.699047127267, 0.0169648143179, 0.714664722709)
    cartesian_pose_target(arm, -0.675, -0.88, 0.1, -0.0173102007701, 0.699047127267, 0.0169648143179, 0.714664722709)
    cartesian_pose_target(arm, -0.675, -0.88, 0.08, -0.0173102007701, 0.699047127267, 0.0169648143179, 0.714664722709)
                        
    success = False
    gripper.set_start_state_to_current_state()
    while not success and not rospy.is_shutdown():
        gripper.set_named_target("gripper_close")
        success = gripper.go(wait = True)
        if not success:
            rospy.sleep(1)
        gripper.stop()
    print(success)

    cartesian_pose_target(arm, -0.5, -0.775, 0.15, -0.0173102007701, 0.699047127267, 0.0169648143179, 0.714664722709)
    cartesian_pose_target(arm, -0.4, -0.775, 0.15, -0.0173102007701, 0.699047127267, 0.0169648143179, 0.714664722709)
    cartesian_pose_target(arm, -0.3, -0.775, 0.15, -0.0173102007701, 0.699047127267, 0.0169648143179, 0.714664722709)
    cartesian_pose_target(arm, -0.2, -0.775, 0.15, -0.0173102007701, 0.699047127267, 0.0169648143179, 0.714664722709)
    cartesian_pose_target(arm, -0.1, -0.775, 0.15, -0.0173102007701, 0.699047127267, 0.0169648143179, 0.714664722709)
    cartesian_pose_target(arm, 0.1, -0.775, 0.15, -0.0173102007701, 0.699047127267, 0.0169648143179, 0.714664722709)
    cartesian_pose_target(arm, 0.28, -0.775, 0.15, -0.0173102007701, 0.699047127267, 0.0169648143179, 0.714664722709)
    cartesian_pose_target(arm, 0.40, -0.775, 0.15, -0.0173102007701, 0.699047127267, 0.0169648143179, 0.714664722709)
    cartesian_pose_target(arm, 0.50, -0.775, 0.15, -0.0173102007701, 0.699047127267, 0.0169648143179, 0.714664722709)
    cartesian_pose_target(arm, 0.60, -0.775, 0.15, -0.0173102007701, 0.699047127267, 0.0169648143179, 0.714664722709)
    cartesian_pose_target(arm, 0.70, -0.6, 0.15, -0.0173102007701, 0.699047127267, 0.0169648143179, 0.714664722709)
                
    success = False
    gripper.set_start_state_to_current_state()
    while not success and not rospy.is_shutdown():
        gripper.set_named_target("gripper_open")
        success = gripper.go(wait = True)
        if not success:
            rospy.sleep(1)
        gripper.stop()
    print(success)

    success = False
    arm.set_start_state_to_current_state()
    while not success and not rospy.is_shutdown():
        arm.set_named_target("home")        
        success = arm.go(wait = True)
        if not success:
            rospy.sleep(1)
        arm.stop()
    print(success)
    
    success = False
    gripper.set_start_state_to_current_state()
    while not success and not rospy.is_shutdown():
        gripper.set_named_target("gripper_open")
        success = gripper.go(wait = True)
        if not success:
            rospy.sleep(1)
        gripper.stop()
    print(success)

def main():
    ## First initialize moveit_commander and rospy.
    print("============ Starting dual arms moveit")
    moveit_commander.roscpp_initializer.roscpp_initialize(sys.argv)
    rospy.init_node('ur10_1_arm_moveit',
                  anonymous=True)

    PLANNING_GROUP_GRIPPER = "gripper"
    PLANNING_GROUP_ARM = "manipulator"
    PLANNING_NS = "/ur10_3/"
    REFERENCE_FRAME = "/ur10_3/world"

    ## Instantiate a RobotCommander object.  This object is an interface to
    ## the robot as a whole.
    robot = moveit_commander.RobotCommander("%srobot_description"%PLANNING_NS, ns="/ur10_3/")

    ## Instantiate the MoveGroupCommander objects.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the ur10
    ## arm and gripper. This interface can be used to plan and execute motions on the ur10
    ## arm and gripper.
    
    #gripper = robot.get_group(PLANNING_GROUP_GRIPPER)
    #arm = robot.get_group(PLANNING_GROUP_ARM)
    arm = moveit_commander.move_group.MoveGroupCommander(PLANNING_GROUP_ARM,"%srobot_description"%PLANNING_NS, ns="/ur10_3/")
    gripper = moveit_commander.move_group.MoveGroupCommander(PLANNING_GROUP_GRIPPER, "%srobot_description"%PLANNING_NS, ns="/ur10_3/")

    ## We create this DisplayTrajectory publisher which is used below to publish
    ## trajectories for RVIZ to visualize.
    display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory, queue_size=10)

    rospy.sleep(2)

    ## We can get the name of the reference frame for this robot
    print( "============ Reference frame: %s" % arm.get_planning_frame() )

    ## We can also print the name of the end-effector link for this group
    print( "============ Reference frame: %s" % arm.get_end_effector_link() )

    ## We can get a list of all the groups in the robot
    print( "============ Robot Groups:" )
    print( robot.get_group_names())
    
    #arm.set_planner_id("RRT")
    arm.set_num_planning_attempts(15)
    arm.set_planning_time(5)
    arm.allow_looking(True)
    arm.allow_replanning(True)
    arm.set_pose_reference_frame(REFERENCE_FRAME)
    arm.set_goal_position_tolerance(0.001)
    arm.set_goal_orientation_tolerance(0.001)
    minX = 1.5
    maxX = -1.5
    minY = 0.0
    maxY = -1.5
    minZ = 0
    maxZ = 1.5
    arm.set_workspace([minX, minY, minZ, maxX, maxY, maxZ])

    #gripper.set_planner_id("RRTConnect")
    gripper.set_num_planning_attempts(15)
    gripper.allow_replanning(True)
    gripper.allow_looking(True)

    pick_place(arm, gripper)

    ## When finished shut down moveit_commander.
    moveit_commander.roscpp_shutdown()

if __name__=='__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass