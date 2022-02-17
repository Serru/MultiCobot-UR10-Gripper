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


# r, p and y in rads
def plan_cartesian_path_orientation(group, r, p, y, scale=1, bool_wait = True):
    joint_target = group.get_current_joint_values()
    joint_target[3] += scale * r
    joint_target[4] += scale * p
    joint_target[5] += scale * y
    succeeded = group.go(joint_target, wait=bool_wait)
    return succeeded

def plan_cartesian_path_pose(group, x, y, z, w, scale=1):
    ## BEGIN_SUB_TUTORIAL plan_cartesian_path
    ##
    ## Cartesian Paths
    ## ^^^^^^^^^^^^^^^
    ## You can plan a Cartesian path directly by specifying a list of waypoints
    ## for the end-effector to go through:
    ##
    waypoints = []

    wpose = group.get_current_pose().pose
    wpose.position.x += scale * x  # Move forward/backwards in (x)
    wpose.position.z += scale * z  # Move up/down (z)
    wpose.position.y += scale * y  # Move sideways (y)
    wpose.orientation.w += scale * w  # Rotation of the arm
    waypoints.append(copy.deepcopy(wpose))

    # wpose = group.get_current_rpy().pose
    # wpose.position.x += scale * x  # Move forward/backwards in (x)
    # wpose.position.z -= scale * z  # Move up/down (z)
    # wpose.position.y += scale * y  # Move sideways (y)
    # waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan, fraction) = group.compute_cartesian_path(
        waypoints,  # waypoints to follow
        0.01,  # eef_step
        0.0)  # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction


def execute_plan(group, plan, bool_wait  = True):
    ## BEGIN_SUB_TUTORIAL execute_plan
    ##
    ## Executing a Plan
    ## ^^^^^^^^^^^^^^^^
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    succeeded = group.execute(plan, wait=bool_wait)
    return succeeded

    ## **Note:** The robot's current joint state must be within some tolerance of the
    ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
    ## END_SUB_TUTORIAL

def error_back_initial_state(arm, gripper, y, ymax):
    
    succeeded = False
    print("\033[91m ur10_1 Error Point 1 - Open Gripper \033[0m")
    while (not succeeded) and (y < ymax):
        gripper.set_named_target("open")
        succeeded = gripper.go(wait=True)
        if succeeded and (y < ymax):
          print("\033[92m Completed trajectory succesfully \033[0m")
        y+=1

    succeeded = False
    print("\033[91m ur10_1 Error Point 2 - Moving arm up \033[0m")
    while (not succeeded) and (y < ymax):
        
        cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0.0, 0, 0.7, 0, 1)
        succeeded = execute_plan(arm, cartesian_plan)
        if succeeded and (y < ymax):
          print("\033[92m Completed trajectory succesfully \033[0m")
        y+=1

    succeeded = False
    print("\033[91m ur10_1 Error Point 3 - back home \033[0m")
    while (not succeeded) and (y < ymax):
        arm.set_named_target("home")
        succeeded = arm.go(wait=True)
        if succeeded and (y < ymax):
          print("\033[92m Completed trajectory succesfully \033[0m")
        y+=1

    if (not succeeded) and (y > ymax):
        print("\033[91m ur10_1 Error Point 4 - Too many errors exiting \033[0m")
        moveit_commander.roscpp_shutdown()
        #error_back_initial_state(arm, gripper, 0, ymax)

def main():
    ## BEGIN_TUTORIAL
    ##
    ## Setup
    ## ^^^^^
    ## CALL_SUB_TUTORIAL imports
    ##
    ## First initialize moveit_commander and rospy.
    print("============ Starting dual arms moveit")
    #moveit_commander.roscpp_initialize(sys.argv)
    moveit_commander.roscpp_initializer.roscpp_initialize(sys.argv)
    rospy.init_node('ur10_1_dual_moveit',
                  anonymous=True)

    PLANNING_GROUP_GRIPPER = "gripper"
    PLANNING_GROUP_ARM = "manipulator"
    PLANNING_NS = ""

    ## Instantiate a RobotCommander object.  This object is an interface to
    ## the robot as a whole.
    robot = moveit_commander.RobotCommander("%srobot_description"%PLANNING_NS, ns="")

    ## Instantiate the MoveGroupCommander objects.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the ur10
    ## arm and gripper. This interface can be used to plan and execute motions on the ur10
    ## arm and gripper.
    
    #gripper = robot.get_group(PLANNING_GROUP_GRIPPER)
    #arm = robot.get_group(PLANNING_GROUP_ARM)
    arm = moveit_commander.move_group.MoveGroupCommander(PLANNING_GROUP_ARM,"%srobot_description"%PLANNING_NS, ns="")
    gripper = moveit_commander.move_group.MoveGroupCommander(PLANNING_GROUP_GRIPPER, "%srobot_description"%PLANNING_NS, ns="")

    ## Instantiate a PlanningSceneInterface object.  This object is an interface
    ## to the world surrounding the robot.
    scene = moveit_commander.PlanningSceneInterface()

    ## We create this DisplayTrajectory publisher which is used below to publish
    ## trajectories for RVIZ to visualize.
    display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory, queue_size=10)

    rospy.sleep(2)
    #scene.remove_world_object("floor")
    
    # publish a demo scene
    #p = geometry_msgs.msg.PoseStamped()
    #p.header.frame_id = robot.get_planning_frame()
    #p.pose.position.x = 0.0
    #p.pose.position.y = 0.0
    #p.pose.position.z = -0.01
    #p.pose.orientation.w = 1.0
    #scene.add_box("floor", p, (2.0, 2.0, 0.02))
    
    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    ##
    ## We can get the name of the reference frame for this robot
    print( "============ Reference frame: %s" % arm.get_planning_frame() )

    ## We can also print the name of the end-effector link for this group
    print( "============ Reference frame: %s" % arm.get_end_effector_link() )

    ## We can get a list of all the groups in the robot
    print( "============ Robot Groups:" )
    print( robot.get_group_names())

    ## Sometimes for debugging it is useful to print the entire state of the
    ## robot.
    #print( "============ Printing robot state" )
    #print( robot.get_current_state())
    #print( "============" )
    
    #arm.set_planner_id("RRT")
    #arm.set_num_planning_attempts(15)
    #arm.allow_looking(True)
    #arm.allow_replanning(True)

    #gripper.set_planner_id("RRTConnect")
    #gripper.set_num_planning_attempts(15)
    #gripper.allow_replanning(True)
    #gripper.allow_looking(True)


    ## Planning to a Pose goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the 
    ## end-effector

    # Primer Cubo
    arm.set_named_target("home")
    print(arm.get_current_pose())
    #x: 0.239820825156
    #y: -0.631965677562
    #z: 0.62069143596

    arm.go(wait=True)
    gripper.set_named_target("gripper_open")
    gripper.go(wait=True)
    
    cartesian_plan, fraction = plan_cartesian_path_pose(arm, -0.54, 0, 0, 0, 1)
    execute_plan(arm, cartesian_plan)
    cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, -0.143, 0, 0, 1)
    execute_plan(arm, cartesian_plan)
    cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0, -0.47, 0, 1)
    execute_plan(arm, cartesian_plan)
    cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0, -0.07, 0, 1)

    execute_plan(arm, cartesian_plan)
    gripper.set_named_target("gripper_close")
    gripper.go(wait=True)
    
    cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0.01, 0.0, 0.07, 0, 1)
    print(arm.get_current_pose())
    execute_plan(arm, cartesian_plan)
    cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0.98, 0, 0, 0, 1)
    execute_plan(arm, cartesian_plan)
    cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0.175, 0, 0, 1)
    print(arm.get_current_pose())
    execute_plan(arm, cartesian_plan)
    gripper.set_named_target("gripper_open")
    gripper.go(wait=True)
    
    # Segundo Cubo
    arm.set_named_target("home")
    arm.go(wait=True)
    #x: 0.239820825156 (-0.24)
    #y: -0.631965677562 (0.632)
    #z: 0.62069143596   (0.62)

    cartesian_plan, fraction = plan_cartesian_path_pose(arm, -0.53, 0, 0, 0, 1)
    execute_plan(arm, cartesian_plan)
    cartesian_plan, fraction = plan_cartesian_path_pose(arm, -0.61, 0, 0, 0, 1)
    execute_plan(arm, cartesian_plan)
    cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0, -0.42, 1.0)
    execute_plan(arm, cartesian_plan)
    cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0.132, 0, 0, 1)
    execute_plan(arm, cartesian_plan)
    cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0, -0.12, 0, 1)
    execute_plan(arm, cartesian_plan)
    gripper.set_named_target("gripper_close")
    gripper.go(wait=True)
    
    cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0.62, -0.275, 0.07, 0, 1)
    execute_plan(arm, cartesian_plan)
    cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0.98, 0, 0, 0, 1)
    execute_plan(arm, cartesian_plan)
    cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0.175, 0, 0, 1)
    execute_plan(arm, cartesian_plan)
    gripper.set_named_target("gripper_open")
    gripper.go(wait=True)
    
    # Tercer Cubo
    arm.set_named_target("home")
    arm.go(wait=True)
    #x: 0.239820825156
    #y: -0.631965677562
    #z: 0.62069143596

    cartesian_plan, fraction = plan_cartesian_path_pose(arm, -0.53, 0, 0, 0, 1)
    execute_plan(arm, cartesian_plan)
    cartesian_plan, fraction = plan_cartesian_path_pose(arm, -0.385, 0, 0, 0, 1)
    execute_plan(arm, cartesian_plan)
    cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0, -0.42, 1.0)
    execute_plan(arm, cartesian_plan)
    cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, -0.248, 0, 0, 1)
    execute_plan(arm, cartesian_plan)
    cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0, -0.12, 0, 1)
    execute_plan(arm, cartesian_plan)
    gripper.set_named_target("gripper_close")
    gripper.go(wait=True)
    
    cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0.395, 0.105, 0.07, 0, 1)
    execute_plan(arm, cartesian_plan)
    cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0.98, 0, 0, 0, 1)
    execute_plan(arm, cartesian_plan)
    cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0.175, 0, 0, 1)
    execute_plan(arm, cartesian_plan)
    gripper.set_named_target("gripper_open")
    gripper.go(wait=True)

    arm.set_named_target("home")
    arm.go(wait=True)
    gripper.set_named_target("gripper_open")
    gripper.go(wait=True)

    ## When finished shut down moveit_commander.
    moveit_commander.roscpp_shutdown()

if __name__=='__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass

