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
    waypoints = []

    wpose = group.get_current_pose().pose
    wpose.position.x += scale * x  # Move forward/backwards in (x)
    wpose.position.z += scale * z  # Move up/down (z)
    wpose.position.y += scale * y  # Move sideways (y)
    wpose.orientation.w += scale * w  # Rotation of the arm
    waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan, fraction) = group.compute_cartesian_path(
        waypoints,  # waypoints to follow
        0.001,  # eef_step
        0.0)  # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction


def execute_plan(group, plan, bool_wait = True):
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    succeeded = group.execute(plan, wait=bool_wait)
    return succeeded

    ## **Note:** The robot's current joint state must be within some tolerance of the
    ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail

def main():
    ## First initialize moveit_commander and rospy.
    print("============ Starting dual arms moveit")
    moveit_commander.roscpp_initializer.roscpp_initialize(sys.argv)
    rospy.init_node('ur10_1_arm_moveit',
                  anonymous=True)

    PLANNING_GROUP_GRIPPER = "gripper"
    PLANNING_GROUP_ARM = "manipulator"
    PLANNING_NS = "/ur10_1/"

    ## Instantiate a RobotCommander object.  This object is an interface to
    ## the robot as a whole.
    robot = moveit_commander.RobotCommander("%srobot_description"%PLANNING_NS, ns="/ur10_1/")

    ## Instantiate the MoveGroupCommander objects.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the ur10
    ## arm and gripper. This interface can be used to plan and execute motions on the ur10
    ## arm and gripper.
    
    #gripper = robot.get_group(PLANNING_GROUP_GRIPPER)
    #arm = robot.get_group(PLANNING_GROUP_ARM)
    arm = moveit_commander.move_group.MoveGroupCommander(PLANNING_GROUP_ARM,"%srobot_description"%PLANNING_NS, ns="/ur10_1/")
    gripper = moveit_commander.move_group.MoveGroupCommander(PLANNING_GROUP_GRIPPER, "%srobot_description"%PLANNING_NS, ns="/ur10_1/")

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
    arm.allow_looking(True)
    arm.allow_replanning(True)

    #gripper.set_planner_id("RRTConnect")
    gripper.set_num_planning_attempts(15)
    gripper.allow_replanning(True)
    gripper.allow_looking(True)


    ## Planning to a Pose goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the 
    ## end-effector

    # Primer Cubo
    succeess = False
    all_ok = False
    while not all_ok:
        arm.set_named_target("home")
    
        arm.go(wait=True)
        gripper.set_named_target("gripper_open")
        gripper.go(wait=True)
        
        cartesian_plan, fraction = plan_cartesian_path_pose(arm, -0.54, 0, 0, 0, 1)
        succeess = execute_plan(arm, cartesian_plan)
        if succeess:
            cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, -0.143, 0, 0, 1)
            succeess = execute_plan(arm, cartesian_plan)
            if succeess:
                cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0, -0.47, 0, 1)
                succeess = execute_plan(arm, cartesian_plan)
                if succeess:
                    cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0, -0.07, 0, 1)
                    succeess = execute_plan(arm, cartesian_plan)
                    if succeess:
                        pos_check = arm.get_current_pose().pose.position
                        print(round(pos_check.x,2))
                        print(round(pos_check.y,2))
                        if round(pos_check.x,2) == -0.30:
                            if round(pos_check.y,2) == -0.77:
                                all_ok = True
                                print(arm.get_current_pose())
        print(succeess)
        print(all_ok)

    succeess = False
    all_ok = False
    while not all_ok:
    
        gripper.set_named_target("gripper_close")
        gripper.go(wait=True)

        arm.set_named_target("home")    
        arm.go(wait=True)

        cartesian_plan, fraction = plan_cartesian_path_pose(arm, -0.53, 0.0, 0.07, 0, 1)
        succeess = execute_plan(arm, cartesian_plan)
        if succeess:
            cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0.98, 0, 0, 0, 1)
            succeess = execute_plan(arm, cartesian_plan)
            if succeess:
                cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0.175, 0, 0, 1)
                succeess = execute_plan(arm, cartesian_plan)
                if succeess:
                    pos_check = arm.get_current_pose().pose.position
                    print(round(pos_check.x,2))
                    print(round(pos_check.y,2))
                    if round(pos_check.x,2) == 0.69:
                        if round(pos_check.y,2) == -0.46:
                            all_ok = True
                            print(arm.get_current_pose())
        print(succeess)

    succeess = False
    all_ok = False
    while not all_ok:
        gripper.set_named_target("gripper_open")
        gripper.go(wait=True)
    
        # Segundo Cubo
        arm.set_named_target("home")
        arm.go(wait=True)

        cartesian_plan, fraction = plan_cartesian_path_pose(arm, -0.53, 0, 0, 0, 1)
        succeess = execute_plan(arm, cartesian_plan)
        if succeess:
            cartesian_plan, fraction = plan_cartesian_path_pose(arm, -0.61, 0, 0, 0, 1)
            succeess = execute_plan(arm, cartesian_plan)
            if succeess:
                cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0, -0.42, 1.0)
                succeess = execute_plan(arm, cartesian_plan)
                if succeess:
                    cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0.132, 0, 0, 1)
                    succeess = execute_plan(arm, cartesian_plan)
                    if succeess:
                        cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0, -0.12, 0, 1)
                        succeess = execute_plan(arm, cartesian_plan)
                        if succeess:
                            pos_check = arm.get_current_pose().pose.position
                            print(round(pos_check.x,2))
                            print(round(pos_check.y,2))
                            if round(pos_check.x,2) == -0.9:
                                if round(pos_check.y,2) == -0.50:
                                    all_ok = True
                                    print(arm.get_current_pose())
    
    succeess = False
    all_ok = False
    while not all_ok:
        gripper.set_named_target("gripper_close")
        gripper.go(wait=True)

        arm.set_named_target("home")    
        arm.go(wait=True)

        cartesian_plan, fraction = plan_cartesian_path_pose(arm, -0.53, 0.0, 0.07, 0, 1)
        succeess = execute_plan(arm, cartesian_plan)
        if succeess:
            cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0.98, 0, 0, 0, 1)
            succeess = execute_plan(arm, cartesian_plan)
            if succeess:
                cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0.175, 0, 0, 1)
                succeess = execute_plan(arm, cartesian_plan)
                if succeess:
                    pos_check = arm.get_current_pose().pose.position
                    print(round(pos_check.x,2))
                    print(round(pos_check.y,2))
                    if round(pos_check.x,2) == 0.69:
                        if round(pos_check.y,2) == -0.46:
                            all_ok = True
                            print(arm.get_current_pose())    
    succeess = False
    all_ok = False
    while not all_ok:
        gripper.set_named_target("gripper_open")
        gripper.go(wait=True)
        
        # Tercer Cubo
        arm.set_named_target("home")
        arm.go(wait=True)
    
        cartesian_plan, fraction = plan_cartesian_path_pose(arm, -0.53, 0, 0, 0, 1)
        succeess = execute_plan(arm, cartesian_plan)
        if succeess:
            cartesian_plan, fraction = plan_cartesian_path_pose(arm, -0.385, 0, 0, 0, 1)
            succeess = execute_plan(arm, cartesian_plan)
            if succeess:
                cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0, -0.42, 1.0)
                succeess = execute_plan(arm, cartesian_plan)
                if succeess:
                    cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, -0.248, 0, 0, 1)
                    succeess = execute_plan(arm, cartesian_plan)
                    if succeess:
                        cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0, -0.12, 0, 1)
                        succeess = execute_plan(arm, cartesian_plan)
                        if succeess:
                            pos_check = arm.get_current_pose().pose.position
                            print(round(pos_check.x,2))
                            print(round(pos_check.y,2))
                            if round(pos_check.x,2) == -0.68:
                                if round(pos_check.y,2) == -0.88:
                                    all_ok = True
                                    print(arm.get_current_pose())
    succeess = False
    all_ok = False
    while not all_ok:
        gripper.set_named_target("gripper_close")
        gripper.go(wait=True)

        arm.set_named_target("home")    
        arm.go(wait=True)

        cartesian_plan, fraction = plan_cartesian_path_pose(arm, -0.53, 0.0, 0.07, 0, 1)
        succeess = execute_plan(arm, cartesian_plan)
        if succeess:
            cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0.98, 0, 0, 0, 1)
            succeess = execute_plan(arm, cartesian_plan)
            if succeess:
                cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0.175, 0, 0, 1)
                succeess = execute_plan(arm, cartesian_plan)
                if succeess:
                    pos_check = arm.get_current_pose().pose.position
                    print(round(pos_check.x,2))
                    print(round(pos_check.y,2))
                    if round(pos_check.x,2) == 0.69:
                        if round(pos_check.y,2) == -0.46:
                            all_ok = True
                            print(arm.get_current_pose())
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