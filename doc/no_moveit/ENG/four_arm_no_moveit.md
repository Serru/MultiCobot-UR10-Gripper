# Installation and configuration for four UR10 robots without 'MoveIt! 

**Spanish** | [English](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/no_moveit/ ENG /four_arm_no_moveit.md) 

![image](/doc/imgs_md/Diseno-no-moveit-general-cuatro-cobots-leap-motion.png "Loads the URDF model of the UR10 robot") 

This time the solution for four robots is done in the same way as for two robots, changing the content of the files and adapting them to the simulation with four robots. 

The phases you see in the diagram are for orientation. They can be executed in the order you wish. The scheme has been divided into phases for the sake of order, so you know which element of the scheme is being worked on. In this case, it starts with Phase 1, followed by Phase 2, and finally ends with Phase 3. Note that there may be configurations in one phase that actually belong in another phase. If this is the case, it will be marked accordingly.

## Prerequisite
- Successfully install the [base system configuration](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/setup-doc/proyect_setup.md). 

## Index
- [Initial setup: setup for one robot](#setup-initial)
- [Phase 1: Configuration of the 'Gazebo' simulator](#phase1)
- [Phase 2: *URDF* configuration](#phase2)
- [Phase 3: Implementation of a custom scheduler that performs a 'pick & place'](#phase3)
- [Implementation of tests](#tests)
- [Mods: multi-robot system consisting of four robots](#mods)
- [Phase 1: configuration of the 'Gazebo' simulator](#modifications1 )
- Phase 2: *URDF* configuration](#modifications2)
- [Phase 3: Implementation of a custom scheduler that performs a 'pick & place'](#modifications3)
- [Execution of tests](#tests2)


<a name="setup-inicial">
<h2>
Initial setup: Setup for a robot
</h2>
</a>

In this section, a replication of the configuration for a single robot is made, which serves as a basis and thus later explains the changes made to be able to control two robots. 

### :warning: Contents of the file
Do not blindly copy and paste the contents of the files into this section. When you explain the configuration for one robot, the information is identical to that from the **one_arm_no_moveit** package, but you must adapt the contents to the new package, in this case **four_arm_no_moveit**. 

There are two possibilities:
- If you created the solution for a [one_arm_no_moveit] robot (https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/no_moveit/ ESP /one_arm_no_moveit.md), you can continue with what you implemented, but in the *modifications* section you need to change **four_arm_no_moveit** to **one_arm_no_moveit**.
- If the solution for a [one_arm_no_moveit](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/no_moveit/ ESP /one_arm_no_moveit.md) robot has not yet been created, you can follow the steps presented in this section. Note, however, that the contents of the files at this stage of *initial configuration* are linked to the files of **one_arm_no_moveit**. Therefore, you must replace references to the **one_arm_no_moveit** package with **four_arm_no_moveit** when creating the copy. 

If this information is not taken into account, the package will not compile correctly and the compiler itself will force a choice between one of the previously suggested options.

<a name="fase1">
<h3>
Phase 1: Gazebo Simulator Setup
</h3>
</a>

#### :book: `Gazebo` configuration 

The first thing to do is to configure the 'Gazebo' and the controllers so that it can properly simulate the cobot's movements. The package 'one_arm_no_ moveit_gazebo' will be created, which will contain all the configurations for 'Gazebo', including the controllers. 

Once the package is created, you will need to configure the controllers that are stored in the 'controller' directory. The controllers are defined in files with the extension *yaml*. To define these controllers, you must give them a name and specify the type of controller, the dynamic 'joints' you want to control, the constraints it has, the publishing ratio and other options. 

The contents of the driver configuration files are shown below. All of these drivers generally follow the structure mentioned above. The definition of the controllers can be contained in a single file, the only important thing is that 'Gazebo' loads them correctly. These controllers are briefly explained: 

- [arm_controller_ur10.yaml](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/controller/arm_controller_ur10.yaml) file: This file defines the controller for the UR10 cobot. It defines the controller name 'arm_controller', the controller type element 'controllers/JointTrajectoryController' which contains the definition of the message type and the correct formatting of the information needed to communicate with the controller. Then there is the 'joints' field that specifies which 'joints' of the robot are part of the controller, all of these 'joints' being dynamic. The other fields are left untouched, but care must be taken to name them consistently. 

- File [joint_state_controller.yaml](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/controller/joint_state_controller.yaml): This file does not actually define a controller as such, its function is that of an interface that translates the 'joint' information coming from the real cobot and converts it into 'JointState' type messages that are later published. It is essential for correct operation both in simulation and with the real robot. It is part of the package * ROS * *ros_control*.


##### :computer: Create solution directory
```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot
mkdir four_arm_no_moveit
``` 

##### :computer: Starting `Gazebo`
Currently it is not possible to run 'Gazebo' from the created directory of the project with the robot. The first step is to start it with the UR10 robot. 

To bring some order to the structure of the project to be implemented, the use of the different tools and their files will be separated if possible. This allows a better understanding of the structure and facilitates debugging later. 

Therefore, instead of a directory, a package will be created that contains everything related to 'Gazebo': 

'''bash cd ~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit catkin_create_pkg four_arm_no_moveit_gazebo rospy
''' 

In the directory created for 'Gazebo' it copies the directory *ur_gazebo*, the folders *controller* and *launch*. 

'''bash cd ~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_gazebo cp -r ~/MultiCobot-UR10-Gripper/src/universal_robot/ur_gazebo/controller .
cp -r ~/MultiCobot-UR10-Gripper/src/universal_robot/ur_gazebo/launch .
'''
It compiles: 

'''bash cd ~/MultiCobot-UR10-Gripper catkin_make
''' 

With this you can now start 'Gazebo' from the project directory. 

'''bash roslaunch four_arm_no_moveit_gazebo ur10.launch
'''

##### `Gazebo` world setup
Now we proceed to create the world with the robot and an environment where it can perform simple tasks. This project focuses on the tasks that the robot can perform, so it is not necessary for the world to be very detailed. 

For this purpose, you must first create the *world* directory where the created worlds will be stored: 

'''bash cd ~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_gazebo mkdir world
''' 

The file 'world' from another [repository] (https://github.com/Infinity8sailor/multiple_arm_setup/tree/main/multiple_ur_description/) is used, which provides a very simple scenario that allows later verification of tasks in the robot. 

'''bash cd ~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_gazebo git clone https://github.com/Infinity8sailor/multiple_arm_setup.git cp -r multiple_arm_setup/multiple_ur_description/models/ .
cp -r multiple_arm_setup/multiple_ur_description/world/ .
sudo rm -r multiple_arm_setup
''' 

To add the 'world' with the robot in 'Gazebo' you have to change the file [ur10.launch] (https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/ one_arm_no_moveit/one_arm_no_moveit_gazebo/launch/ur10.launch) in the directory *launch*: 

'''bash cd ~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_gazebo/launch nano ur10.launch
''' 

And add the world to *launch* by adding the 'world' argument and replacing the value of *default* in the *world_name* argument:

```xml
<arg name="world" default="$(find four_arm_no_moveit_gazebo)/world/multiarm_bot.world"></arg><!-- startup simulated world --><include file="$(find gazebo_ros)/launch/empty_world.launch"><arg name="world_name" default="$(arg world)"></arg><arg name="paused" value="$(arg paused)"></arg><arg name="gui" value="$(arg gui)"></arg></include>
``` 

We will now start 'Gazebo' with the new world: 

'''bash roslaunch four_arm_no_moveit_gazebo ur10.launch
''' 

When launching, two errors are displayed: 

'''bash
Error [parser.cc:581] Cannot find uri[model://dropbox].
''' 

This error can only be fixed if the model is built from scratch, as it is not provided by 'Gazebo' or has been removed. Therefore, in the [multiarm_bot.world](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/world/multiarm_bot.world) file, the 'dropbox' object is commented:

``` {xml}<!--include>
      <pose frame=''>0.5 0.0 0.3 0 0 0</pose> 
      <uri>model://dropbox</uri>
      <name>DropBox</name>
</include-->``` 

--- 

```bash
[ERROR] [1639740881.902412642, 0.057000000]: GazeboRosControlPlugin missing <legacymodens> while using DefaultRobotHWSim, defaults to true.
This setting assumes you have an old package with an old implementation of DefaultRobotHWSim, where the robotNamespace is disregarded and absolute paths are used instead.
If you do not want to fix this issue in an old package just set <legacymodens> to true.
``` 

To fix this error, we make the change from the file [~/MultiCobot-UR10-Gripper/src/universal_robot/ur_descriptiom/urdf/common.gazebo.xacro] (https://github.com/Serru/MultiCobot -UR10-Gripper/blob/main/src/universal_robot/ur_description/urdf/common.gazebo.xacro) and add *<legacymodens>* to the file:

```xml
<plugin name="ros_control" filename="libgazebo_ros_control.so"><!--robotNamespace> / </robotNamespace--><!--robotSimType> gazebo_ros_control/DefaultRobotHWSim </robotSimType--><legacymodens> true </legacymodens></plugin>
``` 

##### :computer: Installing 'Gazebo' 9 [Optional].
You may see a *warning* that can be a problem if you ignore it when simulating arm behavior in the future.

```bash
[ WARN] [1639745803.729749460, 0.061000000]: The default_robot_hw_sim plugin is using the Joint::SetPosition method without preserving the link velocity.
[ WARN] [1639745803.729772883, 0.061000000]: As a result, gravity will not be simulated correctly for your model.
[ WARN] [1639745803.729786659, 0.061000000]: Please set gazebo_pid parameters, switch to the VelocityJointInterface or EffortJointInterface, or upgrade to Gazebo 9. 

``` 

For this purpose, we chose to install 'Gazebo' 9 in the * ROS Kinetic Kame* environment.

```bash
sudo apt-get remove ros-kinetic-desktop-full
sudo apt-get remove ros-kinetic-gazebo*
sudo apt-get upgrade 

sudo apt-get install ros-kinetic-ros-base 

sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" &gt; /etc/apt/sources.list.d/gazebo-stable.list'
cat /etc/apt/sources.list.d/gazebo-stable.list
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update 

sudo apt-get install ros-kinetic-gazebo9-ros-pkgs ros-kinetic-gazebo9-ros-control ros-kinetic-gazebo9* 

sudo apt-get install ros-kinetic-catkin 

sudo apt-get install rviz 

sudo apt-get install ros-kinetic-controller-manager ros-kinetic-joint-state-controller ros-kinetic-joint-trajectory-controller ros-kinetic-rqt ros-kinetic-rqt-controller-manager ros-kinetic-rqt- joint-trajectory-controller ros-kinetic-ros-control ros-kinetic-rqt-gui 

sudo apt-get install ros-kinetic-rqt-plot ros-kinetic-rqt-graph ros-kinetic-rqt-rviz ros-kinetic-rqt-tf-tree 

sudo apt-get install ros-kinetic-gazebo9-ros ros-kinetic-kdl-conversions ros-kinetic-kdl-parser ros-kinetic-forward-command-controller ros-kinetic-tf-conversions ros-kinetic-xacro ros-kinetic -joint-state-publisher ros-kinetic-robot-state-publisher 

sudo apt-get install ros-kinetic-ros-control ros-kinetic-ros-controllers
``` 

The result after installing 'Gazebo' 9 looks something like this.
![setup stage](/doc/imgs_md/one-arm-no-moveit-gazebo-setup.png "Gazebo9-world-setup")

<a name="fase2">
<h3>
Phase 2: URDF configuration
</h3>
</a>

#### :book: File Description *URDF*
The *URDF* (United Robotics Description Format) file models the cobot in *XML* format, which is used by the various applications that * ROS * requires, but mainly to perform a simulation of the modeled robot. 

The file is built in the form of a tree in which there are three main tags: ` <robot> `, ` <link> ` and ` <joint> `. For clarification, you can take the arm of the human body as a reference. If you want to model the arm of a person, the tag ` <robot> ` stands for the arm as a whole. This arm consists of several bones (humerus, ulna and radius) represented by the tags ` <link> ` and a joint connecting these bones (elbow) represented by the tag ` <joint> `. 

As with bones, additional information can be added to these tags, providing information about size, geometry, inertia, orientation, etc. Finally, the modeling of a robot can be connected to another model to form a more complex model that can be represented by adding the hand to the arm, where the wrist is a joint that connects the two. Note that ` <joint> ` tags are connected to ` <link> ` tags via a parent-child relationship. 

In this way, a representation of the robot components is created: 

![image](/doc/imgs_md/urdf-robot.png "URDF file representation") 

The image shows the contents of the [*URDF* file](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_description/urdf/ur10_robot.urdf.xacro ) that models the robot next to the gripper. You can see how the component of the UR10 robot arm is connected to the link 'world' which represents the 'world' (yellow color) and the base of the UR10 arm 'base_link' (green color) which is directly above it, also the *joint* 'world_joint' is the yellow ball located between both links. In the same way we connected the component of the gripper 'robotiq_85_gripper' to the arm of the UR10 ('ur10_robot'). The ball representing the *joint* 'robotiq_85_base_joint' connecting both components (purple color) connects the link 'robotiq_85_base_link' of the gripper with the link 'ee_link' of the UR10 arm.

##### :computer: :warning: Adding robotiq_2f_85_gripper to the UR10 robot [Not possible, problems with 'Gazebo']

To add the gripper correctly, you must first understand how it works. Installation instructions can be found [here](https://github.com/Danfoa/robotiq_2finger_grippers).

Take a look at the following image to understand how the gripper control works:
![gripper schematic](/doc/imgs_md/robotiq_2f_85_gripper.png "gripper controller") 

You can see 2 nodes, one acting as a client and the other as a server. The *server* is responsible for sending jobs to the gripper and feedback to the client and the *client* sends jobs to the server. 

If we compare the configuration of the controller with that of the UR10, we notice that the concept is different because there is no *yaml* file that loads the necessary information to control the gripper. Instead, it is the server that creates the *topic*s */command_robotiq_action* and */robotiq_controller/follow_joint_trajectory*, both of which are of type *action*.

```bash
miguel@Omen:~$ rostopic list
/clicked_point
/command_robotiq_action/cancel
/command_robotiq_action/feedback
/command_robotiq_action/goal
/command_robotiq_action/result
/command_robotiq_action/status
/initialpose
/joint_states
/move_base_simple/goal
/robotiq_controller/follow_joint_trajectory/cancel
/robotiq_controller/follow_joint_trajectory/feedback
/robotiq_controller/follow_joint_trajectory/goal
/robotiq_controller/follow_joint_trajectory/result
/robotiq_controller/follow_joint_trajectory/status
/rosout
/rosout_agg
/tf
/tf_static
``` 

To control the gripper, there is an example in the file [robotiq_2f_action_client_example.py] (https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/robotiq_2finger_grippers/robotiq_2f_gripper_control/scripts/robotiq_2f_action_client_example. py) and can work with a simulated gripper or the real gripper. So if you add the gripper to the UR10 robot, the *yaml* file that would load the driver controller is not required. However, the node that acts as the server must be started correctly, and when you look at the server code and the client, you must pay special attention to the configuration of the topic*s and namespace*s. 

The file that uploads the UR10 robot is [~/MultiCobot-UR10-Gripper/src/universal_robot/ur_description/launch/ur10_upload.launch](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/ src/universal_robot/ur_description/launch/ur10_upload.launch), so we copy what is necessary from the *ur_description* package and modify it to add the gripper to the robot and also modify the [ur10.launch](https:// github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/universal_robot/ur_gazebo/launch/ur10.launch) file that loads the *URDF* of the *ur_gazebo* package:

```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit
catkin_create_pkg four_arm_no_moveit_description rospy
cd four_arm_no_moveit_description
mkdir launch
cp ~/MultiCobot-UR10-Gripper/src/universal_robot/ur_description/launch/ur10_upload.launch launch/
mkdir urdf
cp ~/MultiCobot-UR10-Gripper/src/universal_robot/ur_description/urdf/ur10_robot.urdf.xacro udrf/
cp ~/MultiCobot-UR10-Gripper/src/universal_robot/ur_description/urdf/ur10_joint_limited_robot.urdf.xacro udrf/
``` 

The exact reason why it is not possible to simulate the gripper correctly is not known, as 'Gazebo' prints the following error:

```bash
[ERROR] [1640030309.858708751, 1040.204000000]: This robot has a joint named "finger_joint" which is not in the gazebo model.
[FATAL] [1640030309.858805517, 1040.204000000]: Could not initialize robot simulation interface
``` 

It does not work for simulation with 'Gazebo', but it works in the *Rosviz* tool. This indicates that the package * ROS * works correctly for the gripper, but the simulated robot configuration in 'Gazebo' (real robot) is not correct. Surely they should be treated as independent elements that physically work as a whole, which is not the case with this simulation in 'Gazebo'. 

![rviz-robotiq-2f-85-gripper](/doc/imgs_md/robotiq_2f_85_gripper_rviz.png "rviz-robotiq-2f-85-gripper") 

Since the gripper in the real robot is an independent element of the robot arm, this repository may be necessary, but for the simulations another repository compatible with the 'Gazebo' simulator is used. 

##### :computer: Adding the robotiq_85_gripper to the UR10 robot
I am in the process of adding the gripper to the UR10 robot. There are no clear instructions on how to do this properly. So let us first look at how the package is structured to get an idea of how to adapt it to the project in question. 

'''bash
LICENSE robotiq_85_description robotiq_85_moveit_config si_utils
README.md robotiq_85_driver robotiq_85_msgs robotiq_85_bringup robotiq_85_gripper robotiq_85_simulation
''' 

At first glance, the packages we are interested in are *robotiq_85_description*, *robotiq_85_bringup* and *robotiq_85_simulation*, the rest are resources for use with 'MoveIt!' or scripts that will be used later to control the gripper in the simulation. 

In this sense, the gripper is built into the UR10 robot. 

For example, take the file [~/MultiCobot-UR10-Gripper/src/robotiq_85_gripper/robotiq_85_description/urdf/robotiq_85_gripper.xacro] (https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/robotiq_85_gripper /robotiq_85_description/urdf/robotiq_85_gripper.xacro) and insert the gripper into the robot by changing the files: 

- [~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_description/urdf/ur10_robot.urdf.xacro](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot /one_arm_no_moveit/one_arm_no_moveit_description/urdf/ur10_robot.urdf.xacro)
- [~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_description/urdf/ur10_joint_limited_robot.urdf.xacro](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot /one_arm_no_moveit/one_arm_no_moveit_description/urdf/ur10_joint_limited_robot.urdf.xacro). 

And with that you have the gripper on the UR10 robot. You can see the gripper on the picture.
![ ](/doc/imgs_md/ur10_con_gripper_85.png "ur10 with gripper")

--- 

##### :computer: Gripper controller aggregation (`Gazebo`, phase 1) 

Now the controllers are missing to send jobs to the terminal. You can check this by getting the list of active *topic*s and checking if there is no controller for the terminal:

```bash
miguel@Omen:~$ rostopic list
/arm_controller/command
/arm_controller/follow_joint_trajectory/cancel
/arm_controller/follow_joint_trajectory/feedback
/arm_controller/follow_joint_trajectory/goal
/arm_controller/follow_joint_trajectory/result
/arm_controller/follow_joint_trajectory/status
/arm_controller/state
/calibrated
/clock
/gazebo/link_states
/gazebo/model_states
/gazebo/parameter_descriptions
/gazebo/parameter_updates
/gazebo/performance_metrics
/gazebo/set_link_state
/gazebo/set_model_state
/gazebo_gui/parameter_descriptions
/gazebo_gui/parameter_updates
/joint_group_position_controller/command
/joint_states
/rosout
/rosout_agg
/tf
/tf_static
``` 

You can see that the UR10 robot controllers are loaded (**/arm_controller**), but there is no *topic* for the gripper controller. To change this, you need to add the controllers as follows and load them correctly into 'Gazebo'. 

The files with the gripper controller information are located in the directories [~/MultiCobot-UR10-Gripper/src/robotiq_85_gripper/robotiq_85_simulation/robotiq_85_gazebo/controller](https://github.com/Serru/MultiCobot-UR10-Gripper /tree/main/src/robotiq_85_gripper/robotiq_85_simulation/robotiq_85_gazebo/controller) and [~/MultiCobot-UR10-Gripper/src/robotiq_85_gripper/robotiq_85_simulation/robotiq_85_gazebo/launch](https://github.com/Serru/MultiCobot-UR10- Gripper/tree/main/src/robotiq_85_gripper/robotiq_85_simulation/robotiq_85_gazebo/launch), which points to the *controller* and *launch* directories of the [four_arm_no_moveit_gazebo](https://github.com/Serru/MultiCobot-UR10- Gripper/tree/main/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo). 

Finally, the following changes were made: 

'''bash cd ~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_gazebo/controller cp ~/MultiCobot-UR10-Gripper/src/robotiq_85_gripper/robotiq_85_simulation/robotiq_85_gazebo/controller/gripper_controller_robotiq.yaml .
''' 

And at the end of the file [~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_gazebo/launch/ur10.launch](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/ main/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/launch/ur10.launch) added the gripper controller:

```xml
<!-- robotiq_85_gripper controller -->
<rosparam file="$(find four_arm_no_moveit_gazebo)/controller/gripper_controller_robotiq.yaml" command="load"></rosparam><node name="gripper_controller_spawner" pkg="controller_manager" type="spawner" args="gripper"></node>
``` 

Restart 'Gazebo' (*roslaunch four_arm_no_moveit_gazebo ur10.launch*) and with the *rostopic list* command you can see that the gripper controllers appear correctly (**/gripper**): 

```bash
/arm_controller/command
/arm_controller/follow_joint_trajectory/cancel
/arm_controller/follow_joint_trajectory/feedback
/arm_controller/follow_joint_trajectory/goal
/arm_controller/follow_joint_trajectory/result
/arm_controller/follow_joint_trajectory/status
/arm_controller/state
/calibrated
/clock
/gazebo/link_states
/gazebo/model_states
/gazebo/parameter_descriptions
/gazebo/parameter_updates
/gazebo/performance_metrics
/gazebo/set_link_state
/gazebo/set_model_state
/gazebo_gui/parameter_descriptions
/gazebo_gui/parameter_updates
/gripper/command
/gripper/follow_joint_trajectory/cancel
/gripper/follow_joint_trajectory/feedback
/gripper/follow_joint_trajectory/goal
/gripper/follow_joint_trajectory/result
/gripper/follow_joint_trajectory/status
/gripper/state
/joint_group_position_controller/command
/joint_states
/rosout
/rosout_agg
/tf
/tf_static
``` 


<a name="fase3">
<h2>
Phase 3: Implementation of a custom scheduler that performs a <i> pick &amp; place </i>
</h2>
</a>

#### :book: Creating the Scheduler and the Helper Nodes 

The * ROS * policy of splitting tasks into nodes is followed whenever possible to facilitate future reuse. 

![image](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/imgs_md/Design-planner-phase-2.png "Schema for creating your own scheduler") 

In the figure you can see the scheme for building the implemented scheduler architecture. You can see two sections: 'Gazebo' (the nodes in red color and the *themes* in orange color) and the scheduler (the nodes in blue color and the *themes* in green color), divided in this way to facilitate the explanation. 

The group of nodes and *topics* of the group 'Gazebo' did not need to be touched. Their existence was exploited to obtain the necessary information to allow the scheduler to perform its task. This group communicates with the scheduler group through three *topic*s '/tf', which contain information about robot transformations, and the *topic*s '/gripper/command' and '/arm_controller/command', which receive the necessary information to execute movements in the robot. 

Since these nodes and *topic*s are known to the 'Gazebo' group, the solution for the planner is to communicate with them to get the needed information. The node 'ur10_robot_pose' receives the transformation information and sends the position of the *end-effector* to the node 'robot_manipulator', which performs two main functions: The first is to control the gripper and the second is to plan the trajectory of the effector. The 'cmd_gripper_value_pub' and 'cmd_ik_trajectory_pub' nodes receive commands from the 'robot_manipulator' node and send them directly to the *topic* 'Gazebo' controllers ('/gripper/command' and '/arm_controller/command'). 

Having explained how the solution works in general, we now explain what each node does in detail: 

- [ur10_robot_pose](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/scripts/ur10_robot_pose.py): This script creates a node that only posts the position of the *end-effector* with a certain frequency (10 Hz) via the *tf* library of * ROS *, i.e. it obtains the position of the *end-effector* from the information of the *topic* '/tf' published in the *topic* '/robot_pose'. The position of the *end-effector* is obtained from the difference between the positions of '/base_link' and '/ee_link' (it is the link associated with the clip). This node is not really necessary, the 'robot_manipulator' node could do this itself, albeit redundantly, but very useful for debugging and understanding how the scheduler works.
- [pub_gripper_cmd](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/scripts/pub_gripper_cmd.py): The script creates a node that only receives commands from *topic* '/pub_gripper_control' and forwards them to *topic* gripper controller '/gripper/command'. Although the functionality is simple, this node can change the value sent to the controller at any time because it does not check if the move was done correctly, which is a desired behavior. And if it does not receive an order, it continues to publish the previous order and keeps the value of the parenthesis. When it receives a new order, it immediately discards the previous value so that changes can be made while a movement is being executed.
- [pub_ik_trajectory](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/scripts/pub_ik_trajectory.py): Works the same way as described for the 'cmd_gripper_value_pub' node, this script instantiates the 'cmd_ik_trajectory_pub' node. It receives commands from the *topic* '/pub_ik_trajectory' to which it subscribes and transmits them to the arm controller *topic* '/arm_controller/command'. Also, changes can be made during the execution of a trajectory, allowing sudden changes of direction without having to wait until the previously specified position is reached.
- [robot_manipulator](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_manipulator/scripts/robot_manipulator.py): This script instantiates the 'robot_manipulator' node and determines the current position of the *end-effector* of the *subject* '/robot_pose'. This information is necessary to determine the values of each joint required to achieve the desired Cartesian position. Once the value that the joints must have is determined, they are published via the *topic* '/pub_ik_trajectory'. I had to implement a [script that executes the library function](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_manipulator/scripts/kinematics_utils.py). This library consists of several functions that perform the necessary calculations to obtain forward kinematics and inverse kinematics. In the case of the bracket, since only the value of a joint needs to be controlled, no calculations are required; the desired value is simply passed to the function. In the implementation, it is necessary to take into account the type of messages that the controllers must receive in order to set them correctly, otherwise they will perform no movements or undesirable ones. This solution can lead to problems because singularities can occur. This is because divisions by zero can occur when calculating the equations, especially when two joints are aligned.

--- 

##### :computer: robot_pose_publisher script
We will implement a node that publishes the position of the end effector 'ee_link' at any time, using the library *tf*. 

Since this is a tool and not a script that gives commands to the robot, this script will be provided in the 'Gazebo' directory. 

'''bash cd ~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_gazebo/scripts touch ur10_robot_pose.py chmod +x ur10_robot_pose.py
''' 

- View the contents of the [ur10_robot_pose.py](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/scripts/ur10_robot_pose.py) file. 

This node calculates the position of the 'ee_link' relative to the position of the 'base_link' using the *tf* library and publishes it in the *topic* ' \r obot_pose'. This allows you to quickly determine the current position of the grabber. 

- Node launch automation, add the following to the [~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_gazebo/launch/controller_utils.launch](https://github.com/Serru/MultiCobot- UR10-Gripper/blob/main/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/launch/controller_utils.launch) file.

```xml<!-- get the robot position [Own Script]--><node name="ur10_robot_pose" pkg="four_arm_no_moveit_gazebo" type="ur10_robot_pose.py" respawn="true"></node>
``` 

--- 

##### :computer: pub_gripper_cmd script
It enables the control of the gripper. For this purpose, a node is created that listens to a *topic* ('/pub_gripper_control') from which it receives the value it sends to the controller via the *topic* '/gripper/command'. 

This node is a support node, so it is located next to the 'gazebo' scripts, near the driver files:
'''bash cd ~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_gazebo/scripts touch pub_gripper_cmd.py chmod +x pub_gripper_cmd.py
''' 

- View the contents of the [pub_gripper_cmd.py](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/scripts/pub_gripper_cmd.py) file. 

- Node launch automation, add the following to the [~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_gazebo/launch/controller_utils.launch](https://github.com/Serru/MultiCobot- UR10-Gripper/blob/main/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/launch/controller_utils.launch) file.

```xml<!-- send the gripper commands [Own Script]--><node name="cmd_gripper_value_pub" pkg="four_arm_no_moveit_gazebo" type="pub_gripper_cmd.py" respawn="true"></node>
``` 

--- 

##### :computer: pub_ik_trajectory script
A node is implemented that receives commands via the *topic* '/pub_ik_trajectory' and sends them repeatedly to the robot controllers. The position it has to travel to is changed when it receives new commands. 

Since this script is considered a tool and not the script that gives the commands to the robots, it is implemented in the 'Gazebo' directory, just like in the previous section. 

'''bash cd ~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_gazebo/scripts touch pub_ik_trajectory.py chmod +x pub_ik_trajectory.py
''' 

- View the contents of the [pub_ik_trajectory.py](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/scripts/pub_ik_trajectory.py) file. 

- Node launch automation, add the following to the [~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_gazebo/launch/controller_utils.launch](https://github.com/Serru/MultiCobot- UR10-Gripper/blob/main/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/launch/controller_utils.launch) file.

```xml<!-- send the arms commands [Own Script]--><node name="cmd_ik_trajectory_pub" pkg="four_arm_no_moveit_gazebo" type="pub_ik_trajectory.py" respawn="true"></node>
``` 

--- 

##### :computer: robot_manipulator script
We will implement a node that receives the information received from the 'robot_pose_publisher' node and sends the trajectories to the 'pub_ik_trajectory' node. 

To get the values of the joints based on the Cartesian position you want to move to, you need to implement the *Inverse Kinematics* and *Forward Kinematics* functions. These functions were taken from [The Construct] (https://www.theconstructsim.com/) and adapted. 

For them:
'''bash cd ~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_manipulator/scripts touch kinematics_utils.py chmod +x kinematics_utils.py
'''
- Displays the contents of the [kinematics_utils.py](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_manipulator/scripts/kinematics_utils.py) library. 

Once the library is implemented, we proceed to develop the node that sends commands to the robot to perform the desired tasks, in this case *pick & place*, where the cobot picks a cube from the table and places it in a basket until there are no more cubes on the table. 

'''bash cd ~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_manipulator/scripts touchrobot_manipulator.py chmod +x robot_manipulator.py
''' 

It contains a set of functions to send the final trajectories correctly and get the values of the 'joints' in the robot's position. 

- See the contents of the library [robot_manipulator.py](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_manipulator/scripts/robot_manipulator.py). 

The robot has problems depending on the coordinates passed, since singularities can occur if the robot is blocked by constraints on the mathematical domain (e.g. if theta2 = acos(1), the value is undefined and generates an error or divisions for zero). 

To avoid this, a *workspace* can be defined where these singularities do not occur. 

Execution of the tests

#### :computer: test in 'Gazebo'.
A test was created where the robot grabs three wooden cubes and sends them into a container. 

The file with the handler code to run the test: [~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_manipulator/scripts/robot_manipulator.py](https://github.com/Serru/MultiCobot-UR10- Gripper/blob/main/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_manipulator/scripts/robot_manipulator.py) 

commands to start the test: 

-Terminal 1: 

'''bash cd ~/MultiCobot-UR10-Gripper/
source devel/setup.bash roslaunch four_arm_no_moveit_gazebo ur10_joint_limited.launch
'''
-Terminal 2: 

'''bash cd ~/MultiCobot-UR10-Gripper/
source devel/setup.bash rosrun four_arm_no_moveit_manipulator robot_manipulator.py
''' 

#### :book: Problems with pick-and-place tests
At the beginning of the tests, it was found that it is not possible to pick up the objects on the table in the simulation in 'Gazebo'. 

The reason for this is the absence of the 'Gazebo' plugin *gazebo_grasp*, which is included in the package *gazebo-pkgs*, previously installed as a resource. 

##### :computer: gazbebo_grasp plugin
We now perform the necessary steps to load the plugin that will allow the robot to interact with the objects in the simulation. 

First, we need the file *gzplugin_grasp_fix.urdf.xacro* (you can get it from the repository of [Jennifer Buehler](https://github-wiki-see.page/m/JenniferBuehler/gazebo-pkgs/wiki/The-Gazebo -grasp-fix-plugin)). It could be stored in the *universal robots* package, but since it is a modification for this project, it was decided to move it to the [~/MultiCobot-UR10-Gripper/src/multirobot /four_arm_no_moveit/four_arm_no_moveit_description/urdf](https://github.com/Serru/MultiCobot-UR10-Gripper/tree/main/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_description/urdf) directory. 

'''bash cd ~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_description/urdf touch gzplugin_grasp_fix.urdf.xacro
''' 

- See the contents of the file [gzplugin_grasp_fix.urdf.xacro](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_description/urdf/gzplugin_grasp_fix.urdf.xacro) . 

Once you have the 'Gazebo' plugin, you need to add it to the robot arm. The code to increase friction has also been added to the gripper to make it easier to grab objects in the simulation. 

'''bash nano ~/MultiCobot-UR10-Gripper/src/multirobot/four_arms_no_moveit/four_arms_no_moveit_description/urdf/ur10_robot.urdf.xacro
''' 

Add the plugin at the end:
```xml
<include filename="$(find four_arm_no_moveit_description)/urdf/gzplugin_grasp_fix.urdf.xacro"></include><gzplugin_grasp_fix prefix=""></gzplugin_grasp_fix>
``` 

- Take a look at the contents of the [ur10_robot.urdf.xacro](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_description/urdf/ur10_robot.urdf.xacro) file. 

In the same way we edit the gripper file ([~/MultiCobot-UR10-Gripper/src/robotiq_85_gripper/robotiq_85_description/urdf/robotiq_85_gripper.urdf.xacro](https://github.com/Serru/MultiCobot-UR10- Gripper/blob/main/src/robotiq_85_gripper/robotiq_85_description/urdf/robotiq_85_gripper.urdf.xacro)): 

'''bash nano ~/MultiCobot-UR10-Gripper/src/robotiq_85_gripper/robotiq_85_description/urdf/robotiq_85_gripper.urdf.xacro
''' 

- View the contents of the [robotiq_85_gripper.urdf.xacro](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/robotiq_85_gripper/robotiq_85_description/urdf/robotiq_85_gripper.urdf.xacro) file. 

This is all you need to configure the plugin correctly. If there are multiple robots, the plugin must be assigned correctly for each gripper, otherwise these grippers will not be able to grab objects during the simulation.

<a name="modificaciones">
<h2>
Modifications: Multirobot system composed of four robots
</h2>
</a>

![image](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/imgs_md/Diseno-planificador-fase-2-cuatro.png "Own planner design scheme") 

Before making changes to add four robots to the system, the picture shows how the nodes and communication between 'Gazebo', the controllers and the planner are set up for four robots. There are 16 blue nodes (4 planners) and 8 controllers (4 for UR10 arms and 4 for Robotiq grippers). 

This is a replication of what was previously developed in [Phase 3](#phase3) for a single robot. Therefore, note that changes are made to the 'Gazebo' controllers and their scripts that enable communication between them and the planner, changes are made to the robot modeling (URDF), and changes are made to the script that performs the *pick & place* task. 

Be careful with the scaling in this solution, the name of the joints must be unique, this is due to the configuration of the robot model. Since you are not in a *namespace* that automatically generates unique names for the joints, you will have to make the changes manually, which will also affect the *subject names*. You will also need to modify the scripts to publish and subscribe to the correct *subjects*. 

This does not mean that the system itself is not scalable, because it is easy to give the scripts an argument to use as a prefix, which automatically generates unique names from *start* files, just like a *namespace*.

<a name="modificaciones1">
<h3>
Phase 1: Gazebo Simulator Setup
</h3>
</a>

#### :book: `Gazebo` configuration
In this configuration, we need to redefine the files that configure the 'Gazebo' controllers. Essentially, we need to duplicate them and add the prefix 'ur10_1', 'ur10_2', 'ur10_3' and 'ur10_4' to the names of the joints, both for the clamp and for the arm of UR10. 

Likewise, the replicated and customised nodes for communication with the cobot controllers must be added to 'Gazebo' and the nodes that obtain the current positions of each cobot from the *topic* '/tf' must also be added to the 'controller_utils.launch' file.

##### :computer: Modifications to be made to simulate four robots in `Gazebo`
First we need to decide on the *namespace* for each robot, i.e. the group name under which we will group the configurations for each of the robots. 

We will start with the controllers: 

- File *ur10_1_arm_controller.yaml*.
'''bash cd ~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_gazebo/controllers mv arm_controller_ur10.yaml ur10_1_arm_controller.yaml
''' 

View the contents of the [ur10_1_arm_controller.yaml](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_gazebo/controller/ur10_1_arm_controller.yaml) file with the changes made. 

You can see that the prefix*ur10_1_* was simply added. This allows to distinguish to which 'joints' the commands should be sent, which also requires a change of the *URDF* file. 

We now proceed to modify the rest of the files and add the second set of controllers for the second robot, which get the prefix *ur10_2_*. 

- File *ur10_1_gripper_controller_robotiq.yaml*
'''bash cd ~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_gazebo/controllers mv gripper_controller_robotiq.yaml ur10_1_gripper_controller_robotiq.yaml
''' 

View the contents of the [ur10_1_gripper_controller_robotiq.yaml](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_gazebo/controller/ur10_1_gripper_controller_robotiq.yaml) file with the changes made. 

- File *ur10_2_arm_controller.yaml*.
'''bash cd ~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_gazebo/controllers cp ur10_1_arm_controller.yaml ur10_2_arm_controller.yaml
''' 

View the contents of the [ur10_2_arm_controller.yaml](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_gazebo/controller/ur10_2_arm_controller.yaml) file with the changes made to cape. 

- File *ur10_2_gripper_controller_robotiq.yaml*
'''bash cd ~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_gazebo/controllers mv ur10_1_gripper_controller_robotiq.yaml ur10_2_gripper_controller_robotiq.yaml
''' 

View the contents of the [ur10_2_gripper_controller_robotiq.yaml](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_gazebo/controller/ur10_2_gripper_controller_robotiq.yaml) file with the changes made. 

- File *ur10_3_arm_controller.yaml*.
'''bash cd ~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_gazebo/controllers cp ur10_1_arm_controller.yaml ur10_3_arm_controller.yaml
''' 

View the contents of the [ur10_3_arm_controller.yaml](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_gazebo/controller/ur10_3_arm_controller.yaml) file with the changes made. 

- File *ur10_3_gripper_controller_robotiq.yaml*
'''bash cd ~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_gazebo/controllers mv ur10_1_gripper_controller_robotiq.yaml ur10_3_gripper_controller_robotiq.yaml
''' 

View the contents of the [ur10_3_gripper_controller_robotiq.yaml](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_gazebo/controller/ur10_3_gripper_controller_robotiq.yaml) file with the changes made to cape. 

- File *ur10_4_arm_controller.yaml*.
'''bash cd ~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_gazebo/controllers cp ur10_1_arm_controller.yaml ur10_4_arm_controller.yaml
''' 

View the contents of the [ur10_4_arm_controller.yaml](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_gazebo/controller/ur10_4_arm_controller.yaml) file with the changes made. 

- File *ur10_4_gripper_controller_robotiq.yaml*
'''bash cd ~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_gazebo/controllers mv ur10_1_gripper_controller_robotiq.yaml ur10_4_gripper_controller_robotiq.yaml
''' 

View the contents of the [ur10_4_gripper_controller_robotiq.yaml](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_gazebo/controller/ur10_4_gripper_controller_robotiq.yaml) file with the changes made.

##### :computer: Modification of the `Gazebo` *launch* file
Now we have to change the *Launch* file to start the control of both robots in 'Gazebo'. 

''' {bash}
cd ~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_gazebo/launch nano ur10.launch
''' 

View the contents of the [ur10.launch]https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_gazebo/launch/ur10.launch) file with the changes made. 

'''bash cd ~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_gazebo/launch nano ur10_joint_limited.launch
''' 

View the contents of the [ur10_joint_limited.launch](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_gazebo/launch/ur10_joint_limited.launch) file with the changes made. 

You still need to change the file that launches the previously created scripts, as well as these files, as they have been corrected to correctly match the *namespaces*: 

- [~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_gazebo/launch/controller_utils.launch](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/four_arm_no_moveit /four_arm_no_moveit_gazebo/launch/controller_utils.launch) 

- [~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_gazebo/scripts/ur10_1_pub_gripper_cmd.py](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/four_arm_no_moveit /four_arm_no_moveit_gazebo/scripts/ur10_1_pub_gripper_cmd.py) 

- [~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_gazebo/scripts/ur10_1_pub_ik_trajectory.py](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/four_arm_no_moveit /four_arm_no_moveit_gazebo/scripts/ur10_1_pub_ik_trajectory.py) 

- [~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_gazebo/scripts/ur10_1_robot_pose.py](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/four_arm_no_moveit /four_arm_no_moveit_gazebo/scripts/ur10_1_robot_pose.py) 

- [~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_gazebo/scripts/ur10_2_pub_gripper_cmd.py](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/four_arm_no_moveit /four_arm_no_moveit_gazebo/scripts/ur10_2_pub_gripper_cmd.py) 

- [~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_gazebo/scripts/ur10_2_pub_ik_trajectory.py](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/four_arm_no_moveit /four_arm_no_moveit_gazebo/scripts/ur10_2_pub_ik_trajectory.py) 

- [~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_gazebo/scripts/ur10_2_robot_pose.py](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/four_arm_no_moveit /four_arm_no_moveit_gazebo/scripts/ur10_2_robot_pose.py) 

- [~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_gazebo/scripts/ur10_3_pub_gripper_cmd.py](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/four_arm_no_moveit /four_arm_no_moveit_gazebo/scripts/ur10_3_pub_gripper_cmd.py) 

- [~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_gazebo/scripts/ur10_3_pub_ik_trajectory.py](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/four_arm_no_moveit /four_arm_no_moveit_gazebo/scripts/ur10_3_pub_ik_trajectory.py) 

- [~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_gazebo/scripts/ur10_3_robot_pose.py](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/four_arm_no_moveit /four_arm_no_moveit_gazebo/scripts/ur10_3_robot_pose.py) 

- [~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_gazebo/scripts/ur10_4_pub_gripper_cmd.py](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/four_arm_no_moveit /four_arm_no_moveit_gazebo/scripts/ur10_4_pub_gripper_cmd.py) 

- [~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_gazebo/scripts/ur10_4_pub_ik_trajectory.py](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/four_arm_no_moveit /four_arm_no_moveit_gazebo/scripts/ur10_4_pub_ik_trajectory.py) 

- [~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_gazebo/scripts/ur10_4_robot_pose.py](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/four_arm_no_moveit /four_arm_no_moveit_gazebo/scripts/ur10_4_robot_pose.py)

<a name="modificaciones2">
<h3>
Phase 2: *URDF* configuration
</h3>
</a>

#### :book: File Description *URDF*
![image](/doc/imgs_md/solo-cuatro-urdf-robot.png "URDF file representation for two robots") 

It's similar to [Phase 1](#phase1) of the *Initial Configuration*, except this time there are four UR10 cobots, not just one. There are only two cobots in the picture, but it is very easy to add cobots to the model. 

In the file [URDF](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_description/urdf/ur10_robot.urdf.xacro) you can see how to connect the components of the 'ur10_robot' arms to the 'world' link, as you can see in the picture. It is not explained for four robots, because what was explained for modeling two robots is also valid for modeling N robots. 

##### :computer: directory configuration description 

You have to change the files *.urdf*, *ur10_joint_limited_robot.urdf.xacro* and *ur10_robot.urdf.xacro*: 

- Modify the file [~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_description/urdf/ur10_joint_limited_robot.urdf.xacro] (https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/ src/multirobot/four_arm_no_moveit/four_arm_no_moveit_description/urdf/ur10_joint_limited_robot.urdf.xacro) with:

```xml
[...] 

<link name="world"><joint name="ur10_1_world_joint" type="fixed"><parent link="world"></parent><child link="ur10_1_base_link"></child><origin xyz="0.6 -0.6 1.1" rpy="0.0 0.0 0.0"></origin></joint><!-- gripper --><robotiq_85_gripper prefix="ur10_1_" parent="ur10_1_ee_link"><origin xyz="0 0 0" rpy="0 0 0"></origin></robotiq_85_gripper><gzplugin_grasp_fix prefix="ur10_1_" prefix2="ur10_2_" prefix3="ur10_3_" prefix4="ur10_4_"></gzplugin_grasp_fix><!-- arm --><ur10_robot prefix="ur10_2_" joint_limited="true" shoulder_pan_lower_limit="${-pi}" shoulder_pan_upper_limit="${pi}" shoulder_lift_lower_limit="${-pi}" shoulder_lift_upper_limit="${pi}" elbow_joint_lower_limit="${-pi}" elbow_joint_upper_limit="${pi}" wrist_1_lower_limit="${-pi}" wrist_1_upper_limit="${pi}" wrist_2_lower_limit="${-pi}" wrist_2_upper_limit="${pi}" wrist_3_lower_limit="${-pi}" wrist_3_upper_limit="${pi}" transmission_hw_interface="$(arg transmission_hw_interface)"></ur10_robot><joint name="ur10_2_world_joint" type="fixed"><parent link="world"></parent><child link="ur10_2_base_link"></child><origin xyz="0.6 1.38 1.1" rpy="0.0 0.0 0.0"></origin></joint><!-- gripper --><robotiq_85_gripper prefix="ur10_2_" parent="ur10_2_ee_link"><origin xyz="0 0 0" rpy="0 0 0"></origin></robotiq_85_gripper><!-- arm --><ur10_robot prefix="ur10_3_" joint_limited="true" shoulder_pan_lower_limit="${-pi}" shoulder_pan_upper_limit="${pi}" shoulder_lift_lower_limit="${-pi}" shoulder_lift_upper_limit="${pi}" elbow_joint_lower_limit="${-pi}" elbow_joint_upper_limit="${pi}" wrist_1_lower_limit="${-pi}" wrist_1_upper_limit="${pi}" wrist_2_lower_limit="${-pi}" wrist_2_upper_limit="${pi}" wrist_3_lower_limit="${-pi}" wrist_3_upper_limit="${pi}" transmission_hw_interface="$(arg transmission_hw_interface)"></ur10_robot><joint name="ur10_3_world_joint" type="fixed"><parent link="world"></parent><child link="ur10_3_base_link"></child><origin xyz="0.6 3.36 1.1" rpy="0.0 0.0 0.0"></origin></joint><!-- gripper --><robotiq_85_gripper prefix="ur10_3_" parent="ur10_3_ee_link"><origin xyz="0 0 0" rpy="0 0 0"></origin></robotiq_85_gripper><!-- arm --><ur10_robot prefix="ur10_4_" joint_limited="true" shoulder_pan_lower_limit="${-pi}" shoulder_pan_upper_limit="${pi}" shoulder_lift_lower_limit="${-pi}" shoulder_lift_upper_limit="${pi}" elbow_joint_lower_limit="${-pi}" elbow_joint_upper_limit="${pi}" wrist_1_lower_limit="${-pi}" wrist_1_upper_limit="${pi}" wrist_2_lower_limit="${-pi}" wrist_2_upper_limit="${pi}" wrist_3_lower_limit="${-pi}" wrist_3_upper_limit="${pi}" transmission_hw_interface="$(arg transmission_hw_interface)"></ur10_robot><joint name="ur10_4_world_joint" type="fixed"><parent link="world"></parent><child link="ur10_4_base_link"></child><origin xyz="0.6 5.34 1.1" rpy="0.0 0.0 0.0"></origin></joint><!-- gripper --><robotiq_85_gripper prefix="ur10_4_" parent="ur10_4_ee_link"><origin xyz="0 0 0" rpy="0 0 0"></origin></robotiq_85_gripper>
``` 


- Modify the file [~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_description/urdf/ur10_robot.urdf.xacro](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/ src/multirobot/four_arm_no_moveit/four_arm_no_moveit_description/urdf/ur10_robot.urdf.xacro) with:

```xml
[...]<!-- arm --><ur10_robot prefix="ur10_1_" joint_limited="false" transmission_hw_interface="$(arg transmission_hw_interface)"></ur10_robot><link name="world"><joint name="ur10_1_world_joint" type="fixed"><parent link="world"></parent><child link="ur10_1_base_link"></child><origin xyz="0.6 -0.6 1.1" rpy="0.0 0.0 0.0"></origin></joint><!-- gripper --><robotiq_85_gripper prefix="ur10_1_" parent="ur10_1_ee_link"><origin xyz="0 0 0" rpy="0 0 0"></origin></robotiq_85_gripper><gzplugin_grasp_fix prefix="ur10_1_" prefix2="ur10_2_"></gzplugin_grasp_fix><!-- arm --><gzplugin_grasp_fix prefix="ur10_1_" prefix2="ur10_2_" prefix3="ur10_3_" prefix4="ur10_4_"></gzplugin_grasp_fix><joint name="ur10_2_world_joint" type="fixed"><parent link="world"></parent><child link="ur10_2_base_link"></child><origin xyz="0.6 1.38 1.1" rpy="0.0 0.0 0.0"></origin></joint><!-- gripper --><robotiq_85_gripper prefix="ur10_2_" parent="ur10_2_ee_link"><origin xyz="0 0 0" rpy="0 0 0"></origin></robotiq_85_gripper><!-- arm --><ur10_robot prefix="ur10_3_" joint_limited="false" transmission_hw_interface="$(arg transmission_hw_interface)"></ur10_robot><joint name="ur10_3_world_joint" type="fixed"><parent link="world"></parent><child link="ur10_3_base_link"></child><origin xyz="0.6 3.36 1.1" rpy="0.0 0.0 0.0"></origin></joint><!-- gripper --><robotiq_85_gripper prefix="ur10_3_" parent="ur10_3_ee_link"><origin xyz="0 0 0" rpy="0 0 0"></origin></robotiq_85_gripper><!-- arm --><ur10_robot prefix="ur10_4_" joint_limited="false" transmission_hw_interface="$(arg transmission_hw_interface)"></ur10_robot><joint name="ur10_4_world_joint" type="fixed"><parent link="world"></parent><child link="ur10_4_base_link"></child><origin xyz="0.6 5.34 1.1" rpy="0.0 0.0 0.0"></origin></joint><!-- gripper --><robotiq_85_gripper prefix="ur10_4_" parent="ur10_4_ee_link"><origin xyz="0 0 0" rpy="0 0 0"></origin></robotiq_85_gripper>
``` 

<a name="modificaciones3">
<h3>
Phase 3: Implementation of a custom scheduler that performs a <i> pick &amp; place </i>
</h3>
</a>

#### :book: Creating the scheduler and helper nodes
The scheduler was already developed in [Phase 1](#Modifications1) of this section on modifications for a four-robot multirobot system, where the nodes that are part of the scheduler were implemented. 

This section is now about implementing the scripts that use the [library](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_manipulator/scripts/kinematics_utils.py) developed to perform the direct kinematics to control the robot arms. 

For the *Pick & Place* task, four scripts are implemented that are started in different terminals so that they send commands to each robot simultaneously.

##### :computer: Modification of the pick and place
In the directory *four_arm_no_moveit_manipulator* the following files are changed:
- [~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_manipulator/ur10_1_robot_manipulator.py*](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/four_arm_no_moveit/ four_arm_no_moveit_manipulator/scripts/ur10_1_robot_manipulator.py).
- [~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_manipulator/ur10_2_robot_manipulator.py](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_manipulator /scripts/ur10_2_robot_manipulator.py).
- [~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_manipulator/ur10_3_robot_manipulator.py*](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/four_arm_no_moveit/ four_arm_no_moveit_manipulator/scripts/ur10_3_robot_manipulator.py).
- [~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_manipulator/ur10_4_robot_manipulator.py](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_manipulator /scripts/ur10_4_robot_manipulator.py). 

The 'Gazebo' plugin needs to be fixed so it can grab objects with both grippers:
- File [~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_description/urdf/gzplugin_grasp_fix.urdf.xacro](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/ multirobot/four_arm_no_moveit/four_arm_no_moveit_description/urdf/gzplugin_grasp_fix.urdf.xacro). 

And finally, customise the 'world' file to run the simulations in a suitable environment:
- File [~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/world/multiarm_bot.world](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/ four_arm_no_moveit/four_arm_no_moveit_gazebo/world/multiarm_bot.world).



<a name="pruebas2">
<h3>
Execution of the tests
</h3>
</a>

#### :computer: Launch simulation tests 

5 terminals are needed: 

-Terminal 1:
```bash
roslaunch four_arm_no_moveit_gazebo ur10_joint_limited.launch
``` 

-Terminal 2:
```bash
rosrun four_arm_no_moveit_manipulator ur10_1_robot_manipulator.py
``` 

-Terminal 3:
```bash
rosrun four_arm_no_moveit_manipulator ur10_2_robot_manipulator.py
``` 

-Terminal 4:
```bash
rosrun four_arm_no_moveit_manipulator ur10_3_robot_manipulator.py
``` 

-Terminal 5:
```bash
rosrun four_arm_no_moveit_manipulator ur10_4_robot_manipulator.py
``` 

#### :book: Final result information 

- Visual result in `Gazebo`
![image](/doc/imgs_md/four-arm-no-moveit-gazebo.png "Result in Gazebo") 

- Scheme of the nodes and *topic*s of the system
![image](/doc/imgs_md/four-arm-no-moveit-graph.png "System nodes and topics") 

- Tree of transformations of the robot model
![image](/doc/imgs_md/four-arm-no-moveit-tree.png "Transform Tree") 

---

 <p align="left">
   <button name="button"><a rel="license" href="https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/no-moveit-intro.md"> Back </a></button>
 </p>
