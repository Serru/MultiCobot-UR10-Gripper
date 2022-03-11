# Installation and configuration for two UR10 robots with `MoveIt!` 

**Spanish** | [English](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/moveit/ENG/one_arm_moveit.md) 

![image](/doc/imgs_md/Diseno-moveit-general-dos-cobots-leap-motion.png "Loaded the URDF model of the UR10 robot") 

This time, the solution for two robots is created in the same way as for one robot. However, the content of the files is changed to adapt them to the simulation with two robots.

## Prerequisite
- Successfully install the [base system configuration](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/setup-doc/proyect_setup.md).
- Implement the [Solution for a robot without the 'MoveIt!' scheduler](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/no_moveit/ ESP /one_arm_no_moveit.md).

## Index
- [Initial setup: setup for a robot](#setup-initial).
- [Phase 1: *URDF* configuration](#phase1).
- [Phase 2: Configuration of 'MoveIt!'](#phase2).
- [Phase 3: Simulation of a 'Pick & Place' in 'Gazebo'](#phase3)
- [Execution of tests](#tests)
- Modifications: Multirobot system consisting of two robots](#modifications)
- [Configuration in 'Gazebo' for two cobots](#modifications1)
- [Configuration in 'MoveIt!' for two cobots](#modifications2)
- Execution of tests](#tests2)

<a name="setup-inicial">
<h2>
Initial setup: Setup for a robot
</h2>
</a>

In this section, a replication of the configuration for a single robot is made, which serves as a basis and thus later explains the changes made to be able to control two robots. 

### :warning: Contents of the file
Do not blindly copy and paste the contents of the files into this section. When you explain the configuration for one robot, the information is identical to that from the **one_arm_moveit** package, but you must adapt the contents to the new package, in this case **two_arm_moveit**. 

There are two ways to do this:
- If you created the solution for a [one_arm_moveit] robot (https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/moveit/ ESP /one_arm_moveit.md), you can continue with what you implemented, but in the *Modifications* section you need to change **two_arm_moveit** to **one_arm_moveit**.
- If the solution for a [one_arm_moveit](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/moveit/ ESP /one_arm_moveit.md) robot has not yet been created, you can follow the steps presented in this section. Note, however, that the contents of the files at this stage of *initial configuration* are linked to the files of **one_arm_moveit**. Therefore, you must replace references to the **one_arm_moveit** package with **two_arm_moveit** when you create the copy. 

If this information is not taken into account, the package will not compile correctly and the compiler itself will force a choice between one of the previously suggested options.

<a name="fase1">
<h3>
Phase 1: URDF configuration
</h3>
</a>

#### :book: File description *URDF*
The *URDF* (United Robotics Description Format) file models the cobot using the *XML* format, which will be used by the different applications that *ROS* needs, but mainly to perform a simulation of the modeled robot. 

The file is built in the form of a tree, where there are three main tags: ` <robot> `, ` <link> ` and ` <joint> `. To explain it well, the arm of the human body can be taken as a reference. If you want to model a person's arm, the ` <robot> ` tag represents the arm as a whole. This arm is made up of several bones (humerus, ulna, and radius) that are represented by the ` <link> ` tags, and a joint that joins those bones (elbow) that is represented by the ` <joint> ` tag. 

In addition, as in the bones, these labels can go with additional information contained in them that give information on size, geometry, inertia, orientation, etc. Finally, the modeling of a robot can be joined to another model and form a more complex one, which could be represented with the addition of the hand to the arm, with the wrist as a joint that connects both. Note that ` <joint> ` tags connect ` <link> ` tags through a parent-child relationship. 

Having said that, a representation of the robot components is made: 

![image](/doc/imgs_md/urdf-robot.png "URDF file representation") 

The image shows the content of the [*URDF* file](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_description/urdf/ur10_robot.urdf.xacro ) that models the robot next to the gripper, you can see how the UR10 robot arm component is connected with the `world` link, representing `world` (yellow color) and the base of the UR10 arm `base_link` (green color ) located just above, also the joint `world_joint` is the yellow sphere located between both links. In the same way, there is the component of the gripper `robotiq_85_gripper`, it is connected to the arm of the UR10 (ur10 robot), where the sphere that represents the joint `robotiq_85_base_joint` that unites both components (purple color), uniting the link `robotiq_85_base_link` of the gripper with the link `ee_link` of the UR10 arm. 


#### :computer: Create solution directory
```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot
mkdir two_arm_moveit
``` 

#### :computer: directory configuration description
A new package is created and the *one_arm_no_moveit* project directories are copied for later modification. 

```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_moveit
catkin_create_pkg two_arm_moveit_description rospy
``` 

In the directory created for *description*, the *one_arm_no_moveit_description* directory, *launch* and *urdf* folders will be copied. 

```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_moveit/two_arm_moveit_description
cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_description/launch .
cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_description/urdf .
``` 

#### :computer: Modification of *description* files 

- [~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_moveit/two_arm_moveit_description/launch/ur10_upload.launch](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit /one_arm_moveit_description/launch/ur10_upload.launch) 

- [~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_moveit/two_arm_moveit_description/urdf/ur10_robot.urdf.xacro](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot /one_arm_moveit/one_arm_moveit_description/urdf/ur10_robot.urdf.xacro) 

- [~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_moveit/two_arm_moveit_description/urdf/ur10_joint_limited_robot.urdf.xacro](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot /one_arm_moveit/one_arm_moveit_description/urdf/ur10_joint_limited_robot.urdf.xacro) 

It compiles:
```bash
cd ~/MultiCobot-UR10-Gripper
catkin_make
``` 

<a name="fase2">
<h3>
Phase 2: Configuring MoveIt!
</h3>
</a>

Before configuring with the `Setup Assistant` you must have the `URDF` well defined previously, with this fact the configuration assistant is launched with the following command in the terminal: 

```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_moveit/
mkdir two_arm_moveit_config
roslaunch moveit_setup_assistant setup_assistant.launch
``` 

#### :computer: `MoveIt!` Setup Assistant
The *URDF* file will be chosen as the robot model: [~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_moveit/two_arm_moveit_description/urdf/ur10_joint_limited_robot.urdf.xacro](https://github.com/Serru/ MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_description/urdf/ur10_joint_limited_robot.urdf.xacro) (could be perfectly [ur10_robot.urdf.xacro](https://github.com/Serru/MultiCobot- UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_description/urdf/ur10_robot.urdf.xacro)). 

- Starting the configuration of the Setup Assistant, you have to tell it where the `UR10_joint_limited_robot.urdf.xacro` file is, that is, the model of the robot that you want to configure:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_1.png "Load UR10 robot URDF model") 

- Subsequently, the *Load Files* button is given.
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_2.png "URDF model of UR10 robot loaded") 

- In the *Self-Collisions* tab, click the *Generate Collision Matrix* button, which will generate a matrix between the different robot components that can generate self-collision during trajectory planning:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_3.png "Collision matrix generation") 

- In the *Virtual Joints* tab it is directed especially for robotic arms installed on a mobile base, in this case it does not affect the configuration, since the base is fixed, but it will be configured equally for future configurations, a joint must be created between the `base_link` robot and the `world` frame, with the following configuration:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_4.png "Defining Virtual Joint")
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_5.png "Virtual Joint Defined") 

- One of the most important tabs is to define the *Planning groups* well, in this case there are two groups, the *manipulator* group that will control the robot arm and the *gripper* group that will control the gripper:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_6.png "Manipulator kdl") 

- Then you have to hit the *Add Kin button. Chain*:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_7.png "Manipulator Kinetic Chain setup") 

- Finally, the configuration is saved:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_8.png "Manipulator group set up") 

- Now you have to do it for the *Gripper* group that will control the gripper, press the *Add Group* button, fill in the name of the group and put *kdl* as *Kinematic Solver*:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_9.png "Gripper kdl") 

- After clicking the *Add Joints* button, you have to search for *robotiq_85_left_knucle_joint* and add it to with the arrow *- &gt; * and the configuration is saved:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_10.png "Gripper Joint setup") 

- After saving, the result in the *Planning Group* tab should be the following:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_11.png "Planning Group Final Setup") 

- In the *Robot Poses* tab is where fixed poses are configured in advance in the robot, the *home* pose will be configured, which will reflect the initial position of the cobot:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_12.png "Setting up *home* 1/3")
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_13.png "Setting up *home* 2/3")
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_14.png "Setting up *home* 3/3") 

- In the *Robot Poses* tab, the *gripper_open* pose is going to be configured:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_24.png "Setting up *gripper open*") 

- In the *Robot Poses* tab, the *gripper_close* pose is going to be configured:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_25.png "Setting *gripper close*") 


- In the *End Effectors* tab the clamp will be added:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_15.png "Setting up end effector 1/3")
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_16.png "Setting up end effector 2/3")
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_17.png "Setting up end effector 3/3") 

- In the *Passive Joints* tab it represents the joints that cannot be moved actively, in the case of the *gripper* defined in your *URDF* file they are the `joints` that mimic the movement of another joint. The `joints` defined as passive will not be taken into account in the planning, in this case the configuration is as follows:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_18.png "Setting up Passive Joints") 

- In the *ROS Control* tab, it will be added automatically, the generated files will be later manually modified.
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_19.png "Setting up ROS Control")
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_20.png "Setting up ROS Control") 

- You have to fill in the *Author Information* tab so that the configuration can finish:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_21.png "Setting Author Information") 

- The last tab *Configuration Files*, allows you to decide where the `MoveIt!` configuration will be saved, in this case in `one_arm_moveit_config` previously created, the configuration is generated using the *Generate Package* button and finally *Exit Setup Assistant* to end:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_22.png "Setting Author Information")
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_23.png "Setting Author Information") 


<a name="fase3">
<h3>
Phase 3: Simulation of a <i> pick &amp; place </i> in Gazebo
</h3>
</a>

This phase is divided into two stages 

#### :book: Connection between `Gazebo` and `MoveIt!` 

The first thing to do in this phase is to configure the `Gazebo` and the controllers so that it can properly simulate the movements of the cobot. The *two_arm_moveit_gazebo* package is created, which will contain all the configuration related to `Gazebo`, including controllers. Once the package is created, you have to configure the controllers that are stored in the *controller* directory, although all the controllers can be defined in a single file for clarity it has been distributed in three files. 

The controllers are defined in files with *yaml* extension, to define these controllers you have to give them a name and define the type of the controller, the dynamic `joints` that you want to control, the restrictions that it has, the publication ratio and other options . 

These controllers are briefly explained: 

- File [arm_controller_ur10.yaml](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_gazebo/controller/arm_controller_ur10.yaml): The controller is defined in this file for the UR10 cobot, this defines the controller name `arm_controller`, the controller type position `controllers/JointTrajectoryController`, which implies the definition of the type of messages and the proper formatting of the information needed to communicate with it. Then there is the `joints` field, which is where it is indicated which `joints` of the cobot are part of the controller, all these `joints` are dynamic. The rest of the fields have not been touched, but consistency must be maintained in how they are named. 

- File [gripper_controller_robotiq.yaml](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_gazebo/controller/gripper_controller_robotiq.yaml): The controller is defined in this file for the *Robotiq* gripper, here you define the controller name `gripper`, the controller type `position controllers/JointTrajectoryController` which defines the type of messages and the information needed to communicate with it. Then there is the `joints` field, which is where it is indicated which `joints` of the cobot are part of the controller, in this case a single `joint_robotiq_85_left_knuckle_joint` because the rest of the `joints` of the controller imitate the movements of this one. The rest of the fields have not been touched, but consistency must be maintained in how they are named as in the previous case. 

- File [joint_state_controller.yaml](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_gazebo/controller/joint_state_controller.yaml): What this file defines is not really it is a controller as such, its function is that of an interface that translates the `joints` information that comes from the real cobot and translates it into `JointState` type messages to later publish it. It is essential for proper operation, both in
simulation as with the real robot, it is part of the *ROS* *ros_control* package. 

##### :computer: Starting `Gazebo` 

We are going to create the package for `Gazebo`, and copy the content of the solution starting from the *one_arm_no_moveit* solution for later modification:
```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_moveit
catkin_create_pkg two_arm_moveit_gazebo rospy
```
In the directory created for `Gazebo`, it will copy the *one_arm_no_moveit_gazebo* directory, the *controller*, *launch*, *models*, and *world* folders.
```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_moveit/two_arm_moveit_gazebo
cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/controller .
cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/launch .
cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/models .
cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/world .
``` 

##### :computer: Modifying the `Gazebo` files
Files with the following content are created: 

- [~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_moveit/two_arm_moveit_gazebo/launch/ur10_joint_limited.launch](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit /one_arm_moveit_gazebo/launch/ur10_joint_limited.launch) 

- [~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_moveit/two_arm_moveit_gazebo/launch/ur10.launch](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit /one_arm_moveit_gazebo/launch/ur10.launch) 

It compiles:
```bash
cd ~/MultiCobot-UR10-Gripper
catkin_make
``` 

If the configuration is left as it is, communication between `Gazebo` and `MoveIt!` is not possible. When launching the *demo.launch* file it also launches *Rviz* which allows the visualization of the data and it can be seen that it works correctly, but with the automated configuration, it is not possible to observe its operation in the `Gazebo` simulator. 

This is because there is no communication between `MoveIt!` and `Gazebo`, so this communication is solved. 

For convenience, for possible evaluation in the real robot, we will keep the launch `Gazebo` and `MoveIt!` separate. 

To get started and better understand the `MoveIt!` package, let's separate the contents of the packages into `MoveIt!` and `Gazebo`: 

`Gazebo`:
-launch:
- gazebo.launch
- ros_controllers.launch
-config:
- **ros_controllers.yaml** 

The *gazebo.launch* file provided by `MoveIt!` is not necessary, since the `Gazebo` package to be used has already been configured. The configuration is very similar if you take a look at the files involved. 

`MoveIt!`:
-launch:
- **demo.launch**
- planning_context.launch
-config:
- ur10.srdf
- joint_limits.yaml
- kinematics.yaml
- **move_group.launch**
- planning_context.launch
-config:
- ur10.srdf
- joint_limits.yaml
- kinematics.yaml
- planning_pipeline.launch.xml
- ompl_planning_pipeline.launch.xml
- **trajectory_execution.launch.xml**
- **ur10_moveit_controller_manager.launch.xml**
-config:
- **ros_controllers.yaml**
- sensor_manager.launch.xml
-config:
-sensors_3d.yaml
- ur10_moveit_sensor_manager.launch.xml
- moveit_rviz.launch (Rviz)
- moveit.rviz
-config:
- kinematics.yaml 


You can see in the directory scheme how the files are related. Among them, those that are going to be modified manually are indicated in bold. 

The controllers configuration file is the same for both `Gazebo` and `MoveIt!`, this means that in order to connect the controllers properly, their configuration must be the same as what was done for `Gazebo` previously and that is configured in the *Commissioning in `Gazebo`* section. 

Therefore, you can add the `Gazebo` controllers (.yaml files) to the `MoveIt!` package to correctly connect the robot controllers (manipulator and gripper). 

If you launch `Gazebo` and `MoveIt!` without making any changes, you get the following plot of nodes and *topic*s: 

- terminal 1
```bash
roslaunch two_arm_moveit_config demo.launch
``` 

- terminal 2
```bash
roslaunch two_arm_moveit_gazebo ur10_joint_limited.launch
``` 

![ ](/doc/imgs_md/one_arm_moveit_graph_no_changes.png "Schematic without changes") 

It is seen that `Gazebo` correctly loads the `arm_controller` and `gripper` controllers, but there is no communication between the `move_group` node and the controllers of the `gazebo` node, the only point in common is the *topic* `/joint_states `. This means that if trajectories are planned and executed with the `MoveIt!` plugin *Motion Planning*, they will not be rendered in `Gazebo` but in *Rviz*. This is not the desired result, so we proceed to modify the configuration of `MoveIt!` so that it can communicate with the controllers loaded in `Gazebo`. 

Therefore, the following files must be modified. The modified files will be in a different package with the intention of facilitating the compression of the modifications made to the configuration. 

To do this, the following files will be modified based on their original files *demo.launch*, *move_group.launch*, *trajectory_execution.launch.xml* and *ur10_moveit_controller_manager.launch.xml* and the controllers will be added by creating two files *controllers.yaml* and *joint_names.yaml*: 

```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_moveit
catkin_create_pkg two_arm_moveit_manipulator rospy
mkdir config
touchconfig/controllers.yaml
touch config/joint_names.yaml
mkdir launch
touch launch/two_arm_moveit_execution.launch
touch launch/move_group.launch
touch launch/trajectory_execution.launch.xml
touch launch/ur10_moveit_controller_manager.launch.xml
mkdir scripts
```
##### :computer: We will proceed to add the controllers for its interaction with `Gazebo` 

File [~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_moveit/two_arm_moveit_manipulator/config/controllers.yaml](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit /one_arm_moveit_manipulator/config/controllers.yaml): The definition of the *ROS* controllers in `MoveIt!` is similar to that for `Gazebo`, the name of the controllers must match the names of the controllers described in `Gazebo`, the action server `follow_joint_trajectory` is defined, the type must be `FollowJointTrajectory` so that the message type sent between them are compatible and finally the names of the `joints` involved must be identical as well. 

Another file to modify is [~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_moveit/two_arm_moveit_manipulator/config/joint_names.yaml](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src /multirobot/one_arm_moveit/one_arm_moveit_manipulator/config/joint_names.yaml): This file defines the name of the `joints` of the cobot controller, it will be stored as a server parameter and will be used as part of the `MoveIt!` configuration. 

Of the files in the *launch* directory, we are going to modify the *two_arm_moveit_execution* file which is the entry point for using the `MoveIt!` package and *Rviz* which is based on the *demo.launch* file. 

File [~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_moveit/two_arm_moveit_manipulator/launch/two_arm_moveit_execution.launch](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit /one_arm_moveit_manipulator/launch/one_arm_moveit_execution.launch) 


Then tweak the controller launcher and `MoveIt!`: 

File [~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_moveit/two_arm_moveit_manipulator/launch/move_group.launch](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit /one_arm_moveit_manipulator/launch/move_group.launch) 


File [~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_moveit/two_arm_moveit_manipulator/launch/trajectory_execution.launch.xml](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot /one_arm_moveit/one_arm_moveit_manipulator/launch/trajectory_execution.launch.xml): Set up communication with a real robot, in this case `Gazebo` is the one that simulates the robot, but `MoveIt!` is not aware of that and treats it as a actual robots. 


File [~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_moveit/two_arm_moveit_manipulator/launch/ur10_moveit_controller_manager.launch.xml](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot /one_arm_moveit/one_arm_moveit_manipulator/launch/ur10_moveit_controller_manager.launch.xml): Load controllers defined for `MoveIt!` 


##### :computer: Finally, a test is performed:
```bash
cd ~/MultiCobot-UR10-Gripper
catkin_make
``` 

-Terminal 1
```bash
roslaunch two_arm_moveit_gazebo ur10_joint_limited.launch
``` 

-Terminal 2
```bash
roslaunch two_arm_moveit_manipulator two_arm_moveit_execution.launch
``` 

![ ](/doc/imgs_md/one_arm_moveit_26.png "Example gazebo+rviz+moveit! (1/2)") 

![ ](/doc/imgs_md/one_arm_moveit_27.png "Example gazebo+rviz+moveit! (2/2)") 

And the graph of the nodes and the *topic*s, after the modifications, you can see how now the *move_group* node has communication with the controllers and it is *Gazebo* that listens to what is published for perform the desired movements. These moves modify the current state of the robot which is published to the *topic* `/jont_states` and that information is passed to the `robot_state_publisher` node and the `move_group` node. The `move_group` node can recalculate a new path with the information it gets from the *topic*s `/tf` and `/jont_states`. 

![ ](/doc/imgs_md/one_arm_moveit_graph_changes.png "rqt_graph representation of nodes and topics") 

##### :computer: Pick and Place 

To script *pick &amp; place* in Python, the Python `moveit_commander` interface is used to communicate with the `move_group` node and its services and actions. It will not go into detail because for the control of a single robot it is not very problematic and there is good documentation, therefore the script will be described in detail for the solution with two or more cobots. 

Very simple tests are carried out. To do this, first you have to create the necessary scripts to control the robot arm and the gripper correctly and then the movements will be made so that the robot picks up a cube from the table and leaves it in the basket. 

```bash
cd scripts
touch two_arm_moveit.py
``` 

File [~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_moveit/two_arm_moveit_manipulator/scripts/two_arm_moveit.py](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit /one_arm_moveit_manipulator/scripts/one_arm_moveit.py) 

<a name="pruebas">
<h2>
Execution of the tests
</h2>
</a>

```bash
#Terminal 1
roslaunch two_arm_moveit_gazebo ur10_joint_limited.launch 

#Terminal 2
roslaunch two_arm_moveit_manipulator two_arm_moveit_execution.launch 

#Terminal 3
rosrun two_arm_moveit_manipulator two_arm_moveit.py
``` 

<a name="modificaciones">
<h2>
Modifications: Multirobot system composed of two robots
</h2>
</a>

To be able to control two or more cobots at the same time and with different controllers, which means that they can be cobots of different brands and models, the 'MoveIt!' node and the simulated robot are replicated in 'Gazebo'. 

If you want to do a proper replication, you need to apply the concept of *namespace*, which can be seen like a *directory* containing nodes, *themes* or even other directories (*namespaces*) that also allow nested organization and * ROS * allows to run instances of the same node as long as they are in different *namespaces*. Building on what has been done so far, changes are made to the 'two_arm_moveit_gazebo' and 'two_arm_moveit_manipulator' packages, which include the changes to the 'MoveIt!' package ('two_arm_moveit_config)' previously configured by the setup wizard. 

The configuration process is divided into two parts: the configuration in 'Gazebo' for two cobots and the configuration in 'MoveIt! 

<a name="modificaciones1">
<h3>
Configuration made in Gazebo for two cobots
</h3>
</a>

To have two or more cobots in the simulation, it is explained what changes must be made in the *Launch* files to allow adding two or more cobots in the simulation. 

The idea is to create an external file that replicates (launches instances of) as many cobots as you want to add to the simulation. Therefore, the 'Gazebo' files from the 'two_arm_moveit_gazebo' package are prepared first. 

- File [ur10_joint_limited.launch](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/two_arm_moveit/two_arm_moveit_gazebo/launch/ur10_joint_limited.launch): Two arguments have been added to the original file, the robot name ('robot_name') and the initial pose ('init_pose'). These two arguments are taken from the file that contains this *launch*. 

- File [ur10.launch](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/two_arm_moveit/two_arm_moveit_gazebo/launch/ur10.launch): Similar to the *launch* file above ('ur10_joint_limited.launch'), two arguments have been added to this file, namely the robot name ('robot_name') and the initial pose ('init_pose'), which are defined in the file that contains it. Apart from the addition of the arguments, the instantiation of the virtual world 'Gazebo' and the loading of the robot model into the parameter server ('robot_description') have been removed. 

- File [controller_utils.launch]https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/two_arm_moveit/two_arm_moveit_gazebo/launch/controller_utils.launch): In this file you only need to comment out the 'tf_prefix' parameter. Its default value is an empty string, but this interferes with changing the default value.  

It compiles:
```bash
cd ~/MultiCobot-UR10-Gripper
catkin_make
``` 

##### :computer: Replication of Cobots in `Gazebo` 

Once the files of the 'Gazebo' package 'two_arm_moveit_gazebo' are configured, multiple cobots are instantiated in it. This is done by creating a *launch* file named 'two_arm_moveit_gazebo' inside the 'two_arm_moveit_manipulator' package (it can be any other package). launch', which contains the following: 

```xml
<launch><param name="/use_sim_time" value="true"><arg name="robot_name"></arg><arg name="init_pose"></arg><arg name="paused" default="false"></arg><arg name="gui" default="true"></arg><arg name="limited" default="true" doc="If true, limits joint range [-PI, PI] on all joints."></arg><!-- send robot urdf to param server --><include file="$(find two_arm_moveit_description)/launch/ur10_upload.launch"><arg name="limited" value="$(arg limited)"></arg></include><include file="$(find gazebo_ros)/launch/empty_world.launch"><arg name="verbose" value="true"></arg><arg name="world_name" default="$(find two_arm_moveit_gazebo)/world/multiarm_bot.world"></arg><arg name="paused" value="$(arg paused)"></arg><arg name="gui" value="$(arg gui)"></arg></include><group ns="ur10_1"><param name="tf_prefix" value="ur10_1"><include file="$(find two_arm_moveit_gazebo)/launch/ur10_joint_limited.launch"><arg name="init_pose" value="-x 0.6 -y -0.6 -z 1.1"></arg><arg name="robot_name" value="ur10_1"></arg></include></group><group ns="ur10_2"><param name="tf_prefix" value="ur10_2"><include file="$(find two_arm_moveit_gazebo)/launch/ur10_joint_limited.launch"><arg name="robot_name" value="ur10_2"></arg><arg name="init_pose" value="-x 0.6 -y 1.38 -z 1.1"></arg></include></group><node pkg="tf" type="static_transform_publisher" name="world_frames_connection_1" args="0 0 0 0 0 0 /world /ur10_1/world 100"></node><node pkg="tf" type="static_transform_publisher" name="world_frames_connection_2" args="0 0 0 0 0 0 /world /ur10_2/world 100"></node></launch>
``` 

The first thing to note in the contents of the file ([two_arm_moveit_gazebo.launch](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/two_arm_moveit/two_arm_moveit_manipulator/launch/two_arm_moveit_gazebo.launch )) is that it loads the robot model into the parameter server and instantiates the virtual world 'Gazebo' that was removed from the file [ur10.launch](https://github.com/Serru/MultiCobot-UR10- Gripper/blob/main/src/multirobot/two_arm_moveit/two_arm_moveit_gazebo/launch/ur10.launch). This needs to be here because you only want to instantiate the world 'Gazebo' once. 

Then two groups appear, 'ur10_1' and 'ur10_2', this is how you define the *namespaces*. The configuration of these cobots is identical except for three things, namely the value of the 'tf_prefix' parameter, which is the prefix for the transformations. It is important that the group name (*namespace*), the name ('robot_name') and the initial position ('init_pose') match. 

If you want to add more cobots to the system, just copy the content of *group* and change the content accordingly. And note that the last two lines of code bind the base of the cobots to the 'world' frame.


<a name="modificaciones2">
<h3>
Configuration made in MoveIt! for two cobots
</h3>
</a>


The changes to the 'MoveIt!' configuration are very minor, basically you need to group the code implemented for a *single cobot* under a *namespace*, adjust the *map* with the namespace, and then repeat the process as many times as cobots are simulated. Comparing the contents of the [one_arm_moveit_execution.launch](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_manipulator/launch/one_arm_moveit_execution.launch) file implemented for a single cobot to the contents of the [two_arm_moveit_execution.launch](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/two_arm_moveit/two_arm_moveit_manipulator/launch/two_arm_moveit_execution.launch) file, essentially everything has been grouped into a *namespace* and the name of the *namespace* has been added as a prefix in the names of the *themes* of the remap for proper communication with the controllers. 

Contents of the file 'two_arm_moveit_execution.launch':
```xml
<launch><arg name="sim" default="false"></arg><arg name="debug" default="false"></arg><!-- By default, we do not start a database (it can be large) --><arg name="demo" default="false"></arg><group ns="ur10_1"><rosparam command="load" file="$(find two_arm_moveit_manipulator)/config/joint_names.yaml"></rosparam><include file="$(find two_arm_moveit_manipulator)/launch/move_group.launch"><arg name="debug" default="$(arg debug)"></arg><arg name="publish_monitored_planning_scene" value="true"></arg></include><!-- If database loading was enabled, start mongodb as well --><include file="$(find two_arm_moveit_config)/launch/default_warehouse_db.launch" if="$(arg demo)"></include><!-- Remap follow_joint_trajectory --><remap from="/ur10_1/follow_joint_trajectory" to="/ur10_1/arm_controller/follow_joint_trajectory"></remap><include file="$(find two_arm_moveit_config)/launch/moveit_rviz.launch"><arg name="config" value="true"></arg><arg name="debug" default="false"></arg></include></group><group ns="ur10_2"><rosparam command="load" file="$(find two_arm_moveit_manipulator)/config/joint_names.yaml"></rosparam><include file="$(find two_arm_moveit_manipulator)/launch/move_group.launch"><arg name="debug" default="$(arg debug)"></arg><arg name="publish_monitored_planning_scene" value="true"></arg></include><!-- If database loading was enabled, start mongodb as well --><include file="$(find two_arm_moveit_config)/launch/default_warehouse_db.launch" if="$(arg demo)"></include><!-- Remap follow_joint_trajectory --><remap from="/ur10_2/follow_joint_trajectory" to="/ur10_2/arm_controller/follow_joint_trajectory"></remap><include file="$(find two_arm_moveit_config)/launch/moveit_rviz.launch"><arg name="config" value="true"></arg><arg name="debug" default="false"></arg></include></group></launch>
``` 

A test of what has been implemented so far is performed:
```bash
cd ~/MultiCobot-UR10-Gripper
catkin_make clean
catkin_make
rospack profile
``` 

-Terminal 1
```bash
roslaunch two_arm_moveit_manipulator two_arm_moveit_gazebo.launch
``` 

-Terminal 2
```bash
roslaunch two_arm_moveit_manipulator two_arm_moveit_execution.launch
``` 



##### :computer: Pick and Place
As with the previous solutions, a very simple test is performed. To do this, you must first create the necessary scripts to correctly control the robot arms and grippers. Then, the same movements as in the previously proposed solutions will be executed. 

The following piece of code corresponds to the configuration to be able to communicate with the API of the node 'move_group' when it is in a *namespace*: 

```python
[...] 

1 def main():
2 moveit_commander.roscpp_initializer.roscpp_initialize(sys.argv)
3 rospy.init_node('ur10_1_arm_moveit',
4 anonymous=True)
5
6 PLANNING_GROUP_GRIPPER = "gripper"
7 PLANNING_GROUP_ARM = "handler"
8 PLANNING_NS = "/ur10_1/"
9 REFERENCE_FRAME = "/ur10_1/world"
10
11 ## Instantiate a RobotCommander object. This object is an interface to
12 ## the robot as a whole.
13 robot = moveit_commander.RobotCommander(
14 " %s robot_description"%PLANNING_NS,
15 ns="/ur10_1/"
16)
17
18 arm = moveit_commander.move_group.MoveGroupCommander(
19 PLANNING_GROUP_ARM,
20 " %s robot_description"%PLANNING_NS,
21 ns="/ur10_1/"
22)
23
24 gripper = moveit_commander.move_group.MoveGroupCommander(
25 PLANNING_GROUP_GRIPPER,
26 " %s robot_description"%PLANNING_NS,
27 ns="/ur10_1/"
28)
29
30 ## We create this DisplayTrajectory publisher which is used below to publish
31 ## trajectories for RVIZ to visualize.
32 display_trajectory_publisher = rospy.Publisher(
33 '/move_group/display_planned_path',
34 moveit_msgs.msg.DisplayTrajectory,
35 queue_size=10
36)
37
38 rospy.sleep(2)
39
40 arm.set_num_planning_attempts(15)
41 arm.set_planning_time(5)
42 arm.allow_looking(True)
43 arm.allow_replanning(True)
44 arm.set_pose_reference_frame(REFERENCE_FRAME)
45 arm.set_goal_position_tolerance(0.001)
46 arm.set_goal_orientation_tolerance(0.001)
47
48 gripper.set_num_planning_attempts(15)
49 gripper.allow_replanning(True)
50 gripper.allow_looking(True)
51
52 pick_place(arm, gripper)
53
54 ## When finished shut down moveit_commander.
55 moveit_commander.roscpp_shutdown()
``` 

The configuration was defined in the function 'main', but it is not necessary to have it here. It is better to modulate it and pass the configuration via parameters, but it is sufficient for this explanation. 

- **Line 2:** The first thing you need to do is initialize 'moveit_commander'. This is an API for the interface developed in 'C++', which is defined as a *wrapper* and provides most of the functionality of the interface of the 'C++' version, but not all of the functionality of *'MoveIt! ' It is necessary because, among other things, it allows the calculation of Cartesian trajectories, which is the functionality that is mainly needed.
- **Line 3:** Initialize the node named 'ur10_1_arm' moveit.
- **Lines 6-9:** The constants are defined to facilitate configuration. The first two lines (6 and 7) define the names given to the planning groups, in this case they were named 'gripper' for the gripper and 'manipulator' for the UR10 cobot arm. Lines 8 and 9 define the information needed to configure the planner. 'PLANNING _NS' contains the name of the *namespace* containing the node 'move_group' you want to communicate with, and 'REFERENCE _FRAME' is the link used as reference for calculating the trajectory with respect to the *end-effector* ('ee_link'), in this case it would be the same as '/ur10_1/world', but correct would be to take the link '/ur10_1/base_link' as reference.
- **Line 13:** Initializes the 'RobotCommander', which is used to control the robot, as specified in the code. You need to pass the *namespace* as a parameter and which 'robot_description' to use to define the robot, because at the moment there are three descriptions, namely the one for 'Gazebo' ('robot_description') and the other two defined in the 'planning_context.launch' which are instantiated in their respective *namespaces*, so we have '/ur10_1/robot_description ' and '/ur10_2/robot_description'.
- **Lines 18 and 24:** An interface is created for a group of joints, in this case the 'arm' interface for the group of dynamic 'joints' of the UR10 cobot arm, and the 'gripper' interface for the joint that controls the gripper. It is through these interfaces that the planning of the trajectories and their execution is done (it is possible to do it through the robot interface as it contains these two interfaces, but it is clearer and more convenient to do it this way.
- **Line 32:** As it says in the comment, 'display_trajectory_publisher' publishes in the *topic* '/move_group/display_planned_path' that *Rviz* subscribes to in order to display trajectories, it is not necessary, but for debugging it is recommended.
- **Line 38:** Just wait two seconds, make sure that the previous instances have been correctly loaded into the system, you must take into account that some of them instantiate nodes and if the computer you start on is slow, you can cause an undesirable situation.
- **Lines 40-50:** Here you configure some options of the scheduler to use, by default it is RTT but this can be changed. The most important options are the last ones in lines 44, 45 and 46. Line 44 defines the reference link that will be used to perform the scheduling, lines 45 and 45 define the acceptable margin of error of the result obtained by the scheduler for position and orientation. You have to be careful because the smaller the error you define, the longer the planner will take to give an answer. The same procedure applies to the 'grab' interface.
- **Line 52:** Here is the *pick & place.
- **Line 55:** Once the task is finished, *Pick & Place* ends. 

After the detailed description of the code, it is easy for the script to control another robot that is in a different *namespace* to change the value of the variable 'PLANNING _NS' to the name of the *namespace* where the target robot is defined. If you run *Pick & Place* with both arms, you can see that they perform the task at the same time, with a small delay because one starts a little later than the other. You can run tests by starting the scripts at the wrong time and checking that they do indeed move at the same time .

##### :computer: Creation of the scripts that will carry out the *Pick &amp; place*
```bash
cd scripts
touch two_arm_moveit_1.py
touch two_arm_moveit_2.py
``` 

- File [~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_moveit/two_arm_moveit_manipulator/scripts/two_arm_moveit_1.py](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/ two_arm_moveit/two_arm_moveit_manipulator/scripts/two_arm_moveit_1.py) 

- File [~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_moveit/two_arm_moveit_manipulator/scripts/two_arm_moveit_2.py](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/ two_arm_moveit/two_arm_moveit_manipulator/scripts/two_arm_moveit_2.py) 


<a name="pruebas2">
<h2>
Execution of the tests
</h2>
</a>



To run the test:
-Terminal 1
```bash
roslaunch two_arm_moveit_manipulator two_arm_moveit_gazebo.launch
``` 

-Terminal 2
```bash
roslaunch two_arm_moveit_manipulator two_arm_moveit_execution.launch
``` 

-Terminal 3
```bash
rosrun two_arm_moveit_manipulator two_arm_moveit_1.py
``` 

-Terminal 4
```bash
rosrun two_arm_moveit_manipulator two_arm_moveit_2.py
``` 

#### :book: End result information 

- Visual result in `Gazebo`
![image](/doc/imgs_md/two-arm-moveit-gazebo.png "Result in Gazebo") 

- Scheme of the nodes and *topic*s of the system
![image](/doc/imgs_md/two-arm-moveit-graph.png "System nodes and topics") 

It checks if the communication between 'Gazebo' and the replicas of 'MoveIt!' works correctly. You can see two big groups where each cobot communicates with its assigned 'move_group'. The communication with the controllers and the path of the transformations and values of the 'joins' has a single origin, namely the 'Gazebo' node ', this is very important to avoid strange movements, depending on the frequency with which these disturbances occur. 

The path part of the 'gazebo' node is the only one that can be found in the *topic*s
'/joint_states' of both *namespaces*, the 'move_group' node is also subscribed to this *topic*, then it reaches the 'robot_state_publisher' node which performs the transformations and sends them through the *topic* '/tf' to which the 'move_group' node is also subscribed, this is important because 'move_group' uses the information coming from both for path planning.

- Tree of transformations of the robot model
![image](/doc/imgs_md/two-arm-moveit-tree.png "Moveit Tree") 

--- 

<p align="left">
<button name="button">
<a rel="license" href="https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/moveit-intro.md"> Previous </a>
</button>
</p>
