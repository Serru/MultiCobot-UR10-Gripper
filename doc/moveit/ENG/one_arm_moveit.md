# One UR10 with gripper using the `MoveIt!` package

[Español](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/moveit/ESP/one_arm_moveit.md) | **English**

![image](/doc/imgs_md/Diseno-moveit-general-un-cobot-leap-motion.png "Loaded the URDF model of the UR10 robot") 

## Prerequisite
- Successfully install the [Basic System Configuration](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/setup-doc/proyect_setup_eng.md). 
- Implement the [Solution for one robot without the `MoveIt!` motion planner](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/no_moveit/ENG/one_arm_no_moveit.md).

## Index
- [Phase 1: *URDF* configuration](#phase1)
- [Phase 2: Configuration of `MoveIt!`](#phase2)
- [Phase 3: Simulation of a `Pick & Place` in `Gazebo`](#phase3)
- [Execution of tests](#tests)


<a name="phase1">
<h2>
Phase 1: URDF configuration
</h2>
</a>

### :book: Description of the file *URDF*

The *URDF* (United Robotics Description Format) file models the cobot in *XML* format, which is used by the various applications that *ROS* requires, but mainly to perform a simulation of the modeled robot. 

The file is built in the form of a tree in which there are three main tags: `<robot>`, `<link>` and `<joint>`. For clarification, you can take the arm of the human body as a reference. If you want to model the arm of a person, the tag `<robot>` stands for the arm as a whole. This arm consists of several bones (humerus, ulna and radius) represented by the tags `<link>` and a joint connecting these bones (elbow) represented by the tag `<joint>`. 

As with bones, additional information can be added to these tags, providing information about size, geometry, inertia, orientation, etc. Finally, the modeling of a robot can be connected to another model to form a more complex model that can be represented by adding the hand to the arm, where the wrist is a joint that connects the two. Note that `<joint>` tags are connected to `<link>` tags via a parent-child relationship. 

In this way, a representation of the robot components is created: 

![image](/doc/imgs_md/urdf-robot.png "URDF file representation") 

The image shows the contents of the [*URDF* file](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_description/urdf/ur10_robot.urdf.xacro ) that models the robot next to the gripper. You can see how the component of the UR10 robot arm is connected to the link `world` which represents the `world` (yellow color) and the base of the UR10 arm `base_link` (green color) which is directly above it, also the joint `world_joint` is the yellow ball located between both links. In the same way, the component of the gripper `robotiq_85_gripper` is connected to the arm of the UR10 (`ur10_robot`). The ball representing the joint `robotiq_85_base_joint` connecting both components (purple color) connects the link `robotiq_85_base_link` of the gripper with the link `ee_link` of the UR10 arm.


#### :computer: Create solution directory
```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot
mkdir one_arm_moveit
``` 

#### :computer: Description of the directory configuration
A new package is created and the *one_arm_no_moveit* project directories are copied for later editing. 

```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit
catkin_create_pkg one_arm_moveit_description rospy
```

The *one_arm_no_moveit_description* directory and the *launch* and *urdf* folders are copied to the directory created for *description*. 

```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit/one_arm_moveit_description
cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_description/launch .
cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_description/urdf .
```

#### :computer: Modification of the *description* files 

- [~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit/one_arm_moveit_description/launch/ur10_upload.launch](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_description/launch/ur10_upload.launch) 

- [~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit/one_arm_moveit_description/urdf/ur10_robot.urdf.xacro](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_description/urdf/ur10_robot.urdf.xacro) 

- [~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit/one_arm_moveit_description/urdf/ur10_joint_limited_robot.urdf.xacro](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_description/urdf/ur10_joint_limited_robot.urdf.xacro) 

It compiles:

```bash
cd ~/MultiCobot-UR10-Gripper
catkin_make
```


<a name="phase2">
<h2>
Phase 2: Configuring MoveIt!
</h2>
</a>

### Configuration of `MoveIt!` 

Before configuring with the `Setup Assitant`, you must already have the `URDF` well defined. Therefore the `Setup Assitant` configuration is started with the following command in the terminal:

```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit/
mkdir one_arm_moveit_config
roslaunch moveit_setup_assistant setup_assistant.launch
```

#### :computer: `MoveIt!` Setup Assistant
The *URDF* file is selected as the robot model: [~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit/one_arm_moveit_description/urdf/ur10_joint_limited_robot.urdf.xacro](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_description/urdf/ur10_joint_limited_robot.urdf.xacro) (could be perfectly be [ur10_robot.urdf.xacro](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_description/urdf/ur10_robot.urdf.xacro)). 

- When you start the `Setup Assitant` configuration, you must tell it where the file `UR10_joint_limited_robot.urdf.xacro` is located, the model of the robot you want to configure:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_1.png "Load UR10 robot URDF model"). 

- Then the *Load files* button is specified.
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_2.png "URDF model of UR10 robot loaded") 

- On the *Self Collisions* tab, click on the *Generate Collision Matrix* button. This will create a matrix between the various robot components that may cause a self-collision during trajectory planning:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_3.png "Generate collision matrix") 

- The *Virtual Joints* tab is specifically for robot arms installed on a mobile base. In this case it does not affect the configuration as the base is fixed, but for future configurations a joint must be created between the robot `base_link` and the `world` frame with the following configuration:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_4.png "Defining Virtual Joint")
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_5.png "Defining Virtual Joint") 

- One of the most important tabs is the definition of *planning groups*. In this case, there are two groups: the *manipulator* group, which controls the robot arm, and the *gripper* group, which controls the gripper:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_6.png "Manipulator kdl") 

- Then you need to click on the *Add Kin. Chain* button:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_7.png "Manipulator Kinetic Chain setup") 

- Finally the configuration is saved:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_8.png "Manipulator Group setup") 

- Now you need to do this for the *gripper* group that will control the gripper. Click the *Add Group* button, enter the name of the group, and enter *kdl* as *Kinematic Solver*:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_9.png "gripper kdl") 

- After clicking the *Add joints* button, you need to search for *robotiq_85_left_knucle_joint* and add it with the arrow *->*:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_10.png "grab joint setup") 

- After saving, the result on the *Planning group* tab should look like this:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_11.png "Planning Group Final Setup") 

- The *Robot Poses* tab is used to configure fixed poses in the robot in advance. The *home* pose is configured to reflect the initial position of the robot:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_12.png "Setup *Home* 1/3")
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_13.png "Setup *Home* 2/3")
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_14.png "Setup *Home* 3/3") 

- On the *Robot Poses* tab, the *gripper_open* pose is configured:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_24.png "Setup *Open_gripper*") 

- The *gripper_close* pose is configured on the *robot poses* tab:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_25.png "Set up *close_gripper*") 

- On the *End Effectors* tab, the gripper is added:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_15.png "Set up end effector 1/3")
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_16.png "Set up end effector 2/3")
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_17.png "Set up end effector 3/3") 

- The *Passive Joints* tab contains the joints that cannot be actively moved. In the case of the *gripper* defined in your *URDF* file, these are the `joints` that mimic the movement of another joint. The `joints` defined as passive are not considered in the design, in which case the configuration is as follows:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_18.png "Set up passive joints"). 

- On the *ROS control* tab it will be added automatically, the created files will be changed manually later.
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_19.png " ROS control setup")
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_20.png "Setting up the ROS controller") 

- To complete the configuration, you need to fill in the *Author Information* tab:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_21.png "Set up author information"). 

- On the last tab *Configuration Files* you can specify where the `MoveIt!` configuration should be stored, in this case in the previously created file `one_arm_moveit_config`. The configuration is created using the *Generate Package* button and finally.
*Exit Setup Assitant* to exit the programme:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_22.png "Set author information")
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_23.png "Set author information")


<a name="phase3">
<h2>
Phase 3: Simulation of a <i>pick & place</i> in Gazebo
</h2>
</a>

This phase is divided into two stages 

#### :book: Connection between `Gazebo` and `MoveIt!`

This stage is first about configuring `Gazebo` and the controllers to correctly simulate the robot's movements. The package *one_arm_moveit_gazebo* is created, which contains all the configuration of `Gazebo`, including the controllers. Once the package is created, you need to configure the controllers, which are located in the *controller* directory. Although all controllers can be defined in a single file, we have spread them over three files for clarity. 

The controllers are defined in files with the extension *yaml*. To define these controllers, you must give them a name and specify the type of controller, the dynamic `joints` you want to control, the constraints it has, the publishing rate, and other options. 

These controllers are briefly explained below: 

- [arm_controller_ur10.yaml](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_gazebo/controller/arm_controller_ur10.yaml) file: This file defines the controller for the UR10 cobot. It defines the controller name `arm_controller`, the controller type position `controllers/JointTrajectoryController`, which requires the definition of the message type and the correct formatting of the information needed to communicate with the controller. Then there is the `joints` field, which specifies which `joints` of the robot are part of the controller, all of these `joints` being dynamic. The other fields are left untouched, but consistency must be maintained in their naming. 

- File [gripper_controller_robotiq.yaml](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_gazebo/controller/gripper_controller_robotiq.yaml): This file defines the controller for the *Robotiq* gripper. Here you define the controller name `gripper`, the controller type `position controllers/JointTrajectoryController` which defines the type of messages and the information needed to communicate with it. Then there is the `joints` field, which specifies which `joints` of the robot are part of the controller, in this case a single `joint_robotiq_85_left_knuckle_joint`, since the other `joints` of the controller mimic the movements of this one. The other fields remain untouched, but naming consistency must be maintained as in the previous case. 

- File [joint_state_controller.yaml](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_gazebo/controller/joint_state_controller.yaml): This file is not a controller in the strict sense. Its function is that of an interface that translates the `joint` information coming from the real cobot and converts it into messages of type `JointState` that will be published later. It is essential for correct operation both in simulation and with the real robot. It is part of the package *ROS* *ros_control*. 

#### :computer: Starting `Gazebo`

We create the package for `Gazebo` and copy the contents of the solution from *one_arm_no_moveit* for later changes:

```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit
catkin_create_pkg one_arm_moveit_gazebo rospy
```

In the directory created for `Gazebo` it copies the directory *one_arm_no_moveit_gazebo*, the folders *controller*, *launch*, *models* and *world*.

```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit/one_arm_moveit_gazebo
cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/controller .
cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/launch .
cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/models .
cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/world .
```

#### :computer: Modification of `Gazebo` files
The files are created with the following content: 

- [~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit/one_arm_moveit_gazebo/launch/ur10_joint_limited.launch](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_gazebo/launch/ur10_joint_limited.launch) 

- [~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit/one_arm_moveit_gazebo/launch/ur10.launch](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_gazebo/launch/ur10.launch) 

It compiles:

```bash
cd ~/MultiCobot-UR10-Gripper
catkin_make
```

If you leave the configuration as it is, communication between `Gazebo` and `MoveIt!` is not possible. If you start *demo.launch*, it will also start *Rviz*, which you can use to visualize the data. As you can see it works correctly, but with the automatic configuration it is not possible to observe the operation in the `Gazebo` simulator. 

This is because there is no communication between `MoveIt!` and `Gazebo`, so this communication is solved. 

For simplicity, we will keep the start of `Gazebo` and `MoveIt!` separate for possible evaluation in the real robot. 

To make it easier to get started and to better understand the `MoveIt!` package, we separate the contents of the packages into `MoveIt!` and `Gazebo`:

`Gazebo`:
- launch:
	- gazebo.launch
	- ros_controllers.launch
- config:
	- **ros_controllers.yaml** 

The *gazebo.launch* file provided by `MoveIt!` is not required, as the `Gazebo` package to be used has already been configured. The configuration is very similar if you take a look at the files involved.

`MoveIt!`:
- launch:
  - **demo.launch**
    - planning_context.launch
      - config:
        - ur10.srdf
        - joint_limits.yaml
        - kinematics.yaml
    - **move_group.launch**
      - planning_context.launch
      - config:
        - ur10.srdf
        - joint_limits.yaml
        - kinematics.yaml
      - planning_pipeline.launch.xml
        - ompl_planning_pipeline.launch.xml
      - **trajectory_execution.launch.xml**
        - **ur10_moveit_controller_manager.launch.xml**
        - config:
          - **ros_controllers.yaml**
      - sensor_manager.launch.xml
        - config:
          - sensors_3d.yaml
        - ur10_moveit_sensor_manager.launch.xml
      - moveit_rviz.launch (Rviz)
        - moveit.rviz
        - config:
          - kinematics.yaml



In the directory diagram you can see how the files are connected. The files that have to be changed manually are printed in bold. 

The configuration file for the controllers is the same for `Gazebo` and `MoveIt!` This means that the configuration of the controllers must match the one previously created for `Gazebo` and configured in the [Starting `Gazebo`](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/no_moveit/ENG/one_arm_no_moveit.md) section in order for the controllers to connect properly. 

You can therefore add the `Gazebo` controllers (.yaml files) to the `MoveIt!` package to connect the robot controllers (manipulator and gripper) correctly. 

If you start `Gazebo` and `MoveIt!` without making any changes, you will get the following representation of nodes and *topic*s: 

- Terminal 1
```bash
roslaunch one_arm_moveit_config demo.launch
```

- Terminal 2
```bash
roslaunch one_arm_moveit_gazebo ur10_joint_limited.launch
```

![ ](/doc/imgs_md/one_arm_moveit_graph_no_changes.png "Schematic without changes") 

You can see that `gazebo` correctly loads the controllers `arm_controller` and `gripper`, but there is no communication between the node `move_group` and the controllers of the node `gazebo`, the only common point is the *topic* `/joint_states `. This means that trajectories planned and executed with the `MoveIt!` plugin *Motion Planning* will not be rendered in `gazebo`, but only in *Rviz*. Since this is not the desired result, we need to change the configuration of `MoveIt!` so that it can communicate with the controllers loaded in `gazebo`. 

For this purpose the following files have to be changed. The modified files are in a different package to facilitate compression of the changes made to the configuration. 

For this purpose, the following files are modified based on their original files *demo.launch*, *move_group.launch*, *trajectory_execution.launch.xml* and *ur10_moveit_controller_manager.launch.xml* and the controllers are added by creating two files *controllers.yaml* and *joint_names.yaml*: 

```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit
catkin_create_pkg one_arm_moveit_manipulator rospy
mkdir config
touch config/controllers.yaml
touch config/joint_names.yaml
mkdir launch
touch launch/one_arm_moveit_execution.launch
touch launch/move_group.launch
touch launch/trajectory_execution.launch.xml
touch launch/ur10_moveit_controller_manager.launch.xml
mkdir scripts
```

#### :computer: Adding the controllers to interact with `Gazebo`

- File [~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit/one_arm_moveit_manipulator/config/controllers.yaml](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_manipulator/config/controllers.yaml): The definition of the *ROS* controllers in `MoveIt!` is similar to that for `Gazebo`, the name of the controllers must match the names of the controllers described in `Gazebo`, the action server `follow_joint_trajectory` is defined, the type must be `FollowJointTrajectory` so that the message types sent between them are compatible and finally the names of the `joins` involved must be identical. 

- Another file to modify is [~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit/one_arm_moveit_manipulator/config/joint_names.yaml](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_manipulator/config/joint_names.yaml): This file defines the name of the `joints` of the cobot controller. It is saved as a server parameter and used as part of the `MoveIt!` configuration. 

From the files in the *launch* directory we will modify the *one_arm_moveit_execution* file, which is the entry point for using the `MoveIt!` package and *Rviz*, which is based on the *demo.launch* file:

- File [~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit/one_arm_moveit_manipulator/launch/one_arm_moveit_execution.launch](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_manipulator/launch/one_arm_moveit_execution.launch).

Then optimise the controller launcher and `MoveIt!`:

- File [~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit/one_arm_moveit_manipulator/launch/move_group.launch](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_manipulator/launch/move_group.launch).

- File [~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit/one_arm_moveit_manipulator/launch/trajectory_execution.launch.xml](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_manipulator/launch/trajectory_execution.launch.xml): set up communication with a real robot. In this case, `Gazebo` is the one simulating the robot, but `MoveIt!` is not aware of it and treats it like a real robot. 

- File [~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit/one_arm_moveit_manipulator/launch/ur10_moveit_controller_manager.launch.xml](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_manipulator/launch/ur10_moveit_controller_manager.launch.xml): Loading controllers designed for `MoveIt!`. 

#### :computer: Finally, a test is performed: 

```bash
cd ~/MultiCobot-UR10-Gripper
catkin_make
```

- Terminal 1
```bash
roslaunch one_arm_moveit_gazebo ur10_joint_limited.launch
```

- Terminal 2
```bash
roslaunch one_arm_moveit_manipulator one_arm_moveit_execution.launch
```

![ ](/doc/imgs_md/one_arm_moveit_26.png "example gazebo+rviz+moveit! (1/2)") 

![ ](/doc/imgs_md/one_arm_moveit_27.png "example gazebo+rviz+moveit! (2/2)") 

And the graph of nodes and *topic*s, after the changes, you can see how now the node *move_group* communicates with the controllers and it is *Gazebo* that listens to what is published to perform the desired movements. These movements change the current state of the robot, which is published in the *topic* `/jont_states`, and this information is passed to the `robot_state_publisher` node and the `move_group` node. The `move_group` node can recalculate a new path with the information it receives from the *topic*s `/tf` and `/jont_states`. 

![ ](/doc/imgs_md/one_arm_moveit_graph_changes.png "rqt_graph representation of nodes and topics") 

#### :computer: Pick and Place 

To script *pick & place* in Python, the Python interface `moveit_commander` is used to communicate with the node `move_group` and its services and actions. Since it is not very problematic for controlling a single robot and there is good documentation, the script is described in detail for the solution with two or more cobots. 

Very simple tests are performed. To do this, you must first create the necessary scripts to correctly control the robot arm and gripper. Then the movements are performed so that the robot picks up a cube from the table and puts it into the basket. 

```bash
cd scripts
touch one_arm_moveit.py
```

File [~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit/one_arm_moveit_manipulator/scripts/one_arm_moveit.py](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_manipulator/scripts/one_arm_moveit.py)

<a name="tests">
<h2>
Execution of the tests
</h2>
</a>

- Terminal 1
```bash
roslaunch one_arm_moveit_gazebo ur10_joint_limited.launch
```

- Terminal 2
```bash
roslaunch one_arm_moveit_manipulator one_arm_moveit_execution.launch
```

- Terminal 3
```bash
rosrun one_arm_moveit_manipulator one_arm_moveit.py 
```

#### :book: Final results information 

- Visual result in `Gazebo`
![image](/doc/imgs_md/one-arm-moveit-gazebo.png "Result in Gazebo") 

- Scheme of the nodes and *topic*s of the system
![image](/doc/imgs_md/one-arm-moveit-graph.png "System nodes and topics") 

- Tree of transformations of the robot model
![image](/doc/imgs_md/one-arm-moveit-tree.png "Transform Tree") 

--- 

<div>
<p align="left">
<button name="button">
<a rel="license" href="https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/moveit-intro-eng.md"> Back </a>
</button>
</p>
</div>