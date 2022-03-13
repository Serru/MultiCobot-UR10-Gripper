---
title: "Introduction"
permalink: /docs/introduction/
excerpt: "Multi-robot system of collaborative robots (cobots) UR10s with grippers from Robotiq (robotiq_85_gripper) that allows simultaneous execution of tasks with different types of controllers and brands of cobots, as well as direct control of the cobot by a person via the Leap Motion device.
"
redirect_from:
  - /theme-setup/
toc: true
---

<p align="center">

<img alt="MultiCobot-UR10-Gripper" style="border-width:0" src="https://raw.githubusercontent.com/Serru/MultiCobot-UR10-Gripper/main/doc/imgs_md/proyect-logo.png" />
</p>

# MultiCobot-UR10-Gripper

<p align="center">
Multi-robot system of collaborative robots (cobots) <a rel="UR10s" href="https://www.universal-robots.com/products/ur10-robot/">UR10s</a> with grippers from Robotiq (<a rel="robotiq_85_gripper" href="https://robotiq.com/products/2f85-140-adaptive-robot-gripper">robotiq_85_gripper</a>) that allows simultaneous execution of tasks with different types of controllers and brands of cobots, as well as direct control of the cobot by a person via the Leap Motion device.
</p>

## About this project
This thesis project focuses on developing a multi-robot system that can cooperatively perform tasks such as transporting objects. There is not much documentation on how to develop a system where multiple robots can be controlled simultaneously in the environment of ROS, which is widely used in research and prototyped for testing before going into production.

Two solutions have been designed, developed, implemented and experimentally evaluated: the first is the *ROS* package called `MoveIt!`, where the work is mainly focused on the configuration to allow the simultaneous control of different cobots; the second is the creation or use of a third-party planner that sends the commands directly to the controllers in charge of executing the movements of the cobots, and each of them has a built-in gripper that allows them to perform different tasks.

The Leap Motion device is also integrated into the system. It is capable of detecting, tracking and recognizing the user's hand gestures and serves as an interface for the simultaneous control of up to two cobots, enabling the manipulation of objects.

[Read more...](https://deposita.unizar.es/record/66296?ln=es)

## System requirements
- Ubuntu 16.04
- Python 2.7
- ROS Kinetic Kame

## Quick start

- Clone this repository:
```bash
git clone https://github.com/Serru/MultiCobot-UR10-Gripper
```

- Set the catkin Workspace:
```bash
cd ~/MultiCobot-UR10-Gripper/src
catkin_init_workspace
```

- Install all the dependecies:
```bash
source /opt/ros/kinetic/setup.bash
cd ~/MultiCobot-UR10-Gripper
rosdep update
rosdep install -r --rosdistro kinetic --ignore-src --from-paths src
```

- Build the repository
```bash
catkin_make
rospack profile
source devel/setup.bash
```
- Launch Gazebo with two robots doing a simple Pick & Place

   - Terminal 1:
```bash
roslaunch two_arm_no_moveit_gazebo ur10_joint_limited.launch
``` 

   - Terminal 2:
```bash
rosrun two_arm_no_moveit_manipulator ur10_1_robot_manipulator.py
``` 

   - Terminal 3:
```bash
rosrun two_arm_no_moveit_manipulator ur10_2_robot_manipulator.py
```

## Video with the results
Here is a video with the results of the simulations performed in `Gazebo`. The video shows two and four robots performing a *pick & place* without human intervention. Then it shows how a person controls two cobots with Leap Motion, and finally the developed result was tested with the physical Campero robot.

<p>
<a href="https://drive.google.com/file/d/1oqVyre4vlfHqH9SrQuyXH00GcmwIuP97/view?usp=sharing" title="Link Title">
   <img src="https://raw.githubusercontent.com/Serru/MultiCobot-UR10-Gripper/main/doc/imgs_md/img-fondo-video.png" alt="Results of the project." />
</a>
</p>


## Recognition

Please cite this work if the content of this repository has been useful to you:

BibTeX: 
```
@article{
    BurghOliván:66296,
    author = "Burgh Oliván, Miguel Yankan and López Nicolás, Gonzalo",
    title = "{Sistema multirobot para el transporte colaborativo de objetos}",
    year  = "2022",
}
```
