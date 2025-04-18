<p align="center">

<img alt="MultiCobot-UR10-Gripper" style="border-width:0" src="https://raw.githubusercontent.com/Serru/MultiCobot-UR10-Gripper/main/doc/imgs_md/proyect-logo.png" />
</p>

# MultiCobot-UR10-Gripper

<p align="center">
Multi-robot system of collaborative robots (cobots) <a rel="UR10s" href="https://www.universal-robots.com/products/ur10-robot/">UR10s</a> with grippers from Robotiq (<a rel="robotiq_85_gripper" href="https://robotiq.com/products/2f85-140-adaptive-robot-gripper">robotiq_85_gripper</a>) that allows simultaneous execution of tasks with different types of controllers and brands of cobots, as well as direct control of the cobot by a person via the Leap Motion device.
</p>

<p align="center">
  <img alt="Doc" style="border-width:0" src="https://img.shields.io/badge/doc-complete-green?logo=markdown&style=plastic" />
  <a rel="Build Status" href="https://app.travis-ci.com/github/Serru/MultiCobot-UR10-Gripper"><img alt="Build Status" style="border-width:0" src="https://img.shields.io/travis/com/Serru/MultiCobot-UR10-Gripper?logo=travis&style=plastic" />
    </a>
  <a rel="license" href="http://creativecommons.org/licenses/by/4.0/"><img alt="Creative Commons License" style="border-width:0" src="https://img.shields.io/github/license/Serru/MultiCobot-UR10-Gripper?style=plastic" />
  </a>
      <a rel="Python" href="https://github.com/Serru/MultiCobot-UR10-Gripper">
  <img alt="Python 2.7" style="border-width:0" src="https://img.shields.io/badge/Python-14354C?logo=python&logoColor=white&style=plastic" />
  </a>
    <a rel="C++" href="https://github.com/Serru/MultiCobot-UR10-Gripper">
  <img alt="C++" style="border-width:0" src="https://img.shields.io/badge/C%2B%2B-00599C?logo=c%2B%2B&style=plastic" />
  </a>
  <a rel="stars" href="https://github.com/Serru/MultiCobot-UR10-Gripper/stargazers"><img alt="Stars of the repository" style="border-width:0" src="https://img.shields.io/github/stars/Serru/MultiCobot-UR10-Gripper?style=plastic" />
  </a> 
    <a rel="Repository Size" href="https://github.com/Serru/MultiCobot-UR10-Gripper"><img alt="Repository Size" style="border-width:0" src="https://img.shields.io/github/repo-size/Serru/MultiCobot-UR10-Gripper?style=plastic" />
  </a>
  <a rel="ubuntu16.04" href="https://releases.ubuntu.com/16.04/"><img alt="Ubuntu 16.04" style="border-width:0" src="https://img.shields.io/badge/OS-ubuntu%2016.04-important?style=plastic&logo=ubuntu" />
  </a>
  <a rel="ros" href="http://wiki.ros.org/kinetic"><img alt="ROS Kinetic Kame" style="border-width:0" src="https://img.shields.io/badge/ROS-Kinetic%20Kame-important?style=plastic&logo=ros" />
  </a>
</p>

[Español](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/README_ESP.md) | **English**

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

Terminal 1:
```bash
roslaunch two_arm_no_moveit_gazebo ur10_joint_limited.launch
``` 

Terminal 2:
```bash
rosrun two_arm_no_moveit_manipulator ur10_1_robot_manipulator.py
``` 

Terminal 3:
```bash
rosrun two_arm_no_moveit_manipulator ur10_2_robot_manipulator.py
```

## Documentation
- [Basic System Configuration](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/setup-doc/proyect_setup_eng.md)

### Configuration in the `Gazebo` simulator
- [Design and development for one, two and four robots](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/design-eng.md)
- [Design and integration of the Leap Motion device into the system to control one and two robots](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/design-lm-eng.md)

### Configuration of the physical robot
- [Configuration for a UR10 with gripper using its motion planning on the physical robot](https://github.com/Serru/MultiCobot-UR10-Gripper-Campero)

## Video with the results
Here is a video with the results of the simulations performed in `Gazebo`. The video shows two and four robots performing a *pick & place* without human intervention. Then it shows how a person controls two cobots with Leap Motion, and finally the developed result was tested with the physical Campero robot.

<p>
<a href="https://drive.google.com/file/d/1tSQhoj_FoAtpLjpX_puJgrlwuJCzKOLm/view?usp=sharing" title="Link Title">
	<img src="https://raw.githubusercontent.com/Serru/MultiCobot-UR10-Gripper/main/doc/imgs_md/img-fondo-video.png" alt="Results of the project." />
</a>
</p>


## Help & Support
Unfortunately, this repository is not actively maintained. The main goal is to publish what has been learned for the community who might need the knowledge and content of this repository to develop their project or research.

A response is not guaranteed, but you can contact the authors using the information in the [Authors](#autores) section.

## License

<p align="left">
  <a href="https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/LICENSE">
    <img src="https://licensebuttons.net/l/by/4.0/88x31.png" alt="This repository is published under the Creative Commons Attribution 4.0 International license." />
  </a>
  </br>
  </br>
This repository is published under the <a href="https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/LICENSE">Creative Commons Attribution 4.0 International</a> license.
</p>

## Authors
- [Burgh Oliván, Miguel](https://github.com/Serru) - *Author of the Final Degree Project entitled **Multirobot system for the collaborative transport of objects**.*
- [López Nicolás, Gonzalo](https://i3a.unizar.es/es/investigadores/gonzalo-lopez-nicolas) - *Director of the Final Degree Project entitled **Multirobot system for the collaborative transport of objects**.*

The memory of the Final Degree Project can be found in the [Repository](https://deposita.unizar.es/record/66296?ln=es) of TFGs of the [University of Zaragoza](http://www.unizar.es/).

## Acknowledgement

This work is part of the [RoPeRT](https://i3a.unizar.es/es/grupos-de-investigacion/ropert) research group of the [i3A](https://i3a.unizar.es), the [University of Zaragoza](http://www.unizar.es/).

![image](https://i3a.unizar.es/sites/default/files/logo_i3a.jpg)

---

The developed work has been experimentally evaluated and validated, showing a correct operation of the physical robot [Campero](http://commandia.unizar.es/wp-content/uploads/camperoRobot.jpg). For this reason, this work is part of the activities of the project [COMMANDIA (2019)](http://commandia.unizar.es/), co-funded by the [Interreg Sudoe Program](https://www.interreg-sudoe.eu/inicio) and the [European Regional Development Fund (ERDF)](https://ec.europa.eu/regional_policy/es/funding/erdf/).

![image](http://commandia.unizar.es/wp-content/uploads/cropped-logoCommandia-1.png)

## Recognition

Please cite this work if the content of this repository has been useful to you:

BibTeX: 
```
@InProceedings{10.1007/978-3-031-21065-5_34,
author="Burgh-Oliv{\'a}n, Miguel
and Arag{\"u}{\'e}s, Rosario
and L{\'o}pez-Nicol{\'a}s, Gonzalo",
editor="Tardioli, Danilo
and Matell{\'a}n, Vicente
and Heredia, Guillermo
and Silva, Manuel F.
and Marques, Lino",
title="ROS-Based Multirobot System for Collaborative Interaction",
booktitle="ROBOT2022: Fifth Iberian Robotics Conference",
year="2023",
publisher="Springer International Publishing",
address="Cham",
pages="411--422",
abstract="This paper presents the design and implementation of a collaborative multi-robot system based on ROS. The goal is to manipulate objects with multiple cobots simultaneously by following commands given by a user via gestures. Two methods have been designed, developed, implemented and experimentally evaluated: The first one is based on the ROS package called MoveIt! and focuses mainly on configuration to allow simultaneous control of different cobots. The second method involves the development of a third-party motion planner that sends commands directly to the controllers responsible for executing the cobots' movements. The Leap Motion, a device that can be used for gesture recognition, is also integrated into the system to enable user interaction in object manipulation. The system has been tested in simulation using Gazebo and evaluated in a real UR10 robot. The main contribution of the proposed architecture is that it solves the problem of controlling multiple robots simultaneously in ROS. In particular, our approach allows simultaneous execution of tasks with different types of controllers, brands and models, as well as direct control of the robots by using the Leap Motion device.",
isbn="978-3-031-21065-5"
}
```
