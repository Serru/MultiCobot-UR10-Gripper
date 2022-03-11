# Installation of *ROS* packages

[Español](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/setup-doc/ESP/install-ros-packages.md) | **English**

## Create working directory 

```bash
mkdir -p ~/ MultiCobot-UR10-Gripper/src
cd ~/MultiCobot-UR10-Gripper/src
``` 

## Warning: *ROS* Third-party packages.

No direct changes were made to the contents of *ROS* packages during the installation process. Any changes made to these packages were to accommodate *ROS Kinetic* during compilation. 

- The licenses of each installed package have been respected. If there is a licensing issue due to an update, change, or other situation, please contact the authors to arrange for the package to be changed or removed from this repository.

## Installation of third-party packages
Installation of third-party packages to be used in whole or in part. 

### [Universal Robots](https://github.com/ros-industrial/universal_robot)

This package of *Universal Robots* serves as a base for the necessary changes that the project requires. The package provides modeling of branded robots, such as *UR3*, *UR5*, *UR10*, etc., as well as examples for use with other packages (`MoveIt!`) and tools (`Gazebo`). The modified files are detailed accordingly.

Clone the repository:

```bash
cd ~/MultiCobot-UR10-Gripper/src
git clone -b kinetic-devel https://github.com/ros-industrial/universal_robot.git
``` 

Install all missing dependencies for *ROS Kinetic Kame*:
```bash
cd ~/MultiCobot-UR10-Gripper
rosdep update
rosdep install --rosdistro kinetic --ignore-src --from-paths src
``` 

Compile:
```bash
cd ~/MultiCobot-UR10-Gripper
catkin_make
``` 

### [Robotiq_2finger_grippers](https://github.com/Danfoa/robotiq_2finger_grippers)

This *Robotiq* repository is used to add the *gripper* that the robot will use and the necessary drivers for its operation. This package is intended to communicate with *the physical robot gripper*, which is connected to the robot via *USB*. The functions of this package include communicating with the gripper via USB and providing a communication interface with *ROS*.

Clone the repository:
```bash
cd ~/MultiCobot-UR10-Gripper/src
git clone https://github.com/Danfoa/robotiq_2finger_grippers.git
``` 

Install all missing dependencies for *ROS Kinetic Kame*:
```bash
cd ~/MultiCobot-UR10-Gripper
rosdep update
rosdep install --rosdistro kinetic --ignore-src --from-paths src
``` 

Compile:
```bash
cd ~/MultiCobot-UR10-Gripper
catkin_make
``` 

### [Robotiq_85_gripper](https://github.com/PickNikRobotics/robotiq_85_gripper)

This *Robotiq* repository is used to add the *gripper* to be used to the robot, as well as the drivers required for its operation. Similar to the previous package, but focusing on simulating `Gazebo` grippers.

Clone the repository:
```bash
cd ~/MultiCobot-UR10-Gripper/src
git clone -b develop https://github.com/PickNikRobotics/robotiq_85_gripper.git
``` 

Install all missing dependencies for *ROS Kinetic Kame*:
```bash
cd ~/MultiCobot-UR10-Gripper
rosdep update
rosdep install --rosdistro kinetic --ignore-src --from-paths src
```

### [ros_control](https://github.com/ros-controls/ros_control)

This repository contains: "*A set of packages that include controller interfaces, controller managers, transmissions and hardware_interfaces*".

Clone the repository:
```bash
cd ~/MultiCobot-UR10-Gripper/src
git clone -b kinetic-devel https://github.com/ros-controls/ros_control.git
``` 

Install all missing dependencies for *ROS Kinetic Kame*:
```bash
cd ~/MultiCobot-UR10-Gripper
rosdep update
rosdep install --rosdistro kinetic --ignore-src --from-paths src
``` 

Compile:
```bash
cd ~/MultiCobot-UR10-Gripper
catkin_make
``` 


### [ur_modern_driver](https://github.com/ros-industrial/ur_modern_driver)

This repository is deprecated, but you must use it for compatibility with *ROS Kinetic Kame*. Contains drivers for *Universal Robots* robots (UR3/UR5/UR10) and is compatible with *ros_control*.

Clone the repository:
```bash
cd ~/MultiCobot-UR10-Gripper/src
git clone -b kinetic-devel https://github.com/ros-industrial/ur_modern_driver.git
``` 

Install all missing dependencies for *ROS Kinetic Kame*:
```bash
cd ~/MultiCobot-UR10-Gripper
rosdep update
rosdep install --rosdistro kinetic --ignore-src --from-paths src
``` 

Compile:
```bash
cd ~/MultiCobot-UR10-Gripper
catkin_make
``` 

A compilation error may occur with *ROS Kinetic Kame*. To fix this, you must modify the *ur_hardware_interface.cpp* file. Replace the contents of the file with the contents located [here](https://github.com/iron-ox/ur_modern_driver/blob/883070d0b6c0c32b78bb1ca7155b8f3a1ead416c/src/ur_hardware_interface.cpp) and recompile it.

### [gazebo-pkgs](https://github.com/JenniferBuehler/gazebo-pkgs)
This is a collection of plugins for `Gazebo`. We are mostly interested in the plugin that allows grabbing objects.


Package dependencies:
- **gazebo_ros**
- **eigen_conversions**
- **object_recognition_msgs**
- **roslint**
- **general-message-pkgs** 

Installation of package dependencies:

ROS uses **Gazebo 7.x**

```bash
sudo apt-get install -y libgazebo7-dev
```

Clone the repository of **[gazebo_ros_pkgs](https://github.com/ros-simulation/gazebo_ros_pkgs)**

```bash
cd ~/MultiCobot-UR10-Gripper/src
git clone -b kinetic-devel https://github.com/ros-simulation/gazebo_ros_pkgs.git
``` 

Clone the repository from **[eigen_conversions](https://github.com/ros/geometry)**

```bash
cd ~/MultiCobot-UR10-Gripper/src
git clone -b indigo-devel https://github.com/ros/geometry.git
``` 

Clone repository of **[object_recognition_msgs](https://github.com/wg-perception/object_recognition_msgs)**

```bash
cd ~/MultiCobot-UR10-Gripper/src
git clone -b master https://github.com/wg-perception/object_recognition_msgs.git
``` 

Clone the **[roslint](https://github.com/ros/roslint)** repository

```bash
cd ~/MultiCobot-UR10-Gripper/src
git clone -b master https://github.com/ros/roslint.git
``` 

Clone the repository of **[general-message-pkgs](https://github.com/JenniferBuehler/general-message-pkgs)**

```bash
cd ~/MultiCobot-UR10-Gripper/src
git clone -b master https://github.com/JenniferBuehler/general-message-pkgs.git
``` 

Install all missing dependencies for *ROS Kinetic Kame*:

```bash
cd ~/MultiCobot-UR10-Gripper
rosdep update
rosdep check --from-paths . --ignore-src --rosdistro kinetic
rosdep install --from-paths . --ignore-src --rosdistro kinetic -y
``` 

Compile:

```bash
cd ~/MultiCobot-UR10-Gripper
catkin_make
``` 

---

Clone repository **[gazebo-pkgs](https://github.com/JenniferBuehler/gazebo-pkgs)**:

```bash
cd ~/MultiCobot-UR10-Gripper/src
git clone https://github.com/JenniferBuehler/gazebo-pkgs.git
``` 

Install all missing dependencies for *ROS Kinetic Kame*:

```bash
cd ~/MultiCobot-UR10-Gripper
rosdep update
rosdep install --rosdistro kinetic --ignore-src --from-paths src
``` 

Compile:

```bash
cd ~/MultiCobot-UR10-Gripper
catkin_make
``` 

---

During compilation you may get an **error message** asking you to compile to the C++11 standard:

```bash
/usr/include/c++/5/bits/c++0x_warning.h:32:2: error: #error This file requires compiler and library support for the ISO C++ 2011 standard. This support must be enabled with the -std=c++11 or -std=gnu++11 compiler options.
``` 

To fix this, you need to change the `make` of *catkin* when building the project:

- Simply modify the `~/MultiCobot-UR10-Gripper/src/CMakeLists.txt` file and add *add_compile_options(-std=c++11)* to the beginning of the file as shown below.

```bash
# toplevel CMakeLists.txt for a catkin workspace
# catkin/cmake/toplevel.cmake 

cmake_minimum_required(VERSION 3.0.2) 

project(Project) 

set(CATKIN_TOPLEVEL TRUE)
add_compile_options(-std=c++11)
[...]
``` 

---

<div>
<p align="left">
<button name="button"><a rel="license" href="https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/setup-doc/proyect_setup_eng.md"> Menu </a></button>
</p>



<p><span style="float:left;">
<button name="button">
<a rel="license" href="https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/setup-doc/ENG/install-ROS.md"> Previous </a>
</button>
</span>
<span style="float:right;">
<button name="button">
<a rel="license" href="https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/setup-doc/ENG/leap-motion.md"> Next </a>
</button>
</span>
</p>
</div>