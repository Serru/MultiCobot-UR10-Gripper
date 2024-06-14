# Installation of *ROS Kinetic and dependencies* 

[Español](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/setup-doc/ESP/install-ROS.md)  | **English**

## Installation of *ROS Kinetic Kame*

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt-get update

sudo apt-get install ros-kinetic-desktop-full


sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo apt install libignition-math2-dev

sudo apt install python-rosdep
sudo rosdep init
rosdep update

echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
source /opt/ros/kinetic/setup.bash
``` 

## Installation of *ROS* dependencies 

```bash
sudo apt-get install ros-kinetic-$(dependency_name)
``` 

If the dependency is *moveit_ros_planning*, then *dependency_name* is *moveit-ros-planning*, this pattern is followed to install all dependencies, for the previous example it would be:

```bash
sudo apt-get install ros-kinetic-moveit-ros-planning
``` 

### Dependencies to be installed
- moveit_ros_planning
- moveit_kinematics
- moveit_simple_controller_manager
- position_controllers
- moveit_fake_controller_manager
- gazebo_ros_control
- joint_trajectory_controller
- moveit_ros_visualization
- moveit_planners_ompl
-joint_state_controller
- effort_controllers
- ros_controllers
- industrial_msgs 

*The dependencies may differ from those suggested in this guide. During the installation of the * ROS * tools and packages, the different dependencies that ROS requires will be displayed.*

 ---
<div>
<p align="left">
    <button name="button"><a rel="license" href="https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/setup-doc/proyect_setup_eng.md"> Menu </a></button>
</p>



<p><span style="float:right;">
 <button name="button">
 <a rel="license" href="https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/setup-doc/ENG/install-ros-packages.md"> Next </a>
 </button>
 </span>
</p>
</div>
