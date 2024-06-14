# Instalación de *ROS Kinetic y dependencias*

**Español** | [English](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/setup-doc/ENG/install-ROS.md)

## Instalación de *ROS Kinetic Kame*
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

## Instalación de dependencias de *ROS*

```bash
sudo apt-get install ros-kinetic-$(dependencia_name)
```

Si la dependencia es *moveit_ros_planning*, entonces *dependencia_name* será *moveit-ros-planning*, se sigue este patrón para instalar todas las dependencias, para el ejemplo anterior quedaría:

```bash
sudo apt-get install ros-kinetic-moveit-ros-planning
```

### Dependencias a instalar
- moveit_ros_planning 
- moveit_kinematics
- moveit_simple_controller_manager
- position_controllers
- moveit_fake_controller_manager
- gazebo_ros_control
- joint_trajectory_controller
- moveit_ros_visualization
- moveit_planners_ompl
- joint_state_controller
- effort_controllers
- ros_controllers
- industrial_msgs


*Las dependencias pueden ser distintas a lo que se propone en esta guía, durante la instalación de las herramientas y paquetes de *ROS* irán apareciendo las distintas dependencias que necesita ROS.*

---

<div>
<p align="left">
<button name="button">
            	<a rel="license" href="https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/setup-doc/proyect_setup.md">Menú</a>
</button>
</p>



<p>
    <span style="float:right;">
        <button name="button">
            	<a rel="license" href="https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/setup-doc/ESP/install-ros-packages.md">Siguiente</a>
            	</button>
    </span>
</p>
