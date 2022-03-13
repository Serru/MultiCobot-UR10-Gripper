---
layout: splash
permalink: /
hidden: true
header:
  overlay_color: "#2e2322"
  overlay_image: /assets/images/proyect-logo.png
  actions:
    - label: "<i class='fab fa-fw fa-github'></i> View on Github"
      url: "https://github.com/Serru/MultiCobot-UR10-Gripper"
excerpt: >
  Multirobot system consisting of UR10s that allows simultaneous execution of tasks. It enables the control of different cobots with different controllers.
feature_row_1:
  - image_path: /assets/images/mm-customizable-feature.png
    alt: "Basic System Configuration"
    title: "Basic System Configuration"
    excerpt: "Here is a step by step explanation of how to install **ROS Kinetic Kame** and all the necessary tools and packages."
    url: "/docs/requirements/"
    btn_class: "btn--primary"
    btn_label: "Learn more"
feature_row_2:
  - image_path: /assets/images/solo-dos-urdf-robot.png
    alt: "Design and development for one, two and four robots"
    title: "Design and development for one, two and four robots"
    excerpt: "The proposed solutions revolve around the changes and combinations between the robotic modeling (`URDF`) and the `ROS packages` to be used. Organizationally, the proposed solutions are divided into those that use the `MoveIt!` package and those that do not."
    url: "/docs/design-multi-robot-system/"
    btn_class: "btn--primary"
    btn_label: "Learn more"
feature_row_3:
  - image_path: /assets/images/lm-moveit-up.png
    alt: "Design and integration of the Leap Motion device into the system to control one and two robots"
    title: "Design and integration of the Leap Motion device into the system to control one and two robots"
    excerpt: "Here we explain how Leap Motion is integrated into the system developed so far to control up to two robots simultaneously for the different solutions proposed."
    url: "/docs/lm-design-integration/"
    btn_class: "btn--primary"
    btn_label: "Learn more"     
feature_row_4:
  - image_path: /assets/images/lm-campero-suela-2.png
    alt: "Configuration for a UR10 with gripper using its motion planning on the physical robot"
    title: "Configuration for a UR10 with gripper using its motion planning on the physical robot"
    excerpt: "All of the configuration previously done in the Gazebo simulator was tested on the physical robot named **Campero** owned by the university."
    url: "/docs/robot-campero/"
    btn_class: "btn--primary"
    btn_label: "Learn more"
---

Multi-robot system of collaborative robots (cobots) [UR10s](https://www.universal-robots.com/products/ur10-robot/) with grippers from Robotiq ([robotiq_85_gripper](https://robotiq.com/products/2f85-140-adaptive-robot-gripper)) that allows simultaneous execution of tasks with different types of controllers and brands of cobots, as well as direct control of the cobot by a person via the Leap Motion device.


## Documentation

{% include feature_row id="feature_row_1" type="left"%}
{% include feature_row id="feature_row_2" type="right"%}
{% include feature_row id="feature_row_3" type="left"%}
{% include feature_row id="feature_row_4" type="right"%}

## Video with the results
Here is a video with the results of the simulations performed in `Gazebo`. The video shows two and four robots performing a *pick & place* without human intervention. Then it shows how a person controls two cobots with Leap Motion, and finally the developed result was tested with the physical Campero robot.

<p>
<a href="https://drive.google.com/file/d/1oqVyre4vlfHqH9SrQuyXH00GcmwIuP97/view?usp=sharing" title="Link Title">
  <img src="https://raw.githubusercontent.com/Serru/MultiCobot-UR10-Gripper/main/doc/imgs_md/img-fondo-video.png" alt="Results of the project." />
</a>
</p>


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

## Acknowledgement

This work is part of the [RoPeRT](https://i3a.unizar.es/es/grupos-de-investigacion/ropert) research group of the [i3A](https://i3a.unizar.es), the [University of Zaragoza](http://www.unizar.es/).

![image](https://www.unizar.es/sites/default/files/i3a.png)

---

The developed work has been experimentally evaluated and validated, showing a correct operation of the physical robot [Campero](http://commandia.unizar.es/wp-content/uploads/camperoRobot.jpg). For this reason, this work is part of the activities of the project [COMMANDIA (2019)](http://commandia.unizar.es/), co-funded by the [Interreg Sudoe Program](https://www.interreg-sudoe.eu/inicio) and the [European Regional Development Fund (ERDF)](https://ec.europa.eu/regional_policy/es/funding/erdf/).

![image](http://commandia.unizar.es/wp-content/uploads/cropped-logoCommandia-1.png)


## Authors
- [Burgh Oliván, Miguel](https://github.com/Serru) - *Author of the Final Degree Project entitled **Multirobot system for the collaborative transport of objects**.*
- [López Nicolás, Gonzalo](https://i3a.unizar.es/es/investigadores/gonzalo-lopez-nicolas) - *Director of the Final Degree Project entitled **Multirobot system for the collaborative transport of objects**.*

The memory of the Final Degree Project can be found in the [Repository](https://deposita.unizar.es/record/66296?ln=es) of TFGs of the [University of Zaragoza](http://www.unizar.es/).

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