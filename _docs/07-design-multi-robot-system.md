---
title: "Design of a multi-robot system"
permalink: /docs/design-multi-robot-system/
excerpt: "After running numerous tests in the ROS environment to create a multi-robot system, it turns out that the solutions must come from the combinations between the URDF file that defines the robot model and the ROS packages included in the schema which are represented by the MoveIt! package, whose main function is motion planning."
toc: true
---

After running numerous tests in the *ROS* environment to create a multi-robot system, it turns out that the solutions must come from the combinations between the *URDF* file that defines the robot model and the *ROS* packages included in the schema which are represented by the `MoveIt!` package, whose main function is motion planning.

![image](https://raw.githubusercontent.com/Serru/MultiCobot-UR10-Gripper/main/doc/imgs_md/Diseno-General-focus.png "ROS Tools and Drivers in Design") 

Possible variants are: 

- *URDF file*: Describes the robot model. Multiple robots, objects or whatever you want to model can be integrated into this model.
- *ROS Packages*: They can be your own or installed by third parties to which modifications are made to adapt them to the solution being developed. 

The proposed solutions revolve around the changes and combinations between the robotic modeling (`URDF`) and the `ROS packages` to be used. Organizationally, the proposed solutions are divided into those that use the `MoveIt!` package and those that do not. It must be taken into account that the working environment is complex and there are many elements that interact or depend on each other. Therefore, problems may arise during the development of the proposed designs for which there is no solution or which are very expensive to fix. 