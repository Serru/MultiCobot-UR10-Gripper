# Proposed solutions with the package [`MoveIt!`](https://github.com/ros-planning/moveit) 

[Español](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/moveit-intro.md) | **English**

## Index
- [Modeling of multiple robots in the *URDF* file](#modeling )
- [Replicating the `MoveIt!` node](#replication) 

<a name="modeling">
<h2>
Modeling of multiple robots in the <i>URDF</i> file
</h2>
</a>

![image](/doc/imgs_md/Diseno-moveit-general-dos-cobots-leap-motion-urdf.png "Multirobot system by modeling the URDF file") 

In the schematic design, the main component that performs the planner functions is the `MoveIt!` package. It is based on adding robots in the *URDF* file and the `MoveIt!` planner recognizes them as a single robot that receives the shared values with the motion planner of its `end-effectors` (API of the `move_group` node) and sends these received values as a single trajectory to the controller to execute the desired motion.

**The advantages of this proposal are:** 

- The power of the functions offered by `MoveIt!` for each robot as a whole.
- The simplicity of setup, which is largely automatic with the `MoveIt Setup Assistant`.
- The flexibility to adapt or integrate future elements that require the functionalities of `MoveIt!`, such as devices for the development of computer vision, *deep learning*, navigation, etc. 

**The disadvantages are:** 

- Time required to plan trajectories, since the trajectories of each `end-effector` must be determined in advance, to be combined later, as if it were the movement of a single robot.
- There is only one group that manages the movements of the cobots, which means that it has to be reconfigured each time a robot is added to the system, which affects the scalability of the system.
- It is not possible to control different models and brands of cobots at the same time, as they may require different controls. 

This solution has a very important advantage: it knows the movements of all robots and can avoid them when planning paths to reach the desired position. But the package architecture of `MoveIt!` does not allow to control multiple robots with different controllers. 

If you want to control multiple robots with this design, you could achieve this by executing the movements of each robot one after the other, instead of achieving the desired simultaneity. 

If you force this (by running two scripts that send instructions to schedule the desired trajectories for each controller), the `MoveIt!` node `move_group` sends a message saying that the instruction is *PREEMTED*, which means that the system scheduler has deferred it as future work. 

To work around this problem, the `move_group` node would need to be set to assume that it is processing a single controller. To achieve this, you need to create at least three manipulator groups during setup with the `MoveIt Setup Assistant`, one for each arm along with its `end-effector` and another group containing the two previously created groups. 

Once you have this configuration, you need to create the trajectory of each arm without executing it and properly enter the values of the joints of each robot for this trajectory into the group with both robots to be executed later. 

This proposal has already been implemented by [*TEAM O2AC for the World Robot Summit 2020 Assembly Challenge*](https://github.com/o2ac/o2ac-ur/).


<a name="replication">
<h2>
Replicating the <a href="https://github.com/ros-planning/moveit"><i> MoveIt!</i></a> node
</h2>
</a>

![image](/doc/imgs_md/Diseno-moveit-general-dos-cobots-leap-motion.png "Multirobot system through node replication") 

In the schematic design, the main component that performs the motion planning functions is the `MoveIt!` package. This design is based on replication of the components that motion planner needs for its operation (the main node is `move_group`). Each replication is a motion planner assigned to each of the cobots, which are delimited in their own space or `namespace`.

In the robot description, only one cobot description is needed and the number of robots it can control simultaneously is directly related to the number of motion planner replications with different `namespaces`. The numbers in the figure indicate the order of the phases during development. 

**The advantages of this proposal are:** 

- The scalability of the system and the number of cobots.
- The power of the features offered by `MoveIt!` for each robot.
- A simple configuration, most of which is created automatically with the `MoveIt Setup Assistant`.
- Flexibility to adapt or integrate future elements that require the functionalities of `MoveIt!`, such as devices for computer vision work development, *deep learning*, navigation, etc.
- Enables simultaneous control of different models and brands of cobots.
- Behind the `MoveIt!` package is a very active community that supports its users. 

**The disadvantages are:** 

- Little documentation on proper configuration for simultaneous control of multiple robots.
- Restriction of the functionalities of the `MoveIt!` package to each robot but not as a whole, e.g.: the function to plan paths to avoid collisions between robots would not work properly because it has no knowledge of it.
- Loss of efficiency when scaling: if you scale the system and increase the number of cobots it controls, the functions that currently occupy a resource will also be replicated, but will not perform any work.
- It is difficult to make changes to the source code of the `MoveIt!` package functions.

### Development and implementation of the solution and its tests (phases 1, 2 and 3)
- [One UR10 with gripper using the `MoveIt!` package](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/moveit/ENG/one_arm_moveit.md)
- [Two UR10s with grippers using the `MoveIt!` package](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/moveit/ENG/two_arm_moveit.md)
- [Four UR10 with grippers using the `MoveIt!` package](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/moveit/ENG/four_arm_moveit.md) 

---

<div>
<p align="left">
<button name="button"><a rel="license" href="https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/design-eng.md"> Main Menu </a></button>
</p>
</div>