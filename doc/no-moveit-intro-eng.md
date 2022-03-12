# Proposed solutions using a proprietary or third-party motion planning (without the `MoveIt!` package) 

[Español](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/no-moveit-intro.md) | **English**

![image](/doc/imgs_md/Diseno-no-moveit-general-dos-cobots-leap-motion.png "Loaded the URDF model of the UR10 robot") 

In the schematic design, the main component that performs the motion planner functions must be implemented from scratch. The motion planner communicates directly with the controllers needed to execute the cobot's motions, it does not need to be contained in a *namespace*. 

You must describe all the cobots you want to control at the same time in the *URDF* file that defines the model. The numbers in the figure indicate the order of the phases this solution goes through. 

**The advantages of this proposal are:** 

- The scalability of the system and the number of cobots is more complex.
- It is very efficient compared to previous solutions.
- It is easy to make changes to the implemented source code.
- Allows simultaneous control of different models and brands of cobots. 

**The disadvantages are:** 

- Low adaptability when integrating into another project, since it is a custom solution.
- It does not have the ability to plan trajectories to avoid collisions with other cobots or with itself, since these functions are provided by `MoveIt!`.
- It is necessary to implement the functionality to perform Cartesian motions, as this is a prerequisite for the future integration of *Leap Motion* into the system.
- Configuration can be tedious (controllers, *topic*s, message handling, interaction between what was created and what was created by third parties, etc.), some familiarity with the environment is required. 

#### Development and implementation of the solution and its tests (phases 1, 2 and 3)
- [One UR10 with gripper using its own motion planner](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/no_moveit/ENG/one_arm_no_moveit.md)
- [Two UR10s with grippers using their own motion planner](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/no_moveit/ENG/two_arm_no_moveit.md)
- [Four UR10s with grippers using their own motion planner](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/no_moveit/ENG/four_arm_no_moveit.md)

---

<div>
<p align="left">
<button name="button"><a rel="license" href="https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/design-eng.md"> Main Menu </a></button>
</p>
</div>
