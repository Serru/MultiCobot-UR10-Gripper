# Two UR10s with grippers using their own motion planner and *Leap Motion*

[Español](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/no_moveit/ESP/two_arm_no_moveit_lm.md) | **English** 

![image](/doc/imgs_md/Diseno-no-moveit-general-dos-cobots-leap-motion.png "Loads the URDF model of the UR10 robot") 

This section is the continuation of phases 1, 2 and 3, where the integration of *Leap Motion* into the system takes place, i.e. phases 4 and 5. In the previous phases, the robots were controlled by sending the previously defined trajectories to perform the *pick & place*. Now this is done by Leap Motion, which is controlled by a person who sends the commands to the two robots. 

For this purpose, part of the *Leap Motion* repository is directly modified and adapted to properly control the simulated robots. The idea is identical to what was presented above, but the data input is the information provided by *Leap Motion*. This information needs to be properly processed and adapted for use with the API that provides it. 

## :book: Implementing the manipulator with *Leap Motion*

There are a few things to keep in mind when implementing this function: 

- The robot's pose references, which you get from the *ur10_robot_pose.py* script.
- The working environment of the *robot*.
- The working environment of *Leap Motion*. 

Since we want the robot to follow the movements of *Leap Motion*, we will implement a controller whose workspace is contained in the robot's workspace. 

### :computer: Create package 

```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit
catkin_create_pkg two_arm_no_moveit_leap_motion rospy
cd two_arm_no_moveit_leap_motion
mkdir scripts
cd scripts
cp ~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_manipulator/scripts/kinematics_utils.py .
touch ur10_1_lm_robot_manipulator.py
touch ur10_2_lm_robot_manipulator.py
touch sender.py
touch leap_interface.py
```

- Contents of the file [ur10_1_lm_robot_manipulator.py](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_leap_motion/scripts/ur10_1_lm_robot_manipulator.py). 

- Contents of file [ur10_2_lm_robot_manipulator.py](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_leap_motion/scripts/ur10_2_lm_robot_manipulator.py). 

- Contents of file [sender.py](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_leap_motion/scripts/sender.py). 

### :computer: Modified from the original repository. 

The following files were created: *leapcobotright.msg* and *leapcobotleft.msg* in the *msg* directory of the [Leap Motion](https://github.com/Serru/MultiCobot-UR10-Gripper/tree/main/src/leap_motion) repository: 

- Contents of [leapcobotright.msg](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/leap_motion/msg/leapcobotright.msg). 

- Contents of [leapcobotleft.msg](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/leap_motion/msg/leapcobotleft.msg). 

- And the file [CMakeLists.txt](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/leap_motion/CMakeLists.txt), where the newly created message files are added for compilation:

```bash
[...]
## Generate messages in the 'msg' folder

add_message_files(
  FILES
  Arm.msg
  Bone.msg
  Finger.msg
  Gesture.msg
  Hand.msg
  Human.msg

  # For backwards compatibility with the old driver files
  leap.msg
  leapros.msg
  leapcobotright.msg
  leapcobotleft.msg
)
[...]
```

## :computer: Tests with the *Gazebo* simulator
At least 4 terminals are needed to run the tests, although they can be reduced in the startup files to run them automatically. But to better visualize the information sent and for debugging, it was left like this. 

- Terminal 1:
```bash
cd ~/MultiCobot-UR10-Gripper
catkin_make
source devel/setup.bash
roslaunch two_arm_no_moveit_gazebo ur10_joint_limited.launch
```

- Terminal 2:
```bash
cd ~/MultiCobot-UR10-Gripper
source devel/setup.bash
rosrun two_arm_no_moveit_leap_motion sender.py
```

- Terminal 3:
```bash
cd ~/MultiCobot-UR10-Gripper
source devel/setup.bash
rosrun two_arm_no_moveit_leap_motion ur10_1_lm_robot_manipulator.py 
```

- Terminal 4:
```bash
cd ~/MultiCobot-UR10-Gripper
source devel/setup.bash
rosrun two_arm_no_moveit_leap_motion ur10_2_lm_robot_manipulator.py 
```

- Terminal 5 (in case the *Leap Motion* device fails):
```bash
sudo service leapd restart
```

---

<div>
 <p align="left">
   <button name="button"><a rel="license" href="https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/design-lm-eng.md"> Back </a></button>
 </p>
</div>