# One UR10 with gripper using its own motion planner and *Leap Motion*

[Español](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/no_moveit/ESP/one_arm_no_moveit_lm.md) | **English** 

![image](/doc/imgs_md/Diseno-no-moveit-general-un-cobot-leap-motion.png "Loaded the URDF model of the UR10 robot") 

This section is the continuation of phases 1, 2 and 3, where the integration of *Leap Motion* into the system takes place, i.e. phases 4 and 5. In the previous phases, the robot was controlled by sending the previously defined trajectories to perform the *Pick & Place*. Now this is done by *Leap Motion*, controlled by a person who sends the commands to the robot. 

For this purpose, part of the *Leap Motion* repository is directly modified and adapted to properly control the simulated robot. The idea is identical to what was presented above, but the data input is the information provided by *Leap Motion*. This information needs to be handled properly with the API that provides it and adapted for its use. 

## :book: Implementing the Manipulator with *Leap Motion*

There are a few things to keep in mind when implementing this: 

- The references of the robot position, which you get from the script *ur10_robot_pose.py*
- The working environment of the *robot*
- The working environment of *Leap Motion* 

Since we want the robot to follow the movements of *Leap Motion*, we will implement a controller whose workspace is contained in the robot's workspace. 


### :computer: Package creation
```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit
catkin_create_pkg one_arm_no_moveit_leap_motion rospy
cd one_arm_no_moveit_leap_motion
mkdir scripts
cd scripts
cp ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_manipulator/scripts/kinematics_utils.py .
touch lm_robot_manipulator.py
touch sender.py
touch leap_interface.py
``` 

- Contents of the file [lm_robot_manipulator.py](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_leap_motion/scripts/lm_robot_manipulator.py). 

- Contents of file [sender.py](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_leap_motion/scripts/sender.py). 

- Contents of file [leap_interface.py](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_leap_motion/scripts/leap_interface.py). 

### :computer: Modified from the original repository 

The following files were created: *leapcobotright.msg* and *leapcobotleft.msg* in the *msg* directory of the [Leap Motion](https://github.com/Serru/MultiCobot-UR10-Gripper/tree/main/src/leap_motion) repository: 

- Contents of [leapcobotright.msg](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/leap_motion/msg/leapcobotright.msg).  

- Contents of [leapcobotleft.msg](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/leap_motion/msg/leapcobotleft.msg). 

- And the file [CMakeLists.txt](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/leap_motion/CMakeLists.txt), where the newly created message files are added for compilation:

```bash
[...]
## Generate messages in the 'msg' folder
add_message_files(
files
Arm.msg
Bone.msg
Finger.msg
gesture.msg
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
At least 3 terminals are needed to run the tests, although they can be reduced in the *launch* files to start them automatically. But to better visualize and debug the information sent, it was left that way. 

- Terminal 1:
```bash
cd ~/MultiCobot-UR10-Gripper
source devel/setup.bash
roslaunch one_arm_no_moveit_gazebo ur10_joint_limited.launch
``` 

- Terminal 2:
```bash
cd ~/MultiCobot-UR10-Gripper
source devel/setup.bash
rosrun one_arm_no_moveit_leap_motion sender.py
``` 

- Terminal 3:
```bash
cd ~/MultiCobot-UR10-Gripper
source devel/setup.bash
rosrun one_arm_no_moveit_leap_motion lm_robot_manipulator.py
``` 

- Terminal 4 (in case the *Leap Motion* device fails):
```bash
sudo service leapd restart
``` 

---

<div>
 <p align="left">
   <button name="button"><a rel="license" href="https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/design-lm-eng.md"> Back </a></button>
 </p>
</div>
