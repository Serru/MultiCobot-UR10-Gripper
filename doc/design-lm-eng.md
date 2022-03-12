# Leap Motion Design and Integration 

[Español](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/design-lm.md) | **English**

![image](/doc/imgs_md/Diseno-moveit-general-un-cobot-leap-motion.png "Leap Motion Design and Integration") 

Here we explain how *Leap Motion* is integrated into the system developed so far to control up to two robots simultaneously for the different solutions proposed in this repository. In the schematic representation of the design, this section represents stages 4 and 5.

## Prerequisite
- Successful installation of [Basic System Configuration](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/setup-doc/proyect_setup_eng.md).
- [Implementation of one of the proposed solutions](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/design-eng.md) (corresponds to phases 1, 2 and 3 of the scheme) . 

## Index
- [Phase 4: *Leap Motion* interface design](#phase4)
    - [General operation](#lm1)
    - [Coordinate systems](#lm2)
    - [Workspace of *Leap Motion*](#lm3)
    - [Control modes](#lm4)
    - [Gesture recognition](#lm5)
    - [Calibration of motion speed](#lm6)
    - [Creation of the `msgs` that convey the information](#lm7)
    - [Obtaining and publishing data by *topic*](#lm8)
- [Phase 5: Integration of *Leap Motion* into the system](#phase5)

<a name="fase4">
 <h2>
 Phase 4: <i>Leap Motion</i> interface design
 </h2>
</a>

<a name="lm1">
 <h3>
 General operation
 </h3>
</a>

![image](/doc/imgs_md/Diseno-leap-motion.png "Scheme of how Leap Motion works") 

The *Leap Motion* libraries are used to identify the gestures and obtain the data needed to control the gripper, the movements of the cobot, and the orientations of the *end- effector*. The figure shows a schematic representation of the general flow. The file `leap_interface.py` is created using the `Leap` library of *Leap Motion*, which waits for events (`frames` from *Leap Motion*) and on each event retrieves the required data and stores it in an `object`, which is then accessed by the `sender` node using the interface created to access that object. The `sender` node receives the information, stores it in a message and publishes it via the *topic* `leapmotion/data1`. The `UR10_lm_arm_1` node is subscribed to this *topic* and sends the commands to the cobot with the information received from the *topic*. Finally, the `UR10_lm_arm_1` node is basically the *script* that executes the *pick & place*, but the inputs are taken from the *topic* `leapmotion/data_1` instead of being entered manually. 

In the schematic representation of the image, you can see that two *topic*s originate from the `sender` node. This is because it was taken into account during development that *Leap Motion* can recognize up to two hands. Therefore, the data of the right hand (`right.msg`) and the left hand (`left.msg`) were separated in the schema, because the information is sent via different *topic*s and not all information was entered in a single message, because this way you can play with the ratio of the releases, get more clarity and debugging is easier.
 
<a name="lm2">
 <h3>
Coordinate systems
 </h3>
</a>

![image](/doc/imgs_md/distintintos-sistemas-referencia.png "Different reference coordinate systems") 

When designing, note that the reference coordinates of *ROS* and the coordinates used by *Leap Motion* are different (as shown in the figure), so you will need to adjust them accordingly during implementation.

<a name="lm3">
 <h3>
 Workspace of <i>Leap Motion</i>
 </h3>
</a>

It must also be taken into account that the workspace of *Leap Motion* is quite small compared to that of the UR10 cobot. So depending on what tasks you want to perform, you have to take this into account, but a simple *pick & place* as in this case is not a problem.

<a name="lm4">
 <h3>
Control modes
 </h3>
</a>

With the data obtained by *Leap Motion* two forms of control can be realised in a simple way: 

- **Joystick:** This type of control has a dead zone (*dead zone*) that takes an origin as a reference, and no motion is performed in this dead zone. The moment it leaves this dead zone, the value in this coordinate is increased/decreased depending on how far it is from the reference origin. This must be calibrated so that no sudden movements are executed. 

- **Imitation:** This solution was chosen because it is more intuitive when executing movements. It consists in matching the reference origin of *Leap Motion* and the reference origin of the *end-effector* of the mapped robot, i.e. the coordinates taken as reference for *Leap Motion* refer to the initial position of the UR10 robot. In this way, the robot can imitate the movements of the hand. It must also be calibrated correctly to avoid sudden movements.

<a name="lm5">
 <h3>
Gesture recognition
 </h3>
</a>

When identifying gestures, note that *Leap Motion* may give false positives for very similar gestures. Also, obscured parts of the hand during movement may lead you to believe you have recognized a gesture that was not performed. 

Four types of gestures have been implemented, which you can see in the image. The *fist* gesture indicates to stop sending instructions, the *pincer* gesture is used to control the cobot's gripper, the *thumbs up* gesture indicates that it is ready, and the *rock* gesture indicates that it takes the current position of the hand as a reference source. It was prepared for implementation to control the orientation of the *end-effector*, but since this causes identification problems with some of the gestures, it was decided that it would be better to have a fixed orientation. 

![image](/doc/imgs_md/gestos-leap-motion.png "Definition of gestures for Leap Motion") 


In the [leap_interface.py](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_leap_motion/scripts/leap_interface.py) file where the gestures are defined, we will analyze the part of the code that identifies a gesture as an example. The source code shown below attempts to verify that the *thumbs up* gesture has been executed each time *Leap Motion* sends a `frame`. Each time a `frame` is received, it checks whether it is from the *right* or *left* hand. Then it checks how closed the hand is by checking if the value of the `grab_strength` attribute, if greater than the defined value, indicates that the hand is closed. Since we know that the hand is closed, we want to know if the thumb is extended or not. This is determined by checking if the attribute `thumb_finger.extended()` is equal to `1`.

```python
def on_frame(self, controller):
        frame = controller.frame()

        for hand in frame.hands:
            handType = "Left hand" if hand.is_left else "Right hand"

            if handType == "Right hand":
                if hand.grab_strength > self.GRAB_STRENGTH_THRESHOLD:
                    thumb_finger = hand.fingers.finger_type(0)
                    for _ in thumb_finger:
                        if len(thumb_finger.extended()) == 0:
                            self.right_hand_fist = True
                            self.right_hand_thumb_up = False
                        elif len(thumb_finger.extended()) == 1:
                            self.right_hand_thumb_up = True
                            self.right_hand_fist = False
            else:
                if hand.grab_strength > self.GRAB_STRENGTH_THRESHOLD:
                    thumb_finger = hand.fingers.finger_type(0)
                    for _ in thumb_finger:
                        if len(thumb_finger.extended()) == 0:
                            self.left_hand_fist = True
                            self.left_hand_thumb_up = False
                        elif len(thumb_finger.extended()) == 1:
                            self.left_hand_fist = False
                            self.left_hand_thumb_up = True
``` 

<a name="lm6">
 <h3>
Calibration of motion speed
 </h3>
</a>

To avoid sudden movements, the velocity of the *end-effector* was used as a limiting factor, for which a maximum velocity of *0.05 rad/s* was assumed. The distance is based on the largest error between the values of the *joints* of the current position and the *joints* of the future position, with a division indicating the time this movement should take.

<a name="lm7">
 <h3>
Creation of the <i>msgs</i> that convey the information
 </h3>
</a>

Two types of messages were created for the implementation, one to identify the movements and gestures of the right hand and the other for the left hand, which also serves to define the type of data transmitted by this *topic*. These files, with the extension `msg`, must be compiled into a directory named *msg* so that they can be integrated into the system *ROS*, otherwise it will not find them. Specifically, the content of the following source code is for the right hand, for the left hand it would be the same, except that you replace `right` with `left`.

```bash
Header header

# Right hand information
bool is_right_hand                      # Right hand detected
geometry_msgs/Point right_hand_palmpos  # Palm's position
bool right_hand_fist                    # Fist gesture recognize
bool right_hand_thumb_up                # Thumb up gesture recognize
bool right_hand_pinch                   # Pinch gesture recognize
float32 right_hand_pinch_value			# Pinch gesture value
bool right_hand_origin_frame            # Reference frame set
bool right_hand_set_origin_frame_detected # Detect gesture
float32 right_hand_rotate_value         # Values between [-1..0..1] rads
float32 right_hand_turn_value           # Values between [-1..0..1] rads
float32 right_hand_swing_value
``` 

<a name="lm8">
 <h3>
Obtaining and publishing data by topic
 </h3>
</a>

To get *Leap Motion* data, the interface defined in the `leap_interface.py` file (e.g. `li.get_is_right_hand()`) is used to access the *object* that stores the desired data, as shown below in the source code. Once the message is created, it is sent via the *topic* `leapmotion/data` with the message type `leapcobotright`.

```python
pub_ros_right   = rospy.Publisher('leapmotion/data', leapcobotright, queue_size=1)

while not rospy.is_shutdown():
        right_hand_palm_pos_    = li.get_right_hand_palmpos()   # Palm's position

        # Right hand information
        msg_right = leapcobotright()
        msg_right.is_right_hand = li.get_is_right_hand()                     # Right hand detected
        msg_right.right_hand_palmpos.x = right_hand_palm_pos_[0]
        msg_right.right_hand_palmpos.y = right_hand_palm_pos_[1]
        msg_right.right_hand_palmpos.z = right_hand_palm_pos_[2]

        msg_right.right_hand_fist = li.get_right_hand_fist()                 # Fist gesture recognize
        msg_right.right_hand_thumb_up = li.get_right_hand_thumb_up()         # Thumb up gesture recognize
[...]
        pub_ros_right.publish(msg_right)
        rospy.sleep(rospy.get_param(PARAMNAME_FREQ_ENTIRE, FREQUENCY_ROSTOPIC_DEFAULT))
``` 

<a name="fase5">
 <h2>
 Phase 5: Integration of <i> Leap Motion </i> into the system
 </h2>
</a>

The integration of *Leap Motion* into the *ROS* system and as part of the developed solution is very simple. The *script* that controls the cobot's movements only needs to subscribe to the *topic* `leapmotion/data` (for two cobots, one of the scripts will subscribe to the *topic* that sends information from the right hand and the other from the left hand) and use this data input accordingly to control the cobot's movements. 


The picture shows the architecture of the system with nodes and *topic*s. You can see the integration of *Leap Motion* into the system. At the bottom of the image, you can see the *sender* node (`one_arm_no_moveit_lm_pub`), the *topic* that publishes the data received from the *Leap Motion* device (`/leapmotion/data`), and the * script* that processes the information to send commands to the robot is the node `ur10_dual_moveit`.

![image](/doc/imgs_md/one-arm-moveit-rqt-graph-gazebo-moveit-fase-4-detalle.png "Integrating Leap Motion into the system") 



### Adding the Leap Motion device to the system `without` the `MoveIt!` package
- [One UR10 with gripper using its own motion planner and Leap Motion](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/no_moveit/ENG/one_arm_no_moveit_lm.md)
- [Two UR10s with grippers using their own motion planner and Leap Motion](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/no_moveit/ENG/two_arm_no_moveit_lm.md)


### Adding the Leap Motion device to the system `with` the `MoveIt!` package
- [One UR10 with gripper `with` the `MoveIt!` package and Leap Motion](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/moveit/ENG/one_arm_moveit_lm.md)
- [Two UR10s with grippers `with` the `MoveIt!` package and Leap Motion](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/moveit/ENG/two_arm_moveit_lm.md)

---

<div> 
<p align="left">
<button name="button"><a rel="license" href="https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/README.md"> Home </a></button>
</p>
</div>
