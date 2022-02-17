<!--- Para cuatro robots opción A --->
## Instalación y configuración para cuatro robots UR10 sin MoveIt!
Se va a realizar la solución para cuatro robots esta vez, de la misma manera que se ha realizado para dos, pero modificando el contenido de los ficheros adaptándolo para su similación con cuatro robots.

### Creación del directorio
```{bash}
cd ~/MultiCobot-UR10-Gripper/src/multirobot
mkdir four_arm_no_moveit
```
### Puesta en marcha de Gazebo para dos robots
Se va a crear el paquete para gazebo, y copiar el contenido de la solución anterior para su posterior modificación:
```{bash}
cd ~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit
catkin_create_pkg four_arm_no_moveit_gazebo rospy
```
En el directorio creado para gazebo, se copiara del directorio de two_arm_no_moveit_gazebo*, las carpetas *controller*, *launch*, *models*, *scripts* y *world*.
```{bash}
cd ~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_gazebo
cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_gazebo/controller .
cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_gazebo/launch .
cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_gazebo/models .
cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_gazebo/scripts .
cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_gazebo//world .
```

Se compila:
```{bash}
cd ~/MultiCobot-UR10-Gripper
catkin_make
```
### Modificaciones a realizar para simular dos robots en gazebo
Primero, hay que decidir en el *namespace* para cada robot, es decir el nombre de grupo sobre el que agruparemos las configuraciones para cada uno de los robots.

Se procede a modificar los ficheros así como la adicción de los grupos de controladores para el los robots que se llevarán los prefijos *ur10_1_*, *ur10_2_*,*ur10_3_* y *ur10_4_*


* Fichero *ur10_1_arm_controller.yaml*
```{C}
ur10_1_arm_controller:
  type: position_controllers/JointTrajectoryController
  joints:
     - ur10_1_shoulder_pan_joint
     - ur10_1_shoulder_lift_joint
     - ur10_1_elbow_joint
     - ur10_1_wrist_1_joint
     - ur10_1_wrist_2_joint
     - ur10_1_wrist_3_joint
  constraints:
      goal_time: 0.6
      stopped_velocity_tolerance: 0.05
      ur10_1_shoulder_pan_joint: {trajectory: 0.1, goal: 0.1}
      ur10_1_shoulder_lift_joint: {trajectory: 0.1, goal: 0.1}
      ur10_1_elbow_joint: {trajectory: 0.1, goal: 0.1}
      ur10_1_wrist_1_joint: {trajectory: 0.1, goal: 0.1}
      ur10_1_wrist_2_joint: {trajectory: 0.1, goal: 0.1}
      ur10_1_wrist_3_joint: {trajectory: 0.1, goal: 0.1}
  stop_trajectory_duration: 0.5
  state_publish_rate:  25
  action_monitor_rate: 10
ur10_1_joint_group_position_controller:
  type: position_controllers/JointGroupPositionController
  joints:
     - ur10_1_shoulder_pan_joint
     - ur10_1_shoulder_lift_joint
     - ur10_1_elbow_joint
     - ur10_1_wrist_1_joint
     - ur10_1_wrist_2_joint
     - ur10_1_wrist_3_joint
```

* Fichero *ur10_1_gripper_controller_robotiq.yaml*
```{C}
ur10_1_gripper:
  type: position_controllers/JointTrajectoryController
  joints:
     - ur10_1_robotiq_85_left_knuckle_joint
  constraints:
      goal_time: 0.6
      stopped_velocity_tolerance: 0.05
      ur10_1_robotiq_85_left_knuckle_joint: {trajectory: 0.1, goal: 0.1}
  stop_trajectory_duration: 0.5
  state_publish_rate:  25
  action_monitor_rate: 10
```

* Fichero *ur10_2_arm_controller.yaml*
```{C}
ur10_2_arm_controller:
  type: position_controllers/JointTrajectoryController
  joints:
     - ur10_2_shoulder_pan_joint
     - ur10_2_shoulder_lift_joint
     - ur10_2_elbow_joint
     - ur10_2_wrist_1_joint
     - ur10_2_wrist_2_joint
     - ur10_2_wrist_3_joint
  constraints:
      goal_time: 0.6
      stopped_velocity_tolerance: 0.05
      ur10_2_shoulder_pan_joint: {trajectory: 0.1, goal: 0.1}
      ur10_2_shoulder_lift_joint: {trajectory: 0.1, goal: 0.1}
      ur10_2_elbow_joint: {trajectory: 0.1, goal: 0.1}
      ur10_2_wrist_1_joint: {trajectory: 0.1, goal: 0.1}
      ur10_2_wrist_2_joint: {trajectory: 0.1, goal: 0.1}
      ur10_2_wrist_3_joint: {trajectory: 0.1, goal: 0.1}
  stop_trajectory_duration: 0.5
  state_publish_rate:  25
  action_monitor_rate: 10
ur10_2_joint_group_position_controller:
  type: position_controllers/JointGroupPositionController
  joints:
     - ur10_2_shoulder_pan_joint
     - ur10_2_shoulder_lift_joint
     - ur10_2_elbow_joint
     - ur10_2_wrist_1_joint
     - ur10_2_wrist_2_joint
     - ur10_2_wrist_3_joint
```

* Fichero *ur10_2_gripper_controller_robotiq.yaml*
```{C}
ur10_2_gripper:
  type: position_controllers/JointTrajectoryController
  joints:
     - ur10_2_robotiq_85_left_knuckle_joint
  constraints:
      goal_time: 0.6
      stopped_velocity_tolerance: 0.05
      ur10_2_robotiq_85_left_knuckle_joint: {trajectory: 0.1, goal: 0.1}
  stop_trajectory_duration: 0.5
  state_publish_rate:  25
  action_monitor_rate: 10
```

* Fichero *ur10_3_arm_controller.yaml*
```{C}
ur10_3_arm_controller:
  type: position_controllers/JointTrajectoryController
  joints:
     - ur10_3_shoulder_pan_joint
     - ur10_3_shoulder_lift_joint
     - ur10_3_elbow_joint
     - ur10_3_wrist_1_joint
     - ur10_3_wrist_2_joint
     - ur10_3_wrist_3_joint
  constraints:
      goal_time: 0.6
      stopped_velocity_tolerance: 0.05
      ur10_3_shoulder_pan_joint: {trajectory: 0.1, goal: 0.1}
      ur10_3_shoulder_lift_joint: {trajectory: 0.1, goal: 0.1}
      ur10_3_elbow_joint: {trajectory: 0.1, goal: 0.1}
      ur10_3_wrist_1_joint: {trajectory: 0.1, goal: 0.1}
      ur10_3_wrist_2_joint: {trajectory: 0.1, goal: 0.1}
      ur10_3_wrist_3_joint: {trajectory: 0.1, goal: 0.1}
  stop_trajectory_duration: 0.5
  state_publish_rate:  25
  action_monitor_rate: 10
ur10_3_joint_group_position_controller:
  type: position_controllers/JointGroupPositionController
  joints:
     - ur10_3_shoulder_pan_joint
     - ur10_3_shoulder_lift_joint
     - ur10_3_elbow_joint
     - ur10_3_wrist_1_joint
     - ur10_3_wrist_2_joint
     - ur10_3_wrist_3_joint
```

* Fichero *ur10_3_gripper_controller_robotiq.yaml*
```{C}
ur10_3_gripper:
  type: position_controllers/JointTrajectoryController
  joints:
     - ur10_3_robotiq_85_left_knuckle_joint
  constraints:
      goal_time: 0.6
      stopped_velocity_tolerance: 0.05
      ur10_3_robotiq_85_left_knuckle_joint: {trajectory: 0.1, goal: 0.1}
  stop_trajectory_duration: 0.5
  state_publish_rate:  25
  action_monitor_rate: 10
```

* Fichero *ur10_4_arm_controller.yaml*
```{C}
ur10_4_arm_controller:
  type: position_controllers/JointTrajectoryController
  joints:
     - ur10_4_shoulder_pan_joint
     - ur10_4_shoulder_lift_joint
     - ur10_4_elbow_joint
     - ur10_4_wrist_1_joint
     - ur10_4_wrist_2_joint
     - ur10_4_wrist_3_joint
  constraints:
      goal_time: 0.6
      stopped_velocity_tolerance: 0.05
      ur10_4_shoulder_pan_joint: {trajectory: 0.1, goal: 0.1}
      ur10_4_shoulder_lift_joint: {trajectory: 0.1, goal: 0.1}
      ur10_4_elbow_joint: {trajectory: 0.1, goal: 0.1}
      ur10_4_wrist_1_joint: {trajectory: 0.1, goal: 0.1}
      ur10_4_wrist_2_joint: {trajectory: 0.1, goal: 0.1}
      ur10_4_wrist_3_joint: {trajectory: 0.1, goal: 0.1}
  stop_trajectory_duration: 0.5
  state_publish_rate:  25
  action_monitor_rate: 10
ur10_4_joint_group_position_controller:
  type: position_controllers/JointGroupPositionController
  joints:
     - ur10_4_shoulder_pan_joint
     - ur10_4_shoulder_lift_joint
     - ur10_4_elbow_joint
     - ur10_4_wrist_1_joint
     - ur10_4_wrist_2_joint
     - ur10_4_wrist_3_joint
```

* Fichero *ur10_4_gripper_controller_robotiq.yaml*
```{C}
ur10_4_gripper:
  type: position_controllers/JointTrajectoryController
  joints:
     - ur10_4_robotiq_85_left_knuckle_joint
  constraints:
      goal_time: 0.6
      stopped_velocity_tolerance: 0.05
      ur10_4_robotiq_85_left_knuckle_joint: {trajectory: 0.1, goal: 0.1}
  stop_trajectory_duration: 0.5
  state_publish_rate:  25
  action_monitor_rate: 10
```
### Modificación del launch file de Gazebo
Hay que modificar ahora el launch file para lanzar los controladores de ambos orbots en gazebo.

```{bash}
cd ~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_gazebo/launch
nano ur10.launch
```

Tras modificarlo, el fichero queda de la siguiente manera:
```{xml}
<?xml version="1.0"?>
<launch>
  <arg name="limited" default="false"  doc="If true, limits joint range [-PI, PI] on all joints." />
  <arg name="paused" default="false" doc="Starts gazebo in paused mode" />
  <arg name="gui" default="true" doc="Starts gazebo gui" />
  
  <arg name="world" default="$(find four_arm_no_moveit_gazebo)/world/multiarm_bot.world" />

  <!-- startup simulated world -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" default="$(arg world)"/>
    <arg name="paused" value="$(arg paused)"/>
    <arg name="gui" value="$(arg gui)"/>
  </include>

  <!-- send robot urdf to param server -->
  <include file="$(find four_arm_no_moveit_description)/launch/ur10_upload.launch">
    <arg name="limited" value="$(arg limited)"/>
  </include>

  <!-- push robot_description to factory and spawn robot in gazebo -->
  <node name="spawn_gazebo_model" pkg="gazebo_ros" type="spawn_model" respawn="false" output="screen" args="-urdf -param robot_description -model robot " />

  <include file="$(find four_arm_no_moveit_gazebo)/launch/controller_utils.launch"/>

  <!-- ur10_1_arm_controller -->
  <rosparam file="$(find four_arm_no_moveit_gazebo)/controller/ur10_1_arm_controller.yaml" command="load"/>
  <node name="ur10_1_arm_controller_spawner" pkg="controller_manager" type="controller_manager" args="spawn ur10_1_arm_controller" respawn="false" output="screen"/>

  <!-- ur10_1_robotiq_85_gripper controller -->
  <rosparam file="$(find four_arm_no_moveit_gazebo)/controller/ur10_1_gripper_controller_robotiq.yaml" command="load"/> 
  <node name="ur10_1_gripper_controller_spawner" pkg="controller_manager" type="spawner" args="ur10_1_gripper" />

  <!-- ur10_2_arm_controller -->
  <rosparam file="$(find four_arm_no_moveit_gazebo)/controller/ur10_2_arm_controller.yaml" command="load"/>
  <node name="ur10_2_arm_controller_spawner" pkg="controller_manager" type="controller_manager" args="spawn ur10_2_arm_controller" respawn="false" output="screen"/>

  <!-- ur10_2_robotiq_85_gripper controller -->
  <rosparam file="$(find four_arm_no_moveit_gazebo)/controller/ur10_2_gripper_controller_robotiq.yaml" command="load"/> 
  <node name="ur10_2_gripper_controller_spawner" pkg="controller_manager" type="spawner" args="ur10_2_gripper" />

  <!-- ur10_3_arm_controller -->
  <rosparam file="$(find four_arm_no_moveit_gazebo)/controller/ur10_3_arm_controller.yaml" command="load"/>
  <node name="ur10_3_arm_controller_spawner" pkg="controller_manager" type="controller_manager" args="spawn ur10_3_arm_controller" respawn="false" output="screen"/>

  <!-- ur10_3_robotiq_85_gripper controller -->
  <rosparam file="$(find four_arm_no_moveit_gazebo)/controller/ur10_3_gripper_controller_robotiq.yaml" command="load"/> 
  <node name="ur10_3_gripper_controller_spawner" pkg="controller_manager" type="spawner" args="ur10_3_gripper" />

  <!-- ur10_4_arm_controller -->
  <rosparam file="$(find four_arm_no_moveit_gazebo)/controller/ur10_4_arm_controller.yaml" command="load"/>
  <node name="ur10_4_arm_controller_spawner" pkg="controller_manager" type="controller_manager" args="spawn ur10_4_arm_controller" respawn="false" output="screen"/>

  <!-- ur10_4_robotiq_85_gripper controller -->
  <rosparam file="$(find four_arm_no_moveit_gazebo)/controller/ur10_4_gripper_controller_robotiq.yaml" command="load"/> 
  <node name="ur10_4_gripper_controller_spawner" pkg="controller_manager" type="spawner" args="ur10_4_gripper" />

  <!-- load other controllers -->
  <node name="ur10_1_ros_control_controller_manager" pkg="controller_manager" type="controller_manager" respawn="false" output="screen" args="load ur10_1_joint_group_position_controller" />

  <node name="ur10_2_ros_control_controller_manager" pkg="controller_manager" type="controller_manager" respawn="false" output="screen" args="load ur10_2_joint_group_position_controller" />

  <node name="ur10_3_ros_control_controller_manager" pkg="controller_manager" type="controller_manager" respawn="false" output="screen" args="load ur10_3_joint_group_position_controller" />

  <node name="ur10_4_ros_control_controller_manager" pkg="controller_manager" type="controller_manager" respawn="false" output="screen" args="load ur10_4_joint_group_position_controller" />

</launch>
```

```{bash}
cd ~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_gazebo/launch
nano ur10_joint_limited.launch
```

Tras modificarlo, el fichero queda de la siguiente manera:
```{xml}
<?xml version="1.0"?>
<launch>
  <arg name="gui" default="true" doc="Starts gazebo gui" />

  <include file="$(find four_arm_no_moveit_gazebo)/launch/ur10.launch">
    <arg name="limited" value="true"/>
    <arg name="gui" value="$(arg gui)"/>
  </include>

</launch>
```

Falta modificar el fichero que lanza los scripts que se crearon, previamente así como esos ficheros, ya que se han corregido para adaptarse a los namesapces correctamente:

* *~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_gazebo/launch/controller_utils.launch*
```{xml}
<?xml version="1.0"?>
<launch>

  <!-- Robot state publisher -->
  <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher" respawn="true" output="screen">
    <param name="publish_frequency" type="double" value="50.0" />
    <param name="tf_prefix" type="string" value="" />
  </node>

  <!-- Fake Calibration -->
  <node pkg="rostopic" type="rostopic" name="fake_joint_calibration"
        args="pub /calibrated std_msgs/Bool true" />
  
  <!-- joint_state_controller -->
  <rosparam file="$(find ur_gazebo)/controller/joint_state_controller.yaml" command="load"/>
  
  <node name="joint_state_controller_spawner" pkg="controller_manager" type="controller_manager" args="spawn joint_state_controller" respawn="false" output="screen"/>
  
  <!-- get the robot position [Own Script]-->
  <node name="ur10_1_robot_pose" pkg="four_arm_no_moveit_gazebo" type="ur10_1_robot_pose.py" respawn="true" />

  <!-- send the arms commands [Own Script]-->
  <node name="ur10_1_cmd_ik_trajectory_pub" pkg="four_arm_no_moveit_gazebo" type="ur10_1_pub_ik_trajectory.py" respawn="true" />

  <!-- send the gripper commands [Own Script]-->
  <node name="ur10_1_cmd_gripper_value_pub" pkg="four_arm_no_moveit_gazebo" type="ur10_1_pub_gripper_cmd.py" respawn="true" />

  <!-- get the robot position [Own Script]-->
  <node name="ur10_2_robot_pose" pkg="four_arm_no_moveit_gazebo" type="ur10_2_robot_pose.py" respawn="true" />

  <!-- send the arms commands [Own Script]-->
  <node name="ur10_2_cmd_ik_trajectory_pub" pkg="four_arm_no_moveit_gazebo" type="ur10_2_pub_ik_trajectory.py" respawn="true" />

  <!-- send the gripper commands [Own Script]-->
  <node name="ur10_2_cmd_gripper_value_pub" pkg="four_arm_no_moveit_gazebo" type="ur10_2_pub_gripper_cmd.py" respawn="true" />

  <!-- get the robot position [Own Script]-->
  <node name="ur10_3_robot_pose" pkg="four_arm_no_moveit_gazebo" type="ur10_3_robot_pose.py" respawn="true" />

  <!-- send the arms commands [Own Script]-->
  <node name="ur10_3_cmd_ik_trajectory_pub" pkg="four_arm_no_moveit_gazebo" type="ur10_3_pub_ik_trajectory.py" respawn="true" />

  <!-- send the gripper commands [Own Script]-->
  <node name="ur10_3_cmd_gripper_value_pub" pkg="four_arm_no_moveit_gazebo" type="ur10_3_pub_gripper_cmd.py" respawn="true" />

  <!-- get the robot position [Own Script]-->
  <node name="ur10_4_robot_pose" pkg="four_arm_no_moveit_gazebo" type="ur10_4_robot_pose.py" respawn="true" />

  <!-- send the arms commands [Own Script]-->
  <node name="ur10_4_cmd_ik_trajectory_pub" pkg="four_arm_no_moveit_gazebo" type="ur10_4_pub_ik_trajectory.py" respawn="true" />

  <!-- send the gripper commands [Own Script]-->
  <node name="ur10_4_cmd_gripper_value_pub" pkg="four_arm_no_moveit_gazebo" type="ur10_4_pub_gripper_cmd.py" respawn="true" />
</launch>
```

* *~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_gazebo/scripts/ur10_1_pub_gripper_cmd.py*
```{C}
#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory
import roslib
roslib.load_manifest('four_arm_no_moveit_gazebo')
import sys

class PubGripperCmd():
    def __init__(self):
        self.trajectory_gripper_cmd = JointTrajectory()
        self.namenode = "ur10_1_pub_gripper_control"
        self.cmd_gripper_pub = rospy.Publisher('/ur10_1_gripper/command', JointTrajectory, queue_size=10)
        self.gripper_control_sub = rospy.Subscriber('/ur10_1_pub_gripper_control', JointTrajectory, self.update_cmd)

    # Se puede optimizar dejando esta tarea a unos wokers y solo se procesaria el resultado final.
    # Deben estar ordenados y desechar resultados viejos con respecto a un resultado mas reciente.
    def update_cmd(self, data):
        #global trajectory_gripper_cmd
        self.trajectory_gripper_cmd = data
        #print("update")
        #print(data)

    def publisher_gripper_cmd(self):
        rate = rospy.Rate(10) # 10hz  
        while not rospy.is_shutdown():
            #print("publishing")
            self.trajectory_gripper_cmd.header.stamp = rospy.Time.now()
            self.cmd_gripper_pub.publish(self.trajectory_gripper_cmd)
            #print(trajectory_gripper_cmd)
            rate.sleep()

    def callGripperCmdService(self):
        rospy.init_node(self.namenode, anonymous=True)
        try:
            self.publisher_gripper_cmd()
        except rospy.ROSInterruptException:
            pass

if __name__ == "__main__":
    sm = PubGripperCmd()
    sm.callGripperCmdService()
```

* *~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_gazebo/scripts/ur10_1_pub_ik_trajectory.py*
```{C}
#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory
import roslib
roslib.load_manifest('four_arm_no_moveit_gazebo')
import sys

class PubIkTrajectory():
    def __init__(self):
        self.ik_trajectory = JointTrajectory()
        self.namenode = "ur10_1_pub_ik_trajectory"
        self.cmd_pose_pub = rospy.Publisher('/ur10_1_arm_controller/command', JointTrajectory, queue_size=10)
        self.trajectory_sub = rospy.Subscriber('/ur10_1_pub_ik_trajectory', JointTrajectory, self.update_trajectory)

    # Se puede optimizar dejando esta tarea a unos wokers y solo se procesaria el resultado final.
    # Deben estar ordenados y desechar resultados viejos con respecto a un resultado mas reciente.
    def update_trajectory(self, data):
        #global ik_trajectory
        self.ik_trajectory = data
        #print("update")
        #print(data)

    def publisher_trajectory(self):
        rate = rospy.Rate(10) # 10hz  
        while not rospy.is_shutdown():
            #print("publishing")
            self.ik_trajectory.header.stamp = rospy.Time.now()
            self.cmd_pose_pub.publish(self.ik_trajectory)
            #print(ik_trajectory)
            rate.sleep()

    def callIkTrajectoryService(self):
        rospy.init_node(self.namenode, anonymous=True)
        try:
            self.publisher_trajectory()
        except rospy.ROSInterruptException:
            pass

if __name__ == "__main__":
    sm = PubIkTrajectory()
    sm.callIkTrajectoryService()
```

* *~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_gazebo/scripts/ur10_1_robot_pose.py*
```{C}
#!/usr/bin/env python  
import roslib
roslib.load_manifest('four_arm_no_moveit_gazebo')
import rospy, sys
import math
import tf
from geometry_msgs.msg import Point, Pose, Quaternion

class RobotPose():
    def __init__(self):
        self.namenode = "ur10_1_robot_pose"
        self.robot_pose_pub = rospy.Publisher('/ur10_1_robot_pose', Pose, queue_size=10)

    def callRobotPoseService(self):
        # wait for model to exist
        rospy.init_node(self.namenode)
        listener = tf.TransformListener()
    
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                (trans,rot) = listener.lookupTransform('/ur10_1_base_link', '/ur10_1_ee_link', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            #print trans
            #print rot
            rate.sleep()

            # setting initial pose
            robot_pose = Pose()
            robot_pose.position.x = trans[0]
            robot_pose.position.y = trans[1]
            robot_pose.position.z = trans[2]
            # convert rpy to quaternion for Pose message
            #tmpq = tft.quaternion_from_euler(self.initial_rpy[0],self.initial_rpy[1],self.initial_rpy[2])
            q = Quaternion(rot[0],rot[1],rot[2],rot[3])
            robot_pose.orientation = q
            self.robot_pose_pub.publish(robot_pose)
            print robot_pose
            

if __name__ == "__main__":
    sm = RobotPose()
    sm.callRobotPoseService()
```
* *~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_gazebo/scripts/ur10_2_pub_gripper_cmd.py*
```{C}
#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory
import roslib
roslib.load_manifest('four_arm_no_moveit_gazebo')
import sys

class PubGripperCmd():
    def __init__(self):
        self.trajectory_gripper_cmd = JointTrajectory()
        self.namenode = "ur10_2_pub_gripper_control"
        self.cmd_gripper_pub = rospy.Publisher('/ur10_2_gripper/command', JointTrajectory, queue_size=10)
        self.gripper_control_sub = rospy.Subscriber('/ur10_2_pub_gripper_control', JointTrajectory, self.update_cmd)

    # Se puede optimizar dejando esta tarea a unos wokers y solo se procesaria el resultado final.
    # Deben estar ordenados y desechar resultados viejos con respecto a un resultado mas reciente.
    def update_cmd(self, data):
        #global trajectory_gripper_cmd
        self.trajectory_gripper_cmd = data
        #print("update")
        #print(data)


    def publisher_gripper_cmd(self):
        rate = rospy.Rate(10) # 10hz  
        while not rospy.is_shutdown():
            #print("publishing")
            self.trajectory_gripper_cmd.header.stamp = rospy.Time.now()
            self.cmd_gripper_pub.publish(self.trajectory_gripper_cmd)
            #print(trajectory_gripper_cmd)
            rate.sleep()

    def callGripperCmdService(self):
        rospy.init_node(self.namenode, anonymous=True)
        try:
            self.publisher_gripper_cmd()
        except rospy.ROSInterruptException:
            pass

if __name__ == "__main__":
    sm = PubGripperCmd()
    sm.callGripperCmdService()
```

* *~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_gazebo/scripts/ur10_2_pub_ik_trajectory.py*
```{C}
#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory
import roslib
roslib.load_manifest('four_arm_no_moveit_gazebo')
import sys

class PubIkTrajectory():
    def __init__(self):
        self.ik_trajectory = JointTrajectory()
        self.namenode = "ur10_2_pub_ik_trajectory"
        self.cmd_pose_pub = rospy.Publisher('/ur10_2_arm_controller/command', JointTrajectory, queue_size=10)
        self.trajectory_sub = rospy.Subscriber('/ur10_2_pub_ik_trajectory', JointTrajectory, self.update_trajectory)

    # Se puede optimizar dejando esta tarea a unos wokers y solo se procesaria el resultado final.
    # Deben estar ordenados y desechar resultados viejos con respecto a un resultado mas reciente.
    def update_trajectory(self, data):
        #global ik_trajectory
        self.ik_trajectory = data
        #print("update")
        #print(data)

    def publisher_trajectory(self):
        rate = rospy.Rate(10) # 10hz  
        while not rospy.is_shutdown():
            #print("publishing")
            self.ik_trajectory.header.stamp = rospy.Time.now()
            self.cmd_pose_pub.publish(self.ik_trajectory)
            #print(ik_trajectory)
            rate.sleep()

    def callIkTrajectoryService(self):
        rospy.init_node(self.namenode, anonymous=True)
        try:
            self.publisher_trajectory()
        except rospy.ROSInterruptException:
            pass

if __name__ == "__main__":
    sm = PubIkTrajectory()
    sm.callIkTrajectoryService()
```

* *~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_gazebo/scripts/ur10_2_robot_pose.py*
```{C}
#!/usr/bin/env python  
import roslib
roslib.load_manifest('four_arm_no_moveit_gazebo')
import rospy, sys
import math
import tf
from geometry_msgs.msg import Point, Pose, Quaternion

class RobotPose():
    def __init__(self):
        self.namenode = "ur10_2_robot_pose"
        self.robot_pose_pub = rospy.Publisher('/ur10_2_robot_pose', Pose, queue_size=10)

    def callRobotPoseService(self):
        # wait for model to exist
        rospy.init_node(self.namenode)
        listener = tf.TransformListener()

        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                (trans,rot) = listener.lookupTransform('/ur10_2_base_link', '/ur10_2_ee_link', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            #print trans
            #print rot
            rate.sleep()


            # setting initial pose
            robot_pose = Pose()
            robot_pose.position.x = trans[0]
            robot_pose.position.y = trans[1]
            robot_pose.position.z = trans[2]
            # convert rpy to quaternion for Pose message
            #tmpq = tft.quaternion_from_euler(self.initial_rpy[0],self.initial_rpy[1],self.initial_rpy[2])
            q = Quaternion(rot[0],rot[1],rot[2],rot[3])
            robot_pose.orientation = q
            self.robot_pose_pub.publish(robot_pose)
            print robot_pose
            
if __name__ == "__main__":
    sm = RobotPose()
    sm.callRobotPoseService()
```

* *~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_gazebo/scripts/ur10_3_pub_gripper_cmd.py*
```{C}
#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory
import roslib
roslib.load_manifest('four_arm_no_moveit_gazebo')
import sys

class PubGripperCmd():
    def __init__(self):
        self.trajectory_gripper_cmd = JointTrajectory()
        self.namenode = "ur10_3_pub_gripper_control"
        self.cmd_gripper_pub = rospy.Publisher('/ur10_3_gripper/command', JointTrajectory, queue_size=10)
        self.gripper_control_sub = rospy.Subscriber('/ur10_3_pub_gripper_control', JointTrajectory, self.update_cmd)

    # Se puede optimizar dejando esta tarea a unos wokers y solo se procesaria el resultado final.
    # Deben estar ordenados y desechar resultados viejos con respecto a un resultado mas reciente.
    def update_cmd(self, data):
        #global trajectory_gripper_cmd
        self.trajectory_gripper_cmd = data
        #print("update")
        #print(data)


    def publisher_gripper_cmd(self):
        rate = rospy.Rate(10) # 10hz  
        while not rospy.is_shutdown():
            #print("publishing")
            self.trajectory_gripper_cmd.header.stamp = rospy.Time.now()
            self.cmd_gripper_pub.publish(self.trajectory_gripper_cmd)
            #print(trajectory_gripper_cmd)
            rate.sleep()

    def callGripperCmdService(self):
        rospy.init_node(self.namenode, anonymous=True)
        try:
            self.publisher_gripper_cmd()
        except rospy.ROSInterruptException:
            pass

if __name__ == "__main__":
    sm = PubGripperCmd()
    sm.callGripperCmdService()
```

* *~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_gazebo/scripts/ur10_3_pub_ik_trajectory.py*
```{C}
#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory
import roslib
roslib.load_manifest('four_arm_no_moveit_gazebo')
import sys

class PubIkTrajectory():
    def __init__(self):
        self.ik_trajectory = JointTrajectory()
        self.namenode = "ur10_3_pub_ik_trajectory"
        self.cmd_pose_pub = rospy.Publisher('/ur10_3_arm_controller/command', JointTrajectory, queue_size=10)
        self.trajectory_sub = rospy.Subscriber('/ur10_3_pub_ik_trajectory', JointTrajectory, self.update_trajectory)

    # Se puede optimizar dejando esta tarea a unos wokers y solo se procesaria el resultado final.
    # Deben estar ordenados y desechar resultados viejos con respecto a un resultado mas reciente.
    def update_trajectory(self, data):
        #global ik_trajectory
        self.ik_trajectory = data
        #print("update")
        #print(data)

    def publisher_trajectory(self):
        rate = rospy.Rate(10) # 10hz  
        while not rospy.is_shutdown():
            #print("publishing")
            self.ik_trajectory.header.stamp = rospy.Time.now()
            self.cmd_pose_pub.publish(self.ik_trajectory)
            #print(ik_trajectory)
            rate.sleep()

    def callIkTrajectoryService(self):
        rospy.init_node(self.namenode, anonymous=True)
        try:
            self.publisher_trajectory()
        except rospy.ROSInterruptException:
            pass

if __name__ == "__main__":
    sm = PubIkTrajectory()
    sm.callIkTrajectoryService()
```

* *~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_gazebo/scripts/ur10_3_robot_pose.py*
```{C}
#!/usr/bin/env python  
import roslib
roslib.load_manifest('four_arm_no_moveit_gazebo')
import rospy, sys
import math
import tf
from geometry_msgs.msg import Point, Pose, Quaternion

class RobotPose():
    def __init__(self):
        self.namenode = "ur10_3_robot_pose"
        self.robot_pose_pub = rospy.Publisher('/ur10_3_robot_pose', Pose, queue_size=10)

    def callRobotPoseService(self):
        # wait for model to exist
        rospy.init_node(self.namenode)
        listener = tf.TransformListener()

        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                (trans,rot) = listener.lookupTransform('/ur10_3_base_link', '/ur10_3_ee_link', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            #print trans
            #print rot
            rate.sleep()


            # setting initial pose
            robot_pose = Pose()
            robot_pose.position.x = trans[0]
            robot_pose.position.y = trans[1]
            robot_pose.position.z = trans[2]
            # convert rpy to quaternion for Pose message
            #tmpq = tft.quaternion_from_euler(self.initial_rpy[0],self.initial_rpy[1],self.initial_rpy[2])
            q = Quaternion(rot[0],rot[1],rot[2],rot[3])
            robot_pose.orientation = q
            self.robot_pose_pub.publish(robot_pose)
            print robot_pose
            
if __name__ == "__main__":
    sm = RobotPose()
    sm.callRobotPoseService()
```

* *~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_gazebo/scripts/ur10_4_pub_gripper_cmd.py*
```{C}
#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory
import roslib
roslib.load_manifest('four_arm_no_moveit_gazebo')
import sys

class PubGripperCmd():
    def __init__(self):
        self.trajectory_gripper_cmd = JointTrajectory()
        self.namenode = "ur10_4_pub_gripper_control"
        self.cmd_gripper_pub = rospy.Publisher('/ur10_4_gripper/command', JointTrajectory, queue_size=10)
        self.gripper_control_sub = rospy.Subscriber('/ur10_4_pub_gripper_control', JointTrajectory, self.update_cmd)

    # Se puede optimizar dejando esta tarea a unos wokers y solo se procesaria el resultado final.
    # Deben estar ordenados y desechar resultados viejos con respecto a un resultado mas reciente.
    def update_cmd(self, data):
        #global trajectory_gripper_cmd
        self.trajectory_gripper_cmd = data
        #print("update")
        #print(data)


    def publisher_gripper_cmd(self):
        rate = rospy.Rate(10) # 10hz  
        while not rospy.is_shutdown():
            #print("publishing")
            self.trajectory_gripper_cmd.header.stamp = rospy.Time.now()
            self.cmd_gripper_pub.publish(self.trajectory_gripper_cmd)
            #print(trajectory_gripper_cmd)
            rate.sleep()

    def callGripperCmdService(self):
        rospy.init_node(self.namenode, anonymous=True)
        try:
            self.publisher_gripper_cmd()
        except rospy.ROSInterruptException:
            pass

if __name__ == "__main__":
    sm = PubGripperCmd()
    sm.callGripperCmdService()
```

* *~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_gazebo/scripts/ur10_4_pub_ik_trajectory.py*
```{C}
#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory
import roslib
roslib.load_manifest('four_arm_no_moveit_gazebo')
import sys

class PubIkTrajectory():
    def __init__(self):
        self.ik_trajectory = JointTrajectory()
        self.namenode = "ur10_4_pub_ik_trajectory"
        self.cmd_pose_pub = rospy.Publisher('/ur10_4_arm_controller/command', JointTrajectory, queue_size=10)
        self.trajectory_sub = rospy.Subscriber('/ur10_4_pub_ik_trajectory', JointTrajectory, self.update_trajectory)

    # Se puede optimizar dejando esta tarea a unos wokers y solo se procesaria el resultado final.
    # Deben estar ordenados y desechar resultados viejos con respecto a un resultado mas reciente.
    def update_trajectory(self, data):
        #global ik_trajectory
        self.ik_trajectory = data
        #print("update")
        #print(data)

    def publisher_trajectory(self):
        rate = rospy.Rate(10) # 10hz  
        while not rospy.is_shutdown():
            #print("publishing")
            self.ik_trajectory.header.stamp = rospy.Time.now()
            self.cmd_pose_pub.publish(self.ik_trajectory)
            #print(ik_trajectory)
            rate.sleep()

    def callIkTrajectoryService(self):
        rospy.init_node(self.namenode, anonymous=True)
        try:
            self.publisher_trajectory()
        except rospy.ROSInterruptException:
            pass

if __name__ == "__main__":
    sm = PubIkTrajectory()
    sm.callIkTrajectoryService()
```
* *~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_gazebo/scripts/ur10_4_robot_pose.py*
```{C}
#!/usr/bin/env python  
import roslib
roslib.load_manifest('four_arm_no_moveit_gazebo')
import rospy, sys
import math
import tf
from geometry_msgs.msg import Point, Pose, Quaternion

class RobotPose():
    def __init__(self):
        self.namenode = "ur10_4_robot_pose"
        self.robot_pose_pub = rospy.Publisher('/ur10_4_robot_pose', Pose, queue_size=10)

    def callRobotPoseService(self):
        # wait for model to exist
        rospy.init_node(self.namenode)
        listener = tf.TransformListener()

        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                (trans,rot) = listener.lookupTransform('/ur10_4_base_link', '/ur10_4_ee_link', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            #print trans
            #print rot
            rate.sleep()


            # setting initial pose
            robot_pose = Pose()
            robot_pose.position.x = trans[0]
            robot_pose.position.y = trans[1]
            robot_pose.position.z = trans[2]
            # convert rpy to quaternion for Pose message
            #tmpq = tft.quaternion_from_euler(self.initial_rpy[0],self.initial_rpy[1],self.initial_rpy[2])
            q = Quaternion(rot[0],rot[1],rot[2],rot[3])
            robot_pose.orientation = q
            self.robot_pose_pub.publish(robot_pose)
            print robot_pose
            
if __name__ == "__main__":
    sm = RobotPose()
    sm.callRobotPoseService()
```



### Configuración del directorio descripción
Siguiendo la misma línea, se crea un nuevo paquete y se copia los directorios del proyecto anterior para su posterior modificación.

```{bash}
cd ~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit
catkin_create_pkg four_arm_no_moveit_description rospy
```
En el directorio creado para description, se copiara del directorio de *one_arm_no_moveit_description*, las carpetas *launch* y *urdf*.
```{bash}
cd ~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_description
cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_description/launch .
cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_description/urdf .
```
Ligera modificación el el fichero *~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_descriptio/launch/ur10_upload.launch*:
```{xml}
<?xml version="1.0"?>
<launch>
  <arg name="limited" default="false" doc="If true, limits joint range [-PI, PI] on all joints." />
  <arg name="transmission_hw_interface" default="hardware_interface/PositionJointInterface" />

  <param unless="$(arg limited)" name="robot_description" command="$(find xacro)/xacro --inorder '$(find four_arm_no_moveit_description)/urdf/ur10_robot.urdf.xacro' transmission_hw_interface:=$(arg transmission_hw_interface)" />
  <param if="$(arg limited)" name="robot_description" command="$(find xacro)/xacro --inorder '$(find four_arm_no_moveit_description)/urdf/ur10_joint_limited_robot.urdf.xacro' transmission_hw_interface:=$(arg transmission_hw_interface)" />
</launch>
```
 Después hay que modificar los ficheros *.urdf*, *ur10_joint_limited_robot.urdf.xacro* y *ur10_robot.urdf.xacro*:
 
 * Modificando el fichero *~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_description/urdf/ur10_joint_limited_robot.urdf.xacro* con:
 ```{xml}
<?xml version="1.0"?>
<robot xmlns:xacro="http://wiki.ros.org/xacro"
       name="ur10" >

  <xacro:arg name="transmission_hw_interface" default="hardware_interface/PositionJointInterface"/>

  <!-- common stuff -->
  <xacro:include filename="$(find ur_description)/urdf/common.gazebo.xacro" />

  <!-- ur10 -->
  <xacro:include filename="$(find ur_description)/urdf/ur10.urdf.xacro" />

  <!-- robotiq_85_gripper [Para robot simulación en Gazebo] -->
  <xacro:include filename="$(find robotiq_85_description)/urdf/robotiq_85_gripper.urdf.xacro" />

  <xacro:include filename="$(find four_arm_no_moveit_description)/urdf/gzplugin_grasp_fix.urdf.xacro"/>

  <!-- arm -->
  <xacro:ur10_robot prefix="ur10_1_" joint_limited="true"
    shoulder_pan_lower_limit="${-pi}" shoulder_pan_upper_limit="${pi}"
    shoulder_lift_lower_limit="${-pi}" shoulder_lift_upper_limit="${pi}"
    elbow_joint_lower_limit="${-pi}" elbow_joint_upper_limit="${pi}"
    wrist_1_lower_limit="${-pi}" wrist_1_upper_limit="${pi}"
    wrist_2_lower_limit="${-pi}" wrist_2_upper_limit="${pi}"
    wrist_3_lower_limit="${-pi}" wrist_3_upper_limit="${pi}"
    transmission_hw_interface="$(arg transmission_hw_interface)"
/>

  <!-- Limita los movimientos del brazo, es importante la pose inicial
    shoulder_pan: rotacion des de a base, 6*pi, para que no haga rotaciones raras
    shoulder_lift: de -180 a 0, impide que el brazo se mueva bajo la mesa
  -->
  <!--xacro:ur10_robot prefix="" joint_limited="true"
    shoulder_pan_lower_limit="${0}" shoulder_pan_upper_limit="${4*pi}"
    shoulder_lift_lower_limit="${-pi}" shoulder_lift_upper_limit="${0}"
    elbow_joint_lower_limit="${-pi}" elbow_joint_upper_limit="${pi}"
    wrist_1_lower_limit="${-pi}" wrist_1_upper_limit="${pi}"
    wrist_2_lower_limit="${-pi}" wrist_2_upper_limit="${pi}"
    wrist_3_lower_limit="${-pi}" wrist_3_upper_limit="${pi}"
    transmission_hw_interface="$(arg transmission_hw_interface)"
/-->

  <link name="world" />

  <joint name="ur10_1_world_joint" type="fixed">
    <parent link="world" />
    <child link = "ur10_1_base_link" />
    <origin xyz="0.6 -0.6 1.1" rpy="0.0 0.0 0.0" />
  </joint>

  <!-- gripper -->
  <xacro:robotiq_85_gripper prefix="ur10_1_" parent="ur10_1_ee_link" >
      <origin xyz="0 0 0" rpy="0 0 0"/>
  </xacro:robotiq_85_gripper>

  <xacro:gzplugin_grasp_fix prefix="ur10_1_" prefix2="ur10_2_" prefix3="ur10_3_" prefix4="ur10_4_"/>
  <!--xacro:gzplugin_grasp_fix prefix="ur10_1_"/-->

  <!-- arm -->
  <xacro:ur10_robot prefix="ur10_2_" joint_limited="true"
    shoulder_pan_lower_limit="${-pi}" shoulder_pan_upper_limit="${pi}"
    shoulder_lift_lower_limit="${-pi}" shoulder_lift_upper_limit="${pi}"
    elbow_joint_lower_limit="${-pi}" elbow_joint_upper_limit="${pi}"
    wrist_1_lower_limit="${-pi}" wrist_1_upper_limit="${pi}"
    wrist_2_lower_limit="${-pi}" wrist_2_upper_limit="${pi}"
    wrist_3_lower_limit="${-pi}" wrist_3_upper_limit="${pi}"
    transmission_hw_interface="$(arg transmission_hw_interface)" />

  <joint name="ur10_2_world_joint" type="fixed">
    <parent link="world" />
    <child link = "ur10_2_base_link" />
    <origin xyz="0.6 1.38 1.1" rpy="0.0 0.0 0.0" />
  </joint>

  <!-- gripper -->
  <xacro:robotiq_85_gripper prefix="ur10_2_" parent="ur10_2_ee_link" >
      <origin xyz="0 0 0" rpy="0 0 0"/>
  </xacro:robotiq_85_gripper>


  <!-- arm -->
  <xacro:ur10_robot prefix="ur10_3_" joint_limited="true"
    shoulder_pan_lower_limit="${-pi}" shoulder_pan_upper_limit="${pi}"
    shoulder_lift_lower_limit="${-pi}" shoulder_lift_upper_limit="${pi}"
    elbow_joint_lower_limit="${-pi}" elbow_joint_upper_limit="${pi}"
    wrist_1_lower_limit="${-pi}" wrist_1_upper_limit="${pi}"
    wrist_2_lower_limit="${-pi}" wrist_2_upper_limit="${pi}"
    wrist_3_lower_limit="${-pi}" wrist_3_upper_limit="${pi}"
    transmission_hw_interface="$(arg transmission_hw_interface)" />

  <joint name="ur10_3_world_joint" type="fixed">
    <parent link="world" />
    <child link = "ur10_3_base_link" />
    <origin xyz="0.6 3.36 1.1" rpy="0.0 0.0 0.0" />
  </joint>

  <!-- gripper -->
  <xacro:robotiq_85_gripper prefix="ur10_3_" parent="ur10_3_ee_link" >
      <origin xyz="0 0 0" rpy="0 0 0"/>
  </xacro:robotiq_85_gripper>


    <!-- arm -->
  <xacro:ur10_robot prefix="ur10_4_" joint_limited="true"
    shoulder_pan_lower_limit="${-pi}" shoulder_pan_upper_limit="${pi}"
    shoulder_lift_lower_limit="${-pi}" shoulder_lift_upper_limit="${pi}"
    elbow_joint_lower_limit="${-pi}" elbow_joint_upper_limit="${pi}"
    wrist_1_lower_limit="${-pi}" wrist_1_upper_limit="${pi}"
    wrist_2_lower_limit="${-pi}" wrist_2_upper_limit="${pi}"
    wrist_3_lower_limit="${-pi}" wrist_3_upper_limit="${pi}"
    transmission_hw_interface="$(arg transmission_hw_interface)" />

  <joint name="ur10_4_world_joint" type="fixed">
    <parent link="world" />
    <child link = "ur10_4_base_link" />
    <origin xyz="0.6 5.34 1.1" rpy="0.0 0.0 0.0" />
  </joint>

  <!-- gripper -->
  <xacro:robotiq_85_gripper prefix="ur10_4_" parent="ur10_4_ee_link" >
      <origin xyz="0 0 0" rpy="0 0 0"/>
  </xacro:robotiq_85_gripper>

  <!--xacro:gzplugin_grasp_fix prefix="ur10_2_"/-->

</robot>

 ```
 
 
 * Modificando el fichero *~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_description/urdf/ur10_robot.urdf.xacro* con:
 ```{xml}
<?xml version="1.0"?>
<robot xmlns:xacro="http://wiki.ros.org/xacro"
       name="ur10" >

  <xacro:arg name="transmission_hw_interface" default="hardware_interface/PositionJointInterface"/>

  <!-- common stuff -->
  <xacro:include filename="$(find ur_description)/urdf/common.gazebo.xacro" />

  <!-- ur10 -->
  <xacro:include filename="$(find ur_description)/urdf/ur10.urdf.xacro" />

  <!-- robotiq_2f_85_gripper [Para robot real?] -->
  <!--xacro:include filename="$(find robotiq_2f_85_gripper_visualization)/urdf/robotiq_arg2f_85_model_macro.xacro" /-->

  <!-- robotiq_85_gripper [Para robot simulación en Gazebo] -->
  <xacro:include filename="$(find robotiq_85_description)/urdf/robotiq_85_gripper.urdf.xacro" />

  <xacro:include filename="$(find four_arm_no_moveit_description)/urdf/gzplugin_grasp_fix.urdf.xacro"/>

  <!-- arm -->
  <xacro:ur10_robot prefix="ur10_1_" joint_limited="false"
    transmission_hw_interface="$(arg transmission_hw_interface)"
  />

  <!-- gripper -->
  <!--xacro:robotiq_arg2f_85 prefix="">
    <parent link="wrist_3_link"/>
    <origin xyz="0 0 0" rpy="0 0 0" />
  </xacro:robotiq_arg2f_85-->
  <!--xacro:robotiq_arg2f_85 prefix="" /-->

  <link name="world" />

  <joint name="ur10_1_world_joint" type="fixed">
    <parent link="world" />
    <child link = "ur10_1_base_link" />
    <origin xyz="0.6 -0.6 1.1" rpy="0.0 0.0 0.0" />
  </joint>

  <!-- gripper -->
  <xacro:robotiq_85_gripper prefix="ur10_1_" parent="ur10_1_ee_link" >
      <origin xyz="0 0 0" rpy="0 0 0"/>
  </xacro:robotiq_85_gripper>

  <xacro:gzplugin_grasp_fix prefix="ur10_1_" prefix2="ur10_2_"/>

  <!--joint name="ee_gripper_joint" type="fixed">
    <parent link="wrist_3_link" />
    <child link = "robotiq_arg2f_base_link" />
    <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0" />
  </joint-->

  <!-- arm -->
  <xacro:gzplugin_grasp_fix prefix="ur10_1_" prefix2="ur10_2_" prefix3="ur10_3_" prefix4="ur10_4_"/>

  <joint name="ur10_2_world_joint" type="fixed">
    <parent link="world" />
    <child link = "ur10_2_base_link" />
    <origin xyz="0.6 1.38 1.1" rpy="0.0 0.0 0.0" />
  </joint>

  <!-- gripper -->
  <xacro:robotiq_85_gripper prefix="ur10_2_" parent="ur10_2_ee_link" >
      <origin xyz="0 0 0" rpy="0 0 0"/>
  </xacro:robotiq_85_gripper>

    <!-- arm -->
  <xacro:ur10_robot prefix="ur10_3_" joint_limited="false"
    transmission_hw_interface="$(arg transmission_hw_interface)" />

  <joint name="ur10_3_world_joint" type="fixed">
    <parent link="world" />
    <child link = "ur10_3_base_link" />
    <origin xyz="0.6 3.36 1.1" rpy="0.0 0.0 0.0" />
  </joint>

  <!-- gripper -->
  <xacro:robotiq_85_gripper prefix="ur10_3_" parent="ur10_3_ee_link" >
      <origin xyz="0 0 0" rpy="0 0 0"/>
  </xacro:robotiq_85_gripper>


    <!-- arm -->
  <xacro:ur10_robot prefix="ur10_4_" joint_limited="false"
    transmission_hw_interface="$(arg transmission_hw_interface)" />

  <joint name="ur10_4_world_joint" type="fixed">
    <parent link="world" />
    <child link = "ur10_4_base_link" />
    <origin xyz="0.6 5.34 1.1" rpy="0.0 0.0 0.0" />
  </joint>

  <!-- gripper -->
  <xacro:robotiq_85_gripper prefix="ur10_4_" parent="ur10_4_ee_link" >
      <origin xyz="0 0 0" rpy="0 0 0"/>
  </xacro:robotiq_85_gripper>

</robot>
 ```

 ### Pruebas de pick and place con lo implementado
 Siguiendo el mismo procedimiento que en los apartados anteriores, se va a crear el paquete para gazebo, y copiar el contenido de la solución anterior para su posterior modificación:
```{bash}
cd ~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit
catkin_create_pkg four_arm_no_moveit_manipulator rospy
```

En el directorio creado, se copiara del directorio de *one_arm_no_moveit_manipulator*, la carpeta *scripts*.
```{bash}
cd ~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_manipulator
cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_manipulator/scripts .
```

En esta carpeta solamente hay que modificarlo para cada robot:
* *~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_manipulator/ur10_1_robot_manipulator.py*
```{C}
#!/usr/bin/env python

import sys
import copy
import rospy
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import tf.transformations as tf
from geometry_msgs.msg import Pose, Quaternion
from kinematics_utils import *

class CmdTrajectory():
    def __init__(self):
        self.send_trajectory_pub = rospy.Publisher('/ur10_1_pub_ik_trajectory', JointTrajectory, queue_size=10)
        self.send_gripper_cmd_pub = rospy.Publisher('/ur10_1_pub_gripper_control', JointTrajectory, queue_size=10)
        self.current_robot_pose = Pose()
        self.robot_pose_sub = rospy.Subscriber('/ur10_1_robot_pose', Pose, self.update_current_pose)
        self.robot_pose_updated = False

    def send_gripper_cmd(self, gripper_distance):
        gripper = JointTrajectory()
        gripper.header.stamp=rospy.Time.now()
        gripper.header.frame_id = "/ur10_1_ee_link"    
        gripper.joint_names = ['ur10_1_robotiq_85_left_knuckle_joint']
        
        points = JointTrajectoryPoint()
        points.positions = [gripper_distance]
        points.time_from_start = rospy.Duration.from_sec(0.4)
        gripper.points.append(points)
        self.send_gripper_cmd_pub.publish(gripper)
        print('\033[93m[' + str(gripper_distance) + ']\033[0m')

    def pick_place(self):
        # Obtener el primer cubo
        self.send_trajectory(0.29, 0.632, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(0.29, 0.775, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(0.29, 0.775, 0.15, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(0.29, 0.775, 0.08, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
    
        # Primer cubo
        self.send_gripper_cmd(0.43)
        rospy.sleep(4)
    
        # Enviar el cubo a la caja
        self.send_trajectory(0.28, 0.775, 0.15, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(-0.70, 0.775, 0.15, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(-0.70, 0.6, 0.15, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(10)
        self.send_gripper_cmd(0.0)
        rospy.sleep(4)
    
        # Obtener el segundo cubo
        self.send_trajectory(0.29, 0.632, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(0.9, 0.632, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(4)
        self.send_trajectory(0.9, 0.632, 0.2, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(0.9, 0.5, 0.2, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(3)
        self.send_trajectory(0.9, 0.5, 0.08, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(5)
    
        # Segundo cubo
        self.send_gripper_cmd(0.43)
        rospy.sleep(4)
    
        # Enviar el cubo a la caja
        self.send_trajectory(0.28, 0.775, 0.15, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(-0.70, 0.775, 0.15, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(-0.70, 0.6, 0.15, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(10)
        self.send_gripper_cmd(0.0)
        rospy.sleep(4)
    
        # Obtener el tercer cubo
        self.send_trajectory(0.29, 0.632, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(0.675, 0.632, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(4)
        self.send_trajectory(0.675, 0.632, 0.2, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(0.675, 0.88, 0.2, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(3)
        self.send_trajectory(0.675, 0.88, 0.1, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(3)
        self.send_trajectory(0.675, 0.88, 0.08, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(5)
    
        # Tercer cubo
        self.send_gripper_cmd(0.43)
        rospy.sleep(4)
    
        # Enviar el cubo a la caja
        self.send_trajectory(0.28, 0.775, 0.15, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(-0.70, 0.775, 0.15, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(-0.70, 0.6, 0.15, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(10)
        self.send_gripper_cmd(0.0)
        rospy.sleep(4)

        # Posicion inicial del brazo
        cmd.send_gripper_cmd(0.0)
        cmd.send_trajectory(-0.24, 0.632, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(7)

    def update_current_pose(self, pose):
        #self.current_robot_pose.position.x = (-1 * pose.position.x)
        #self.current_robot_pose.position.y = (-1 * pose.position.y)
        self.current_robot_pose.position.x = pose.position.x
        self.current_robot_pose.position.y = pose.position.y
        self.current_robot_pose.position.z = pose.position.z
        self.current_robot_pose.orientation.x = pose.orientation.x
        self.current_robot_pose.orientation.y = pose.orientation.y
        self.current_robot_pose.orientation.z = pose.orientation.z
        self.current_robot_pose.orientation.w = pose.orientation.w
        self.robot_pose_updated = True

    def set_init_pose(self, pos_x, pos_y, pos_z, rot_x, rot_y, rot_z):
        position = JointTrajectory()
        position.header.stamp=rospy.Time.now()
        position.header.frame_id = "/ur10_1_base_link"    
        position.joint_names = ['ur10_1_shoulder_pan_joint','ur10_1_shoulder_lift_joint','ur10_1_elbow_joint',
                          'ur10_1_wrist_1_joint','ur10_1_wrist_2_joint','ur10_1_wrist_3_joint']
        
        rcs = [pos_x, pos_y, pos_z, rot_x, rot_y, rot_z]

        points = JointTrajectoryPoint()
        points.positions = rcs
        points.time_from_start = rospy.Duration.from_sec(5)
        position.points.append(points)
        self.send_trajectory_pub.publish(position)

    def send_trajectory(self, pos_x, pos_y, pos_z, rot_x, rot_y, rot_z, rot_w):
        position = JointTrajectory()
        position.header.stamp=rospy.Time.now()
        position.header.frame_id = "/ur10_1_base_link"    
        position.joint_names = ['ur10_1_shoulder_pan_joint','ur10_1_shoulder_lift_joint','ur10_1_elbow_joint',
                          'ur10_1_wrist_1_joint','ur10_1_wrist_2_joint','ur10_1_wrist_3_joint']
        
        rate = rospy.Rate(10)
        while not self.robot_pose_updated:
            rate.sleep()

        #print self.current_robot_pose

        (roll, pitch, yaw) = tf.euler_from_quaternion([
                                            self.current_robot_pose.orientation.x,
                                            self.current_robot_pose.orientation.y,
                                            self.current_robot_pose.orientation.z,
                                            self.current_robot_pose.orientation.w])

        rcs = [ self.current_robot_pose.position.x,
                self.current_robot_pose.position.y,
                self.current_robot_pose.position.z,
                roll, pitch, yaw]

        #print rcs
        #array_pos = fwd_kin(self.current_robot_pose, 'r', 'n')
        #print(cartesian_pos)

        ps = Pose()
        ps.position.x = pos_x
        ps.position.y = pos_y
        ps.position.z = pos_z
        ps.orientation.x = rot_x
        ps.orientation.y = rot_y
        ps.orientation.z = rot_z
        ps.orientation.w = rot_w
    
        #state = []
    
        #sol = inv_kin(ps, array_pos)
        #print(sol)

        
        points = JointTrajectoryPoint()
        try:
            points.positions = inv_kin(ps, rcs)
        except Exception:
            print('\033[91m[ Singularidad, valores:' + str(ps.position.x) + ', ' + str(ps.position.y) + ', ' + str(ps.position.z) + ']\033[0m')
        
        duration = max([abs(points.positions[0])-abs(rcs[0]), abs(points.positions[1])-abs(rcs[1]), abs(points.positions[2])-abs(rcs[2])])
        print duration

        points.time_from_start = rospy.Duration.from_sec(duration)
        position.points.append(points)
        self.send_trajectory_pub.publish(position)
        #state = sol
        #rospy.sleep(0.1)
        self.robot_pose_updated = False
        print points.positions
        print('\033[93m[' + str(ps.position.x) + ', ' + str(ps.position.y) + ', ' + str(ps.position.z) + ']\033[0m')
        
if __name__ == '__main__':
    rospy.init_node('ur10_1_robot_manipulator', anonymous=True)
    cmd = CmdTrajectory()
    rpy = tf.quaternion_from_euler(-3.12, 0.0, 1.62)
    print rpy
    #[-0.68945825 -0.72424496  0.00781949  0.00744391]
    #cmd.send_trajectory(-0.6, -0.16, 0.62, rpy[0], rpy[1], rpy[2], rpy[3])
    
    # Posicion inicial del brazo
    cmd.set_init_pose(2.176, -1.518, -1.671, -1.511, 1.589, -1.014)

    cmd.send_gripper_cmd(0.0)
    cmd.send_trajectory(-0.24, 0.632, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
    rospy.sleep(4)

    # Ejemplo de movmiento no deseado
    #cmd.send_trajectory(-0.30, 0.300, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
    #rospy.sleep(4)
    #cmd.send_trajectory(-0.40, 0.0, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
    #rospy.sleep(4)
    #cmd.send_trajectory(-0.80, -0.3, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
    #rospy.sleep(4)
    
    cmd.pick_place()
```

* *~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_manipulator/ur10_2_robot_manipulator.py*
```{C}
#!/usr/bin/env python

import sys
import copy
import rospy
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import tf.transformations as tf
from geometry_msgs.msg import Pose, Quaternion
from kinematics_utils import *

class CmdTrajectory():
    def __init__(self):
        self.send_trajectory_pub = rospy.Publisher('/ur10_2_pub_ik_trajectory', JointTrajectory, queue_size=10)
        self.send_gripper_cmd_pub = rospy.Publisher('/ur10_2_pub_gripper_control', JointTrajectory, queue_size=10)
        self.current_robot_pose = Pose()
        self.robot_pose_sub = rospy.Subscriber('/ur10_2_robot_pose', Pose, self.update_current_pose)
        self.robot_pose_updated = False

    def send_gripper_cmd(self, gripper_distance):
        gripper = JointTrajectory()
        gripper.header.stamp=rospy.Time.now()
        gripper.header.frame_id = "/ur10_2_ee_link"    
        gripper.joint_names = ['ur10_2_robotiq_85_left_knuckle_joint']
        
        points = JointTrajectoryPoint()
        points.positions = [gripper_distance]
        points.time_from_start = rospy.Duration.from_sec(0.4)
        gripper.points.append(points)
        self.send_gripper_cmd_pub.publish(gripper)
        print('\033[93m[' + str(gripper_distance) + ']\033[0m')

    def pick_place(self):
        # Obtener el primer cubo
        self.send_trajectory(0.29, 0.632, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(0.29, 0.775, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(0.29, 0.775, 0.15, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(0.29, 0.775, 0.08, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
    
        # Primer cubo
        self.send_gripper_cmd(0.43)
        rospy.sleep(4)
    
        # Enviar el cubo a la caja
        self.send_trajectory(0.28, 0.775, 0.15, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(-0.70, 0.775, 0.15, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(-0.70, 0.6, 0.15, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(10)
        self.send_gripper_cmd(0.0)
        rospy.sleep(4)
    
        # Obtener el segundo cubo
        self.send_trajectory(0.29, 0.632, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(0.9, 0.632, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(4)
        self.send_trajectory(0.9, 0.632, 0.2, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(0.9, 0.5, 0.2, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(3)
        self.send_trajectory(0.9, 0.5, 0.08, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(5)
    
        # Segundo cubo
        self.send_gripper_cmd(0.43)
        rospy.sleep(4)
    
        # Enviar el cubo a la caja
        self.send_trajectory(0.28, 0.775, 0.15, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(-0.70, 0.775, 0.15, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(-0.70, 0.6, 0.15, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(10)
        self.send_gripper_cmd(0.0)
        rospy.sleep(4)
    
        # Obtener el tercer cubo
        self.send_trajectory(0.29, 0.632, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(0.675, 0.632, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(4)
        self.send_trajectory(0.675, 0.632, 0.2, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(0.675, 0.88, 0.2, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(3)
        self.send_trajectory(0.675, 0.88, 0.1, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(3)
        self.send_trajectory(0.675, 0.88, 0.08, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(5)
    
        # Tercer cubo
        self.send_gripper_cmd(0.43)
        rospy.sleep(4)
    
        # Enviar el cubo a la caja
        self.send_trajectory(0.28, 0.775, 0.15, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(-0.70, 0.775, 0.15, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(-0.70, 0.6, 0.15, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(10)
        self.send_gripper_cmd(0.0)
        rospy.sleep(4)

        # Posicion inicial del brazo
        cmd.send_gripper_cmd(0.0)
        cmd.send_trajectory(-0.24, 0.632, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(7)

    def update_current_pose(self, pose):
        #self.current_robot_pose.position.x = (-1 * pose.position.x)
        #self.current_robot_pose.position.y = (-1 * pose.position.y)
        self.current_robot_pose.position.x = pose.position.x
        self.current_robot_pose.position.y = pose.position.y
        self.current_robot_pose.position.z = pose.position.z
        self.current_robot_pose.orientation.x = pose.orientation.x
        self.current_robot_pose.orientation.y = pose.orientation.y
        self.current_robot_pose.orientation.z = pose.orientation.z
        self.current_robot_pose.orientation.w = pose.orientation.w
        self.robot_pose_updated = True

    def set_init_pose(self, pos_x, pos_y, pos_z, rot_x, rot_y, rot_z):
        position = JointTrajectory()
        position.header.stamp=rospy.Time.now()
        position.header.frame_id = "/ur10_2_base_link"    
        position.joint_names = ['ur10_2_shoulder_pan_joint','ur10_2_shoulder_lift_joint','ur10_2_elbow_joint',
                          'ur10_2_wrist_1_joint','ur10_2_wrist_2_joint','ur10_2_wrist_3_joint']
        
        rcs = [pos_x, pos_y, pos_z, rot_x, rot_y, rot_z]

        points = JointTrajectoryPoint()
        points.positions = rcs
        points.time_from_start = rospy.Duration.from_sec(5)
        position.points.append(points)
        self.send_trajectory_pub.publish(position)

    def send_trajectory(self, pos_x, pos_y, pos_z, rot_x, rot_y, rot_z, rot_w):
        position = JointTrajectory()
        position.header.stamp=rospy.Time.now()
        position.header.frame_id = "/ur10_2_base_link"    
        position.joint_names = ['ur10_2_shoulder_pan_joint','ur10_2_shoulder_lift_joint','ur10_2_elbow_joint',
                          'ur10_2_wrist_1_joint','ur10_2_wrist_2_joint','ur10_2_wrist_3_joint']
        
        rate = rospy.Rate(10)
        while not self.robot_pose_updated:
            rate.sleep()

        #print self.current_robot_pose

        (roll, pitch, yaw) = tf.euler_from_quaternion([
                                            self.current_robot_pose.orientation.x,
                                            self.current_robot_pose.orientation.y,
                                            self.current_robot_pose.orientation.z,
                                            self.current_robot_pose.orientation.w])

        rcs = [ self.current_robot_pose.position.x,
                self.current_robot_pose.position.y,
                self.current_robot_pose.position.z,
                roll, pitch, yaw]

        #print rcs
        #array_pos = fwd_kin(self.current_robot_pose, 'r', 'n')
        #print(cartesian_pos)

        ps = Pose()
        ps.position.x = pos_x
        ps.position.y = pos_y
        ps.position.z = pos_z
        ps.orientation.x = rot_x
        ps.orientation.y = rot_y
        ps.orientation.z = rot_z
        ps.orientation.w = rot_w
    
        #state = []
    
        #sol = inv_kin(ps, array_pos)
        #print(sol)

        
        points = JointTrajectoryPoint()
        try:
            points.positions = inv_kin(ps, rcs)
        except Exception:
            print('\033[91m[ Singularidad, valores:' + str(ps.position.x) + ', ' + str(ps.position.y) + ', ' + str(ps.position.z) + ']\033[0m')
        
        duration = max([abs(points.positions[0])-abs(rcs[0]), abs(points.positions[1])-abs(rcs[1]), abs(points.positions[2])-abs(rcs[2])])
        print duration

        points.time_from_start = rospy.Duration.from_sec(duration)
        position.points.append(points)
        self.send_trajectory_pub.publish(position)
        #state = sol
        #rospy.sleep(0.1)
        self.robot_pose_updated = False
        print points.positions
        print('\033[93m[' + str(ps.position.x) + ', ' + str(ps.position.y) + ', ' + str(ps.position.z) + ']\033[0m')
        
if __name__ == '__main__':
    rospy.init_node('ur10_2_robot_manipulator', anonymous=True)
    cmd = CmdTrajectory()
    rpy = tf.quaternion_from_euler(-3.12, 0.0, 1.62)
    print rpy
    #[-0.68945825 -0.72424496  0.00781949  0.00744391]
    #cmd.send_trajectory(-0.6, -0.16, 0.62, rpy[0], rpy[1], rpy[2], rpy[3])
    
    # Posicion inicial del brazo
    cmd.set_init_pose(2.176, -1.518, -1.671, -1.511, 1.589, -1.014)

    cmd.send_gripper_cmd(0.0)
    cmd.send_trajectory(-0.24, 0.632, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
    rospy.sleep(4)

    # Ejemplo de movmiento no deseado
    #cmd.send_trajectory(-0.30, 0.300, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
    #rospy.sleep(4)
    #cmd.send_trajectory(-0.40, 0.0, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
    #rospy.sleep(4)
    #cmd.send_trajectory(-0.80, -0.3, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
    #rospy.sleep(4)
    
    cmd.pick_place()
```

* *~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_manipulator/ur10_3_robot_manipulator.py*
```{C}
#!/usr/bin/env python

import sys
import copy
import rospy
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import tf.transformations as tf
from geometry_msgs.msg import Pose, Quaternion
from kinematics_utils import *

class CmdTrajectory():
    def __init__(self):
        self.send_trajectory_pub = rospy.Publisher('/ur10_3_pub_ik_trajectory', JointTrajectory, queue_size=10)
        self.send_gripper_cmd_pub = rospy.Publisher('/ur10_3_pub_gripper_control', JointTrajectory, queue_size=10)
        self.current_robot_pose = Pose()
        self.robot_pose_sub = rospy.Subscriber('/ur10_3_robot_pose', Pose, self.update_current_pose)
        self.robot_pose_updated = False

    def send_gripper_cmd(self, gripper_distance):
        gripper = JointTrajectory()
        gripper.header.stamp=rospy.Time.now()
        gripper.header.frame_id = "/ur10_3_ee_link"    
        gripper.joint_names = ['ur10_3_robotiq_85_left_knuckle_joint']
        
        points = JointTrajectoryPoint()
        points.positions = [gripper_distance]
        points.time_from_start = rospy.Duration.from_sec(0.4)
        gripper.points.append(points)
        self.send_gripper_cmd_pub.publish(gripper)
        print('\033[93m[' + str(gripper_distance) + ']\033[0m')

    def pick_place(self):
        # Obtener el primer cubo
        self.send_trajectory(0.29, 0.632, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(0.29, 0.775, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(0.29, 0.775, 0.15, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(0.29, 0.775, 0.08, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
    
        # Primer cubo
        self.send_gripper_cmd(0.43)
        rospy.sleep(4)
    
        # Enviar el cubo a la caja
        self.send_trajectory(0.28, 0.775, 0.15, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(-0.70, 0.775, 0.15, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(-0.70, 0.6, 0.15, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(10)
        self.send_gripper_cmd(0.0)
        rospy.sleep(4)
    
        # Obtener el segundo cubo
        self.send_trajectory(0.29, 0.632, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(0.9, 0.632, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(4)
        self.send_trajectory(0.9, 0.632, 0.2, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(0.9, 0.5, 0.2, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(3)
        self.send_trajectory(0.9, 0.5, 0.08, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(5)
    
        # Segundo cubo
        self.send_gripper_cmd(0.43)
        rospy.sleep(4)
    
        # Enviar el cubo a la caja
        self.send_trajectory(0.28, 0.775, 0.15, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(-0.70, 0.775, 0.15, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(-0.70, 0.6, 0.15, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(10)
        self.send_gripper_cmd(0.0)
        rospy.sleep(4)
    
        # Obtener el tercer cubo
        self.send_trajectory(0.29, 0.632, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(0.675, 0.632, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(4)
        self.send_trajectory(0.675, 0.632, 0.2, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(0.675, 0.88, 0.2, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(3)
        self.send_trajectory(0.675, 0.88, 0.1, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(3)
        self.send_trajectory(0.675, 0.88, 0.08, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(5)
    
        # Tercer cubo
        self.send_gripper_cmd(0.43)
        rospy.sleep(4)
    
        # Enviar el cubo a la caja
        self.send_trajectory(0.28, 0.775, 0.15, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(-0.70, 0.775, 0.15, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(-0.70, 0.6, 0.15, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(10)
        self.send_gripper_cmd(0.0)
        rospy.sleep(4)

        # Posicion inicial del brazo
        cmd.send_gripper_cmd(0.0)
        cmd.send_trajectory(-0.24, 0.632, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(7)

    def update_current_pose(self, pose):
        #self.current_robot_pose.position.x = (-1 * pose.position.x)
        #self.current_robot_pose.position.y = (-1 * pose.position.y)
        self.current_robot_pose.position.x = pose.position.x
        self.current_robot_pose.position.y = pose.position.y
        self.current_robot_pose.position.z = pose.position.z
        self.current_robot_pose.orientation.x = pose.orientation.x
        self.current_robot_pose.orientation.y = pose.orientation.y
        self.current_robot_pose.orientation.z = pose.orientation.z
        self.current_robot_pose.orientation.w = pose.orientation.w
        self.robot_pose_updated = True

    def set_init_pose(self, pos_x, pos_y, pos_z, rot_x, rot_y, rot_z):
        position = JointTrajectory()
        position.header.stamp=rospy.Time.now()
        position.header.frame_id = "/ur10_3_base_link"    
        position.joint_names = ['ur10_3_shoulder_pan_joint','ur10_3_shoulder_lift_joint','ur10_3_elbow_joint',
                          'ur10_3_wrist_1_joint','ur10_3_wrist_2_joint','ur10_3_wrist_3_joint']
        
        rcs = [pos_x, pos_y, pos_z, rot_x, rot_y, rot_z]

        points = JointTrajectoryPoint()
        points.positions = rcs
        points.time_from_start = rospy.Duration.from_sec(5)
        position.points.append(points)
        self.send_trajectory_pub.publish(position)

    def send_trajectory(self, pos_x, pos_y, pos_z, rot_x, rot_y, rot_z, rot_w):
        position = JointTrajectory()
        position.header.stamp=rospy.Time.now()
        position.header.frame_id = "/ur10_3_base_link"    
        position.joint_names = ['ur10_3_shoulder_pan_joint','ur10_3_shoulder_lift_joint','ur10_3_elbow_joint',
                          'ur10_3_wrist_1_joint','ur10_3_wrist_2_joint','ur10_3_wrist_3_joint']
        
        rate = rospy.Rate(10)
        while not self.robot_pose_updated:
            rate.sleep()

        #print self.current_robot_pose

        (roll, pitch, yaw) = tf.euler_from_quaternion([
                                            self.current_robot_pose.orientation.x,
                                            self.current_robot_pose.orientation.y,
                                            self.current_robot_pose.orientation.z,
                                            self.current_robot_pose.orientation.w])

        rcs = [ self.current_robot_pose.position.x,
                self.current_robot_pose.position.y,
                self.current_robot_pose.position.z,
                roll, pitch, yaw]

        #print rcs
        #array_pos = fwd_kin(self.current_robot_pose, 'r', 'n')
        #print(cartesian_pos)

        ps = Pose()
        ps.position.x = pos_x
        ps.position.y = pos_y
        ps.position.z = pos_z
        ps.orientation.x = rot_x
        ps.orientation.y = rot_y
        ps.orientation.z = rot_z
        ps.orientation.w = rot_w
    
        #state = []
    
        #sol = inv_kin(ps, array_pos)
        #print(sol)

        
        points = JointTrajectoryPoint()
        try:
            points.positions = inv_kin(ps, rcs)
        except Exception:
            print('\033[91m[ Singularidad, valores:' + str(ps.position.x) + ', ' + str(ps.position.y) + ', ' + str(ps.position.z) + ']\033[0m')
        
        duration = max([abs(points.positions[0])-abs(rcs[0]), abs(points.positions[1])-abs(rcs[1]), abs(points.positions[2])-abs(rcs[2])])
        print duration

        points.time_from_start = rospy.Duration.from_sec(duration)
        position.points.append(points)
        self.send_trajectory_pub.publish(position)
        #state = sol
        #rospy.sleep(0.1)
        self.robot_pose_updated = False
        print points.positions
        print('\033[93m[' + str(ps.position.x) + ', ' + str(ps.position.y) + ', ' + str(ps.position.z) + ']\033[0m')
        
if __name__ == '__main__':
    rospy.init_node('ur10_3_robot_manipulator', anonymous=True)
    cmd = CmdTrajectory()
    rpy = tf.quaternion_from_euler(-3.12, 0.0, 1.62)
    print rpy
    #[-0.68945825 -0.72424496  0.00781949  0.00744391]
    #cmd.send_trajectory(-0.6, -0.16, 0.62, rpy[0], rpy[1], rpy[2], rpy[3])
    
    # Posicion inicial del brazo
    cmd.set_init_pose(2.176, -1.518, -1.671, -1.511, 1.589, -1.014)

    cmd.send_gripper_cmd(0.0)
    cmd.send_trajectory(-0.24, 0.632, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
    rospy.sleep(4)

    # Ejemplo de movmiento no deseado
    #cmd.send_trajectory(-0.30, 0.300, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
    #rospy.sleep(4)
    #cmd.send_trajectory(-0.40, 0.0, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
    #rospy.sleep(4)
    #cmd.send_trajectory(-0.80, -0.3, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
    #rospy.sleep(4)
    
    cmd.pick_place()
```

* *~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_manipulator/ur10_4_robot_manipulator.py*
```{C}
#!/usr/bin/env python

import sys
import copy
import rospy
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import tf.transformations as tf
from geometry_msgs.msg import Pose, Quaternion
from kinematics_utils import *

class CmdTrajectory():
    def __init__(self):
        self.send_trajectory_pub = rospy.Publisher('/ur10_4_pub_ik_trajectory', JointTrajectory, queue_size=10)
        self.send_gripper_cmd_pub = rospy.Publisher('/ur10_4_pub_gripper_control', JointTrajectory, queue_size=10)
        self.current_robot_pose = Pose()
        self.robot_pose_sub = rospy.Subscriber('/ur10_4_robot_pose', Pose, self.update_current_pose)
        self.robot_pose_updated = False

    def send_gripper_cmd(self, gripper_distance):
        gripper = JointTrajectory()
        gripper.header.stamp=rospy.Time.now()
        gripper.header.frame_id = "/ur10_4_ee_link"    
        gripper.joint_names = ['ur10_4_robotiq_85_left_knuckle_joint']
        
        points = JointTrajectoryPoint()
        points.positions = [gripper_distance]
        points.time_from_start = rospy.Duration.from_sec(0.4)
        gripper.points.append(points)
        self.send_gripper_cmd_pub.publish(gripper)
        print('\033[93m[' + str(gripper_distance) + ']\033[0m')

    def pick_place(self):
        # Obtener el primer cubo
        self.send_trajectory(0.29, 0.632, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(0.29, 0.775, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(0.29, 0.775, 0.15, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(0.29, 0.775, 0.08, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
    
        # Primer cubo
        self.send_gripper_cmd(0.43)
        rospy.sleep(4)
    
        # Enviar el cubo a la caja
        self.send_trajectory(0.28, 0.775, 0.15, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(-0.70, 0.775, 0.15, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(-0.70, 0.6, 0.15, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(10)
        self.send_gripper_cmd(0.0)
        rospy.sleep(4)
    
        # Obtener el segundo cubo
        self.send_trajectory(0.29, 0.632, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(0.9, 0.632, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(4)
        self.send_trajectory(0.9, 0.632, 0.2, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(0.9, 0.5, 0.2, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(3)
        self.send_trajectory(0.9, 0.5, 0.08, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(5)
    
        # Segundo cubo
        self.send_gripper_cmd(0.43)
        rospy.sleep(4)
    
        # Enviar el cubo a la caja
        self.send_trajectory(0.28, 0.775, 0.15, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(-0.70, 0.775, 0.15, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(-0.70, 0.6, 0.15, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(10)
        self.send_gripper_cmd(0.0)
        rospy.sleep(4)
    
        # Obtener el tercer cubo
        self.send_trajectory(0.29, 0.632, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(0.675, 0.632, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(4)
        self.send_trajectory(0.675, 0.632, 0.2, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(0.675, 0.88, 0.2, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(3)
        self.send_trajectory(0.675, 0.88, 0.1, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(3)
        self.send_trajectory(0.675, 0.88, 0.08, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(5)
    
        # Tercer cubo
        self.send_gripper_cmd(0.43)
        rospy.sleep(4)
    
        # Enviar el cubo a la caja
        self.send_trajectory(0.28, 0.775, 0.15, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(-0.70, 0.775, 0.15, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(2)
        self.send_trajectory(-0.70, 0.6, 0.15, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(10)
        self.send_gripper_cmd(0.0)
        rospy.sleep(4)

        # Posicion inicial del brazo
        cmd.send_gripper_cmd(0.0)
        cmd.send_trajectory(-0.24, 0.632, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(7)

    def update_current_pose(self, pose):
        #self.current_robot_pose.position.x = (-1 * pose.position.x)
        #self.current_robot_pose.position.y = (-1 * pose.position.y)
        self.current_robot_pose.position.x = pose.position.x
        self.current_robot_pose.position.y = pose.position.y
        self.current_robot_pose.position.z = pose.position.z
        self.current_robot_pose.orientation.x = pose.orientation.x
        self.current_robot_pose.orientation.y = pose.orientation.y
        self.current_robot_pose.orientation.z = pose.orientation.z
        self.current_robot_pose.orientation.w = pose.orientation.w
        self.robot_pose_updated = True

    def set_init_pose(self, pos_x, pos_y, pos_z, rot_x, rot_y, rot_z):
        position = JointTrajectory()
        position.header.stamp=rospy.Time.now()
        position.header.frame_id = "/ur10_4_base_link"    
        position.joint_names = ['ur10_4_shoulder_pan_joint','ur10_4_shoulder_lift_joint','ur10_4_elbow_joint',
                          'ur10_4_wrist_1_joint','ur10_4_wrist_2_joint','ur10_4_wrist_3_joint']
        
        rcs = [pos_x, pos_y, pos_z, rot_x, rot_y, rot_z]

        points = JointTrajectoryPoint()
        points.positions = rcs
        points.time_from_start = rospy.Duration.from_sec(5)
        position.points.append(points)
        self.send_trajectory_pub.publish(position)

    def send_trajectory(self, pos_x, pos_y, pos_z, rot_x, rot_y, rot_z, rot_w):
        position = JointTrajectory()
        position.header.stamp=rospy.Time.now()
        position.header.frame_id = "/ur10_4_base_link"    
        position.joint_names = ['ur10_4_shoulder_pan_joint','ur10_4_shoulder_lift_joint','ur10_4_elbow_joint',
                          'ur10_4_wrist_1_joint','ur10_4_wrist_2_joint','ur10_4_wrist_3_joint']
        
        rate = rospy.Rate(10)
        while not self.robot_pose_updated:
            rate.sleep()

        #print self.current_robot_pose

        (roll, pitch, yaw) = tf.euler_from_quaternion([
                                            self.current_robot_pose.orientation.x,
                                            self.current_robot_pose.orientation.y,
                                            self.current_robot_pose.orientation.z,
                                            self.current_robot_pose.orientation.w])

        rcs = [ self.current_robot_pose.position.x,
                self.current_robot_pose.position.y,
                self.current_robot_pose.position.z,
                roll, pitch, yaw]

        #print rcs
        #array_pos = fwd_kin(self.current_robot_pose, 'r', 'n')
        #print(cartesian_pos)

        ps = Pose()
        ps.position.x = pos_x
        ps.position.y = pos_y
        ps.position.z = pos_z
        ps.orientation.x = rot_x
        ps.orientation.y = rot_y
        ps.orientation.z = rot_z
        ps.orientation.w = rot_w
    
        #state = []
    
        #sol = inv_kin(ps, array_pos)
        #print(sol)

        
        points = JointTrajectoryPoint()
        try:
            points.positions = inv_kin(ps, rcs)
        except Exception:
            print('\033[91m[ Singularidad, valores:' + str(ps.position.x) + ', ' + str(ps.position.y) + ', ' + str(ps.position.z) + ']\033[0m')
        
        duration = max([abs(points.positions[0])-abs(rcs[0]), abs(points.positions[1])-abs(rcs[1]), abs(points.positions[2])-abs(rcs[2])])
        print duration

        points.time_from_start = rospy.Duration.from_sec(duration)
        position.points.append(points)
        self.send_trajectory_pub.publish(position)
        #state = sol
        #rospy.sleep(0.1)
        self.robot_pose_updated = False
        print points.positions
        print('\033[93m[' + str(ps.position.x) + ', ' + str(ps.position.y) + ', ' + str(ps.position.z) + ']\033[0m')
        
if __name__ == '__main__':
    rospy.init_node('ur10_4_robot_manipulator', anonymous=True)
    cmd = CmdTrajectory()
    rpy = tf.quaternion_from_euler(-3.12, 0.0, 1.62)
    print rpy
    #[-0.68945825 -0.72424496  0.00781949  0.00744391]
    #cmd.send_trajectory(-0.6, -0.16, 0.62, rpy[0], rpy[1], rpy[2], rpy[3])
    
    # Posicion inicial del brazo
    cmd.set_init_pose(2.176, -1.518, -1.671, -1.511, 1.589, -1.014)

    cmd.send_gripper_cmd(0.0)
    cmd.send_trajectory(-0.24, 0.632, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
    rospy.sleep(4)

    # Ejemplo de movmiento no deseado
    #cmd.send_trajectory(-0.30, 0.300, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
    #rospy.sleep(4)
    #cmd.send_trajectory(-0.40, 0.0, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
    #rospy.sleep(4)
    #cmd.send_trajectory(-0.80, -0.3, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
    #rospy.sleep(4)
    
    cmd.pick_place()
```

Falta arreglar el plugin de gazebo para que pueda agarrar objetos con ambos grippers:
* Fichero *~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_description/urdf/gzplugin_grasp_fix.urdf.xacro*
```{xml}
<?xml version="1.0" encoding="UTF-8"?>
<root 
 xmlns:sensor="http://playerstage.sourceforge.net/gazebo/xmlschema/#sensor"
 xmlns:controller="http://playerstage.sourceforge.net/gazebo/xmlschema/#controller"
 xmlns:interface="http://playerstage.sourceforge.net/gazebo/xmlschema/#interface"
 xmlns:xacro="http://wiki.ros.org/xacro">


<!-- MACRO FOR THE ROBOT ARM ON THE TABLE-->
<xacro:macro name="gzplugin_grasp_fix" params="prefix prefix2 prefix3 prefix4">
    <gazebo>
        <plugin name="gazebo_grasp_fix" filename="libgazebo_grasp_fix.so">
            <!--
            NOTE: The finger tips are linked together with the links before, because they are connected with a
            fixed joint. Gazebo makes one whole link out of this automatically. When listing the 9_*_tip links
            here, they won't be found in the SDF.
            -->
            <arm>
                <arm_name>${prefix}Arm</arm_name>

                <!-- 0.01977<0.0198<0.01999 -->
                <palm_link> ${prefix}robotiq_85_left_inner_knuckle_link </palm_link>
                <gripper_link> ${prefix}robotiq_85_left_finger_tip_link </gripper_link>
                <palm_link> ${prefix}robotiq_85_right_inner_knuckle_link </palm_link>
                <gripper_link> ${prefix}robotiq_85_right_finger_tip_link </gripper_link>
            </arm>

            <arm>
                <arm_name>${prefix2}Arm</arm_name>
                <palm_link> ${prefix2}robotiq_85_left_inner_knuckle_link </palm_link>
                <gripper_link> ${prefix2}robotiq_85_left_finger_tip_link </gripper_link>
                <palm_link> ${prefix2}robotiq_85_right_inner_knuckle_link </palm_link>
                <gripper_link> ${prefix2}robotiq_85_right_finger_tip_link </gripper_link>
            </arm>

            <arm>
                <arm_name>${prefix3}Arm</arm_name>
                <palm_link> ${prefix3}robotiq_85_left_inner_knuckle_link </palm_link>
                <gripper_link> ${prefix3}robotiq_85_left_finger_tip_link </gripper_link>
                <palm_link> ${prefix3}robotiq_85_right_inner_knuckle_link </palm_link>
                <gripper_link> ${prefix3}robotiq_85_right_finger_tip_link </gripper_link>
            </arm>


            <arm>
                <arm_name>${prefix4}Arm</arm_name>
                <palm_link> ${prefix4}robotiq_85_left_inner_knuckle_link </palm_link>
                <gripper_link> ${prefix4}robotiq_85_left_finger_tip_link </gripper_link>
                <palm_link> ${prefix4}robotiq_85_right_inner_knuckle_link </palm_link>
                <gripper_link> ${prefix4}robotiq_85_right_finger_tip_link </gripper_link>
            </arm>

            <forces_angle_tolerance>100</forces_angle_tolerance>
            <update_rate>100</update_rate>
            <grip_count_threshold>10</grip_count_threshold>
            <max_grip_count>20</max_grip_count>
            <release_tolerance>0.005</release_tolerance>
            <!--release_tolerance>0.0198</release_tolerance--> <!-- 0.01977<0.0198<0.01999 -->
            <disable_collisions_on_attach>false</disable_collisions_on_attach>
            <contact_topic>__default_topic__</contact_topic>
        </plugin>
    </gazebo>
</xacro:macro>

</root>
```
 Y finalmente, adecuar el fichero world para que realice las simulaciones en un entorno adecuado:
 * Fichero *~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_gazebo/world/multiarm_bot.world*:
 ```{xml}
<sdf version='1.6'>
  <world name='default'>
    <gravity>0 0 -9.81</gravity>
    <physics name='default_physics' default='0' type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>1</shadows>
    </scene>

    <light name='sun' type='directional'>
      <cast_shadows>1</cast_shadows>
      <pose frame=''>0 0 10 0 -0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    
    <model name='ground_plane'>
      <static>1</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>100</mu>
                <mu2>50</mu2>
              </ode>
              <torsional>
                <ode/>
              </torsional>
            </friction>
            <contact>
              <ode/>
            </contact>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <pose frame=''>0 0 0 0 -0 0</pose>
    </model>

    <model name='table'>
      <static>1</static>
      <link name='link'>
        <collision name='surface'>
          <pose frame=''>0 0 1 0 -0 0</pose>
          <geometry>
            <box>
              <size>1.5 0.8 0.03</size>
            </box>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>0.6</mu>
                <mu2>0.6</mu2>
              </ode>
              <torsional>
                <ode/>
              </torsional>
            </friction>
            <contact>
              <ode/>
            </contact>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='visual1'>
          <pose frame=''>0 0 1 0 -0 0</pose>
          <geometry>
            <box>
              <size>1.5 0.8 0.03</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Wood</name>
            </script>
          </material>
        </visual>
        <collision name='front_left_leg'>
          <pose frame=''>0.68 0.38 0.5 0 -0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.02</radius>
              <length>1</length>
            </cylinder>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='front_left_leg'>
          <pose frame=''>0.68 0.38 0.5 0 -0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.02</radius>
              <length>1</length>
            </cylinder>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
        <collision name='front_right_leg'>
          <pose frame=''>0.68 -0.38 0.5 0 -0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.02</radius>
              <length>1</length>
            </cylinder>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='front_right_leg'>
          <pose frame=''>0.68 -0.38 0.5 0 -0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.02</radius>
              <length>1</length>
            </cylinder>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
        <collision name='back_right_leg'>
          <pose frame=''>-0.68 -0.38 0.5 0 -0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.02</radius>
              <length>1</length>
            </cylinder>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='back_right_leg'>
          <pose frame=''>-0.68 -0.38 0.5 0 -0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.02</radius>
              <length>1</length>
            </cylinder>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
        <collision name='back_left_leg'>
          <pose frame=''>-0.68 0.38 0.5 0 -0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.02</radius>
              <length>1</length>
            </cylinder>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='back_left_leg'>
          <pose frame=''>-0.68 0.38 0.5 0 -0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.02</radius>
              <length>1</length>
            </cylinder>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <pose frame=''>0 -1.38 0 0 -0 0</pose>
    </model>

    <model name='cube1'>
      <pose frame=''>0.3 -1.38 1.012 0 -0 1.56</pose>
      <link name='link'>
        <pose frame=''>0 0 0.025 0 -0 0</pose>
        <inertial>
          <mass>0.0565</mass>
          <inertia>
            <ixx>2.35417e-05</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>2.35417e-05</iyy>
            <iyz>0</iyz>
            <izz>2.35417e-05</izz>
          </inertia>
          <pose frame=''>0 0 0 0 -0 0</pose>
        </inertial>
        <collision name='collision'>
          <geometry>
            <box>
              <size>0.046 0.046 0.046</size>
            </box>
          </geometry>
          <surface>
            <contact>
              <poissons_ratio>0.347</poissons_ratio>
              <elastic_modulus>8.8e+09</elastic_modulus>
              <ode>
                <kp>100000</kp>
                <kd>1</kd>
                <max_vel>1</max_vel>
                <min_depth>0.001</min_depth>
              </ode>
            </contact>
            <friction>
              <torsional>
                <coefficient>10</coefficient>
                <use_patch_radius>0</use_patch_radius>
                <surface_radius>0.05</surface_radius>
                <ode/>
              </torsional>
              <ode/>
            </friction>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>0.046 0.046 0.046</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Wood</name>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>

    <model name='cube2'>
      <pose frame=''>-0.3 -1.1 1.012 0 -0 1.56</pose>
      <link name='link'>
        <pose frame=''>0 0 0.025 0 -0 0</pose>
        <inertial>
          <mass>0.0565</mass>
          <inertia>
            <ixx>2.35417e-05</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>2.35417e-05</iyy>
            <iyz>0</iyz>
            <izz>2.35417e-05</izz>
          </inertia>
          <pose frame=''>0 0 0 0 -0 0</pose>
        </inertial>
        <collision name='collision'>
          <geometry>
            <box>
              <size>0.046 0.046 0.046</size>
            </box>
          </geometry>
          <surface>
            <contact>
              <poissons_ratio>0.347</poissons_ratio>
              <elastic_modulus>8.8e+09</elastic_modulus>
              <ode>
                <kp>100000</kp>
                <kd>1</kd>
                <max_vel>1</max_vel>
                <min_depth>0.001</min_depth>
              </ode>
            </contact>
            <friction>
              <torsional>
                <coefficient>1</coefficient>
                <use_patch_radius>0</use_patch_radius>
                <surface_radius>0.05</surface_radius>
                <ode/>
              </torsional>
              <ode/>
            </friction>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>0.046 0.046 0.046</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/CeilingTiled</name>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>
    <model name='cube3'>
      <pose frame=''>-0.067508 -1.48308 1.012 0 -0 1.56</pose>
      <link name='link'>
        <pose frame=''>0 0 0.025 0 -0 0</pose>
        <inertial>
          <mass>0.0565</mass>
          <inertia>
            <ixx>2.35417e-05</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>2.35417e-05</iyy>
            <iyz>0</iyz>
            <izz>2.35417e-05</izz>
          </inertia>
          <pose frame=''>0 0 0 0 -0 0</pose>
        </inertial>
        <collision name='collision'>
          <geometry>
            <box>
              <size>0.046 0.046 0.046</size>
            </box>
          </geometry>
          <surface>
            <contact>
              <poissons_ratio>0.347</poissons_ratio>
              <elastic_modulus>8.8e+09</elastic_modulus>
              <ode>
                <kp>100000</kp>
                <kd>1</kd>
                <max_vel>1</max_vel>
                <min_depth>0.001</min_depth>
              </ode>
            </contact>
            <friction>
              <torsional>
                <coefficient>1</coefficient>
                <use_patch_radius>0</use_patch_radius>
                <surface_radius>0.05</surface_radius>
                <ode/>
              </torsional>
              <ode/>
            </friction>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>0.046 0.046 0.046</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/WoodPallet</name>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>

    <model name='bin_4_dropping_task'>
      <link name='bin'>
        <pose frame=''>0 0 0.1 0 -0 0</pose>
        <collision name='bin_collision'>
          <geometry>
            <mesh>
              <uri>model://bin_4_dropping_task/meshes/bin_4_dropping_task.dae</uri>
            </mesh>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='bin_visual'>
          <geometry>
            <mesh>
              <uri>model://bin_4_dropping_task/meshes/bin_4_dropping_task.dae</uri>
            </mesh>
          </geometry>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <static>1</static>
      <pose frame=''>1.2229 -1.11492 0 0 -0 0</pose>
    </model>

    <model name='cube4'>
      <pose frame=''>0.3 0.6 1.012 0 -0 1.56</pose>
      <link name='link'>
        <pose frame=''>0 0 0.025 0 -0 0</pose>
        <inertial>
          <mass>0.0565</mass>
          <inertia>
            <ixx>2.35417e-05</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>2.35417e-05</iyy>
            <iyz>0</iyz>
            <izz>2.35417e-05</izz>
          </inertia>
          <pose frame=''>0 0 0 0 -0 0</pose>
        </inertial>
        <collision name='collision'>
          <geometry>
            <box>
              <size>0.046 0.046 0.046</size>
            </box>
          </geometry>
          <surface>
            <contact>
              <poissons_ratio>0.347</poissons_ratio>
              <elastic_modulus>8.8e+09</elastic_modulus>
              <ode>
                <kp>100000</kp>
                <kd>1</kd>
                <max_vel>1</max_vel>
                <min_depth>0.001</min_depth>
              </ode>
            </contact>
            <friction>
              <torsional>
                <coefficient>10</coefficient>
                <use_patch_radius>0</use_patch_radius>
                <surface_radius>0.05</surface_radius>
                <ode/>
              </torsional>
              <ode/>
            </friction>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>0.046 0.046 0.046</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Wood</name>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>

    <model name='cube5'>
      <pose frame=''>-0.3 0.88 1.012 0 -0 1.56</pose>
      <link name='link'>
        <pose frame=''>0 0 0.025 0 -0 0</pose>
        <inertial>
          <mass>0.0565</mass>
          <inertia>
            <ixx>2.35417e-05</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>2.35417e-05</iyy>
            <iyz>0</iyz>
            <izz>2.35417e-05</izz>
          </inertia>
          <pose frame=''>0 0 0 0 -0 0</pose>
        </inertial>
        <collision name='collision'>
          <geometry>
            <box>
              <size>0.046 0.046 0.046</size>
            </box>
          </geometry>
          <surface>
            <contact>
              <poissons_ratio>0.347</poissons_ratio>
              <elastic_modulus>8.8e+09</elastic_modulus>
              <ode>
                <kp>100000</kp>
                <kd>1</kd>
                <max_vel>1</max_vel>
                <min_depth>0.001</min_depth>
              </ode>
            </contact>
            <friction>
              <torsional>
                <coefficient>1</coefficient>
                <use_patch_radius>0</use_patch_radius>
                <surface_radius>0.05</surface_radius>
                <ode/>
              </torsional>
              <ode/>
            </friction>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>0.046 0.046 0.046</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/CeilingTiled</name>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>
    <model name='cube6'>
      <pose frame=''>-0.067508 0.49692 1.012 0 -0 1.56</pose>
      <link name='link'>
        <pose frame=''>0 0 0.025 0 -0 0</pose>
        <inertial>
          <mass>0.0565</mass>
          <inertia>
            <ixx>2.35417e-05</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>2.35417e-05</iyy>
            <iyz>0</iyz>
            <izz>2.35417e-05</izz>
          </inertia>
          <pose frame=''>0 0 0 0 -0 0</pose>
        </inertial>
        <collision name='collision'>
          <geometry>
            <box>
              <size>0.046 0.046 0.046</size>
            </box>
          </geometry>
          <surface>
            <contact>
              <poissons_ratio>0.347</poissons_ratio>
              <elastic_modulus>8.8e+09</elastic_modulus>
              <ode>
                <kp>100000</kp>
                <kd>1</kd>
                <max_vel>1</max_vel>
                <min_depth>0.001</min_depth>
              </ode>
            </contact>
            <friction>
              <torsional>
                <coefficient>1</coefficient>
                <use_patch_radius>0</use_patch_radius>
                <surface_radius>0.05</surface_radius>
                <ode/>
              </torsional>
              <ode/>
            </friction>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>0.046 0.046 0.046</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/WoodPallet</name>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>


    
    <model name='bin_4_dropping_task_0'>
      <link name='bin'>
        <pose frame=''>0 0 0.1 0 -0 0</pose>
        <collision name='bin_collision'>
          <geometry>
            <mesh>
              <uri>model://bin_4_dropping_task/meshes/bin_4_dropping_task.dae</uri>
            </mesh>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='bin_visual'>
          <geometry>
            <mesh>
              <uri>model://bin_4_dropping_task/meshes/bin_4_dropping_task.dae</uri>
            </mesh>
          </geometry>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <static>1</static>
      <pose frame=''>1.2229 0.865080 0 0 -0 0</pose>
    </model>


    

    <model name='<table_0'>
      <static>1</static>
      <link name='link'>
        <collision name='surface'>
          <pose frame=''>0 0 1 0 -0 0</pose>
          <geometry>
            <box>
              <size>1.5 0.8 0.03</size>
            </box>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>0.6</mu>
                <mu2>0.6</mu2>
              </ode>
              <torsional>
                <ode/>
              </torsional>
            </friction>
            <contact>
              <ode/>
            </contact>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='visual1'>
          <pose frame=''>0 0 1 0 -0 0</pose>
          <geometry>
            <box>
              <size>1.5 0.8 0.03</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Wood</name>
            </script>
          </material>
        </visual>
        <collision name='front_left_leg'>
          <pose frame=''>0.68 0.38 0.5 0 -0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.02</radius>
              <length>1</length>
            </cylinder>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='front_left_leg'>
          <pose frame=''>0.68 0.38 0.5 0 -0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.02</radius>
              <length>1</length>
            </cylinder>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
        <collision name='front_right_leg'>
          <pose frame=''>0.68 -0.38 0.5 0 -0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.02</radius>
              <length>1</length>
            </cylinder>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='front_right_leg'>
          <pose frame=''>0.68 -0.38 0.5 0 -0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.02</radius>
              <length>1</length>
            </cylinder>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
        <collision name='back_right_leg'>
          <pose frame=''>-0.68 -0.38 0.5 0 -0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.02</radius>
              <length>1</length>
            </cylinder>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='back_right_leg'>
          <pose frame=''>-0.68 -0.38 0.5 0 -0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.02</radius>
              <length>1</length>
            </cylinder>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
        <collision name='back_left_leg'>
          <pose frame=''>-0.68 0.38 0.5 0 -0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.02</radius>
              <length>1</length>
            </cylinder>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='back_left_leg'>
          <pose frame=''>-0.68 0.38 0.5 0 -0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.02</radius>
              <length>1</length>
            </cylinder>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <pose frame=''>0.0 0.6 0 0 -0 0</pose>
    </model>

        <model name='cube7'>
      <pose frame=''>0.3 2.58 1.012 0 -0 1.56</pose>
      <link name='link'>
        <pose frame=''>0 0 0.025 0 -0 0</pose>
        <inertial>
          <mass>0.0565</mass>
          <inertia>
            <ixx>2.35417e-05</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>2.35417e-05</iyy>
            <iyz>0</iyz>
            <izz>2.35417e-05</izz>
          </inertia>
          <pose frame=''>0 0 0 0 -0 0</pose>
        </inertial>
        <collision name='collision'>
          <geometry>
            <box>
              <size>0.046 0.046 0.046</size>
            </box>
          </geometry>
          <surface>
            <contact>
              <poissons_ratio>0.347</poissons_ratio>
              <elastic_modulus>8.8e+09</elastic_modulus>
              <ode>
                <kp>100000</kp>
                <kd>1</kd>
                <max_vel>1</max_vel>
                <min_depth>0.001</min_depth>
              </ode>
            </contact>
            <friction>
              <torsional>
                <coefficient>10</coefficient>
                <use_patch_radius>0</use_patch_radius>
                <surface_radius>0.05</surface_radius>
                <ode/>
              </torsional>
              <ode/>
            </friction>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>0.046 0.046 0.046</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Wood</name>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>

    <model name='cube8'>
      <pose frame=''>-0.3 2.86 1.012 0 -0 1.56</pose>
      <link name='link'>
        <pose frame=''>0 0 0.025 0 -0 0</pose>
        <inertial>
          <mass>0.0565</mass>
          <inertia>
            <ixx>2.35417e-05</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>2.35417e-05</iyy>
            <iyz>0</iyz>
            <izz>2.35417e-05</izz>
          </inertia>
          <pose frame=''>0 0 0 0 -0 0</pose>
        </inertial>
        <collision name='collision'>
          <geometry>
            <box>
              <size>0.046 0.046 0.046</size>
            </box>
          </geometry>
          <surface>
            <contact>
              <poissons_ratio>0.347</poissons_ratio>
              <elastic_modulus>8.8e+09</elastic_modulus>
              <ode>
                <kp>100000</kp>
                <kd>1</kd>
                <max_vel>1</max_vel>
                <min_depth>0.001</min_depth>
              </ode>
            </contact>
            <friction>
              <torsional>
                <coefficient>1</coefficient>
                <use_patch_radius>0</use_patch_radius>
                <surface_radius>0.05</surface_radius>
                <ode/>
              </torsional>
              <ode/>
            </friction>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>0.046 0.046 0.046</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/CeilingTiled</name>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>
    <model name='cube9'>
      <pose frame=''>-0.067508 2.47692 1.012 0 -0 1.56</pose>
      <link name='link'>
        <pose frame=''>0 0 0.025 0 -0 0</pose>
        <inertial>
          <mass>0.0565</mass>
          <inertia>
            <ixx>2.35417e-05</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>2.35417e-05</iyy>
            <iyz>0</iyz>
            <izz>2.35417e-05</izz>
          </inertia>
          <pose frame=''>0 0 0 0 -0 0</pose>
        </inertial>
        <collision name='collision'>
          <geometry>
            <box>
              <size>0.046 0.046 0.046</size>
            </box>
          </geometry>
          <surface>
            <contact>
              <poissons_ratio>0.347</poissons_ratio>
              <elastic_modulus>8.8e+09</elastic_modulus>
              <ode>
                <kp>100000</kp>
                <kd>1</kd>
                <max_vel>1</max_vel>
                <min_depth>0.001</min_depth>
              </ode>
            </contact>
            <friction>
              <torsional>
                <coefficient>1</coefficient>
                <use_patch_radius>0</use_patch_radius>
                <surface_radius>0.05</surface_radius>
                <ode/>
              </torsional>
              <ode/>
            </friction>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>0.046 0.046 0.046</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/WoodPallet</name>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>


    
    <model name='bin_4_dropping_task_1'>
      <link name='bin'>
        <pose frame=''>0 0 0.1 0 -0 0</pose>
        <collision name='bin_collision'>
          <geometry>
            <mesh>
              <uri>model://bin_4_dropping_task/meshes/bin_4_dropping_task.dae</uri>
            </mesh>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='bin_visual'>
          <geometry>
            <mesh>
              <uri>model://bin_4_dropping_task/meshes/bin_4_dropping_task.dae</uri>
            </mesh>
          </geometry>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <static>1</static>
      <pose frame=''>1.2229 2.84508 0 0 -0 0</pose>
    </model>


    

    <model name='<table_1'>
      <static>1</static>
      <link name='link'>
        <collision name='surface'>
          <pose frame=''>0 0 1 0 -0 0</pose>
          <geometry>
            <box>
              <size>1.5 0.8 0.03</size>
            </box>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>0.6</mu>
                <mu2>0.6</mu2>
              </ode>
              <torsional>
                <ode/>
              </torsional>
            </friction>
            <contact>
              <ode/>
            </contact>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='visual1'>
          <pose frame=''>0 0 1 0 -0 0</pose>
          <geometry>
            <box>
              <size>1.5 0.8 0.03</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Wood</name>
            </script>
          </material>
        </visual>
        <collision name='front_left_leg'>
          <pose frame=''>0.68 0.38 0.5 0 -0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.02</radius>
              <length>1</length>
            </cylinder>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='front_left_leg'>
          <pose frame=''>0.68 0.38 0.5 0 -0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.02</radius>
              <length>1</length>
            </cylinder>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
        <collision name='front_right_leg'>
          <pose frame=''>0.68 -0.38 0.5 0 -0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.02</radius>
              <length>1</length>
            </cylinder>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='front_right_leg'>
          <pose frame=''>0.68 -0.38 0.5 0 -0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.02</radius>
              <length>1</length>
            </cylinder>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
        <collision name='back_right_leg'>
          <pose frame=''>-0.68 -0.38 0.5 0 -0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.02</radius>
              <length>1</length>
            </cylinder>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='back_right_leg'>
          <pose frame=''>-0.68 -0.38 0.5 0 -0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.02</radius>
              <length>1</length>
            </cylinder>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
        <collision name='back_left_leg'>
          <pose frame=''>-0.68 0.38 0.5 0 -0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.02</radius>
              <length>1</length>
            </cylinder>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='back_left_leg'>
          <pose frame=''>-0.68 0.38 0.5 0 -0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.02</radius>
              <length>1</length>
            </cylinder>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <pose frame=''>0.0 2.58 0 0 -0 0</pose>
    </model>

    

    <model name='cube10'>
      <pose frame=''>0.3 4.56 1.012 0 -0 1.56</pose>
      <link name='link'>
        <pose frame=''>0 0 0.025 0 -0 0</pose>
        <inertial>
          <mass>0.0565</mass>
          <inertia>
            <ixx>2.35417e-05</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>2.35417e-05</iyy>
            <iyz>0</iyz>
            <izz>2.35417e-05</izz>
          </inertia>
          <pose frame=''>0 0 0 0 -0 0</pose>
        </inertial>
        <collision name='collision'>
          <geometry>
            <box>
              <size>0.046 0.046 0.046</size>
            </box>
          </geometry>
          <surface>
            <contact>
              <poissons_ratio>0.347</poissons_ratio>
              <elastic_modulus>8.8e+09</elastic_modulus>
              <ode>
                <kp>100000</kp>
                <kd>1</kd>
                <max_vel>1</max_vel>
                <min_depth>0.001</min_depth>
              </ode>
            </contact>
            <friction>
              <torsional>
                <coefficient>10</coefficient>
                <use_patch_radius>0</use_patch_radius>
                <surface_radius>0.05</surface_radius>
                <ode/>
              </torsional>
              <ode/>
            </friction>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>0.046 0.046 0.046</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Wood</name>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>

    <model name='cube11'>
      <pose frame=''>-0.3 4.84 1.012 0 -0 1.56</pose>
      <link name='link'>
        <pose frame=''>0 0 0.025 0 -0 0</pose>
        <inertial>
          <mass>0.0565</mass>
          <inertia>
            <ixx>2.35417e-05</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>2.35417e-05</iyy>
            <iyz>0</iyz>
            <izz>2.35417e-05</izz>
          </inertia>
          <pose frame=''>0 0 0 0 -0 0</pose>
        </inertial>
        <collision name='collision'>
          <geometry>
            <box>
              <size>0.046 0.046 0.046</size>
            </box>
          </geometry>
          <surface>
            <contact>
              <poissons_ratio>0.347</poissons_ratio>
              <elastic_modulus>8.8e+09</elastic_modulus>
              <ode>
                <kp>100000</kp>
                <kd>1</kd>
                <max_vel>1</max_vel>
                <min_depth>0.001</min_depth>
              </ode>
            </contact>
            <friction>
              <torsional>
                <coefficient>1</coefficient>
                <use_patch_radius>0</use_patch_radius>
                <surface_radius>0.05</surface_radius>
                <ode/>
              </torsional>
              <ode/>
            </friction>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>0.046 0.046 0.046</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/CeilingTiled</name>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>
    <model name='cube12'>
      <pose frame=''>-0.067508 4.45692 1.012 0 -0 1.56</pose>
      <link name='link'>
        <pose frame=''>0 0 0.025 0 -0 0</pose>
        <inertial>
          <mass>0.0565</mass>
          <inertia>
            <ixx>2.35417e-05</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>2.35417e-05</iyy>
            <iyz>0</iyz>
            <izz>2.35417e-05</izz>
          </inertia>
          <pose frame=''>0 0 0 0 -0 0</pose>
        </inertial>
        <collision name='collision'>
          <geometry>
            <box>
              <size>0.046 0.046 0.046</size>
            </box>
          </geometry>
          <surface>
            <contact>
              <poissons_ratio>0.347</poissons_ratio>
              <elastic_modulus>8.8e+09</elastic_modulus>
              <ode>
                <kp>100000</kp>
                <kd>1</kd>
                <max_vel>1</max_vel>
                <min_depth>0.001</min_depth>
              </ode>
            </contact>
            <friction>
              <torsional>
                <coefficient>1</coefficient>
                <use_patch_radius>0</use_patch_radius>
                <surface_radius>0.05</surface_radius>
                <ode/>
              </torsional>
              <ode/>
            </friction>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>0.046 0.046 0.046</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/WoodPallet</name>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>


    
    <model name='bin_4_dropping_task_2'>
      <link name='bin'>
        <pose frame=''>0 0 0.1 0 -0 0</pose>
        <collision name='bin_collision'>
          <geometry>
            <mesh>
              <uri>model://bin_4_dropping_task/meshes/bin_4_dropping_task.dae</uri>
            </mesh>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='bin_visual'>
          <geometry>
            <mesh>
              <uri>model://bin_4_dropping_task/meshes/bin_4_dropping_task.dae</uri>
            </mesh>
          </geometry>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <static>1</static>
      <pose frame=''>1.2229 4.82508 0 0 -0 0</pose>
    </model>


    

    <model name='<table_2'>
      <static>1</static>
      <link name='link'>
        <collision name='surface'>
          <pose frame=''>0 0 1 0 -0 0</pose>
          <geometry>
            <box>
              <size>1.5 0.8 0.03</size>
            </box>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>0.6</mu>
                <mu2>0.6</mu2>
              </ode>
              <torsional>
                <ode/>
              </torsional>
            </friction>
            <contact>
              <ode/>
            </contact>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='visual1'>
          <pose frame=''>0 0 1 0 -0 0</pose>
          <geometry>
            <box>
              <size>1.5 0.8 0.03</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Wood</name>
            </script>
          </material>
        </visual>
        <collision name='front_left_leg'>
          <pose frame=''>0.68 0.38 0.5 0 -0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.02</radius>
              <length>1</length>
            </cylinder>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='front_left_leg'>
          <pose frame=''>0.68 0.38 0.5 0 -0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.02</radius>
              <length>1</length>
            </cylinder>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
        <collision name='front_right_leg'>
          <pose frame=''>0.68 -0.38 0.5 0 -0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.02</radius>
              <length>1</length>
            </cylinder>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='front_right_leg'>
          <pose frame=''>0.68 -0.38 0.5 0 -0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.02</radius>
              <length>1</length>
            </cylinder>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
        <collision name='back_right_leg'>
          <pose frame=''>-0.68 -0.38 0.5 0 -0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.02</radius>
              <length>1</length>
            </cylinder>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='back_right_leg'>
          <pose frame=''>-0.68 -0.38 0.5 0 -0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.02</radius>
              <length>1</length>
            </cylinder>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
        <collision name='back_left_leg'>
          <pose frame=''>-0.68 0.38 0.5 0 -0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.02</radius>
              <length>1</length>
            </cylinder>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='back_left_leg'>
          <pose frame=''>-0.68 0.38 0.5 0 -0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.02</radius>
              <length>1</length>
            </cylinder>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <pose frame=''>0.0 4.56 0 0 -0 0</pose>
    </model>


    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose frame=''>4.11691 1.54151 6.92783 0 0.951641 -2.96299</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
      </camera>
    </gui>

    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type='adiabatic'/>
    <wind/>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>0</latitude_deg>
      <longitude_deg>0</longitude_deg>
      <elevation>0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>

    <state world_name='default'>
      <sim_time>39 786000000</sim_time>
      <real_time>48 995378976</real_time>
      <wall_time>1640542535 679198067</wall_time>
      <iterations>39786</iterations>

      <model name='bin_4_dropping_task'>
        <pose frame=''>1.2229 -1.11492 0 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='bin'>
          <pose frame=''>1.2229 -1.11492 0.1 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>

      <model name='bin_4_dropping_task_0'>
        <pose frame=''>1.2229 0.865080 0 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='bin'>
          <pose frame=''>1.2229 0.865080 0.1 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>

      <model name='bin_4_dropping_task_1'>
        <pose frame=''>1.2229 2.84508 0 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='bin'>
          <pose frame=''>1.2229 2.84508 0.1 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>

      <model name='bin_4_dropping_task_2'>
        <pose frame=''>1.2229 4.82508 0 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='bin'>
          <pose frame=''>1.2229 4.82508 0.1 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>

      <model name='cube1'>
        <pose frame=''>0.3 -1.38 1.012 0 -0 1.56</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose frame=''>0.3 -1.38 1.037 0 -0 1.56</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 -0 0 -0 0</acceleration>
          <wrench>0 0 -0 0 -0 0</wrench>
        </link>
      </model>
      
      <model name='cube2'>
        <pose frame=''>-0.3 -1.1 1.012 0 -0 1.56</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose frame=''>-0.3 -1.1 1.037 0 -0 1.56</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <model name='cube3'>
        <pose frame=''>-0.067508 -1.48308 1.012 0 -0 1.56</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose frame=''>-0.067508 -1.48308 1.037 0 -0 1.56</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>-0 -0 0 0 -0 0</acceleration>
          <wrench>-0 -0 0 0 -0 0</wrench>
        </link>
      </model>
      <model name='cube4'>
        <pose frame=''>0.3 0.6 1.012 0 -0 1.56</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose frame=''>0.3 0.6 1.037 0 -0 1.56</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>-0 0 -0 -0 -0 -0</acceleration>
          <wrench>-0 0 -0 0 -0 0</wrench>
        </link>
      </model>
      <model name='cube5'>
        <pose frame=''>-0.3 0.88 1.012 0 -0 1.56</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose frame=''>-0.3 0.88 1.037 0 -0 1.56</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>

      <model name='cube6'>
        <pose frame=''>-0.067508 0.49692 1.012 0 -0 1.56</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose frame=''>-0.067508 0.49692 1.037 0 -0 1.56</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>

      <model name='cube7'>
        <pose frame=''>0.3 2.58 1.012 0 -0 1.56</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose frame=''>0.3 2.58 1.037 0 -0 1.56</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>-0 0 -0 -0 -0 -0</acceleration>
          <wrench>-0 0 -0 0 -0 0</wrench>
        </link>
      </model>
      <model name='cube8'>
        <pose frame=''>-0.3 2.86 1.012 0 -0 1.56</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose frame=''>-0.3 2.86 1.037 0 -0 1.56</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>

      <model name='cube9'>
        <pose frame=''>-0.067508 2.47692 1.012 0 -0 1.56</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose frame=''>-0.067508 2.47692 1.037 0 -0 1.56</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>

      <model name='cube10'>
        <pose frame=''>0.3 4.56 1.012 0 -0 1.56</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose frame=''>0.3 4.56 1.037 0 -0 1.56</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>-0 0 -0 -0 -0 -0</acceleration>
          <wrench>-0 0 -0 0 -0 0</wrench>
        </link>
      </model>
      <model name='cube11'>
        <pose frame=''>-0.3 4.84 1.012 0 -0 1.56</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose frame=''>-0.3 4.84 1.037 0 -0 1.56</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>

      <model name='cube12'>
        <pose frame=''>-0.067508 4.45692 1.012 0 -0 1.56</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose frame=''>-0.067508 4.45692 1.037 0 -0 1.56</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>

      <model name='ground_plane'>
        <pose frame=''>0 0 0 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose frame=''>0 0 0 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>

      <model name='table'>
        <pose frame=''>0 -1.38 0 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose frame=''>0 -1.38 0 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>

      <model name='table_0'>
        <pose frame=''>0.0 0.6 0 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose frame=''>0.0 0.6 0 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>

      <model name='table_1'>
        <pose frame=''>0.0 2.58 0 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose frame=''>0.0 2.58 0 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>

      <model name='table_2'>
        <pose frame=''>0.0 4.56 0 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose frame=''>0.0 4.56 0 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>

      <light name='sun'>
        <pose frame=''>0 0 10 0 -0 0</pose>
      </light>
    </state>
  </world>
</sdf>
 ```
 
 ### Lanzar las las perubas de simulaócin
 Es necesario 5 terminales:
 * Terminal 1:
 ```{bash}
 roslaunch four_arm_no_moveit_gazebo ur10_joint_limited.launch
 ```
 
 * Terminal 2:
 ```{bash}
 rosrun four_arm_no_moveit_manipulator ur10_1_robot_manipulator.py
 ```
 
 * Terminal 3:
 ```{bash}
 rosrun four_arm_no_moveit_manipulator ur10_2_robot_manipulator.py
 ```

 * Terminal 4:
 ```{bash}
 rosrun four_arm_no_moveit_manipulator ur10_3_robot_manipulator.py
 ```
 
 * Terminal 5:
 ```{bash}
 rosrun four_arm_no_moveit_manipulator ur10_4_robot_manipulator.py
 ```