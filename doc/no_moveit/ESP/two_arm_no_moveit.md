<!--- Para dos robots opción A--->
# Instalación y configuración para dos robots UR10 sin MoveIt!

![image](/doc/imgs_md/Diseno-no-moveit-general-dos-cobots-leap-motion.png  "Cargado el modelo URDF del robot UR10")

Se va a realizar la solución para dos robots esta vez, de la misma manera que se ha realizado para uno, pero modificando el contenido de los ficheros adaptándolo para su similación con dos robots.

Las fases que se ven en el esquema son de orientación. Se pueden hacer en el orden que se prefiera, se ha dividido el esquema en fases para mantener un orden y conocer sobre qué elemento del esquema se está trabajando. En este caso se comenzará por la fase 1, seguido de la fase 2 y finalmente se termina con la fase 3. Hay que tener en cuenta que puede existir configuraciones en una fase que pertenece realmente a otra fase, cuando esto suceda se señalará adecuadamente.

## Requisito previo
- Realizar correctamente la instalación de la [configuración base del sistema](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/setup-doc/proyect_setup.md).

## Índice
- [Congiguración Inicial: Configuración para un robot](#setup-inicial)
	- [Fase 1: Configuración del simulador de Gazebo](#fase1)
	- [Fase 2: Configuración del URDF](#fase2)
	- [Fase 3: Implementación de un planificador propio que realiza un `pick & place`](#fase3)
- [Modificaciones: Sistema multirobot compuesto de dos robots](#modificaciones)
	- [Fase 1: Configuración del simulador de Gazebo](#modificaciones1)
	- [Fase 2: Configuración del URDF](#modificaciones2)
	- [Fase 3: Implementación de un planificador propio que realiza un `pick & place`](#modificaciones3)

- [Ejecución de las pruebas](#pruebas)

<a name="fase1">
  <h2>
Fase 1: Configuración del simulador de Gazebo
  </h2>
</a>

### :book: Configuración de Gazebo
Lo primero que hay que hacer es configurar Gazebo y los controladores para que pueda simular adecuadamente los movimientos del cobot. Se crea el paquete `one_arm_no_ moveit_gazebo`, que contendrá toda
la configuración relacionada con Gazebo, entre ellos los controladores. 

Una vez creada el paquete, hay que configurar los controladores que están almacenados en el directorio `controller`. Los controladores se definen en ficheros con extensión *yaml*, para definir estos controladores hay que darles un nombre y definir el tipo del controlador, los joints dinámicos que se quieren controlar, las restricciones que tiene, el ratio de publicación y otras opciones.

A continuación se presenta el contenido de los ficheros de configuración de los controladores, todos estos controladores, en general, siguen la estructura mencionada. La definición de los controladores pueden ser contenidas en un único fichero, lo importante es que en Gazebo los cargue correctamente, se procede a explicar brevemente estos controladores:

- Fichero [arm_controller_ur10.yaml](): En este fichero se define el controlador para el cobot UR10, aquı́ se define el nombre del controlador `arm_controller`, el tipo de controlador position `controllers/JointTrajectoryController`, lo que implica la definición del tipo de mensajes y el formateo adecuado de la información necesaria para comunicarse con éste. Después está el campo `joints`, que es donde se indica qué joints del cobot forma parte del controlador, todos estos joints son dinámicos. El resto de campos no se han tocado, pero hay que mantener la consistencia en cómo se nombran.

- Fichero [joint_state_controller.yaml](): Lo que define este fichero realmente no es un controlador como tal, su función es la de una interfaz que traduce la información de los joints que viene del cobot real y lo traduce a mensajes de tipo `JointState` para después publicarlo. Es fundamental para el correcto funcionamiento, tanto en simulación como con el robot real, forma parte del paquete de ROS *ros_control*.


### :computer: Creación del directorio
```{bash}
cd ~/MultiCobot-UR10-Gripper/src/multirobot
mkdir two_arm_no_moveit
```
### :computer: Puesta en marcha de Gazebo para dos robots
Se va a crear el paquete para gazebo, y copiar el contenido de la solución anterior para su posterior modificación:
```{bash}
cd ~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit
catkin_create_pkg two_arm_no_moveit_gazebo rospy
```
En el directorio creado para gazebo, se copiara del directorio de *one_arm_no_moveit_gazebo*, las carpetas *controller*, *launch*, *models*, *scripts* y *world*.
```{bash}
cd ~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_gazebo
cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/controller .
cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/launch .
cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/models .
cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/scripts .
cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/world .
```

Se compila:
```{bash}
cd ~/MultiCobot-UR10-Gripper
catkin_make
```
### Modificaciones a realizar para simular dos robots en gazebo
Primero, hay que decidir en el *namespace* para cada robot, es decir el nombre de grupo sobre el que agruparemos las configuraciones para cada uno de los robots.

Se comenzará con los controladores:

* Fichero *ur10_1_arm_controller.yaml*
```{bash}
cd ~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_gazebo/controllers
mv arm_controller_ur10.yaml ur10_1_arm_controller.yaml
```

Este fichero será ligeramente modificado por el siguiente contenido:
```yaml
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
Se puede apreciar, que simplemente se ha añadido el prefijo *ur10_1_*, esto permitirá diferencia después a que joints debe enviar los comandos, lo cual implica la modificación del fichero URDF también.

Por ahora se procede a modificar el resto de ficheros así como la adicción del segundo grupo de controladores para el segundo robot que se llevará el prefijo *ur10_2_*.

* Fichero *ur10_1_gripper_controller_robotiq.yaml*
```{bash}
cd ~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_gazebo/controllers
mv gripper_controller_robotiq.yaml ur10_1_gripper_controller_robotiq.yaml
```

Este fichero será ligeramente modificado por el siguiente contenido:
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
```{bash}
cd ~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_gazebo/controllers
cp ur10_1_arm_controller.yaml ur10_2_arm_controller.yaml
```

Este fichero será ligeramente modificado por el siguiente contenido:
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
```{bash}
cd ~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_gazebo/controllers
mv ur10_1_gripper_controller_robotiq.yaml ur10_2_gripper_controller_robotiq.yaml
```

Este fichero será ligeramente modificado por el siguiente contenido:
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
### Modificación del launch file de Gazebo
Hay que modificar ahora el launch file para lanzar los controladores de ambos orbots en gazebo.

```{bash}
cd ~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_gazebo/launch
nano ur10.launch
```

Tras modificarlo, el fichero queda de la siguiente manera:
```{xml}
<?xml version="1.0"?>
<launch>
  <arg name="limited" default="false"  doc="If true, limits joint range [-PI, PI] on all joints." />
  <arg name="paused" default="false" doc="Starts gazebo in paused mode" />
  <arg name="gui" default="true" doc="Starts gazebo gui" />
  
  <arg name="world" default="$(find two_arm_no_moveit_gazebo)/world/multiarm_bot.world" />

  <!-- startup simulated world -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" default="$(arg world)"/>
    <arg name="paused" value="$(arg paused)"/>
    <arg name="gui" value="$(arg gui)"/>
  </include>

  <!-- send robot urdf to param server -->
  <include file="$(find two_arm_no_moveit_description)/launch/ur10_upload.launch">
    <arg name="limited" value="$(arg limited)"/>
  </include>

  <!-- push robot_description to factory and spawn robot in gazebo -->
  <node name="spawn_gazebo_model" pkg="gazebo_ros" type="spawn_model" respawn="false" output="screen" args="-urdf -param robot_description -model robot " />

  <include file="$(find two_arm_no_moveit_gazebo)/launch/controller_utils.launch"/>

  <!-- ur10_1_arm_controller -->
  <rosparam file="$(find two_arm_no_moveit_gazebo)/controller/ur10_1_arm_controller.yaml" command="load"/>
  <node name="ur10_1_arm_controller_spawner" pkg="controller_manager" type="controller_manager" args="spawn ur10_1_arm_controller" respawn="false" output="screen"/>

  <!-- ur10_1_robotiq_85_gripper controller -->
  <rosparam file="$(find two_arm_no_moveit_gazebo)/controller/ur10_1_gripper_controller_robotiq.yaml" command="load"/> 
  <node name="ur10_1_gripper_controller_spawner" pkg="controller_manager" type="spawner" args="ur10_1_gripper" />

    <!-- ur10_2_arm_controller -->
  <rosparam file="$(find two_arm_no_moveit_gazebo)/controller/ur10_2_arm_controller.yaml" command="load"/>
  <node name="ur10_2_arm_controller_spawner" pkg="controller_manager" type="controller_manager" args="spawn ur10_2_arm_controller" respawn="false" output="screen"/>

  <!-- ur10_2_robotiq_85_gripper controller -->
  <rosparam file="$(find two_arm_no_moveit_gazebo)/controller/ur10_2_gripper_controller_robotiq.yaml" command="load"/> 
  <node name="ur10_2_gripper_controller_spawner" pkg="controller_manager" type="spawner" args="ur10_2_gripper" />

  <!-- load other controllers -->
  <node name="ur10_1_ros_control_controller_manager" pkg="controller_manager" type="controller_manager" respawn="false" output="screen" args="load ur10_1_joint_group_position_controller" />

    <!-- load other controllers -->
  <node name="ur10_2_ros_control_controller_manager" pkg="controller_manager" type="controller_manager" respawn="false" output="screen" args="load ur10_2_joint_group_position_controller" />

</launch>
```

```{bash}
cd ~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_gazebo/launch
nano ur10_joint_limited.launch
```

Tras modificarlo, el fichero queda de la siguiente manera:
```{xml}
<?xml version="1.0"?>
<launch>
  <arg name="gui" default="true" doc="Starts gazebo gui" />

  <include file="$(find two_arm_no_moveit_gazebo)/launch/ur10.launch">
    <arg name="limited" value="true"/>
    <arg name="gui" value="$(arg gui)"/>
  </include>

</launch>
```

Falta modificar el fichero que lanza los scripts que se crearon, previamente así como esos ficheros, ya que se han corregido para adaptarse a los namesapces correctamente:

* *~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_gazebo/launch/controller_utils.launch*
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
  <node name="ur10_1_robot_pose" pkg="two_arm_no_moveit_gazebo" type="ur10_1_robot_pose.py" respawn="true" />

  <!-- send the arms commands [Own Script]-->
  <node name="ur10_1_cmd_ik_trajectory_pub" pkg="two_arm_no_moveit_gazebo" type="ur10_1_pub_ik_trajectory.py" respawn="true" />

  <!-- send the gripper commands [Own Script]-->
  <node name="ur10_1_cmd_gripper_value_pub" pkg="two_arm_no_moveit_gazebo" type="ur10_1_pub_gripper_cmd.py" respawn="true" />

    <!-- get the robot position [Own Script]-->
  <node name="ur10_2_robot_pose" pkg="two_arm_no_moveit_gazebo" type="ur10_2_robot_pose.py" respawn="true" />

  <!-- send the arms commands [Own Script]-->
  <node name="ur10_2_cmd_ik_trajectory_pub" pkg="two_arm_no_moveit_gazebo" type="ur10_2_pub_ik_trajectory.py" respawn="true" />

  <!-- send the gripper commands [Own Script]-->
  <node name="ur10_2_cmd_gripper_value_pub" pkg="two_arm_no_moveit_gazebo" type="ur10_2_pub_gripper_cmd.py" respawn="true" />
</launch>

```

* *~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_gazebo/scripts/ur10_1_pub_gripper_cmd.py*
```{C}
#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory
import roslib
roslib.load_manifest('two_arm_no_moveit_gazebo')
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

* *~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_gazebo/scripts/ur10_1_pub_ik_trajectory.py*
```{C}
#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory
import roslib
roslib.load_manifest('two_arm_no_moveit_gazebo')
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

* *~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_gazebo/scripts/ur10_1_robot_pose.py*
```{C}
#!/usr/bin/env python  
import roslib
roslib.load_manifest('two_arm_no_moveit_gazebo')
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
        print self.namenode
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
* *~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_gazebo/scripts/ur10_2_pub_gripper_cmd.py*
```{C}
#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory
import roslib
roslib.load_manifest('two_arm_no_moveit_gazebo')
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

* *~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_gazebo/scripts/ur10_2_pub_ik_trajectory.py*
```{C}
#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory
import roslib
roslib.load_manifest('two_arm_no_moveit_gazebo')
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

* *~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_gazebo/scripts/ur10_2_robot_pose.py*
```{C}
#!/usr/bin/env python  
import roslib
roslib.load_manifest('two_arm_no_moveit_gazebo')
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
        print self.namespace
        print self.namenode
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


### Configuración del directorio descripción
Siguiendo la misma línea, se crea un nuevo paquete y se copia los directorios del proyecto anterior para su posterior modificación.

```{bash}
cd ~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit
catkin_create_pkg two_arm_no_moveit_description rospy
```
En el directorio creado para description, se copiara del directorio de *one_arm_no_moveit_description*, las carpetas *launch* y *urdf*.
```{bash}
cd ~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_description
cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_description/launch .
cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_description/urdf .
```
Ligera modificación el el fichero *~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_descriptio/launch/ur10_upload.launch*:
```{xml}
<?xml version="1.0"?>
<launch>
  <arg name="limited" default="false" doc="If true, limits joint range [-PI, PI] on all joints." />
  <arg name="transmission_hw_interface" default="hardware_interface/PositionJointInterface" />

  <param unless="$(arg limited)" name="robot_description" command="$(find xacro)/xacro --inorder '$(find two_arm_no_moveit_description)/urdf/ur10_robot.urdf.xacro' transmission_hw_interface:=$(arg transmission_hw_interface)" />
  <param if="$(arg limited)" name="robot_description" command="$(find xacro)/xacro --inorder '$(find two_arm_no_moveit_description)/urdf/ur10_joint_limited_robot.urdf.xacro' transmission_hw_interface:=$(arg transmission_hw_interface)" />

</launch>
```
 Después hay que modificar los ficheros *.urdf*, *ur10_joint_limited_robot.urdf.xacro* y *ur10_robot.urdf.xacro*:
 
 * Modificando el fichero *~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_description/urdf/ur10_joint_limited_robot.urdf.xacro* con:
 
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

  <xacro:include filename="$(find two_arm_no_moveit_description)/urdf/gzplugin_grasp_fix.urdf.xacro"/>

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

  <xacro:gzplugin_grasp_fix prefix="ur10_1_" prefix2="ur10_2_"/>
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

  <!--xacro:gzplugin_grasp_fix prefix="ur10_2_"/-->

</robot>
 ```
 
 
 * Modificando el fichero *~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_description/urdf/ur10_robot.urdf.xacro* con:
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

  <xacro:include filename="$(find two_arm_no_moveit_description)/urdf/gzplugin_grasp_fix.urdf.xacro"/>

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
  <xacro:ur10_robot prefix="ur10_2_" joint_limited="false"
    transmission_hw_interface="$(arg transmission_hw_interface)"
  />

  <joint name="ur10_2_world_joint" type="fixed">
    <parent link="world" />
    <child link = "ur10_2_base_link" />
    <origin xyz="0.6 1.38 1.1" rpy="0.0 0.0 0.0" />
  </joint>

  <!-- gripper -->
  <xacro:robotiq_85_gripper prefix="ur10_2_" parent="ur10_2_ee_link" >
      <origin xyz="0 0 0" rpy="0 0 0"/>
  </xacro:robotiq_85_gripper>

</robot>

 ```
 ### Pruebas de pick and place con lo implementado
 Siguiendo el mismo procedimiento que en los apartados anteriores, se va a crear el paquete para gazebo, y copiar el contenido de la solución anterior para su posterior modificación:
```{bash}
cd ~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit
catkin_create_pkg two_arm_no_moveit_manipulator rospy
```

En el directorio creado, se copiara del directorio de *one_arm_no_moveit_manipulator*, la carpeta *scripts*.
```{bash}
cd ~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_manipulator
cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_manipulator/scripts .
```

En esta carpeta solamente hay que modificarlo para cada robot:
- [~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_manipulator/ur10_1_robot_manipulator.py*](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_manipulator/scripts/ur10_1_robot_manipulator.py).
- [~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_manipulator/ur10_2_robot_manipulator.py](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_manipulator/scripts/ur10_2_robot_manipulator.py).

Falta arreglar el plugin de gazebo para que pueda agarrar objetos con ambos grippers:
* Fichero [~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_description/urdf/gzplugin_grasp_fix.urdf.xacro](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_description/urdf/gzplugin_grasp_fix.urdf.xacro).

 Y finalmente, adecuar el fichero world para que realice las simulaciones en un entorno adecuado:
 * Fichero [~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/world/multiarm_bot.world](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_gazebo/world/multiarm_bot.world).
 
 ### Lanzar las las perubas de simulaócin
 Es necesario 3 terminales:
 * Terminal 1:
 ```{bash}
 roslaunch two_arm_no_moveit_gazebo ur10_joint_limited.launch
 ```
 
 * Terminal 2:
 ```{bash}
 rosrun two_arm_no_moveit_manipulator ur10_1_robot_manipulator.py
 ```
 
 * Terminal 3:
 ```{bash}
 rosrun two_arm_no_moveit_manipulator ur10_2_robot_manipulator.py
 ```