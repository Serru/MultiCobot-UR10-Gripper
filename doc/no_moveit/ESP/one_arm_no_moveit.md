<!--- Para un robot  opción A--->
## Instalación y configuración para un único robot UR10 sin MoveIt!

## Requisito previo
- Realizar correctamente la instalación de la [configuración base del sistema](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/setup-doc/proyect_setup.md).

## Índice
- [Fase 1: Configuración del URDF](#fase1)
- [Fase 2: Implementación de un planificador propio](#fase2)
- [Fase 3: Simulación de un `pick & place` en Gazebo](#fase3)
- [Ejecución de las pruebas](#pruebas)

<a name="fase1">
  <h2>
Fase 1: Configuración del URDF
  </h2>
</a>

### :book: Descripción del fichero URDF
El fichero URDF (United Robotics Description Format) modela el cobot utilizando el formato XML el cual será utilizado por las diferentes aplicaciones que ROS necesite, pero principalmente para realizar una simulación del robot modelado.

El fichero está construido en forma de árbol, en donde hay tres etiquetas principales: `<robot>`, `<link>` y `<joint>`. Para explicarlo bien, se puede tomar como referencia el brazo del cuerpo humano. Si lo que se quiere modelar es el brazo de una persona, la etiqueta `<robot>` representarı́a al brazo en su conjunto. Este brazo está compuesto de varios huesos (húmero, cúbito y radio) que son
representados por las etiquetas `<link>` y por una articulación que une esos huesos (codo) que es representado por la etiqueta `<joint>`. 

Además como en los huesos, estas etiquetas pueden ir con información adicional contenida en ellas que den información del tamaño, geometrı́a, inercia, orientación etc. Finalmente, el modelado de un robot se puede unir a otro modelo y formar uno más complejo,
que podrı́a ser representado con la adición de la mano al brazo, con la muñeca como articulación que conectan ambos. Hay que tener en cuenta que las etiquetas `<joint>` conecta las etiquetas `<link>` a través de una relación padre-hijo.

Dicho esto, se realiza una representación de los componentes del robot:
 
 ![image](/doc/imgs_md/urdf-robot.png  "Representación del fichero URDF")

En la imagen se representa el contenido del [fichero URDF](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_description/urdf/ur10_robot.urdf.xacro) que modela el robot junto a la pinza, se puede ver cómo se conecta el componente del brazo UR10 robot con el link `world`, representando world (color amarillo) y la base del brazo del UR10 `base_link` (color verde) situado justo encima, además el joint `world_joint` es la esfera de color amarillo situado entre ambos links. De la misma manera se tiene el componente de la pinza `robotiq_85_gripper`, está conectado al brazo del UR10 (ur10 robot), en donde la esfera que representa el joint `robotiq_85_base_joint` que une ambos componentes (color morado), uniendo el link `robotiq_85_base_link` de la pinza con el link `ee_link` del brazo de UR10.

### :computer: Creación del directorio para la solución
```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot
mkdir one_arm_no_moveit
```

### :computer: :warning: Agregación del robotiq_2f_85_gripper al robot ur10 [No ha sido posible, problemas con gazebo]
Para agregar correctamente el gripper hay que entender primero el funcionamiento de este. Las instrucciones de instalación está en [aquí](https://github.com/Danfoa/robotiq_2finger_grippers).
 
 Tomando la siguiente imagen para entender el funcionamiento del controllador del gripper:
 ![esquema del gripper](/doc/imgs_md/robotiq_2f_85_gripper.png  "controlador del gripper")

Se puede apreciar 2 nodos, uno hace de cliente y otro de servidor. El *servidor* es que se encarga de enviar las ordenes al gripper y feedback al cliente y el *cliente* envía ordenes al servidor.

Si comparamos la configuración del controlador con la del UR10, se aprecia que el concepto es diferente, ya que no existe un fichero *yaml* que cargue con la información necesaria para controlar el gripper. En vez de eso, es el servidor que crea los topics */command_robotiq_action* y */robotiq_controller/follow_joint_trajectory*, ambos son de tipo *action*.

```bash
miguel@Omen:~$ rostopic list
/clicked_point
/command_robotiq_action/cancel
/command_robotiq_action/feedback
/command_robotiq_action/goal
/command_robotiq_action/result
/command_robotiq_action/status
/initialpose
/joint_states
/move_base_simple/goal
/robotiq_controller/follow_joint_trajectory/cancel
/robotiq_controller/follow_joint_trajectory/feedback
/robotiq_controller/follow_joint_trajectory/goal
/robotiq_controller/follow_joint_trajectory/result
/robotiq_controller/follow_joint_trajectory/status
/rosout
/rosout_agg
/tf
/tf_static
```

Sabiendo esto, para controlar el gripper, hay un ejemplo en el fichero [robotiq_2f_action_client_example.py](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/robotiq_2finger_grippers/robotiq_2f_gripper_control/scripts/robotiq_2f_action_client_example.py) y puede funcionar con un gripper simulado o el griper real. Teniendo esto en cuenta, a la hora de agregar el gripper al robot UR10, no es necesario el fichero *yaml* que cargaria el controlador del driver, pero hay que lanzar correctamente el nodo que hará de servidor y si se mira el código de servidor y del cliente, hay que tener especial cuidado con la configuración de los topics y namespaces.

El fichero que carga el robot UR10 es [~/MultiCobot-UR10-Gripper/src/universal_robot/ur_description/launch/ur10_upload.launch](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/universal_robot/ur_description/launch/ur10_upload.launch), por tanto se va a copiar lo necesario del paquete *ur_description* y modificarlo para añadir el gripper al robot y también modificar el fichero [ur10.launch](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/universal_robot/ur_gazebo/launch/ur10.launch) que carga el URDF del paquete *ur_gazebo*:

```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit
catkin_create_pkg one_arm_no_moveit_description rospy
cd one_arm_no_moveit_description
mkdir launch
cp ~/MultiCobot-UR10-Gripper/src/universal_robot/ur_description/laun
ch/ur10_upload.launch launch/
mkdir urdf
cp ~/MultiCobot-UR10-Gripper/src/universal_robot/ur_description/urdf/ur10_robot.urdf.xacro udrf/
cp ~/MultiCobot-UR10-Gripper/src/universal_robot/ur_description/urdf/ur10_joint_limited_robot.urdf.xacro udrf/
```

No se sabe la razón exacta de porqué no es posible simular correctamente el gripper, ya que gazebo provee el siguiente error:
```bash
[ERROR] [1640030309.858708751, 1040.204000000]: This robot has a joint named "finger_joint" which is not in the gazebo model.
[FATAL] [1640030309.858805517, 1040.204000000]: Could not initialize robot simulation interface
```

Para la simulación con Gazebo no funciona, pero sí en la herramienta de rosviz. Esto indica que el paquete de ROS para la pinza funciona correctamente, pero la configuración del robot simulado en Gazebo (robot real) no es la adecuada. Seguramente habría que tratarlos como elementos independientes que funcionan físicamente como un conjunto, que no es el enfoque que se está dando para esta simulación en Gazebo.

![rviz-robotiq-2f-85-gripper](/doc/imgs_md/robotiq_2f_85_gripper_rviz.png  "rviz-robotiq-2f-85-gripper")

Como en el robot real, la pinza es un elemento independiente del brazo robótico puede que sea necesario este repositorio, pero para realizar las simulaciones se va utilizar otro repositorio que sí es compatible con el simulador gazebo.


### :computer: Agregación del robotiq_85_gripper al robot ur10
Se a proceder a agregar el gripper al robot ur10, no hay una referencia clara de cómo hacerlo adecuadamente. Para ello se empezará mirando cómo está estructurado el paquete para tener una idea de cómo adaptarlo al proyecto en cuestión.

```bash
LICENSE             robotiq_85_description  robotiq_85_moveit_config  si_utils
README.md           robotiq_85_driver       robotiq_85_msgs
robotiq_85_bringup  robotiq_85_gripper      robotiq_85_simulation
```

A primera vista los paquetes que nos interesan son el *robotiq_85_description*, *robotiq_85_bringup* y *robotiq_85_simulation*, el resto son recursos para su uso con MoveIt! o scripts, que posteriormente se aplicarán para el control del gripper en simulación.

Teniendo esto en encuenta se va a proceder a la incorporación del gripper en el robot UR10.

Tomando como ejemplo el fichero [~/MultiCobot-UR10-Gripper/src/robotiq_85_gripper/robotiq_85_description/urdf/robotiq_85_gripper.xacro](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/robotiq_85_gripper/robotiq_85_description/urdf/robotiq_85_gripper.xacro), se procede a añadir el gripper al robot, para ello hay que modificar los ficheros:

- [~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_description/urdf/ur10_robot.urdf.xacro](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_description/urdf/ur10_robot.urdf.xacro)
- [~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_description/urdf/ur10_joint_limited_robot.urdf.xacro](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_description/urdf/ur10_joint_limited_robot.urdf.xacro).

Y con esto, se tiene el gripper en el robot UR10, se puede apreciar en la imagen el gripper.
![ ](/doc/imgs_md/ur10_con_gripper_85.png  "ur10 con gripper")

Ahora, faltan los controladores para mandarle ordenes al gripper, eso se puede comprobar obteniendo la lista de topics activos y se comprobará que no hay ningun controlador para el griper:
```bash
miguel@Omen:~$ rostopic list
/arm_controller/command
/arm_controller/follow_joint_trajectory/cancel
/arm_controller/follow_joint_trajectory/feedback
/arm_controller/follow_joint_trajectory/goal
/arm_controller/follow_joint_trajectory/result
/arm_controller/follow_joint_trajectory/status
/arm_controller/state
/calibrated
/clock
/gazebo/link_states
/gazebo/model_states
/gazebo/parameter_descriptions
/gazebo/parameter_updates
/gazebo/performance_metrics
/gazebo/set_link_state
/gazebo/set_model_state
/gazebo_gui/parameter_descriptions
/gazebo_gui/parameter_updates
/joint_group_position_controller/command
/joint_states
/rosout
/rosout_agg
/tf
/tf_static
```
Se puede apreciar, que están cargados los controladores del robot UR10 (**/arm_controller**) pero no existe ningun topic para el control del gripper. Para ello hay que añadirlos y cargarlos en Gazebo correctamente de la siguiente manera. 

Los ficheros que contienen la información para controlar el gripper se encuentran en los directorios [~/MultiCobot-UR10-Gripper/src/robotiq_85_gripper/robotiq_85_simulation/robotiq_85_gazebo/controller](https://github.com/Serru/MultiCobot-UR10-Gripper/tree/main/src/robotiq_85_gripper/robotiq_85_simulation/robotiq_85_gazebo/controller) y  [~/MultiCobot-UR10-Gripper/src/robotiq_85_gripper/robotiq_85_simulation/robotiq_85_gazebo/launch](https://github.com/Serru/MultiCobot-UR10-Gripper/tree/main/src/robotiq_85_gripper/robotiq_85_simulation/robotiq_85_gazebo/launch) que estarían relacionadas respectivamente con los directorios *controller* y *launch* del directorio [one_arm_no_moveit_gazebo](https://github.com/Serru/MultiCobot-UR10-Gripper/tree/main/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo).

Al final, se han realizado las siguientes modificaciones:
```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/controller
cp ~/MultiCobot-UR10-Gripper/src/robotiq_85_gripper/robotiq_85_simulation/robotiq_85_gazebo/controller/gripper_controller_robotiq.yaml .
```

Y se ha agregado al final del fichero [~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/launch/ur10.launch](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/launch/ur10.launch) el controlador del gripper:
```xml
<!-- robotiq_85_gripper controller -->
  <rosparam file="$(find one_arm_no_moveit_gazebo)/controller/gripper_controller_robotiq.yaml" command="load"/> 
  <node name="gripper_controller_spawner" pkg="controller_manager" type="spawner" args="gripper" />
```

Se lanza Gazebo de nuevo (*roslaunch one_arm_no_moveit_gazebo ur10.launch*) y con el comado *rostopic list*, se aprecia que los controladores del gripper aparecen correctamente (**/gripper**):
```bash
/arm_controller/command
/arm_controller/follow_joint_trajectory/cancel
/arm_controller/follow_joint_trajectory/feedback
/arm_controller/follow_joint_trajectory/goal
/arm_controller/follow_joint_trajectory/result
/arm_controller/follow_joint_trajectory/status
/arm_controller/state
/calibrated
/clock
/gazebo/link_states
/gazebo/model_states
/gazebo/parameter_descriptions
/gazebo/parameter_updates
/gazebo/performance_metrics
/gazebo/set_link_state
/gazebo/set_model_state
/gazebo_gui/parameter_descriptions
/gazebo_gui/parameter_updates
/gripper/command
/gripper/follow_joint_trajectory/cancel
/gripper/follow_joint_trajectory/feedback
/gripper/follow_joint_trajectory/goal
/gripper/follow_joint_trajectory/result
/gripper/follow_joint_trajectory/status
/gripper/state
/joint_group_position_controller/command
/joint_states
/rosout
/rosout_agg
/tf
/tf_static
```

<a name="fase2">
  <h2>
Fase 2: Implementación de un planificador propio
  </h2>
</a>

### Fase 2: Implementación de un planificador propio

### :book: Creación del planificador y nodos auxiliares

Se seguirá siempre que se pueda la política de ROS en separar las tareas en nodos, para facilitar su reutilización futura.

#### robot_pose_publisher script
Se va a implementar un nodo que publique la posición del end effector *wrist_3_link* en todo momento, mediante la librería *tf*.


Como se considera más una herramienta que el script que realmente da las órdenes al robot, este script se implementará en el directorio de gazebo.

```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/scripts
touch ur10_robot_pose.py
chmod +x ur10_robot_pose.py
```

Y finalmente, el contenido del fichero será el siguiente:
```python
#!/usr/bin/env python  
import roslib
roslib.load_manifest('one_arm_no_moveit_gazebo')
import rospy, sys
import math
import tf
from geometry_msgs.msg import Point, Pose, Quaternion

def usage():
    print('''Commands:
    -namenode <namenode> - Set the name of the node,can't exists two nodes with the same name. Default value: ur10_robot_pose
    -namespace <namespace> - Let it empty to no add any namespacing.
    ''')
    sys.exit(1)

class RobotPose():
    def __init__(self):
        self.namespace               = ""
        self.namenode               = "ur10_robot_pose"
        self.robot_pose_pub = rospy.Publisher(self.namespace + '/robot_pose', Pose, queue_size=10)

    def parseUserInputs(self):
        # get goal from commandline
        for i in range(0,len(sys.argv)):
          if sys.argv[i] == '-namespace':
            if len(sys.argv) > i+1:
              self.namespace = sys.argv[i+1]
              self.robot_pose_pub = rospy.Publisher(self.namespace + '/robot_pose', Pose, queue_size=10)
          if sys.argv[i] == '-namenode':
            if len(sys.argv) > i+1:
              self.namenode = sys.argv[i+1]


    def callRobotPoseService(self):

        # wait for model to exist
        print self.namespace
        print self.namenode
        rospy.init_node(self.namenode)
        listener = tf.TransformListener()
    
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                (trans,rot) = listener.lookupTransform(self.namespace + '/base_link', self.namespace + '/wrist_3_link', rospy.Time(0))
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
            q = Quaternion(rot[0],rot[1],rot[2],rot[3])
            robot_pose.orientation = q
            self.robot_pose_pub.publish(robot_pose)
            print robot_pose         

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(usage())
    else:
        print("RobotPose script started") # make this a print incase roscore has not been started
        sm = RobotPose()
        sm.parseUserInputs()
        sm.callRobotPoseService()
```

Este nodo calcula la posición del *wrist_3_link* respecto a la posición del *base_link* mediante la libreria *tf* y lo publica en el topic. Esto permitirá obtener rápidamente la posición actual de la pinza.

Para automatizar el lanzamiento del nodo, hay que añadir lo siguiente al fichero *~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/launch/controller_utils.launch*

```xml
  <!-- get the robot position [Own Script]-->
  <node name="ur10_robot_pose" pkg="one_arm_no_moveit_gazebo" type="ur10_robot_pose.py" respawn="true" />
```

#### pub_ik_trajectory script
Se va a implementar un nodo que reciba comandos por el topic */pub_ik_trajectory* y se los irá mandando repetidamente a los controladores del robot, la posición a la que debe ir se irá modificando al recibir nuevas ordenes.

Como se considera más una herramienta que el script que realmente da las órdenes al robot, este script se implementará en el directorio de gazebo, igual que en el apartado anterior.

```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/scripts
touch pub_ik_trajectory.py
chmod +x pub_ik_trajectory.py
```

Y finalmente, el contenido del fichero será el siguiente:

```python
#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory
import roslib
roslib.load_manifest('one_arm_no_moveit_gazebo')
import sys

def usage():
    print('''Commands:
    -namenode <namenode> - Set the name of the node,can't exists two nodes with the same name. Default value: ur10_robot_pose
    -namespace <namespace> - Let it empty to no add any namespacing.
    ''')
    sys.exit(1)

class PubIkTrajectory():
    def __init__(self):
        self.ik_trajectory = JointTrajectory()
        self.namespace               = ""
        self.namenode               = "pub_ik_trajectory"
        self.cmd_pose_pub = rospy.Publisher(self.namespace + '/arm_controller/command', JointTrajectory, queue_size=10)
        self.trajectory_sub =         rospy.Subscriber(self.namespace + '/pub_ik_trajectory', JointTrajectory, self.update_trajectory)

    def parseUserInputs(self):
        # get goal from commandline
        for i in range(0,len(sys.argv)):
          if sys.argv[i] == '-namespace':
            if len(sys.argv) > i+1:
              self.namespace = sys.argv[i+1]
              self.cmd_pose_pub = rospy.Publisher(self.namespace + '/arm_controller/command', JointTrajectory, queue_size=10)
          if sys.argv[i] == '-namenode':
            if len(sys.argv) > i+1:
              self.namenode = sys.argv[i+1]

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
    if len(sys.argv) < 2:
        print(usage())
    else:
        print("PubIkTrajectory script started") # make this a print incase roscore has not been started
        sm = PubIkTrajectory()
        sm.parseUserInputs()
        sm.callIkTrajectoryService()

```

Para automatizar el lanzamiento del nodo, hay que añadir lo siguiente al fichero *~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/launch/controller_utils.launch*

```xml
  <!-- send the arms commands [Own Script]-->
  <node name="cmd_ik_trajectory_pub" pkg="one_arm_no_moveit_gazebo" type="pub_ik_trajectory.py" respawn="true" />
```

#### robot_manipulator script
Se va a implementar un nodo que obtendrá la información obtenida del *robot_pose_publisher* y enviará las trayectorías al nodo *pub_ik_trajectory*.

Para obtener los valores de las articulaciones dado la posición cartesiana que al que se quiere ir, es necesario la implementación la función *Inverse Kinematics* y *Forward Kinematics*. Estas funciones fueron obtenidas y adaptadas de [The Construct](https://www.theconstructsim.com/).

Para ello:
```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_manipulator/scripts
touch kinematics_utils.py
chmod +x kinematics_utils.py
```
Con el siguiente contenido, que se usará como librería:
```python
#!/usr/bin/env python

import sys
import copy
import rospy

import numpy as np
import tf.transformations as tf
from math import *
import cmath
from geometry_msgs.msg import Pose, Quaternion

# DH Parameters from ur10.urdf.xacro file
#<xacro:property name="d1" value="0.1273" />
#<xacro:property name="a2" value="-0.612" />
#<xacro:property name="a3" value="-0.5723" />
#<xacro:property name="d4" value="0.163941" />
#<xacro:property name="d5" value="0.1157" />
#<xacro:property name="d6" value="0.0922" />

# d (unit: mm)
d1 = 0.1273
d2 = d3 = 0
d4 = 0.163941
d5 = 0.1157
d6 = 0.0922

# a (unit: mm)
a1 = a4 = a5 = a6 = 0
a2 = -0.612
a3 = -0.5723


# List type of D-H parameter
d = np.array([d1, d2, d3, d4, d5, d6]) # unit: mm
a = np.array([a1, a2, a3, a4, a5, a6]) # unit: mm
alpha = np.array([pi/2, 0, 0, pi/2, -pi/2, 0]) # unit: radian


# Auxiliary Functions

def ur2ros(ur_pose):
    """Transform pose from UR format to ROS Pose format.
    Args:
        ur_pose: A pose in UR format [px, py, pz, rx, ry, rz] 
        (type: list)
    Returns:
        An HTM (type: Pose).
    """

    # ROS pose
    ros_pose = Pose()

    # ROS position
    ros_pose.position.x = ur_pose[0]
    ros_pose.position.y = ur_pose[1]
    ros_pose.position.z = ur_pose[2]

    # Ros orientation
    angle = sqrt(ur_pose[3] ** 2 + ur_pose[4] ** 2 + ur_pose[5] ** 2)
    direction = [i / angle for i in ur_pose[3:6]]
    np_T = tf.rotation_matrix(angle, direction)
    np_q = tf.quaternion_from_matrix(np_T)
    ros_pose.orientation.x = np_q[0]
    ros_pose.orientation.y = np_q[1]
    ros_pose.orientation.z = np_q[2]
    ros_pose.orientation.w = np_q[3]
    
    return ros_pose


def ros2np(ros_pose):
    """Transform pose from ROS Pose format to np.array format.
    Args:
        ros_pose: A pose in ROS Pose format (type: Pose)
    Returns:
        An HTM (type: np.array).
    """

    # orientation
    np_pose = tf.quaternion_matrix([ros_pose.orientation.x, ros_pose.orientation.y, \
                                    ros_pose.orientation.z, ros_pose.orientation.w])
    
    # position
    np_pose[0][3] = ros_pose.position.x
    np_pose[1][3] = ros_pose.position.y
    np_pose[2][3] = ros_pose.position.z

    return np_pose


def np2ros(np_pose):
    """Transform pose from np.array format to ROS Pose format.
    Args:
        np_pose: A pose in np.array format (type: np.array)
    Returns:
        An HTM (type: Pose).
    """

    # ROS pose
    ros_pose = Pose()

    # ROS position
    ros_pose.position.x = np_pose[0, 3]
    ros_pose.position.y = np_pose[1, 3]
    ros_pose.position.z = np_pose[2, 3]

    # ROS orientation 
    np_q = tf.quaternion_from_matrix(np_pose)
    ros_pose.orientation.x = np_q[0]
    ros_pose.orientation.y = np_q[1]
    ros_pose.orientation.z = np_q[2]
    ros_pose.orientation.w = np_q[3]

    return ros_pose


def select(q_sols, q_d, w=[1]*6):
    """Select the optimal solutions among a set of feasible joint value 
       solutions.
    Args:
        q_sols: A set of feasible joint value solutions (unit: radian)
        q_d: A list of desired joint value solution (unit: radian)
        w: A list of weight corresponding to robot joints
    Returns:
        A list of optimal joint value solution.
    """

    error = []
    for q in q_sols:
        error.append(sum([w[i] * (q[i] - q_d[i]) ** 2 for i in range(6)]))
    
    return q_sols[error.index(min(error))]


def HTM(i, theta):
    """Calculate the HTM between two links.
    Args:
        i: A target index of joint value. 
        theta: A list of joint value solution. (unit: radian)
    Returns:
        An HTM of Link l w.r.t. Link l-1, where l = i + 1.
    """

    Rot_z = np.matrix(np.identity(4))
    Rot_z[0, 0] = Rot_z[1, 1] = cos(theta[i])
    Rot_z[0, 1] = -sin(theta[i])
    Rot_z[1, 0] = sin(theta[i])

    Trans_z = np.matrix(np.identity(4))
    Trans_z[2, 3] = d[i]

    Trans_x = np.matrix(np.identity(4))
    Trans_x[0, 3] = a[i]

    Rot_x = np.matrix(np.identity(4))
    Rot_x[1, 1] = Rot_x[2, 2] = cos(alpha[i])
    Rot_x[1, 2] = -sin(alpha[i])
    Rot_x[2, 1] = sin(alpha[i])

    A_i = Rot_z * Trans_z * Trans_x * Rot_x
        
    return A_i


# Forward Kinematics

def fwd_kin(theta, i_unit='r', o_unit='n'):
    """Solve the HTM based on a list of joint values.
    Args:
        theta: A list of joint values. (unit: radian)
        i_unit: Output format. 'r' for radian; 'd' for degree.
        o_unit: Output format. 'n' for np.array; 'p' for ROS Pose.
    Returns:
        The HTM of end-effector joint w.r.t. base joint
    """

    T_06 = np.matrix(np.identity(4))

    if i_unit == 'd':
        theta = [radians(i) for i in theta]
    
    for i in range(6):
        T_06 *= HTM(i, theta)

    if o_unit == 'n':
        return T_06
    elif o_unit == 'p':
        return np2ros(T_06)


# Inverse Kinematics

def inv_kin(p, q_d, i_unit='r', o_unit='r'):
    """Solve the joint values based on an HTM.
    Args:
        p: A pose.
        q_d: A list of desired joint value solution 
             (unit: radian).
        i_unit: Output format. 'r' for radian; 'd' for degree.
        o_unit: Output format. 'r' for radian; 'd' for degree.
    Returns:
        A list of optimal joint value solution.
    """

    # Preprocessing
    if type(p) == Pose: # ROS Pose format
        T_06 = ros2np(p)
    elif type(p) == list: # UR format
        T_06 = ros2np(ur2ros(p))

    if i_unit == 'd':
        q_d = [radians(i) for i in q_d]

    # Initialization of a set of feasible solutions
    theta = np.zeros((8, 6))
 
    # theta1
    P_05 = T_06[0:3, 3] - d6 * T_06[0:3, 2]
    phi1 = atan2(P_05[1], P_05[0])
    phi2 = acos(d4 / sqrt(P_05[0] ** 2 + P_05[1] ** 2))
    theta1 = [pi / 2 + phi1 + phi2, pi / 2 + phi1 - phi2]
    theta[0:4, 0] = theta1[0]
    theta[4:8, 0] = theta1[1]
  
    # theta5
    P_06 = T_06[0:3, 3]
    theta5 = []
    for i in range(2):
        theta5.append(acos((P_06[0] * sin(theta1[i]) - P_06[1] * cos(theta1[i]) - d4) / d6))
    for i in range(2):
        theta[2*i, 4] = theta5[0]
        theta[2*i+1, 4] = -theta5[0]
        theta[2*i+4, 4] = theta5[1]
        theta[2*i+5, 4] = -theta5[1]
  
    # theta6
    T_60 = np.linalg.inv(T_06)
    theta6 = []
    for i in range(2):
        for j in range(2):
            s1 = sin(theta1[i])
            c1 = cos(theta1[i])
            s5 = sin(theta5[j])
            theta6.append(atan2((-T_60[1, 0] * s1 + T_60[1, 1] * c1) / s5, (T_60[0, 0] * s1 - T_60[0, 1] * c1) / s5))
    for i in range(2):
        theta[i, 5] = theta6[0]
        theta[i+2, 5] = theta6[1]
        theta[i+4, 5] = theta6[2]
        theta[i+6, 5] = theta6[3]

    # theta3, theta2, theta4
    for i in range(8):  
        # theta3
        T_46 = HTM(4, theta[i]) * HTM(5, theta[i])
        T_14 = np.linalg.inv(HTM(0, theta[i])) * T_06 * np.linalg.inv(T_46)
        P_13 = T_14 * np.array([[0, -d4, 0, 1]]).T - np.array([[0, 0, 0, 1]]).T
        if i in [0, 2, 4, 6]:
            theta[i, 2] = -cmath.acos((np.linalg.norm(P_13) ** 2 - a2 ** 2 - a3 ** 2) / (2 * a2 * a3)).real
            theta[i+1, 2] = -theta[i, 2]
        # theta2
        theta[i, 1] = -atan2(P_13[1], -P_13[0]) + asin(a3 * sin(theta[i, 2]) / np.linalg.norm(P_13))
        # theta4
        T_13 = HTM(1, theta[i]) * HTM(2, theta[i])
        T_34 = np.linalg.inv(T_13) * T_14
        theta[i, 3] = atan2(T_34[1, 0], T_34[0, 0])       

    theta = theta.tolist()

    # Select the most close solution
    q_sol = select(theta, q_d)

    # Output format
    if o_unit == 'r': # (unit: radian)
        return q_sol
    elif o_unit == 'd': # (unit: degree)
        return [degrees(i) for i in q_sol]
```
Una vez que se tiene la librería implementada, se procede a desarrollar el nodo que será el que realmente envíe las órdenes al robot para que realice las tareas deseadas.

```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_manipulator/scripts
touch robot_manipulator.py
chmod +x robot_manipulator.py
```

QUe contendrá una serie de funciones para enviar correctamente las trayectorias finales así como la obtención de los valores de los joints en la posición del robot.

```python
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
        self.send_trajectory_pub = rospy.Publisher('/pub_ik_trajectory', JointTrajectory, queue_size=10)
        self.current_robot_pose = Pose()
        self.robot_pose_sub = rospy.Subscriber('/robot_pose', Pose, self.update_current_pose)
        self.robot_pose_updated = False

    def pick_place(self):
        # -0.24, -0.632, 0.62
        self.send_trajectory(-0.24, 0.632, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(5)
#        y = 2000
#        z = 5900
        y = 7000
        z = 2000
        #for x in range(-2000, -5900, -100):
        for x in range(7000, 2000, -100):
            #print x
            #print x*0.0001
            if x > 0:
                y -= 100
                z += 100
            if x < 0:
                y += 100
                z -= 100
            print x*0.0001
            print y*0.0001
            print z*0.0001
            try:
                self.send_trajectory(x*0.0001, y*0.0001, z*0.0001, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
            except Exception:
                print('\033[91m[ Singularidad, valores:' + str(x*0.0001) + ', ' + str(y*0.0001) + ', ' + str(z*0.0001) + ']\033[0m')

#        self.send_trajectory(-0.56, 0.21, 0.51, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
#        self.send_trajectory(-0.56, 0.22, 0.50, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
#        self.send_trajectory(-0.56, 0.23, 0.49, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
#        self.send_trajectory(-0.56, 0.24, 0.48, -0.68945825, -0.72424496, 0.00781949, 0.00744391)

    def update_current_pose(self, pose):
        self.current_robot_pose = pose
        self.robot_pose_updated = True

    def send_trajectory(self, pos_x, pos_y, pos_z, rot_x, rot_y, rot_z, rot_w):
        position = JointTrajectory()
        position.header.stamp=rospy.Time.now()
        position.header.frame_id = "/base_link"    
        position.joint_names = ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint',
                          'wrist_1_joint','wrist_2_joint','wrist_3_joint']
        
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
        points.positions = inv_kin(ps, rcs)
        points.time_from_start = rospy.Duration.from_sec(0.1)
        position.points.append(points)
        self.send_trajectory_pub.publish(position)
        #state = sol
        #rospy.sleep(0.1)
        self.robot_pose_updated = False
#        print points.positions
        print('\033[93m[' + str(ps.position.x) + ', ' + str(ps.position.y) + ', ' + str(ps.position.z) + ']\033[0m')
        
if __name__ == '__main__':
    rospy.init_node('demo', anonymous=True)
    cmd = CmdTrajectory()
    rpy = tf.quaternion_from_euler(-3.12, 0.0, 1.62)
    print rpy
    #[-0.68945825 -0.72424496  0.00781949  0.00744391]
    #cmd.send_trajectory(-0.6, -0.16, 0.62, rpy[0], rpy[1], rpy[2], rpy[3])
    # Posicion inicial del brazo
    cmd.send_trajectory(-0.24, 0.632, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
    rospy.sleep(2)
    cmd.pick_place()
```

El robot da problemas dependiendo de las coordenadas que se pasen, ya que puede surgir singularidades, es cuando el robot se bloquea porque limitaciones que tiene el dominio matemático (ej: si theta2 = acos(1), el valor es indefinido generando un error o divisiones por cero).

Para evitar eso, hay que definir un workspace en donde no sufra de estas singularidades.

#### pub_gripper_cmd script
Finalmente, queda añadir un script que permita el control del gripper, para ello se creará un node que escuche de un topic (*/pub_gripper_control*) delque obtendrá el valor que enviará al controlador mediante el topic */gripper/command*

Este nodo es un nodo de apoyo, por ello estará junto a los scripts de Gazebo, cerca de los ficheros de los controladores:
```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/scripts
touch pub_gripper_cmd.py
chmod +x pub_gripper_cmd.py
```

Y su contenido es el siguiente:
```python
#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory
import roslib
roslib.load_manifest('one_arm_no_moveit_gazebo')
import sys

def usage():
    print('''Commands:
    -namenode <namenode> - Set the name of the node,can't exists two nodes with the same name. Default value: ur10_robot_pose
    -namespace <namespace> - Let it empty to no add any namespacing.
    ''')
    sys.exit(1)

class PubGripperCmd():
    def __init__(self):
        self.trajectory_gripper_cmd = JointTrajectory()
        self.namespace               = ""
        self.namenode               = "pub_gripper_control"
        self.cmd_gripper_pub = rospy.Publisher(self.namespace + '/gripper/command', JointTrajectory, queue_size=10)
        self.gripper_control_sub =         rospy.Subscriber(self.namespace + '/pub_gripper_control', JointTrajectory, self.update_cmd)

    def parseUserInputs(self):
        # get goal from commandline
        for i in range(0,len(sys.argv)):
          if sys.argv[i] == '-namespace':
            if len(sys.argv) > i+1:
              self.namespace = sys.argv[i+1]
              self.cmd_gripper_pub = rospy.Publisher(self.namespace + '/gripper/command', JointTrajectory, queue_size=10)
          if sys.argv[i] == '-namenode':
            if len(sys.argv) > i+1:
              self.namenode = sys.argv[i+1]

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
    if len(sys.argv) < 2:
        print(usage())
    else:
        print("PubGripperCmd script started") # make this a print incase roscore has not been started
        sm = PubGripperCmd()
        sm.parseUserInputs()
        sm.callGripperCmdService()
```

Para automatizar el lanzamiento del nodo, hay que añadir lo siguiente al fichero *~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/launch/controller_utils.launch*

```{xml}
<!-- send the gripper commands [Own Script]-->
  <node name="cmd_gripper_value_pub" pkg="one_arm_no_moveit_gazebo" type="pub_gripper_cmd.py" respawn="true" />
```
#### gripper_manipulator script
Para controlar el gripper, unicamente hay que modificar el fichero *robot_manipulator.py*

```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_manipulator/scripts
nano robot_manipulator.py
```
Con el siguiente contenido:
```python
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
        self.send_trajectory_pub = rospy.Publisher('/pub_ik_trajectory', JointTrajectory, queue_size=10)
        self.send_gripper_cmd_pub = rospy.Publisher('/pub_gripper_control', JointTrajectory, queue_size=10)
        self.current_robot_pose = Pose()
        self.robot_pose_sub = rospy.Subscriber('/robot_pose', Pose, self.update_current_pose)
        self.robot_pose_updated = False

    def send_gripper_cmd(self, gripper_distance):
        gripper = JointTrajectory()
        gripper.header.stamp=rospy.Time.now()
        gripper.header.frame_id = "/ee_link"    
        gripper.joint_names = ['robotiq_85_left_knuckle_joint']
        
        points = JointTrajectoryPoint()
        points.positions = [gripper_distance]
        points.time_from_start = rospy.Duration.from_sec(1)
        gripper.points.append(points)
        self.send_gripper_cmd_pub.publish(gripper)
        print('\033[93m[' + str(gripper_distance) + ']\033[0m')

    def pick_place(self):
        # -0.24, -0.632, 0.62
        self.send_trajectory(-0.24, 0.632, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(5)
#        y = 2000
#        z = 5900
        y = 7000
        z = 2000
        #for x in range(-2000, -5900, -100):
        for x in range(7000, 2000, -100):
            #print x
            #print x*0.0001
            if x > 0:
                y -= 100
                z += 100
            if x < 0:
                y += 100
                z -= 100
            print x*0.0001
            print y*0.0001
            print z*0.0001
            try:
                self.send_trajectory(x*0.0001, y*0.0001, z*0.0001, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
            except Exception:
                print('\033[91m[ Singularidad, valores:' + str(x*0.0001) + ', ' + str(y*0.0001) + ', ' + str(z*0.0001) + ']\033[0m')

#        self.send_trajectory(-0.56, 0.21, 0.51, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
#        self.send_trajectory(-0.56, 0.22, 0.50, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
#        self.send_trajectory(-0.56, 0.23, 0.49, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
#        self.send_trajectory(-0.56, 0.24, 0.48, -0.68945825, -0.72424496, 0.00781949, 0.00744391)

    def pick_place2(self):
        # -0.24, -0.632, 0.62
        self.send_trajectory(-0.24, 0.632, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
        rospy.sleep(5)
        for y in range(7000, -7000, -100):
            try:
                self.send_trajectory(-0.64, y*0.0001, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
            except Exception:
                print('\033[91m[ Singularidad, valores:' + str(x*0.0001) + ', ' + str(y*0.0001) + ', ' + str(z*0.0001) + ']\033[0m')


    def update_current_pose(self, pose):
        self.current_robot_pose = pose
        self.robot_pose_updated = True

    def send_trajectory(self, pos_x, pos_y, pos_z, rot_x, rot_y, rot_z, rot_w):
        position = JointTrajectory()
        position.header.stamp=rospy.Time.now()
        position.header.frame_id = "/base_link"    
        position.joint_names = ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint',
                          'wrist_1_joint','wrist_2_joint','wrist_3_joint']
        
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
        points.positions = inv_kin(ps, rcs)
        points.time_from_start = rospy.Duration.from_sec(0.1)
        position.points.append(points)
        self.send_trajectory_pub.publish(position)
        #state = sol
        #rospy.sleep(0.1)
        self.robot_pose_updated = False
#        print points.positions
        print('\033[93m[' + str(ps.position.x) + ', ' + str(ps.position.y) + ', ' + str(ps.position.z) + ']\033[0m')
        
if __name__ == '__main__':
    rospy.init_node('demo', anonymous=True)
    cmd = CmdTrajectory()
    rpy = tf.quaternion_from_euler(-3.12, 0.0, 1.62)
    print rpy
    #[-0.68945825 -0.72424496  0.00781949  0.00744391]
    #cmd.send_trajectory(-0.6, -0.16, 0.62, rpy[0], rpy[1], rpy[2], rpy[3])
    # Posicion inicial del brazo
    cmd.send_trajectory(-0.24, 0.632, 0.62, -0.68945825, -0.72424496, 0.00781949, 0.00744391)
    rospy.sleep(2)
    cmd.send_gripper_cmd(0.0)
    cmd.send_gripper_cmd(0.8)
    cmd.send_gripper_cmd(0.5)
#    cmd.send_gripper_cmd(0.0)
    #cmd.pick_place2()
```








### Puesta en marcha de Gazebo
Actualmente desde el directorio creado del proyecto no se puede lanzar Gazebo con el robot, y es el primer paso lanzar gazebo con el robot UR10.

Por imponer cierto orden en la estructura del proyecto a implementar, se van a separar el uso de las diferentes herramientas y sus ficheros siempre que se pueda. Esto permitirá una mejor compresión de cómo está estructurado y facilitará su debug en caso de fallo.

Por ello, se creará un paquete, en vez de un directorio que contendrá todo lo relacionado con gazebo:
```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot/on_arm_no_moveit
catkin_create_pkg one_arm_no_moveit_gazebo rospy
```

En el directio creado para gazebo, se copiara del directorio de *ur_gazebo*, las carpetas *controller* y *launch*.
```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot/on_arm_no_moveit/one_arm_no_moveit_gazebo
cp -r ~/MultiCobot-UR10-Gripper/src/universal_robot/ur_gazebo/controller .
cp -r ~/MultiCobot-UR10-Gripper/src/universal_robot/ur_gazebo launch .
```
Se compila:
```bash
cd ~/MultiCobot-UR10-Gripper
catkin_make
```

Con esto ya se puede lanzar Gazebo desde el directorio del proyecto.
```bash
roslaunch one_arm_no_moveit_gazebo ur10.launch
```
### Configuración del mundo de Gazebo
Ahora se procede a crear el mundo con el robot y un entorno sobre el que podrá realizar simples tareas, este proyecto se enfoca en las tareas que pueda realizar el robot, por tanto no es necesario que el mundo sea muy detallado.

Para ello primero se crear el directorio world, en donde se guardará los worlds que se creen:
```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot/on_arm_no_moveit/one_arm_no_moveit_gazebo
mkdir world
```

Se utilizará el fichero world, de otro [repositorio](https://github.com/Infinity8sailor/multiple_arm_setup/tree/main/multiple_ur_description/) que provee de un escenario muy simple que permitirá el testeo posterior de tareas en el robot.

```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot/on_arm_no_moveit/one_arm_no_moveit_gazebo
git clone https://github.com/Infinity8sailor/multiple_arm_setup.git
cp -r multiple_arm_setup/multiple_ur_description/models/ .
cp -r multiple_arm_setup/multiple_ur_description/world/ .
sudo rm -r mutiple_arm_setup
```
Para que añadir el world con el robot en gazebo, hay que modificar el fichero ur10.launch en el directorio launch:
```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot/on_arm_no_moveit/one_arm_no_moveit_gazebo/launch
nano ur10.launch
```
Y incluidir el mundo en el launch, añadiendo el argumento *world* y reemplazando el valor de *default* en el argumento *world_name*:
```xml
 <arg name="world" default="$(find one_arm_no_moveit_gazebo)/world/multiarm_bot.world" />

  <!-- startup simulated world -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" default="$(arg world)"/>
    <arg name="paused" value="$(arg paused)"/>
    <arg name="gui" value="$(arg gui)"/>
  </include>
```
Se procede a lanzar gazebo con el nuevo mundo:
```bash
roslaunch one_arm_no_moveit_gazebo ur10.launch
```

Al lanzarlo, muestra dos errores:
```bash
Error [parser.cc:581] Unable to find uri[model://dropbox]
```
Este error, no se puede solucionar, a no ser que se cree el modelo de cero, ya que gazebo no lo provee o se ha eliminado. Por tanto en el fichero *world/multiarm_bot.world* se comenta el objeto dropbox:
```{xml}
<!--include>
       <pose frame=''>0.5 0.0 0.3 0 0 0</pose> 
       <uri>model://dropbox</uri>
       <name>DropBox</name>
</include-->
```


```bash
[ERROR] [1639740881.902412642, 0.057000000]: GazeboRosControlPlugin missing <legacyModeNS> while using DefaultRobotHWSim, defaults to true.
This setting assumes you have an old package with an old implementation of DefaultRobotHWSim, where the robotNamespace is disregarded and absolute paths are used instead.
If you do not want to fix this issue in an old package just set <legacyModeNS> to true.
```
Para eliminar este error, se procede a realizar el cambio desde el fichero de *~/MultiCobot-UR10-Gripper/src/universal_robot/ur_descriptiom/urdf/common.gazebo.xacro*, hay que añadir *<legacyModeNS>* al fichero:
```xml
<plugin name="ros_control" filename="libgazebo_ros_control.so">
      <!--robotNamespace>/</robotNamespace-->
      <!--robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType-->
      <legacyModeNS>true</legacyModeNS>
</plugin>
```

Y un warning, que puede ser un problema el ignorarlo a la hora de simular el comportamiento de brazo en el futuro, por lo que se procede a resolverlo.
```bash
[ WARN] [1639745803.729749460, 0.061000000]: The default_robot_hw_sim plugin is using the Joint::SetPosition method without preserving the link velocity.
[ WARN] [1639745803.729772883, 0.061000000]: As a result, gravity will not be simulated correctly for your model.
[ WARN] [1639745803.729786659, 0.061000000]: Please set gazebo_pid parameters, switch to the VelocityJointInterface or EffortJointInterface, or upgrade to Gazebo 9.

```
Para ello, se ha decidido instalar Gazebo 9 en el entorno de ROS Kinetic Kame.
```bash
sudo apt-get remove ros-kinetic-desktop-full
sudo apt-get remove ros-kinetic-gazebo*
sudo apt-get upgrade

sudo apt-get install ros-kinetic-ros-base

sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
cat /etc/apt/sources.list.d/gazebo-stable.list
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update

sudo apt-get install ros-kinetic-gazebo9-ros-pkgs ros-kinetic-gazebo9-ros-control ros-kinetic-gazebo9*

sudo apt-get install ros-kinetic-catkin

sudo apt-get install rviz

sudo apt-get install ros-kinetic-controller-manager ros-kinetic-joint-state-controller ros-kinetic-joint-trajectory-controller ros-kinetic-rqt ros-kinetic-rqt-controller-manager ros-kinetic-rqt-joint-trajectory-controller ros-kinetic-ros-control ros-kinetic-rqt-gui

sudo apt-get install ros-kinetic-rqt-plot ros-kinetic-rqt-graph ros-kinetic-rqt-rviz ros-kinetic-rqt-tf-tree

sudo apt-get install ros-kinetic-gazebo9-ros ros-kinetic-kdl-conversions ros-kinetic-kdl-parser ros-kinetic-forward-command-controller ros-kinetic-tf-conversions ros-kinetic-xacro ros-kinetic-joint-state-publisher ros-kinetic-robot-state-publisher

sudo apt-get install ros-kinetic-ros-control ros-kinetic-ros-controllers
```
El resultado tras solventar  configurar Gazebo es similar a la siguiente.
![setup stage](/doc/imgs_md/one-arm-no-moveit-gazebo-setup.png  "Gazebo9-world-setup")
* Se ha realizado modificaciones en el fichero de world y ur10.launch para obtener el escenario como en la imagen, estas modificaciones se pueden encontrar en los anexos.













### Pruebas de pick and place con lo implementado
Durante el inicio de las pruebas se ha visto que en la smulación en Gazebo no es posible agarrar los objetos sobre la mesa.

Esto es debido a que falta incluir el plugin de Gazebo *gazebo_grasp* que está en el paquete *gazebo-pkgs* instalado previamente como recurso.

#### Gazbebo Grasp plugin
Se va a proceder a realizar los pasos necesarios para cargar el plugin que permita al robot interaccionar con los objetos en simulación.

Lo primero es tener el fichero *gzplugin_grasp_fix.urdf.xacro* (se puede obtenerlo del repositorio de [Jennifer Buehler](https://github-wiki-see.page/m/JenniferBuehler/gazebo-pkgs/wiki/The-Gazebo-grasp-fix-plugin)), se podria guardarlo en el paquete de *universal robots* pero como es una modificación realizada para nuestro proyecto, se ha decidido en moverlo al directorio *~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_description/urdf*.

```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_description/urdf
touch gzplugin_grasp_fix.urdf.xacro
```

Y el contenido del fichero:
```xml
<?xml version="1.0" encoding="UTF-8"?>
<root 
 xmlns:sensor="http://playerstage.sourceforge.net/gazebo/xmlschema/#sensor"
 xmlns:controller="http://playerstage.sourceforge.net/gazebo/xmlschema/#controller"
 xmlns:interface="http://playerstage.sourceforge.net/gazebo/xmlschema/#interface"
 xmlns:xacro="http://wiki.ros.org/xacro">


<!-- MACRO FOR THE ROBOT ARM ON THE TABLE-->
<xacro:macro name="gzplugin_grasp_fix" params="prefix">
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

                <!-- 0.01976<0.01977<0.01978 -->
<!--                 <palm_link>wrist_3_link</palm_link>
                <gripper_link>gripper_finger1_inner_knuckle_link</gripper_link>
                <gripper_link>gripper_finger1_finger_tip_link</gripper_link>
                <gripper_link>gripper_finger1_knuckle_link</gripper_link>
                <gripper_link>gripper_finger2_inner_knuckle_link</gripper_link>
                <gripper_link>gripper_finger2_finger_tip_link</gripper_link>
                <gripper_link>gripper_finger2_knuckle_link</gripper_link>
 -->                
            </arm>
            <forces_angle_tolerance>100</forces_angle_tolerance>
            <update_rate>20</update_rate>
            <grip_count_threshold>1</grip_count_threshold>
            <max_grip_count>3</max_grip_count>
            <release_tolerance>0.005</release_tolerance>
            <!--release_tolerance>0.0198</release_tolerance--> <!-- 0.01977<0.0198<0.01999 -->
            <disable_collisions_on_attach>false</disable_collisions_on_attach>
            <contact_topic>__default_topic__</contact_topic>
        </plugin>
    </gazebo>
</xacro:macro>

</root>
```
Una vez que se tiene el plugin de Gazebo, hay que añadirlo al brazo robótico y se ha añadido al gripper código para aumentar su fricción ayudando al agarre de los objetos en la simulación.

```bash
nano ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_description/urdf/ur10_robot.urdf.xacro
```
Añadir al final el plugin:
```xml
  <xacro:include filename="$(find one_arm_no_moveit_description)/urdf/gzplugin_grasp_fix.urdf.xacro"/>

  <xacro:gzplugin_grasp_fix prefix=""/>
```

De la misma manera, editamos el fichero del gripper ([~/MultiCobot-UR10-Gripper/src/robotiq_85_gripper/robotiq_85_description/urdf/robotiq_85_gripper.urdf.xacro](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/robotiq_85_gripper/robotiq_85_description/urdf/robotiq_85_gripper.urdf.xacro)):
```bash
nano ~/MultiCobot-UR10-Gripper/src/robotiq_85_gripper/robotiq_85_description/urdf/robotiq_85_gripper.urdf.xacro
```

Añadir al final el plugin:
```xml
	[...]
        <!-- Improve grasping physics -->
        <gazebo reference="${prefix}robotiq_85_left_finger_tip_link">
          <!--kp>1000000.0</kp>
          <mu1>1.0</mu1>
          <mu2>1.0</mu2-->
          <mu1>100000</mu1>
          <mu2>100000</mu2>
          <kp>100000000.0</kp>
          <kd>1.0</kd>
          <minDepth>0.001</minDepth>
        </gazebo>

        <gazebo reference="${prefix}robotiq_85_right_finger_tip_link">
          <!--kp>1000000.0</kp>
          <mu1>1.0</mu1>
          <mu2>1.0</mu2-->
          <mu1>100000</mu1>
          <mu2>100000</mu2>
          <kp>100000000.0</kp>
          <kd>1.0</kd>
          <minDepth>0.001</minDepth>
        </gazebo>
```
Y esta es toda la configuraóicn necesaria para que el plugin funcione correctamente, en el caso de que hubiese varios robots, hay que asignar correctamente el plugin para cada gripper, sino esos grippers no podrá agrrar objetos durante la simulación.

#### Simple test en Gazebo
Se a creado un test en donde, el robot agarra tres cubos de madera y los envía a un contenedor.

El fichero con el código del manipulador para realizar la prueba: [~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_manipulator/scripts/robot_manipulator.py](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_manipulator/scripts/robot_manipulator.py)


Comandos para lanzar el test:
* En un terminal:
```bash
cd ~/MultiCobot-UR10-Gripper/
source devel/setup.bash
roslaunch one_arm_no_moveit_gazebo ur10_joint_limited.launch
```
* En otro terminal:
```bash
cd ~/MultiCobot-UR10-Gripper/
source devel/setup.bash
rosrun one_arm_no_moveit_manipulator robot_manipulator.py
```

* Tener en cuenta que se ha modificado un poco el world de gazebo para estas pruebas. Todo esto se podrá encontrar en el repositorio del proyecto.