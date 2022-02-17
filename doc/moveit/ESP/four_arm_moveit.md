<!--- Para cuatro robots opción B--->
## Instalación y configuración para cuatro robot UR10 replicando MoveIt!

### Creación del directorio para la solución
```{bash}
cd ~/MultiCobot-UR10-Gripper/src/multirobot
mkdir four_arm_moveit
```

### Puesta en marcha de Gazebo
Se va a crear el paquete para gazebo, y copiar el contenido de la solución partiendo del la solución *two_arm_moveit* para su posterior modificación:
```{bash}
cd ~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_moveit
catkin_create_pkg four_arm_moveit_gazebo rospy
```
En el directorio creado para gazebo, se copiara del directorio de *two_arm_moveit_gazebo*, las carpetas *controller*, *launch*, *models*, y *world* del directorio de *four_arm_no_moveit*.
```{bash}
cd ~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_moveit/four_arm_moveit_gazebo
cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_moveit/two_arm_moveit_gazebo/controller .
cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_moveit/two_arm_moveit_gazebo/launch .
cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_moveit/two_arm_moveit_gazebo/models .
cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_no_moveit/four_arm_no_moveit_gazebo/world .
```

Sustituir el directorio *two_arm_moveit_gazebo* por *four_arm_moveit_gazebo* y *two_arm_moveit_description* por *four_arm_moveit_description* en los ficheros.

### Configuración del directorio descripción
Siguiendo la misma línea, se crea un nuevo paquete y se copia los directorios del proyecto *one_arm_moveit* para su posterior modificación.

```{bash}
cd ~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_moveit
catkin_create_pkg four_arm_moveit_description rospy
```

En el directorio creado para description, se copiara del directorio de *two_arm_moveit_description*, las carpetas *launch* y *urdf*.

```{bash}
cd ~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_moveit/four_arm_moveit_description
cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_moveit/two_arm_moveit_description/launch .
cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_moveit/two_arm_moveit_description/urdf .
```

### Modificación de los ficheros de description
Sustituir el directorio *two_arm_moveit_description* por *four_arm_moveit_description* en los ficheros.

### Configuración de MoveIt!

Tras la preparación de los directorios que contiene el modelo de robot (descripton) y para su simulación (Gazebo), se procede a configurar MoveIt!

```{bash}
cd ~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_moveit
mkdir four_arm_moveit_config
roslaunch moveit_setup_assistant setup_assistant.launch
```
#### MoveIt! Setup Assistant

Se va a escoger como modelo del robot el fichero URDF: *~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_moveit/two_arm_moveit_description/urdf/ur10_joint_limited_robot.urdf.xacro* (podría ser perfectamente ur10_robot.urdf.xacro).

```{bash}
cd ~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_moveit
mkdir two_arm_moveit_config
roslaunch moveit_setup_assistant setup_assistant.launch
```

![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_1.png  "Cargar el modelo URDF del robot UR10")

Posteriormente, se le da al boton *Load Files*.
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_2.png  "Cargado el modelo URDF del robot UR10")

En la pestaña *Self-Collisions*, darle al boton *Generate Collision Matrix*, o que generará una matriz en donde los diferentes componentes del robot puedan tener colisiones:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_3.png  "Generación de matirz de colisiones")

En la pestaña *VIrtual Joints*, hay que crear un joint entre la base del robot y el frame que lo contiene, en este caso, *world*, siendo la configuración la siguiente:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_4.png  "Definiendo Virtual Joint")
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_5.png  "Definido Virtual Joint")

Una de las pestañas más importantes es definir bien los Planning groups, en este caso se tiene dos grupos, el grupo *manipulator* que controlará el brazo del robot y el grupo *gripper* que controlará la pinza:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_6.png  "Manipulator kdl")

Después hay que darle al botón *Add Kin. Chain*:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_7.png  "Manipulator Kinetic Chain configuración")

Finalmente se guarda la configuración:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_8.png  "Grupo manipulator configurado")

Ahora hay que hacerlo para el grupo *Gripper* que controlará la pinza, se pulsa el botón *Add Group*, se rellena con el nombre del grupo y se pone *kdl* como *Kinematic Solver*:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_9.png  "Gripper kdl")

Tras darle al botón *Add Joints*, hay que buscar por *robotiq_85_left_knucle_joint* y añadirlo a con la flecha *->* yse guarda la configuración:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_10.png  "Gripper Joint configuración")

Tras guardar, el resultado en la pestaña de *Planning Group* debería ser la siguiente:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_11.png  "Configuración final del Planning Group")

En la pestaña *Robot Poses* se va a configurar uno denominado "home":
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_12.png  "Configurando "home" 1/3")

![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_13.png  "Configurando "home" 2/3")

![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_14.png  "Configurando "home" 3/3")

![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_24.png  "Configurando "gripper open"")

![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_25.png  "Configurando "gripper close"")


En la pestaña *End Effectors* se va añadir el gripper:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_15.png  "Configurando end effector 1/3")

![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_16.png  "Configurando end effector 2/3")

![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_17.png  "Configurando end effector 3/3")

En la pestaña *Passive Joints*, para este caso, es la siguiente:

![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_18.png  "Configurando Passive Joints")

En la pestaña *ROS Control*, se añadirá de forma automática, los ficheros generados, serán posteriormente modificado manualmente.

![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_19.png  "Configurando ROS Control")

![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_20.png  "Configurando ROS Control")

Hay que rellenar la pestaña *Author Information* para que la configuración pueda terminar:

![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_21.png  "Configurando Author Information")

La útima pestaña *Configuration Files*, permite decidir dónde se guardará la configuración de MoveIt!, en este caso en *one_arm_moveit_config* creado previamente, se genera la configuración mediante el botón *Generate Package* y finalmente
 *Exit Setup Assistant* para terminar:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_22.png  "Configurando Author Information")

![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_23.png  "Configurando Author Information")


### Conexión entre Gazebo y MoveIt!

Se va a modificar los siguientes ficheros teniendo como base sus ficheros originales *demo.launch*, *move_group.launch*, *trajectory_execution.launch.xml*  y *ur10_moveit_controller_manager.launch.xml* y se añadirá los controladores creando dos ficheros *controllers.yaml* y *joint_names.yaml*:

```{bash}
cd ~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_moveit
catkin_create_pkg four_arm_moveit_manipulator rospy
cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_moveit/two_arm_moveit_manipulator/config .
cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_moveit/two_arm_moveit_manipulator/launch .
cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_moveit/two_arm_moveit_manipulator/scripts .
```
Sustituir el directorio *two_arm_moveit_gazebo* por *four_arm_moveit_gazebo* y *two_arm_moveit_description* por *four_arm_moveit_description* en los ficheros y renombrar los ficheros *two_arm_moveit* por *four_arm_moveit*


Se va a proceder a añadir los controladores para su interacción con Gazebo de cuatro robots:

Para ello se crea un fichero *launch* que lanzará la configuración de los cuatro robots con diferente *namespaces*.
```{bash}
touch launch/four_arm_moveit_gazebo.launch
```

Fichero: *~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_moveit/four_arm_moveit_manipulator/launch/four_arm_moveit_gazebo.launch*

```{xml}
<launch>
    <param name="/use_sim_time" value="true"/>
    <arg name="robot_name"/>
    <arg name="init_pose"/>
    <arg name="paused" default="false"/>
    <arg name="gui" default="true"/>
    <arg name="limited" default="true"  doc="If true, limits joint range [-PI, PI] on all joints." />


    <!-- send robot urdf to param server -->
    <include file="$(find four_arm_moveit_description)/launch/ur10_upload.launch">
      <arg name="limited" value="$(arg limited)"/>
    </include>

    <include file="$(find gazebo_ros)/launch/empty_world.launch">
        <!--arg name="world_name" default="worlds/empty.world"/-->
        <arg name="verbose" value="true"/>
        <arg name="world_name" default="$(find four_arm_moveit_gazebo)/world/multiarm_bot.world"/>
        <arg name="paused" value="$(arg paused)"/>
        <!--arg name="gui" value="$(arg gui)"/-->
        <arg name="gui" value="$(arg gui)"/>
    </include>

    <group ns="ur10_1">
        <param name="tf_prefix" value="ur10_1_tf" />
        <include file="$(find four_arm_moveit_gazebo)/launch/ur10_joint_limited.launch">
            <arg name="init_pose" value="-x 0.6 -y -0.6 -z 1.1"/>
            <arg name="robot_name" value="ur10_1"/>
        </include>
    </group>

    <group ns="ur10_2">
        <param name="tf_prefix" value="ur10_2_tf" />
        <include file="$(find four_arm_moveit_gazebo)/launch/ur10_joint_limited.launch">
            <arg name="robot_name" value="ur10_2"/>
            <arg name="init_pose" value="-x 0.6 -y 1.38 -z 1.1"/>
        </include>
    </group>

    <group ns="ur10_3">
        <param name="tf_prefix" value="ur10_3_tf" />
        <include file="$(find four_arm_moveit_gazebo)/launch/ur10_joint_limited.launch">
            <arg name="robot_name" value="ur10_3"/>
            <arg name="init_pose" value="-x 0.6 -y 3.36 -z 1.1"/>
        </include>
    </group>

    <group ns="ur10_4">
        <param name="tf_prefix" value="ur10_4_tf" />
        <include file="$(find four_arm_moveit_gazebo)/launch/ur10_joint_limited.launch">
            <arg name="robot_name" value="ur10_4"/>
            <arg name="init_pose" value="-x 0.6 -y 5.34 -z 1.1"/>
        </include>
    </group>

    <node pkg="tf" type="static_transform_publisher" name="world_frames_connection_1" args="0 0 0 0 0 0 /world /ur10_1_tf/world 100"/>

    <node pkg="tf" type="static_transform_publisher" name="world_frames_connection_2" args="0 0 0 0 0 0 /world /ur10_2_tf/world 100"/>

    <node pkg="tf" type="static_transform_publisher" name="world_frames_connection_3" args="0 0 0 0 0 0 /world /ur10_3_tf/world 100"/>

    <node pkg="tf" type="static_transform_publisher" name="world_frames_connection_4" args="0 0 0 0 0 0 /world /ur10_4_tf/world 100"/>

</launch>
```

Los nodos de tf son muy importantes porque MoveIt! tomo el frame */world* como refrencia como referencia a la hora de realizar el planing. 

Y no es posible modificarlo con la API de python, hay que crear un nodo que realize las transformaciones necesarias como en la solución sin MoveIt! o añadir el frame */world* como se ha realizado que será la raíz del resto de los frames, como aparece enla imagen.
![ ](/doc/imgs_md/four_arm_moveit_frames.pdf  "Robots frames")

Fichero: *~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_moveit/four_arm_moveit_manipulator/launch/four_arm_moveit_execution.launch*

```{xml}
<launch>
    <arg name="sim" default="false" />
    <arg name="debug" default="false" />
    <!-- By default, we do not start a database (it can be large) -->
    <arg name="demo" default="false" />

    <group ns="ur10_1">
      <rosparam command="load" file="$(find four_arm_moveit_manipulator)/config/joint_names.yaml"/>

      <include file="$(find four_arm_moveit_config)/launch/planning_context.launch">
        <arg name="load_robot_description" value="true"/>
      </include>

      <!-- We do not have a robot connected, so publish fake joint states -->
      <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
        <param name="/use_gui" value="false"/>
        <rosparam param="/source_list">[/joint_states]</rosparam>
      </node>

      <!-- New added by Miguel Burgh -->

      <!-- Given the published joint states, publish tf for the robot links -->
      <!--node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" respawn="true" output="screen" /-->

      <!-- End new added by Miguel Burgh -->

      <include file="$(find four_arm_moveit_manipulator)/launch/move_group.launch">
        <arg name="debug" default="$(arg debug)" />
        <arg name="publish_monitored_planning_scene" value="true"/>
        <!--arg name="info" value="true"/-->
      </include>

      <!-- If database loading was enabled, start mongodb as well -->
      <include file="$(find four_arm_moveit_config)/launch/default_warehouse_db.launch" if="$(arg demo)"/>

      <!-- Remap follow_joint_trajectory -->
      <remap from="/ur10_1/follow_joint_trajectory" to="/ur10_1/arm_controller/follow_joint_trajectory"/>

      <include file="$(find four_arm_moveit_config)/launch/moveit_rviz.launch">
        <arg name="config" value="true"/>
        <arg name="debug" default="false"/>
      </include>
    </group>

    <group ns="ur10_2">
      <rosparam command="load" file="$(find four_arm_moveit_manipulator)/config/joint_names.yaml"/>

      <include file="$(find four_arm_moveit_config)/launch/planning_context.launch">
        <arg name="load_robot_description" value="true"/>
      </include>

      <!-- We do not have a robot connected, so publish fake joint states -->
      <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
        <param name="/use_gui" value="false"/>
        <rosparam param="/source_list">[/joint_states]</rosparam>
      </node>

      <!-- New added by Miguel Burgh -->

      <!-- Given the published joint states, publish tf for the robot links -->
      <!--node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" respawn="true" output="screen" /-->

      <!-- End new added by Miguel Burgh -->

      <include file="$(find four_arm_moveit_manipulator)/launch/move_group.launch">
        <arg name="debug" default="$(arg debug)" />
        <arg name="publish_monitored_planning_scene" value="true"/>
        <!--arg name="info" value="true"/-->
      </include>

      <!-- If database loading was enabled, start mongodb as well -->
      <include file="$(find four_arm_moveit_config)/launch/default_warehouse_db.launch" if="$(arg demo)"/>

      <!-- Remap follow_joint_trajectory -->
      <remap from="/ur10_2/follow_joint_trajectory" to="/ur10_2/arm_controller/follow_joint_trajectory"/>

      <include file="$(find four_arm_moveit_config)/launch/moveit_rviz.launch">
        <arg name="config" value="true"/>
        <arg name="debug" default="false"/>
      </include>
    </group>

    <group ns="ur10_3">
      <rosparam command="load" file="$(find four_arm_moveit_manipulator)/config/joint_names.yaml"/>

      <include file="$(find four_arm_moveit_config)/launch/planning_context.launch">
        <arg name="load_robot_description" value="true"/>
      </include>

      <!-- We do not have a robot connected, so publish fake joint states -->
      <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
        <param name="/use_gui" value="false"/>
        <rosparam param="/source_list">[/joint_states]</rosparam>
      </node>

      <!-- New added by Miguel Burgh -->

      <!-- Given the published joint states, publish tf for the robot links -->
      <!--node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" respawn="true" output="screen" /-->

      <!-- End new added by Miguel Burgh -->

      <include file="$(find four_arm_moveit_manipulator)/launch/move_group.launch">
        <arg name="debug" default="$(arg debug)" />
        <arg name="publish_monitored_planning_scene" value="true"/>
        <!--arg name="info" value="true"/-->
      </include>

      <!-- If database loading was enabled, start mongodb as well -->
      <include file="$(find four_arm_moveit_config)/launch/default_warehouse_db.launch" if="$(arg demo)"/>

      <!-- Remap follow_joint_trajectory -->
      <remap from="/ur10_3/follow_joint_trajectory" to="/ur10_3/arm_controller/follow_joint_trajectory"/>

      <include file="$(find four_arm_moveit_config)/launch/moveit_rviz.launch">
        <arg name="config" value="true"/>
        <arg name="debug" default="false"/>
      </include>
    </group>

    <group ns="ur10_4">
      <rosparam command="load" file="$(find four_arm_moveit_manipulator)/config/joint_names.yaml"/>

      <include file="$(find four_arm_moveit_config)/launch/planning_context.launch">
        <arg name="load_robot_description" value="true"/>
      </include>

      <!-- We do not have a robot connected, so publish fake joint states -->
      <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
        <param name="/use_gui" value="false"/>
        <rosparam param="/source_list">[/joint_states]</rosparam>
      </node>

      <!-- New added by Miguel Burgh -->

      <!-- Given the published joint states, publish tf for the robot links -->
      <!--node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" respawn="true" output="screen" /-->

      <!-- End new added by Miguel Burgh -->

      <include file="$(find four_arm_moveit_manipulator)/launch/move_group.launch">
        <arg name="debug" default="$(arg debug)" />
        <arg name="publish_monitored_planning_scene" value="true"/>
        <!--arg name="info" value="true"/-->
      </include>

      <!-- If database loading was enabled, start mongodb as well -->
      <include file="$(find four_arm_moveit_config)/launch/default_warehouse_db.launch" if="$(arg demo)"/>

      <!-- Remap follow_joint_trajectory -->
      <remap from="/ur10_4/follow_joint_trajectory" to="/ur10_4/arm_controller/follow_joint_trajectory"/>

      <include file="$(find four_arm_moveit_config)/launch/moveit_rviz.launch">
        <arg name="config" value="true"/>
        <arg name="debug" default="false"/>
      </include>
    </group>
</launch>
```

Finalmente, se realiza una prueba:
```{bash}
cd ~/MultiCobot-UR10-Gripper
catkin_make clean
catkin_make
rospack profile 

# Terminal 1
roslaunch four_arm_moveit_manipulator four_arm_moveit_gazebo.launch

# Terminal 2
roslaunch two_arm_moveit_manipulator two_arm_moveit_execution.launch
```

![ ](/doc/imgs_md/four_arm_moveit_gazebo.png  "Example gazebo+rviz+moveit!")

Y la gráfica de los nodos y los topis, despues de las modificaciones, se puede apreciar cómo ahorá el nodo *move_group* tiene comunicación con los controladores. En la imagen no se aprecia, pero hay dos nodos *move_group*s con el mismo contenido pero diferente namespaces, se recomienda generar el proyecto y obtener le gráfica mediante la herramienta *rqt_graph*.

![ ](/doc/imgs_md/four_arm_moveit_graph_changes.png  "rqt_graph representación de los nodos y los topics")

### Pick and Place
Como en las soluciones anteriores, se procederáa realizar unas pruebas muy sencillas. Para ello primero hay que crear los scripts necesarios para controlar el brazo del robot y el gripper correctamente y posterioremente, se realizará los mismos movimientos que en las soluciones anteriormente propuestas.

Fichero ~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_moveit/four_arm_moveit_manipulator/scripts/four_arm_moveit_1.py
```{C}
#!/usr/bin/env python
from __future__ import print_function
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi


    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))

# r, p and y in rads
def plan_cartesian_path_orientation(group, r, p, y, scale=1, bool_wait = True):
    joint_target = group.get_current_joint_values()
    joint_target[3] += scale * r
    joint_target[4] += scale * p
    joint_target[5] += scale * y
    succeeded = group.go(joint_target, wait=bool_wait)
    return succeeded

def plan_cartesian_path_pose(group, x, y, z, w, scale=1):
    waypoints = []

    wpose = group.get_current_pose().pose
    wpose.position.x += scale * x  # Move forward/backwards in (x)
    wpose.position.z += scale * z  # Move up/down (z)
    wpose.position.y += scale * y  # Move sideways (y)
    wpose.orientation.w += scale * w  # Rotation of the arm
    waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan, fraction) = group.compute_cartesian_path(
        waypoints,  # waypoints to follow
        0.001,  # eef_step
        0.0)  # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction


def execute_plan(group, plan, bool_wait = True):
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    succeeded = group.execute(plan, wait=bool_wait)
    return succeeded

    ## **Note:** The robot's current joint state must be within some tolerance of the
    ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail

def main():
    ## First initialize moveit_commander and rospy.
    print("============ Starting dual arms moveit")
    moveit_commander.roscpp_initializer.roscpp_initialize(sys.argv)
    rospy.init_node('ur10_1_arm_moveit',
                  anonymous=True)

    PLANNING_GROUP_GRIPPER = "gripper"
    PLANNING_GROUP_ARM = "manipulator"
    PLANNING_NS = "/ur10_1/"

    ## Instantiate a RobotCommander object.  This object is an interface to
    ## the robot as a whole.
    robot = moveit_commander.RobotCommander("%srobot_description"%PLANNING_NS, ns="/ur10_1/")

    ## Instantiate the MoveGroupCommander objects.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the ur10
    ## arm and gripper. This interface can be used to plan and execute motions on the ur10
    ## arm and gripper.
    
    #gripper = robot.get_group(PLANNING_GROUP_GRIPPER)
    #arm = robot.get_group(PLANNING_GROUP_ARM)
    arm = moveit_commander.move_group.MoveGroupCommander(PLANNING_GROUP_ARM,"%srobot_description"%PLANNING_NS, ns="/ur10_1/")
    gripper = moveit_commander.move_group.MoveGroupCommander(PLANNING_GROUP_GRIPPER, "%srobot_description"%PLANNING_NS, ns="/ur10_1/")

    ## We create this DisplayTrajectory publisher which is used below to publish
    ## trajectories for RVIZ to visualize.
    display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory, queue_size=10)

    rospy.sleep(2)

    ## We can get the name of the reference frame for this robot
    print( "============ Reference frame: %s" % arm.get_planning_frame() )

    ## We can also print the name of the end-effector link for this group
    print( "============ Reference frame: %s" % arm.get_end_effector_link() )

    ## We can get a list of all the groups in the robot
    print( "============ Robot Groups:" )
    print( robot.get_group_names())
    
    #arm.set_planner_id("RRT")
    arm.set_num_planning_attempts(15)
    arm.allow_looking(True)
    arm.allow_replanning(True)

    #gripper.set_planner_id("RRTConnect")
    gripper.set_num_planning_attempts(15)
    gripper.allow_replanning(True)
    gripper.allow_looking(True)


    ## Planning to a Pose goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the 
    ## end-effector

    # Primer Cubo
    succeess = False
    all_ok = False
    while not all_ok:
        arm.set_named_target("home")
    
        arm.go(wait=True)
        gripper.set_named_target("gripper_open")
        gripper.go(wait=True)
        
        cartesian_plan, fraction = plan_cartesian_path_pose(arm, -0.54, 0, 0, 0, 1)
        succeess = execute_plan(arm, cartesian_plan)
        if succeess:
            cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, -0.143, 0, 0, 1)
            succeess = execute_plan(arm, cartesian_plan)
            if succeess:
                cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0, -0.47, 0, 1)
                succeess = execute_plan(arm, cartesian_plan)
                if succeess:
                    cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0, -0.07, 0, 1)
                    succeess = execute_plan(arm, cartesian_plan)
                    if succeess:
                        pos_check = arm.get_current_pose().pose.position
                        print(round(pos_check.x,2))
                        print(round(pos_check.y,2))
                        if round(pos_check.x,2) == -0.30:
                            if round(pos_check.y,2) == -0.77:
                                all_ok = True
                                print(arm.get_current_pose())
        print(succeess)
        print(all_ok)

    succeess = False
    all_ok = False
    while not all_ok:
    
        gripper.set_named_target("gripper_close")
        gripper.go(wait=True)

        arm.set_named_target("home")    
        arm.go(wait=True)

        cartesian_plan, fraction = plan_cartesian_path_pose(arm, -0.53, 0.0, 0.07, 0, 1)
        succeess = execute_plan(arm, cartesian_plan)
        if succeess:
            cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0.98, 0, 0, 0, 1)
            succeess = execute_plan(arm, cartesian_plan)
            if succeess:
                cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0.175, 0, 0, 1)
                succeess = execute_plan(arm, cartesian_plan)
                if succeess:
                    pos_check = arm.get_current_pose().pose.position
                    print(round(pos_check.x,2))
                    print(round(pos_check.y,2))
                    if round(pos_check.x,2) == 0.69:
                        if round(pos_check.y,2) == -0.46:
                            all_ok = True
                            print(arm.get_current_pose())
        print(succeess)

    succeess = False
    all_ok = False
    while not all_ok:
        gripper.set_named_target("gripper_open")
        gripper.go(wait=True)
    
        # Segundo Cubo
        arm.set_named_target("home")
        arm.go(wait=True)

        cartesian_plan, fraction = plan_cartesian_path_pose(arm, -0.53, 0, 0, 0, 1)
        succeess = execute_plan(arm, cartesian_plan)
        if succeess:
            cartesian_plan, fraction = plan_cartesian_path_pose(arm, -0.61, 0, 0, 0, 1)
            succeess = execute_plan(arm, cartesian_plan)
            if succeess:
                cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0, -0.42, 1.0)
                succeess = execute_plan(arm, cartesian_plan)
                if succeess:
                    cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0.132, 0, 0, 1)
                    succeess = execute_plan(arm, cartesian_plan)
                    if succeess:
                        cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0, -0.12, 0, 1)
                        succeess = execute_plan(arm, cartesian_plan)
                        if succeess:
                            pos_check = arm.get_current_pose().pose.position
                            print(round(pos_check.x,2))
                            print(round(pos_check.y,2))
                            if round(pos_check.x,2) == -0.9:
                                if round(pos_check.y,2) == -0.50:
                                    all_ok = True
                                    print(arm.get_current_pose())
    
    succeess = False
    all_ok = False
    while not all_ok:
        gripper.set_named_target("gripper_close")
        gripper.go(wait=True)

        arm.set_named_target("home")    
        arm.go(wait=True)

        cartesian_plan, fraction = plan_cartesian_path_pose(arm, -0.53, 0.0, 0.07, 0, 1)
        succeess = execute_plan(arm, cartesian_plan)
        if succeess:
            cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0.98, 0, 0, 0, 1)
            succeess = execute_plan(arm, cartesian_plan)
            if succeess:
                cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0.175, 0, 0, 1)
                succeess = execute_plan(arm, cartesian_plan)
                if succeess:
                    pos_check = arm.get_current_pose().pose.position
                    print(round(pos_check.x,2))
                    print(round(pos_check.y,2))
                    if round(pos_check.x,2) == 0.69:
                        if round(pos_check.y,2) == -0.46:
                            all_ok = True
                            print(arm.get_current_pose())    
    succeess = False
    all_ok = False
    while not all_ok:
        gripper.set_named_target("gripper_open")
        gripper.go(wait=True)
        
        # Tercer Cubo
        arm.set_named_target("home")
        arm.go(wait=True)
    
        cartesian_plan, fraction = plan_cartesian_path_pose(arm, -0.53, 0, 0, 0, 1)
        succeess = execute_plan(arm, cartesian_plan)
        if succeess:
            cartesian_plan, fraction = plan_cartesian_path_pose(arm, -0.385, 0, 0, 0, 1)
            succeess = execute_plan(arm, cartesian_plan)
            if succeess:
                cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0, -0.42, 1.0)
                succeess = execute_plan(arm, cartesian_plan)
                if succeess:
                    cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, -0.248, 0, 0, 1)
                    succeess = execute_plan(arm, cartesian_plan)
                    if succeess:
                        cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0, -0.12, 0, 1)
                        succeess = execute_plan(arm, cartesian_plan)
                        if succeess:
                            pos_check = arm.get_current_pose().pose.position
                            print(round(pos_check.x,2))
                            print(round(pos_check.y,2))
                            if round(pos_check.x,2) == -0.68:
                                if round(pos_check.y,2) == -0.88:
                                    all_ok = True
                                    print(arm.get_current_pose())
    succeess = False
    all_ok = False
    while not all_ok:
        gripper.set_named_target("gripper_close")
        gripper.go(wait=True)

        arm.set_named_target("home")    
        arm.go(wait=True)

        cartesian_plan, fraction = plan_cartesian_path_pose(arm, -0.53, 0.0, 0.07, 0, 1)
        succeess = execute_plan(arm, cartesian_plan)
        if succeess:
            cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0.98, 0, 0, 0, 1)
            succeess = execute_plan(arm, cartesian_plan)
            if succeess:
                cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0.175, 0, 0, 1)
                succeess = execute_plan(arm, cartesian_plan)
                if succeess:
                    pos_check = arm.get_current_pose().pose.position
                    print(round(pos_check.x,2))
                    print(round(pos_check.y,2))
                    if round(pos_check.x,2) == 0.69:
                        if round(pos_check.y,2) == -0.46:
                            all_ok = True
                            print(arm.get_current_pose())
    gripper.set_named_target("gripper_open")
    gripper.go(wait=True)

    arm.set_named_target("home")
    arm.go(wait=True)
    gripper.set_named_target("gripper_open")
    gripper.go(wait=True)

    ## When finished shut down moveit_commander.
    moveit_commander.roscpp_shutdown()

if __name__=='__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass
```

Fichero ~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_moveit/four_arm_moveit_manipulator/scripts/four_arm_moveit_2.py
```{C}
#!/usr/bin/env python
from __future__ import print_function
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi


    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))

# r, p and y in rads
def plan_cartesian_path_orientation(group, r, p, y, scale=1, bool_wait = True):
    joint_target = group.get_current_joint_values()
    joint_target[3] += scale * r
    joint_target[4] += scale * p
    joint_target[5] += scale * y
    succeeded = group.go(joint_target, wait=bool_wait)
    return succeeded

def plan_cartesian_path_pose(group, x, y, z, w, scale=1):
    waypoints = []

    wpose = group.get_current_pose().pose
    wpose.position.x += scale * x  # Move forward/backwards in (x)
    wpose.position.z += scale * z  # Move up/down (z)
    wpose.position.y += scale * y  # Move sideways (y)
    wpose.orientation.w += scale * w  # Rotation of the arm
    waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan, fraction) = group.compute_cartesian_path(
        waypoints,  # waypoints to follow
        0.001,  # eef_step
        0.0)  # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction


def execute_plan(group, plan, bool_wait = True):
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    succeeded = group.execute(plan, wait=bool_wait)
    return succeeded

    ## **Note:** The robot's current joint state must be within some tolerance of the
    ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail

def main():
    ## First initialize moveit_commander and rospy.
    print("============ Starting dual arms moveit")
    moveit_commander.roscpp_initializer.roscpp_initialize(sys.argv)
    rospy.init_node('ur10_2_arm_moveit',
                  anonymous=True)

    PLANNING_GROUP_GRIPPER = "gripper"
    PLANNING_GROUP_ARM = "manipulator"
    PLANNING_NS = "/ur10_2/"

    ## Instantiate a RobotCommander object.  This object is an interface to
    ## the robot as a whole.
    robot = moveit_commander.RobotCommander("%srobot_description"%PLANNING_NS, ns="/ur10_2/")

    ## Instantiate the MoveGroupCommander objects.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the ur10
    ## arm and gripper. This interface can be used to plan and execute motions on the ur10
    ## arm and gripper.
    
    #gripper = robot.get_group(PLANNING_GROUP_GRIPPER)
    #arm = robot.get_group(PLANNING_GROUP_ARM)
    arm = moveit_commander.move_group.MoveGroupCommander(PLANNING_GROUP_ARM,"%srobot_description"%PLANNING_NS, ns="/ur10_2/")
    gripper = moveit_commander.move_group.MoveGroupCommander(PLANNING_GROUP_GRIPPER, "%srobot_description"%PLANNING_NS, ns="/ur10_2/")

    ## We create this DisplayTrajectory publisher which is used below to publish
    ## trajectories for RVIZ to visualize.
    display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory, queue_size=10)

    rospy.sleep(2)

    ## We can get the name of the reference frame for this robot
    print( "============ Reference frame: %s" % arm.get_planning_frame() )

    ## We can also print the name of the end-effector link for this group
    print( "============ Reference frame: %s" % arm.get_end_effector_link() )

    ## We can get a list of all the groups in the robot
    print( "============ Robot Groups:" )
    print( robot.get_group_names())
    
    #arm.set_planner_id("RRT")
    arm.set_num_planning_attempts(15)
    arm.allow_looking(True)
    arm.allow_replanning(True)

    #gripper.set_planner_id("RRTConnect")
    gripper.set_num_planning_attempts(15)
    gripper.allow_replanning(True)
    gripper.allow_looking(True)


    ## Planning to a Pose goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the 
    ## end-effector

    # Primer Cubo
    succeess = False
    all_ok = False
    while not all_ok:
        arm.set_named_target("home")
    
        arm.go(wait=True)
        gripper.set_named_target("gripper_open")
        gripper.go(wait=True)
        
        cartesian_plan, fraction = plan_cartesian_path_pose(arm, -0.54, 0, 0, 0, 1)
        succeess = execute_plan(arm, cartesian_plan)
        if succeess:
            cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, -0.143, 0, 0, 1)
            succeess = execute_plan(arm, cartesian_plan)
            if succeess:
                cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0, -0.47, 0, 1)
                succeess = execute_plan(arm, cartesian_plan)
                if succeess:
                    cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0, -0.07, 0, 1)
                    succeess = execute_plan(arm, cartesian_plan)
                    if succeess:
                        pos_check = arm.get_current_pose().pose.position
                        print(round(pos_check.x,2))
                        print(round(pos_check.y,2))
                        if round(pos_check.x,2) == -0.30:
                            if round(pos_check.y,2) == -0.77:
                                all_ok = True
                                print(arm.get_current_pose())
        print(succeess)
        print(all_ok)

    succeess = False
    all_ok = False
    while not all_ok:
    
        gripper.set_named_target("gripper_close")
        gripper.go(wait=True)

        arm.set_named_target("home")    
        arm.go(wait=True)

        cartesian_plan, fraction = plan_cartesian_path_pose(arm, -0.53, 0.0, 0.07, 0, 1)
        succeess = execute_plan(arm, cartesian_plan)
        if succeess:
            cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0.98, 0, 0, 0, 1)
            succeess = execute_plan(arm, cartesian_plan)
            if succeess:
                cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0.175, 0, 0, 1)
                succeess = execute_plan(arm, cartesian_plan)
                if succeess:
                    pos_check = arm.get_current_pose().pose.position
                    print(round(pos_check.x,2))
                    print(round(pos_check.y,2))
                    if round(pos_check.x,2) == 0.69:
                        if round(pos_check.y,2) == -0.46:
                            all_ok = True
                            print(arm.get_current_pose())
        print(succeess)

    succeess = False
    all_ok = False
    while not all_ok:
        gripper.set_named_target("gripper_open")
        gripper.go(wait=True)
    
        # Segundo Cubo
        arm.set_named_target("home")
        arm.go(wait=True)

        cartesian_plan, fraction = plan_cartesian_path_pose(arm, -0.53, 0, 0, 0, 1)
        succeess = execute_plan(arm, cartesian_plan)
        if succeess:
            cartesian_plan, fraction = plan_cartesian_path_pose(arm, -0.61, 0, 0, 0, 1)
            succeess = execute_plan(arm, cartesian_plan)
            if succeess:
                cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0, -0.42, 1.0)
                succeess = execute_plan(arm, cartesian_plan)
                if succeess:
                    cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0.132, 0, 0, 1)
                    succeess = execute_plan(arm, cartesian_plan)
                    if succeess:
                        cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0, -0.12, 0, 1)
                        succeess = execute_plan(arm, cartesian_plan)
                        if succeess:
                            pos_check = arm.get_current_pose().pose.position
                            print(round(pos_check.x,2))
                            print(round(pos_check.y,2))
                            if round(pos_check.x,2) == -0.9:
                                if round(pos_check.y,2) == -0.50:
                                    all_ok = True
                                    print(arm.get_current_pose())
    
    succeess = False
    all_ok = False
    while not all_ok:
        gripper.set_named_target("gripper_close")
        gripper.go(wait=True)

        arm.set_named_target("home")    
        arm.go(wait=True)

        cartesian_plan, fraction = plan_cartesian_path_pose(arm, -0.53, 0.0, 0.07, 0, 1)
        succeess = execute_plan(arm, cartesian_plan)
        if succeess:
            cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0.98, 0, 0, 0, 1)
            succeess = execute_plan(arm, cartesian_plan)
            if succeess:
                cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0.175, 0, 0, 1)
                succeess = execute_plan(arm, cartesian_plan)
                if succeess:
                    pos_check = arm.get_current_pose().pose.position
                    print(round(pos_check.x,2))
                    print(round(pos_check.y,2))
                    if round(pos_check.x,2) == 0.69:
                        if round(pos_check.y,2) == -0.46:
                            all_ok = True
                            print(arm.get_current_pose())    
    succeess = False
    all_ok = False
    while not all_ok:
        gripper.set_named_target("gripper_open")
        gripper.go(wait=True)
        
        # Tercer Cubo
        arm.set_named_target("home")
        arm.go(wait=True)
    
        cartesian_plan, fraction = plan_cartesian_path_pose(arm, -0.53, 0, 0, 0, 1)
        succeess = execute_plan(arm, cartesian_plan)
        if succeess:
            cartesian_plan, fraction = plan_cartesian_path_pose(arm, -0.385, 0, 0, 0, 1)
            succeess = execute_plan(arm, cartesian_plan)
            if succeess:
                cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0, -0.42, 1.0)
                succeess = execute_plan(arm, cartesian_plan)
                if succeess:
                    cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, -0.248, 0, 0, 1)
                    succeess = execute_plan(arm, cartesian_plan)
                    if succeess:
                        cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0, -0.12, 0, 1)
                        succeess = execute_plan(arm, cartesian_plan)
                        if succeess:
                            pos_check = arm.get_current_pose().pose.position
                            print(round(pos_check.x,2))
                            print(round(pos_check.y,2))
                            if round(pos_check.x,2) == -0.68:
                                if round(pos_check.y,2) == -0.88:
                                    all_ok = True
                                    print(arm.get_current_pose())
    succeess = False
    all_ok = False
    while not all_ok:
        gripper.set_named_target("gripper_close")
        gripper.go(wait=True)

        arm.set_named_target("home")    
        arm.go(wait=True)

        cartesian_plan, fraction = plan_cartesian_path_pose(arm, -0.53, 0.0, 0.07, 0, 1)
        succeess = execute_plan(arm, cartesian_plan)
        if succeess:
            cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0.98, 0, 0, 0, 1)
            succeess = execute_plan(arm, cartesian_plan)
            if succeess:
                cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0.175, 0, 0, 1)
                succeess = execute_plan(arm, cartesian_plan)
                if succeess:
                    pos_check = arm.get_current_pose().pose.position
                    print(round(pos_check.x,2))
                    print(round(pos_check.y,2))
                    if round(pos_check.x,2) == 0.69:
                        if round(pos_check.y,2) == -0.46:
                            all_ok = True
                            print(arm.get_current_pose())
    gripper.set_named_target("gripper_open")
    gripper.go(wait=True)

    arm.set_named_target("home")
    arm.go(wait=True)
    gripper.set_named_target("gripper_open")
    gripper.go(wait=True)

    ## When finished shut down moveit_commander.
    moveit_commander.roscpp_shutdown()

if __name__=='__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass
```

Fichero ~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_moveit/four_arm_moveit_manipulator/scripts/four_arm_moveit_3.py
```{C}
#!/usr/bin/env python
from __future__ import print_function
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi


    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))

# r, p and y in rads
def plan_cartesian_path_orientation(group, r, p, y, scale=1, bool_wait = True):
    joint_target = group.get_current_joint_values()
    joint_target[3] += scale * r
    joint_target[4] += scale * p
    joint_target[5] += scale * y
    succeeded = group.go(joint_target, wait=bool_wait)
    return succeeded

def plan_cartesian_path_pose(group, x, y, z, w, scale=1):
    waypoints = []

    wpose = group.get_current_pose().pose
    wpose.position.x += scale * x  # Move forward/backwards in (x)
    wpose.position.z += scale * z  # Move up/down (z)
    wpose.position.y += scale * y  # Move sideways (y)
    wpose.orientation.w += scale * w  # Rotation of the arm
    waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan, fraction) = group.compute_cartesian_path(
        waypoints,  # waypoints to follow
        0.001,  # eef_step
        0.0)  # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction


def execute_plan(group, plan, bool_wait = True):
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    succeeded = group.execute(plan, wait=bool_wait)
    return succeeded

    ## **Note:** The robot's current joint state must be within some tolerance of the
    ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail

def main():
    ## First initialize moveit_commander and rospy.
    print("============ Starting dual arms moveit")
    moveit_commander.roscpp_initializer.roscpp_initialize(sys.argv)
    rospy.init_node('ur10_3_arm_moveit',
                  anonymous=True)

    PLANNING_GROUP_GRIPPER = "gripper"
    PLANNING_GROUP_ARM = "manipulator"
    PLANNING_NS = "/ur10_3/"

    ## Instantiate a RobotCommander object.  This object is an interface to
    ## the robot as a whole.
    robot = moveit_commander.RobotCommander("%srobot_description"%PLANNING_NS, ns="/ur10_3/")

    ## Instantiate the MoveGroupCommander objects.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the ur10
    ## arm and gripper. This interface can be used to plan and execute motions on the ur10
    ## arm and gripper.
    
    #gripper = robot.get_group(PLANNING_GROUP_GRIPPER)
    #arm = robot.get_group(PLANNING_GROUP_ARM)
    arm = moveit_commander.move_group.MoveGroupCommander(PLANNING_GROUP_ARM,"%srobot_description"%PLANNING_NS, ns="/ur10_3/")
    gripper = moveit_commander.move_group.MoveGroupCommander(PLANNING_GROUP_GRIPPER, "%srobot_description"%PLANNING_NS, ns="/ur10_3/")

    ## We create this DisplayTrajectory publisher which is used below to publish
    ## trajectories for RVIZ to visualize.
    display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory, queue_size=10)

    rospy.sleep(2)

    ## We can get the name of the reference frame for this robot
    print( "============ Reference frame: %s" % arm.get_planning_frame() )

    ## We can also print the name of the end-effector link for this group
    print( "============ Reference frame: %s" % arm.get_end_effector_link() )

    ## We can get a list of all the groups in the robot
    print( "============ Robot Groups:" )
    print( robot.get_group_names())
    
    #arm.set_planner_id("RRT")
    arm.set_num_planning_attempts(15)
    arm.allow_looking(True)
    arm.allow_replanning(True)

    #gripper.set_planner_id("RRTConnect")
    gripper.set_num_planning_attempts(15)
    gripper.allow_replanning(True)
    gripper.allow_looking(True)


    ## Planning to a Pose goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the 
    ## end-effector

    # Primer Cubo
    succeess = False
    all_ok = False
    while not all_ok:
        arm.set_named_target("home")
    
        arm.go(wait=True)
        gripper.set_named_target("gripper_open")
        gripper.go(wait=True)
        
        cartesian_plan, fraction = plan_cartesian_path_pose(arm, -0.54, 0, 0, 0, 1)
        succeess = execute_plan(arm, cartesian_plan)
        if succeess:
            cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, -0.143, 0, 0, 1)
            succeess = execute_plan(arm, cartesian_plan)
            if succeess:
                cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0, -0.47, 0, 1)
                succeess = execute_plan(arm, cartesian_plan)
                if succeess:
                    cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0, -0.07, 0, 1)
                    succeess = execute_plan(arm, cartesian_plan)
                    if succeess:
                        pos_check = arm.get_current_pose().pose.position
                        print(round(pos_check.x,2))
                        print(round(pos_check.y,2))
                        if round(pos_check.x,2) == -0.30:
                            if round(pos_check.y,2) == -0.77:
                                all_ok = True
                                print(arm.get_current_pose())
        print(succeess)
        print(all_ok)

    succeess = False
    all_ok = False
    while not all_ok:
    
        gripper.set_named_target("gripper_close")
        gripper.go(wait=True)

        arm.set_named_target("home")    
        arm.go(wait=True)

        cartesian_plan, fraction = plan_cartesian_path_pose(arm, -0.53, 0.0, 0.07, 0, 1)
        succeess = execute_plan(arm, cartesian_plan)
        if succeess:
            cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0.98, 0, 0, 0, 1)
            succeess = execute_plan(arm, cartesian_plan)
            if succeess:
                cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0.175, 0, 0, 1)
                succeess = execute_plan(arm, cartesian_plan)
                if succeess:
                    pos_check = arm.get_current_pose().pose.position
                    print(round(pos_check.x,2))
                    print(round(pos_check.y,2))
                    if round(pos_check.x,2) == 0.69:
                        if round(pos_check.y,2) == -0.46:
                            all_ok = True
                            print(arm.get_current_pose())
        print(succeess)

    succeess = False
    all_ok = False
    while not all_ok:
        gripper.set_named_target("gripper_open")
        gripper.go(wait=True)
    
        # Segundo Cubo
        arm.set_named_target("home")
        arm.go(wait=True)

        cartesian_plan, fraction = plan_cartesian_path_pose(arm, -0.53, 0, 0, 0, 1)
        succeess = execute_plan(arm, cartesian_plan)
        if succeess:
            cartesian_plan, fraction = plan_cartesian_path_pose(arm, -0.61, 0, 0, 0, 1)
            succeess = execute_plan(arm, cartesian_plan)
            if succeess:
                cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0, -0.42, 1.0)
                succeess = execute_plan(arm, cartesian_plan)
                if succeess:
                    cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0.132, 0, 0, 1)
                    succeess = execute_plan(arm, cartesian_plan)
                    if succeess:
                        cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0, -0.12, 0, 1)
                        succeess = execute_plan(arm, cartesian_plan)
                        if succeess:
                            pos_check = arm.get_current_pose().pose.position
                            print(round(pos_check.x,2))
                            print(round(pos_check.y,2))
                            if round(pos_check.x,2) == -0.9:
                                if round(pos_check.y,2) == -0.50:
                                    all_ok = True
                                    print(arm.get_current_pose())
    
    succeess = False
    all_ok = False
    while not all_ok:
        gripper.set_named_target("gripper_close")
        gripper.go(wait=True)

        arm.set_named_target("home")    
        arm.go(wait=True)

        cartesian_plan, fraction = plan_cartesian_path_pose(arm, -0.53, 0.0, 0.07, 0, 1)
        succeess = execute_plan(arm, cartesian_plan)
        if succeess:
            cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0.98, 0, 0, 0, 1)
            succeess = execute_plan(arm, cartesian_plan)
            if succeess:
                cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0.175, 0, 0, 1)
                succeess = execute_plan(arm, cartesian_plan)
                if succeess:
                    pos_check = arm.get_current_pose().pose.position
                    print(round(pos_check.x,2))
                    print(round(pos_check.y,2))
                    if round(pos_check.x,2) == 0.69:
                        if round(pos_check.y,2) == -0.46:
                            all_ok = True
                            print(arm.get_current_pose())    
    succeess = False
    all_ok = False
    while not all_ok:
        gripper.set_named_target("gripper_open")
        gripper.go(wait=True)
        
        # Tercer Cubo
        arm.set_named_target("home")
        arm.go(wait=True)
    
        cartesian_plan, fraction = plan_cartesian_path_pose(arm, -0.53, 0, 0, 0, 1)
        succeess = execute_plan(arm, cartesian_plan)
        if succeess:
            cartesian_plan, fraction = plan_cartesian_path_pose(arm, -0.385, 0, 0, 0, 1)
            succeess = execute_plan(arm, cartesian_plan)
            if succeess:
                cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0, -0.42, 1.0)
                succeess = execute_plan(arm, cartesian_plan)
                if succeess:
                    cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, -0.248, 0, 0, 1)
                    succeess = execute_plan(arm, cartesian_plan)
                    if succeess:
                        cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0, -0.12, 0, 1)
                        succeess = execute_plan(arm, cartesian_plan)
                        if succeess:
                            pos_check = arm.get_current_pose().pose.position
                            print(round(pos_check.x,2))
                            print(round(pos_check.y,2))
                            if round(pos_check.x,2) == -0.68:
                                if round(pos_check.y,2) == -0.88:
                                    all_ok = True
                                    print(arm.get_current_pose())
    succeess = False
    all_ok = False
    while not all_ok:
        gripper.set_named_target("gripper_close")
        gripper.go(wait=True)

        arm.set_named_target("home")    
        arm.go(wait=True)

        cartesian_plan, fraction = plan_cartesian_path_pose(arm, -0.53, 0.0, 0.07, 0, 1)
        succeess = execute_plan(arm, cartesian_plan)
        if succeess:
            cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0.98, 0, 0, 0, 1)
            succeess = execute_plan(arm, cartesian_plan)
            if succeess:
                cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0.175, 0, 0, 1)
                succeess = execute_plan(arm, cartesian_plan)
                if succeess:
                    pos_check = arm.get_current_pose().pose.position
                    print(round(pos_check.x,2))
                    print(round(pos_check.y,2))
                    if round(pos_check.x,2) == 0.69:
                        if round(pos_check.y,2) == -0.46:
                            all_ok = True
                            print(arm.get_current_pose())
    gripper.set_named_target("gripper_open")
    gripper.go(wait=True)

    arm.set_named_target("home")
    arm.go(wait=True)
    gripper.set_named_target("gripper_open")
    gripper.go(wait=True)

    ## When finished shut down moveit_commander.
    moveit_commander.roscpp_shutdown()

if __name__=='__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass
```

Fichero ~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_moveit/four_arm_moveit_manipulator/scripts/four_arm_moveit_4.py
```{C}
#!/usr/bin/env python
from __future__ import print_function
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi


    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))

# r, p and y in rads
def plan_cartesian_path_orientation(group, r, p, y, scale=1, bool_wait = True):
    joint_target = group.get_current_joint_values()
    joint_target[3] += scale * r
    joint_target[4] += scale * p
    joint_target[5] += scale * y
    succeeded = group.go(joint_target, wait=bool_wait)
    return succeeded

def plan_cartesian_path_pose(group, x, y, z, w, scale=1):
    waypoints = []

    wpose = group.get_current_pose().pose
    wpose.position.x += scale * x  # Move forward/backwards in (x)
    wpose.position.z += scale * z  # Move up/down (z)
    wpose.position.y += scale * y  # Move sideways (y)
    wpose.orientation.w += scale * w  # Rotation of the arm
    waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan, fraction) = group.compute_cartesian_path(
        waypoints,  # waypoints to follow
        0.001,  # eef_step
        0.0)  # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction


def execute_plan(group, plan, bool_wait = True):
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    succeeded = group.execute(plan, wait=bool_wait)
    return succeeded

    ## **Note:** The robot's current joint state must be within some tolerance of the
    ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail

def main():
    ## First initialize moveit_commander and rospy.
    print("============ Starting dual arms moveit")
    moveit_commander.roscpp_initializer.roscpp_initialize(sys.argv)
    rospy.init_node('ur10_4_arm_moveit',
                  anonymous=True)

    PLANNING_GROUP_GRIPPER = "gripper"
    PLANNING_GROUP_ARM = "manipulator"
    PLANNING_NS = "/ur10_4/"

    ## Instantiate a RobotCommander object.  This object is an interface to
    ## the robot as a whole.
    robot = moveit_commander.RobotCommander("%srobot_description"%PLANNING_NS, ns="/ur10_4/")

    ## Instantiate the MoveGroupCommander objects.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the ur10
    ## arm and gripper. This interface can be used to plan and execute motions on the ur10
    ## arm and gripper.
    
    #gripper = robot.get_group(PLANNING_GROUP_GRIPPER)
    #arm = robot.get_group(PLANNING_GROUP_ARM)
    arm = moveit_commander.move_group.MoveGroupCommander(PLANNING_GROUP_ARM,"%srobot_description"%PLANNING_NS, ns="/ur10_4/")
    gripper = moveit_commander.move_group.MoveGroupCommander(PLANNING_GROUP_GRIPPER, "%srobot_description"%PLANNING_NS, ns="/ur10_4/")

    ## We create this DisplayTrajectory publisher which is used below to publish
    ## trajectories for RVIZ to visualize.
    display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory, queue_size=10)

    rospy.sleep(2)

    ## We can get the name of the reference frame for this robot
    print( "============ Reference frame: %s" % arm.get_planning_frame() )

    ## We can also print the name of the end-effector link for this group
    print( "============ Reference frame: %s" % arm.get_end_effector_link() )

    ## We can get a list of all the groups in the robot
    print( "============ Robot Groups:" )
    print( robot.get_group_names())
    
    #arm.set_planner_id("RRT")
    arm.set_num_planning_attempts(15)
    arm.allow_looking(True)
    arm.allow_replanning(True)

    #gripper.set_planner_id("RRTConnect")
    gripper.set_num_planning_attempts(15)
    gripper.allow_replanning(True)
    gripper.allow_looking(True)


    ## Planning to a Pose goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the 
    ## end-effector

    # Primer Cubo
    succeess = False
    all_ok = False
    while not all_ok:
        arm.set_named_target("home")
    
        arm.go(wait=True)
        gripper.set_named_target("gripper_open")
        gripper.go(wait=True)
        
        cartesian_plan, fraction = plan_cartesian_path_pose(arm, -0.54, 0, 0, 0, 1)
        succeess = execute_plan(arm, cartesian_plan)
        if succeess:
            cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, -0.143, 0, 0, 1)
            succeess = execute_plan(arm, cartesian_plan)
            if succeess:
                cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0, -0.47, 0, 1)
                succeess = execute_plan(arm, cartesian_plan)
                if succeess:
                    cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0, -0.07, 0, 1)
                    succeess = execute_plan(arm, cartesian_plan)
                    if succeess:
                        pos_check = arm.get_current_pose().pose.position
                        print(round(pos_check.x,2))
                        print(round(pos_check.y,2))
                        if round(pos_check.x,2) == -0.30:
                            if round(pos_check.y,2) == -0.77:
                                all_ok = True
                                print(arm.get_current_pose())
        print(succeess)
        print(all_ok)

    succeess = False
    all_ok = False
    while not all_ok:
    
        gripper.set_named_target("gripper_close")
        gripper.go(wait=True)

        arm.set_named_target("home")    
        arm.go(wait=True)

        cartesian_plan, fraction = plan_cartesian_path_pose(arm, -0.53, 0.0, 0.07, 0, 1)
        succeess = execute_plan(arm, cartesian_plan)
        if succeess:
            cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0.98, 0, 0, 0, 1)
            succeess = execute_plan(arm, cartesian_plan)
            if succeess:
                cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0.175, 0, 0, 1)
                succeess = execute_plan(arm, cartesian_plan)
                if succeess:
                    pos_check = arm.get_current_pose().pose.position
                    print(round(pos_check.x,2))
                    print(round(pos_check.y,2))
                    if round(pos_check.x,2) == 0.69:
                        if round(pos_check.y,2) == -0.46:
                            all_ok = True
                            print(arm.get_current_pose())
        print(succeess)

    succeess = False
    all_ok = False
    while not all_ok:
        gripper.set_named_target("gripper_open")
        gripper.go(wait=True)
    
        # Segundo Cubo
        arm.set_named_target("home")
        arm.go(wait=True)

        cartesian_plan, fraction = plan_cartesian_path_pose(arm, -0.53, 0, 0, 0, 1)
        succeess = execute_plan(arm, cartesian_plan)
        if succeess:
            cartesian_plan, fraction = plan_cartesian_path_pose(arm, -0.61, 0, 0, 0, 1)
            succeess = execute_plan(arm, cartesian_plan)
            if succeess:
                cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0, -0.42, 1.0)
                succeess = execute_plan(arm, cartesian_plan)
                if succeess:
                    cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0.132, 0, 0, 1)
                    succeess = execute_plan(arm, cartesian_plan)
                    if succeess:
                        cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0, -0.12, 0, 1)
                        succeess = execute_plan(arm, cartesian_plan)
                        if succeess:
                            pos_check = arm.get_current_pose().pose.position
                            print(round(pos_check.x,2))
                            print(round(pos_check.y,2))
                            if round(pos_check.x,2) == -0.9:
                                if round(pos_check.y,2) == -0.50:
                                    all_ok = True
                                    print(arm.get_current_pose())
    
    succeess = False
    all_ok = False
    while not all_ok:
        gripper.set_named_target("gripper_close")
        gripper.go(wait=True)

        arm.set_named_target("home")    
        arm.go(wait=True)

        cartesian_plan, fraction = plan_cartesian_path_pose(arm, -0.53, 0.0, 0.07, 0, 1)
        succeess = execute_plan(arm, cartesian_plan)
        if succeess:
            cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0.98, 0, 0, 0, 1)
            succeess = execute_plan(arm, cartesian_plan)
            if succeess:
                cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0.175, 0, 0, 1)
                succeess = execute_plan(arm, cartesian_plan)
                if succeess:
                    pos_check = arm.get_current_pose().pose.position
                    print(round(pos_check.x,2))
                    print(round(pos_check.y,2))
                    if round(pos_check.x,2) == 0.69:
                        if round(pos_check.y,2) == -0.46:
                            all_ok = True
                            print(arm.get_current_pose())    
    succeess = False
    all_ok = False
    while not all_ok:
        gripper.set_named_target("gripper_open")
        gripper.go(wait=True)
        
        # Tercer Cubo
        arm.set_named_target("home")
        arm.go(wait=True)
    
        cartesian_plan, fraction = plan_cartesian_path_pose(arm, -0.53, 0, 0, 0, 1)
        succeess = execute_plan(arm, cartesian_plan)
        if succeess:
            cartesian_plan, fraction = plan_cartesian_path_pose(arm, -0.385, 0, 0, 0, 1)
            succeess = execute_plan(arm, cartesian_plan)
            if succeess:
                cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0, -0.42, 1.0)
                succeess = execute_plan(arm, cartesian_plan)
                if succeess:
                    cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, -0.248, 0, 0, 1)
                    succeess = execute_plan(arm, cartesian_plan)
                    if succeess:
                        cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0, -0.12, 0, 1)
                        succeess = execute_plan(arm, cartesian_plan)
                        if succeess:
                            pos_check = arm.get_current_pose().pose.position
                            print(round(pos_check.x,2))
                            print(round(pos_check.y,2))
                            if round(pos_check.x,2) == -0.68:
                                if round(pos_check.y,2) == -0.88:
                                    all_ok = True
                                    print(arm.get_current_pose())
    succeess = False
    all_ok = False
    while not all_ok:
        gripper.set_named_target("gripper_close")
        gripper.go(wait=True)

        arm.set_named_target("home")    
        arm.go(wait=True)

        cartesian_plan, fraction = plan_cartesian_path_pose(arm, -0.53, 0.0, 0.07, 0, 1)
        succeess = execute_plan(arm, cartesian_plan)
        if succeess:
            cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0.98, 0, 0, 0, 1)
            succeess = execute_plan(arm, cartesian_plan)
            if succeess:
                cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0.175, 0, 0, 1)
                succeess = execute_plan(arm, cartesian_plan)
                if succeess:
                    pos_check = arm.get_current_pose().pose.position
                    print(round(pos_check.x,2))
                    print(round(pos_check.y,2))
                    if round(pos_check.x,2) == 0.69:
                        if round(pos_check.y,2) == -0.46:
                            all_ok = True
                            print(arm.get_current_pose())
    gripper.set_named_target("gripper_open")
    gripper.go(wait=True)

    arm.set_named_target("home")
    arm.go(wait=True)
    gripper.set_named_target("gripper_open")
    gripper.go(wait=True)

    ## When finished shut down moveit_commander.
    moveit_commander.roscpp_shutdown()

if __name__=='__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass
```

Para ejecutar la prueba:
```{bash}
# Terminal 1
roslaunch four_arm_moveit_manipulator four_arm_moveit_gazebo.launch

# Terminal 2
roslaunch four_arm_moveit_manipulator four_arm_moveit_execution.launch

# Terminal 3
rosrun four_arm_moveit_manipulator four_arm_moveit_1.py 

# Terminal 4
rosrun four_arm_moveit_manipulator four_arm_moveit_2.py

# Terminal 5
rosrun four_arm_moveit_manipulator four_arm_moveit_3.py

# Terminal 6
rosrun four_arm_moveit_manipulator four_arm_moveit_4.py
```

En la terminal al lanzar rosrun puede salir el siguiente mensaje:
![ ](/doc/imgs_md/two_arm_moveit_manipulator_info.png  "")
No ha solución para este mensaje, ya que no es un error de funcionamiento. 

Puede indicar varias razones, porque ha realizado el planining pero los controladores no son capaces de llevarlo acabo, pero si se reintenta llega un momento que lo realiza correctamente, por tanto no es un problema con los controladores.

Y no es un problema del planning porque da una solución.

Afecta en gran medida al rendimiento a la hora de realizar una tarea, ya que tiene que realizar las operaciones varias veces hasta que obtiene la solución correcta.

Se puede aplicar alguna o varias soluciones como trabajo futuro:
- Utilizar un diccionario para almacenar las posiciones con sus correspondientes valores articulares
- Utilizar la utilidad de MoveIt! para implementar *constraints* a las soluciones del planificador
- Utilizar Interligencia artificial para decidir que soluciones propuestas son las más adecuadas
- Puede que tambien sea un problmea de recursos disponibles en el ordenador personal
- También puede tener que ver con el tipo de controlador que se está usando, es decir, *FollowJointTrajectory*, se podría usar el de velocidad y comporbar si funciona adecuadamente.


En la Terminal de Gazebo, al lanzar todos los comandos para realizar las pruebas se obtiene el siguiente warning:
![ ](/doc/imgs_md/four_arm_move_it_gazebo_warning.png  "Warning por flta de recurosos")
Esto es debido a que el sistema avisa de que los recursos son insuficientes para realizar la simulacóin fluidamente, lo que puede acarrear problemas en su velocidad de simulción que puede afectar al resultado de este.
