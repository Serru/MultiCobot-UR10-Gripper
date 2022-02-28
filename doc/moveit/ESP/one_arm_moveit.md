<!--- Para un robot  opción B--->
# Instalación y configuración para un único robot UR10 con MoveIt!

## Requisito previo
- Realizar correctamente la instalación de la [configuración base del sistema](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/setup-doc/proyect_setup.md).

## Índice
- [Fase 1: Configuración del URDF](#fase1)
- [Fase 2: Configuración de MoveIt!](#fase2)
- [Fase 3: Simulación de un `pick & place` en Gazebo](#fase3)
- [Ejecución de las pruebas](#pruebas)


<a name="fase1">
  <h2>
Fase 1: Configuración del URDF
  </h2>
</a>

### Descripción del fichero URDF
El fichero URDF (United Robotics Description Format) modela el cobot utilizando el formato XML el cual será utilizado por las diferentes aplicaciones que ROS necesite, pero principalmente para realizar una simulación del robot modelado.

El fichero está construido en forma de árbol, en donde hay tres etiquetas principales: `<robot>`, `<link>` y `<joint>`. Para explicarlo bien, se puede tomar como referencia el brazo del cuerpo humano. Si lo que se quiere modelar es el brazo de una persona, la etiqueta `<robot>` representarı́a al brazo en su conjunto. Este brazo está compuesto de varios huesos (húmero, cúbito y radio) que son
representados por las etiquetas `<link>` y por una articulación que une esos huesos (codo) que es representado por la etiqueta `<joint>`. 

Además como en los huesos, estas etiquetas pueden ir con información adicional contenida en ellas que den información del tamaño, geometrı́a, inercia, orientación etc. Finalmente, el modelado de un robot se puede unir a otro modelo y formar uno más complejo,
que podrı́a ser representado con la adición de la mano al brazo, con la muñeca como articulación que conectan ambos. Hay que tener en cuenta que las etiquetas `<joint>` conecta las etiquetas `<link>` a través de una relación padre-hijo.

Dicho esto, se realiza una representación de los componentes del robot:
 
 ![image](/doc/imgs_md/urdf-robot.png  "Representación del fichero URDF")

En la imagen se representa el contenido del [fichero URDF](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_description/urdf/ur10_robot.urdf.xacro) que modela el robot junto a la pinza, se puede ver cómo se conecta el componente del brazo UR10 robot con el link `world`, representando world (color amarillo) y la base del brazo del UR10 `base_link` (color verde) situado justo encima, además el joint `world_joint` es la esfera de color amarillo situado entre ambos links. De la misma manera se tiene el componente de la pinza `robotiq_85_gripper`, está conectado al brazo del UR10 (ur10 robot), en donde la esfera que representa el joint `robotiq_85_base_joint` que une ambos componentes (color morado), uniendo el link `robotiq_85_base_link` de la pinza con el link `ee_link` del brazo de UR10.

### Configuración del directorio descripción
Se crea un nuevo paquete y se copia los directorios del proyecto *one_arm_no_moveit* para su posterior modificación.

```{bash}
cd ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit
catkin_create_pkg one_arm_moveit_description rospy
```

En el directorio creado para description, se copiara del directorio de *one_arm_no_moveit_description*, las carpetas *launch* y *urdf*.

```{bash}
cd ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit/one_arm_moveit_description
cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_description/launch .
cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_description/urdf .
```

### Modificación de los ficheros de description

* [~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit/one_arm_moveit_description/launch/ur10_upload.launch](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_description/launch/ur10_upload.launch)

* [~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit/one_arm_moveit_description/urdf/ur10_robot.urdf.xacro](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_description/urdf/ur10_robot.urdf.xacro)

* [~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit/one_arm_moveit_description/urdf/ur10_joint_limited_robot.urdf.xacro](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_description/urdf/ur10_joint_limited_robot.urdf.xacro)

Se compila:
```{bash}
cd ~/MultiCobot-UR10-Gripper
catkin_make
```

<a name="fase2">
  <h2>
Fase 2: Configuración de MoveIt!
  </h2>
</a>

### Configuración de MoveIt!

Tras la preparación de los directorios que contiene el modelo de robot (descripton) y para su simulación (Gazebo), se procede a configurar MoveIt!

```{bash}
cd ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit/
mkdir one_arm_moveit_config
roslaunch moveit_setup_assistant setup_assistant.launch
```

#### MoveIt! Setup Assistant

Se va a escoger como modelo del robot el fichero URDF: *~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit/one_arm_moveit_description/urdf/ur10_joint_limited_robot.urdf.xacro* (podría ser perfectamente ur10_robot.urdf.xacro).

![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_1.png  "Cargar el modelo URDF del robot UR10")

Posteriormente, se le da al boton *Load Files*.
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_2.png  "Cargado el modelo URDF del robot UR10")

En la pestaña *Self-Collisions*, darle al boton *Generate Collision Matrix*, o que generará una matriz en donde los diferentes componentes del robot puedan tener colisiones:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_3.png  "Generación de matirz de colisiones")

En la pestaña *Virtual Joints*, hay que crear un joint entre la base del robot y el frame que lo contiene, en este caso, *world*, siendo la configuración la siguiente:
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

En la pestaña *Robot Poses* se va a configurar uno denominado *home*:

![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_12.png  "Configurando *home* 1/3")

![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_13.png  "Configurando *home* 2/3")

![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_14.png  "Configurando *home* 3/3")

![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_24.png  "Configurando *gripper open*")

![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_25.png  "Configurando *gripper close*")


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


<a name="fase3">
  <h2>
Fase 3: Simulación de un `pick & place` en Gazebo
  </h2>
</a>

### Creación del directorio para la solución
```{bash}
cd ~/MultiCobot-UR10-Gripper/src/multirobot
mkdir one_arm_moveit
```

### Puesta en marcha de Gazebo
Se va a crear el paquete para gazebo, y copiar el contenido de la solución partiendo del la solución *one_arm_no_moveit* para su posterior modificación:
```{bash}
cd ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit
catkin_create_pkg one_arm_moveit_gazebo rospy
```
En el directorio creado para gazebo, se copiara del directorio de *one_arm_no_moveit_gazebo*, las carpetas *controller*, *launch*, *models*, y *world*.
```{bash}
cd ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit/one_arm_moveit_gazebo
cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/controller .
cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/launch .
cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/models .
cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/world .
```

### Modificación de los ficheros de gazebo

* *~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit/one_arm_moveit_gazebo/launch/ur10_joint_limited.launch*

```{xml}
<?xml version="1.0"?>
<launch>
  <arg name="gui" default="true" doc="Starts gazebo gui" />

  <include file="$(find one_arm_moveit_gazebo)/launch/ur10.launch">
    <arg name="limited" value="true"/>
    <arg name="gui" value="$(arg gui)"/>
  </include>

</launch>
```

* *~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit/one_arm_moveit_gazebo/launch/ur10.launch*

```{xml}
<?xml version="1.0"?>
<launch>
  <arg name="limited" default="false"  doc="If true, limits joint range [-PI, PI] on all joints." />
  <arg name="paused" default="false" doc="Starts gazebo in paused mode" />
  <arg name="gui" default="true" doc="Starts gazebo gui" />
  
  <arg name="world" default="$(find one_arm_moveit_gazebo)/world/multiarm_bot.world" />

  <!-- startup simulated world -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" default="$(arg world)"/>
    <arg name="paused" value="$(arg paused)"/>
    <arg name="gui" value="$(arg gui)"/>
  </include>

  <!-- send robot urdf to param server -->
  <include file="$(find one_arm_moveit_description)/launch/ur10_upload.launch">
    <arg name="limited" value="$(arg limited)"/>
  </include>

  <!-- push robot_description to factory and spawn robot in gazebo -->
  <node name="spawn_gazebo_model" pkg="gazebo_ros" type="spawn_model" respawn="false" output="screen" args="-urdf 
                          -param robot_description 
                          -model robot 
                          -x 0.6 
                          -y -0.6 
                          -z 1.1
                          -J shoulder_pan_joint 2.17
                          -J shoulder_lift_joint -1.52  
                          -J elbow_joint -1.67  
                          -J wrist_1_joint -1.51  
                          -J wrist_2_joint 1.58 
                          -J wrist_3_joint -1.01" />
  <!--[2.1760905965722426, -1.5183196482450523, -1.6715145082510372, -1.5111554264043052, 1.5891351190432503, -1.014601195078888]-->


  <include file="$(find one_arm_moveit_gazebo)/launch/controller_utils.launch"/>

  <!-- start this controller -->
  <rosparam file="$(find one_arm_moveit_gazebo)/controller/arm_controller_ur10.yaml" command="load"/>
  <node name="arm_controller_spawner" pkg="controller_manager" type="controller_manager" args="spawn arm_controller" respawn="false" output="screen"/>

  <!-- robotiq_85_gripper controller -->
  <rosparam file="$(find one_arm_moveit_gazebo)/controller/gripper_controller_robotiq.yaml" command="load"/> 
  <node name="gripper_controller_spawner" pkg="controller_manager" type="spawner" args="gripper" />

  <!-- load other controllers -->
  <node name="ros_control_controller_manager" pkg="controller_manager" type="controller_manager" respawn="false" output="screen" args="load joint_group_position_controller" />

</launch>
```

Se compila:
```{bash}
cd ~/MultiCobot-UR10-Gripper
catkin_make
```



### Conexión entre Gazebo y MoveIt!
Si se deja la configuración como está, no es posible la comunicación entre Gazebo y MoveIt!. Al lanzar el fichero *demo.launch* también lanza Rviz lo que permite la visualización de los datos y se aprecia que funciona correctamente, pero con la configuración automatizada, no es posible observar su funcionamiento en el simulador de Gazebo.

Esto es debido a que no hay comunicación entre MoveIt! y Gazebo, con lo que se procede a solucionar esta comunicación.

Por conveniencia, para su posible testeo en el robot, real se mantendrá el lanzamiento Gazebo y MoveIt! separados.

Para comenzar y entender mejor el paquete de MoveIt!, se va a separar el contenido de los paquetes en MoveIt! y Gazebo:

Gazebo:
- launch:
	- gazebo.launch
	- ros_controllers.launch
- config:
	- **ros_controllers.yaml**

No es necesario el gazebo.launch que provee MoveIt!, ya que se ha configurado previamente el paquete de gazebo que se usará. La configuración es muy similar si se da una ojeada a los ficheros involucrados.

MoveIt!:
- launch:
	- **demo.launch**
		- planning_context.launch
			- config:
				- ur10.srdf
				- joint_limits.yaml
				- kinematics.yaml
		- **move_group.launch**
			- planning_context.launch
			- config:
				- ur10.srdf
				- joint_limits.yaml
				- kinematics.yaml
			- planning_pipeline.launch.xml
				- ompl_planning_pipeline.launch.xml
			- **trajectory_execution.launch.xml**
				- **ur10_moveit_controller_manager.launch.xml**
				- config:
					- **ros_controllers.yaml**
			- sensor_manager.launch.xml
				- config:
					- sensors_3d.yaml
				- ur10_moveit_sensor_manager.launch.xml
			- moveit_rviz.launch (Rviz)
				- moveit.rviz
				- config:
					- kinematics.yaml
		
		
Se puede apreciar en el esquema de los directorios comó están relacionados los ficheros. Entre ellos los que se van a modificar manualmente están señalados en negrita.

El fichero de configuración de los controladores es el mismo tanto para Gazebo como para MoveIt!, esto implica que para conectar los controladores adecuadamente, su configurción debe ser la misma que la se ha realizado para Gazebo previamente y eso está configurado en la sección *Puesta en marcha en Gazebo*.

Por tanto se puede añadir los controladores (ficheros .yaml) de Gazebo en el paquete de MoveIt! para conectar correctamente los controladores del robot (manipulator y gripper).

Si se lanza gazebo y moveit! sin realizar ninguna modificación, se obtiene la siguiente gráfica de nodos y topics:
```{bash}
# terminal 1
roslaunch one_arm_moveit_config demo.launch

# terminal 2
roslaunch one_arm_moveit_gazebo ur10_joint_limited.launch
```
![ ](/doc/imgs_md/one_arm_moveit_graph_no_changes.png  "Esquema sin cambios")
		

Se aprecia que MoveIt! no tiene ninguna conexión con los controladores lanzados desde Gazebo, y esto hace que los comandos de trayectorias realizadas desde MoveIt! no sean representadas en Gazebo. 

Por tanto hay que modificar los siguientes ficheros. Los ficheros modificados serán estarán como un paquete diferente con la intención de facilitar la compresión de las modificaciones realizadas sobre la configuración.

Para ello se va a modificar los siguientes ficheros teniendo como base sus ficheros originales *demo.launch*, *move_group.launch*, *trajectory_execution.launch.xml*  y *ur10_moveit_controller_manager.launch.xml* y se añadirá los controladores creando dos ficheros *controllers.yaml* y *joint_names.yaml*:

```{bash}
cd ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit
catkin_create_pkg one_arm_moveit_manipulator rospy
mkdir config
touch config/controllers.yaml
touch config/joint_names.yaml
mkdir launch
touch launch/one_arm_moveit_execution.launch
touch launch/move_group.launch
touch launch/trajectory_execution.launch.xml
touch launch/ur10_moveit_controller_manager.launch.xml
mkdir scripts
```
Se va a proceder a añadir los controladores para su interacción con Gazebo:

Fichero *~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit/one_arm_moveit_manipulator/config/controllers.yaml*
```{yaml}
controller_list:
  - name: "arm_controller"
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:
      - shoulder_pan_joint
      - shoulder_lift_joint
      - elbow_joint
      - wrist_1_joint
      - wrist_2_joint
      - wrist_3_joint
  - name: "gripper"
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:
      - robotiq_85_left_knuckle_joint
```

Otro fichero a modificar es *~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit/one_arm_moveit_manipulator/config/joint_names.yaml*

```{yaml}
controller_joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint]
```

De los ficheros en el direcotrio *launch*, se va a modificar el fichero *one_arm_moveit_execution* que es el punto de entrada para usar el paquete de MoveIt! y Rviz que tiene como base el fichero *demo.launch*.

Fichero: *~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit/one_arm_moveit_manipulator/launch/one_arm_moveit_execution.launch*

```{xml}
<launch>
    <arg name="sim" default="false" />
    <arg name="debug" default="false" />
    <!-- By default, we do not start a database (it can be large) -->
    <arg name="demo" default="false" />

    <rosparam command="load" file="$(find one_arm_moveit_manipulator)/config/joint_names.yaml"/>

    <include file="$(find one_arm_moveit_config)/launch/planning_context.launch">
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

    <include file="$(find one_arm_moveit_manipulator)/launch/move_group.launch">
      <arg name="debug" default="$(arg debug)" />
      <arg name="publish_monitored_planning_scene" value="true"/>
      <!--arg name="info" value="true"/-->
    </include>

    <!-- If database loading was enabled, start mongodb as well -->
    <include file="$(find one_arm_moveit_config)/launch/default_warehouse_db.launch" if="$(arg demo)"/>

    <!-- Remap follow_joint_trajectory -->
    <remap from="/follow_joint_trajectory" to="/arm_controller/follow_joint_trajectory"/>

    <include file="$(find one_arm_moveit_config)/launch/moveit_rviz.launch">
      <arg name="config" value="true"/>
      <arg name="debug" default="false"/>
    </include>

</launch>
```

Después retocar el lanzador de los controladores y MoveIt!:

Fichero: *~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit/one_arm_moveit_manipulator/launch/move_group.launch*

```{xml}
<launch>

  <include file="$(find one_arm_moveit_config)/launch/planning_context.launch" />

  <!-- GDB Debug Option -->
  <arg name="debug" default="false" />
  <arg unless="$(arg debug)" name="launch_prefix" value="" />
  <arg     if="$(arg debug)" name="launch_prefix"
	   value="gdb -x $(find one_arm_moveit_config)/launch/gdb_settings.gdb --ex run --args" />

  <!-- Verbose Mode Option -->
  <arg name="info" default="$(arg debug)" />
  <arg unless="$(arg info)" name="command_args" value="" />
  <arg     if="$(arg info)" name="command_args" value="--debug" />

  <!-- move_group settings -->
  <arg name="allow_trajectory_execution" default="true"/>
  <arg name="fake_execution" default="false"/>
  <arg name="max_safe_path_cost" default="1"/>
  <arg name="jiggle_fraction" default="0.05" />
  <arg name="publish_monitored_planning_scene" default="true"/>

  <arg name="capabilities" default=""/>
  <arg name="disable_capabilities" default=""/>
  <!-- load these non-default MoveGroup capabilities (space seperated) -->
  <!--
  <arg name="capabilities" value="
                a_package/AwsomeMotionPlanningCapability
                another_package/GraspPlanningPipeline
                " />
  -->

  <!-- inhibit these default MoveGroup capabilities (space seperated) -->
  <!--
  <arg name="disable_capabilities" value="
                move_group/MoveGroupKinematicsService
                move_group/ClearOctomapService
                " />
  -->

  <!-- Planning Functionality -->
  <include ns="move_group" file="$(find one_arm_moveit_config)/launch/planning_pipeline.launch.xml">
    <arg name="pipeline" value="ompl" />
  </include>

  <!-- Trajectory Execution Functionality -->
  <include ns="move_group" file="$(find one_arm_moveit_manipulator)/launch/trajectory_execution.launch.xml" if="$(arg allow_trajectory_execution)">
    <arg name="moveit_manage_controllers" value="true" />
    <arg name="moveit_controller_manager" value="ur10" unless="$(arg fake_execution)"/>
    <arg name="moveit_controller_manager" value="fake" if="$(arg fake_execution)"/>
  </include>

  <!-- Sensors Functionality -->
  <include ns="move_group" file="$(find one_arm_moveit_config)/launch/sensor_manager.launch.xml" if="$(arg allow_trajectory_execution)">
    <arg name="moveit_sensor_manager" value="ur10" />
  </include>

  <!-- Start the actual move_group node/action server -->
  <node name="move_group" launch-prefix="$(arg launch_prefix)" pkg="moveit_ros_move_group" type="move_group" respawn="false" output="screen" args="$(arg command_args)">
    <!-- Set the display variable, in case OpenGL code is used internally -->
    <env name="DISPLAY" value="$(optenv DISPLAY :0)" />

    <param name="allow_trajectory_execution" value="$(arg allow_trajectory_execution)"/>
    <param name="max_safe_path_cost" value="$(arg max_safe_path_cost)"/>
    <param name="jiggle_fraction" value="$(arg jiggle_fraction)" />
    <param name="capabilities" value="$(arg capabilities)"/>
    <param name="disable_capabilities" value="$(arg disable_capabilities)"/>

    <!-- MoveGroup capabilities to load -->
    <param name="capabilities" value="move_group/MoveGroupCartesianPathService
              move_group/MoveGroupExecuteTrajectoryAction
              move_group/MoveGroupKinematicsService
              move_group/MoveGroupMoveAction
              move_group/MoveGroupPickPlaceAction
              move_group/MoveGroupPlanService
              move_group/MoveGroupQueryPlannersService
              move_group/MoveGroupStateValidationService
              move_group/MoveGroupGetPlanningSceneService
              move_group/ClearOctomapService
              " />


    <!-- Publish the planning scene of the physical robot so that rviz plugin can know actual robot -->
    <param name="planning_scene_monitor/publish_planning_scene" value="$(arg publish_monitored_planning_scene)" />
    <param name="planning_scene_monitor/publish_geometry_updates" value="$(arg publish_monitored_planning_scene)" />
    <param name="planning_scene_monitor/publish_state_updates" value="$(arg publish_monitored_planning_scene)" />
    <param name="planning_scene_monitor/publish_transforms_updates" value="$(arg publish_monitored_planning_scene)" />
  </node>

</launch>
```

Fichero: *~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit/one_arm_moveit_manipulator/launch/trajectory_execution.launch.xml*

```{xml}
<launch>

  <!-- This file makes it easy to include the settings for trajectory execution  -->

  <!-- Flag indicating whether MoveIt! is allowed to load/unload  or switch controllers -->
  <arg name="moveit_manage_controllers" default="true"/>
  <param name="moveit_manage_controllers" value="$(arg moveit_manage_controllers)"/>

  <!-- When determining the expected duration of a trajectory, this multiplicative factor is applied to get the allowed duration of execution -->
  <param name="trajectory_execution/allowed_execution_duration_scaling" value="1.2"/> <!-- default 1.2 -->
  <!-- Allow more than the expected execution time before triggering a trajectory cancel (applied after scaling) -->
  <param name="trajectory_execution/allowed_goal_duration_margin" value="0.5"/> <!-- default 0.5 -->
  <!-- Allowed joint-value tolerance for validation that trajectory's first point matches current robot state -->
  <param name="trajectory_execution/allowed_start_tolerance" value="0.01"/> <!-- default 0.01 -->

  <!-- Load the robot specific controller manager; this sets the moveit_controller_manager ROS parameter -->
  <arg name="moveit_controller_manager" default="ur10" />
  <include file="$(find one_arm_moveit_manipulator)/launch/$(arg moveit_controller_manager)_moveit_controller_manager.launch.xml" />

</launch>
```

Fichero: *~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit/one_arm_moveit_manipulator/launch/ur10_moveit_controller_manager.launch.xml*

```{xml}
<launch>

  <!-- loads moveit_controller_manager on the parameter server which is taken as argument 
    if no argument is passed, moveit_simple_controller_manager will be set -->
  <!--arg name="moveit_controller_manager" default="moveit_simple_controller_manager/MoveItSimpleControllerManager" />
  <param name="moveit_controller_manager" value="$(arg moveit_controller_manager)"/-->

  <!-- loads ros_controllers to the param server -->
  <!--rosparam file="$(find one_arm_moveit_config)/config/ros_controllers.yaml"/-->

  <!-- loads ros_controllers to the param server -->
  <rosparam file="$(find one_arm_moveit_manipulator)/config/controllers.yaml"/>
  <!--rosparam file="$(find robotiq_85_moveit_config)/config/controllers.yaml"/-->
  <param name="use_controller_manager" value="false"/>
  <param name="trajectory_execution/execution_duration_monitoring" value="false"/>
  <param name="moveit_controller_manager" value="moveit_simple_controller_manager/MoveItSimpleControllerManager"/>

</launch>
```

Finalmente, se realiza una prueba:
```{bash}
cd ~/MultiCobot-UR10-Gripper
catkin_make

## Terminal 1
roslaunch one_arm_moveit_gazebo ur10_joint_limited.launch

## Terminal 2
roslaunch one_arm_moveit_manipulator one_arm_moveit_execution.launch
```
![ ](/doc/imgs_md/one_arm_moveit_26.png  "Example gazebo+rviz+moveit! (1/2)")
		
![ ](/doc/imgs_md/one_arm_moveit_27.png  "Example gazebo+rviz+moveit! (2/2)")

Y la gráfica de los nodos y los topis, despues de las modificaciones, se puede apreciar cómo ahorá el nodo *move_group* tiene comunicación con los controladores.

![ ](/doc/imgs_md/one_arm_moveit_graph_changes.png  "rqt_graph representación de los nodos y los topics")

### Pick and Place
Como en las soluciones anteriores, se procederáa realizar unas pruebas muy sencillas. Para ello primero hay que crear los scripts necesarios para controlar el brazo del robot y el gripper correctamente y posterioremente, se realizará los mismos movimientos que en las soluciones anteriormente propuestas.

```{bash}
cd scripts
touch one_arm_moveit.py
```

Fichero ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit/one_arm_moveit_manipulator/scripts/one_arm_movei.py
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
    ## BEGIN_SUB_TUTORIAL plan_cartesian_path
    ##
    ## Cartesian Paths
    ## ^^^^^^^^^^^^^^^
    ## You can plan a Cartesian path directly by specifying a list of waypoints
    ## for the end-effector to go through:
    ##
    waypoints = []

    wpose = group.get_current_pose().pose
    wpose.position.x += scale * x  # Move forward/backwards in (x)
    wpose.position.z += scale * z  # Move up/down (z)
    wpose.position.y += scale * y  # Move sideways (y)
    wpose.orientation.w += scale * w  # Rotation of the arm
    waypoints.append(copy.deepcopy(wpose))

    # wpose = group.get_current_rpy().pose
    # wpose.position.x += scale * x  # Move forward/backwards in (x)
    # wpose.position.z -= scale * z  # Move up/down (z)
    # wpose.position.y += scale * y  # Move sideways (y)
    # waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan, fraction) = group.compute_cartesian_path(
        waypoints,  # waypoints to follow
        0.01,  # eef_step
        0.0)  # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction


def execute_plan(group, plan, bool_wait  = True):
    ## BEGIN_SUB_TUTORIAL execute_plan
    ##
    ## Executing a Plan
    ## ^^^^^^^^^^^^^^^^
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    succeeded = group.execute(plan, wait=bool_wait)
    return succeeded

    ## **Note:** The robot's current joint state must be within some tolerance of the
    ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
    ## END_SUB_TUTORIAL

def error_back_initial_state(arm, gripper, y, ymax):
    
    succeeded = False
    print("\033[91m ur10_1 Error Point 1 - Open Gripper \033[0m")
    while (not succeeded) and (y < ymax):
        gripper.set_named_target("open")
        succeeded = gripper.go(wait=True)
        if succeeded and (y < ymax):
          print("\033[92m Completed trajectory succesfully \033[0m")
        y+=1

    succeeded = False
    print("\033[91m ur10_1 Error Point 2 - Moving arm up \033[0m")
    while (not succeeded) and (y < ymax):
        
        cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0.0, 0, 0.7, 0, 1)
        succeeded = execute_plan(arm, cartesian_plan)
        if succeeded and (y < ymax):
          print("\033[92m Completed trajectory succesfully \033[0m")
        y+=1

    succeeded = False
    print("\033[91m ur10_1 Error Point 3 - back home \033[0m")
    while (not succeeded) and (y < ymax):
        arm.set_named_target("home")
        succeeded = arm.go(wait=True)
        if succeeded and (y < ymax):
          print("\033[92m Completed trajectory succesfully \033[0m")
        y+=1

    if (not succeeded) and (y > ymax):
        print("\033[91m ur10_1 Error Point 4 - Too many errors exiting \033[0m")
        moveit_commander.roscpp_shutdown()
        #error_back_initial_state(arm, gripper, 0, ymax)

def main():
    ## BEGIN_TUTORIAL
    ##
    ## Setup
    ## ^^^^^
    ## CALL_SUB_TUTORIAL imports
    ##
    ## First initialize moveit_commander and rospy.
    print("============ Starting dual arms moveit")
    #moveit_commander.roscpp_initialize(sys.argv)
    moveit_commander.roscpp_initializer.roscpp_initialize(sys.argv)
    rospy.init_node('ur10_1_dual_moveit',
                  anonymous=True)

    PLANNING_GROUP_GRIPPER = "gripper"
    PLANNING_GROUP_ARM = "manipulator"
    PLANNING_NS = ""

    ## Instantiate a RobotCommander object.  This object is an interface to
    ## the robot as a whole.
    robot = moveit_commander.RobotCommander("%srobot_description"%PLANNING_NS, ns="")

    ## Instantiate the MoveGroupCommander objects.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the ur10
    ## arm and gripper. This interface can be used to plan and execute motions on the ur10
    ## arm and gripper.
    
    #gripper = robot.get_group(PLANNING_GROUP_GRIPPER)
    #arm = robot.get_group(PLANNING_GROUP_ARM)
    arm = moveit_commander.move_group.MoveGroupCommander(PLANNING_GROUP_ARM,"%srobot_description"%PLANNING_NS, ns="")
    gripper = moveit_commander.move_group.MoveGroupCommander(PLANNING_GROUP_GRIPPER, "%srobot_description"%PLANNING_NS, ns="")

    ## Instantiate a PlanningSceneInterface object.  This object is an interface
    ## to the world surrounding the robot.
    scene = moveit_commander.PlanningSceneInterface()

    ## We create this DisplayTrajectory publisher which is used below to publish
    ## trajectories for RVIZ to visualize.
    display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory, queue_size=10)

    rospy.sleep(2)
    #scene.remove_world_object("floor")
    
    # publish a demo scene
    #p = geometry_msgs.msg.PoseStamped()
    #p.header.frame_id = robot.get_planning_frame()
    #p.pose.position.x = 0.0
    #p.pose.position.y = 0.0
    #p.pose.position.z = -0.01
    #p.pose.orientation.w = 1.0
    #scene.add_box("floor", p, (2.0, 2.0, 0.02))
    
    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    ##
    ## We can get the name of the reference frame for this robot
    print( "============ Reference frame: %s" % arm.get_planning_frame() )

    ## We can also print the name of the end-effector link for this group
    print( "============ Reference frame: %s" % arm.get_end_effector_link() )

    ## We can get a list of all the groups in the robot
    print( "============ Robot Groups:" )
    print( robot.get_group_names())

    ## Sometimes for debugging it is useful to print the entire state of the
    ## robot.
    #print( "============ Printing robot state" )
    #print( robot.get_current_state())
    #print( "============" )
    
    #arm.set_planner_id("RRT")
    #arm.set_num_planning_attempts(15)
    #arm.allow_looking(True)
    #arm.allow_replanning(True)

    #gripper.set_planner_id("RRTConnect")
    #gripper.set_num_planning_attempts(15)
    #gripper.allow_replanning(True)
    #gripper.allow_looking(True)


    ## Planning to a Pose goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the 
    ## end-effector

    # Primer Cubo
    arm.set_named_target("home")
    print(arm.get_current_pose())
    #x: 0.239820825156
    #y: -0.631965677562
    #z: 0.62069143596

    arm.go(wait=True)
    gripper.set_named_target("gripper_open")
    gripper.go(wait=True)
    
    cartesian_plan, fraction = plan_cartesian_path_pose(arm, -0.54, 0, 0, 0, 1)
    execute_plan(arm, cartesian_plan)
    cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, -0.143, 0, 0, 1)
    execute_plan(arm, cartesian_plan)
    cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0, -0.47, 0, 1)
    execute_plan(arm, cartesian_plan)
    cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0, -0.07, 0, 1)

    execute_plan(arm, cartesian_plan)
    gripper.set_named_target("gripper_close")
    gripper.go(wait=True)
    
    cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0.01, 0.0, 0.07, 0, 1)
    print(arm.get_current_pose())
    execute_plan(arm, cartesian_plan)
    cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0.98, 0, 0, 0, 1)
    execute_plan(arm, cartesian_plan)
    cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0.175, 0, 0, 1)
    print(arm.get_current_pose())
    execute_plan(arm, cartesian_plan)
    gripper.set_named_target("gripper_open")
    gripper.go(wait=True)
    
    # Segundo Cubo
    arm.set_named_target("home")
    arm.go(wait=True)
    #x: 0.239820825156 (-0.24)
    #y: -0.631965677562 (0.632)
    #z: 0.62069143596   (0.62)

    cartesian_plan, fraction = plan_cartesian_path_pose(arm, -0.53, 0, 0, 0, 1)
    execute_plan(arm, cartesian_plan)
    cartesian_plan, fraction = plan_cartesian_path_pose(arm, -0.61, 0, 0, 0, 1)
    execute_plan(arm, cartesian_plan)
    cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0, -0.42, 1.0)
    execute_plan(arm, cartesian_plan)
    cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0.132, 0, 0, 1)
    execute_plan(arm, cartesian_plan)
    cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0, -0.12, 0, 1)
    execute_plan(arm, cartesian_plan)
    gripper.set_named_target("gripper_close")
    gripper.go(wait=True)
    
    cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0.62, -0.275, 0.07, 0, 1)
    execute_plan(arm, cartesian_plan)
    cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0.98, 0, 0, 0, 1)
    execute_plan(arm, cartesian_plan)
    cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0.175, 0, 0, 1)
    execute_plan(arm, cartesian_plan)
    gripper.set_named_target("gripper_open")
    gripper.go(wait=True)
    
    # Tercer Cubo
    arm.set_named_target("home")
    arm.go(wait=True)
    #x: 0.239820825156
    #y: -0.631965677562
    #z: 0.62069143596

    cartesian_plan, fraction = plan_cartesian_path_pose(arm, -0.53, 0, 0, 0, 1)
    execute_plan(arm, cartesian_plan)
    cartesian_plan, fraction = plan_cartesian_path_pose(arm, -0.385, 0, 0, 0, 1)
    execute_plan(arm, cartesian_plan)
    cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0, -0.42, 1.0)
    execute_plan(arm, cartesian_plan)
    cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, -0.248, 0, 0, 1)
    execute_plan(arm, cartesian_plan)
    cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0, -0.12, 0, 1)
    execute_plan(arm, cartesian_plan)
    gripper.set_named_target("gripper_close")
    gripper.go(wait=True)
    
    cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0.395, 0.105, 0.07, 0, 1)
    execute_plan(arm, cartesian_plan)
    cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0.98, 0, 0, 0, 1)
    execute_plan(arm, cartesian_plan)
    cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0, 0.175, 0, 0, 1)
    execute_plan(arm, cartesian_plan)
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

<a name="pruebas">
  <h2>
Ejecución de las pruebas
  </h2>
</a>

```{bash}
# Terminal 1
roslaunch one_arm_moveit_gazebo ur10_joint_limited.launch

# Terminal 2
roslaunch one_arm_moveit_manipulator one_arm_moveit_execution.launch

# Terminal 3
rosrun one_arm_moveit_manipulator one_arm_moveit.py 
```
