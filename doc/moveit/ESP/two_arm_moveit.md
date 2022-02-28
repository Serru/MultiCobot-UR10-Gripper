<!--- Para un robot  opción B--->
# Instalación y configuración para dos robots UR10 con MoveIt!

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

En la imagen se representa el contenido del [fichero URDF](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/two_arm_moveit/two_arm_moveit_description/urdf/ur10_robot.urdf.xacro) que modela el robot junto a la pinza, se puede ver cómo se conecta el componente del brazo UR10 robot con el link `world`, representando world (color amarillo) y la base del brazo del UR10 `base_link` (color verde) situado justo encima, además el joint `world_joint` es la esfera de color amarillo situado entre ambos links. De la misma manera se tiene el componente de la pinza `robotiq_85_gripper`, está conectado al brazo del UR10 (ur10 robot), en donde la esfera que representa el joint `robotiq_85_base_joint` que une ambos componentes (color morado), uniendo el link `robotiq_85_base_link` de la pinza con el link `ee_link` del brazo de UR10.

#### Creación del directorio para la solución
```{bash}
cd ~/MultiCobot-UR10-Gripper/src/multirobot
mkdir two_arm_moveit
```

### Configuración del directorio descripción
Siguiendo la misma línea, se crea un nuevo paquete y se copia los directorios del proyecto *one_arm_moveit* para su posterior modificación.

```{bash}
cd ~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_moveit
catkin_create_pkg two_arm_moveit_description rospy
```

En el directorio creado para description, se copiara del directorio de *one_arm_moveit_description*, las carpetas *launch* y *urdf*.

```{bash}
cd ~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_moveit/two_arm_moveit_description
cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit/one_arm_moveit_description/launch .
cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit/one_arm_moveit_description/urdf .
```

### Modificación de los ficheros de description
Sustituir el directorio *one_arm_moveit_description* por *two_arm_moveit_description* en los ficheros.


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

Antes de realizar la configuración con el `Setup Assitant` hay que tener el `URDF` bien definido previamente, con ese hecho se lanza el asistente de configuración con el siguiente comando en la terminal:

```{bash}
cd ~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_moveit/
mkdir two_arm_moveit_config
roslaunch moveit_setup_assistant setup_assistant.launch
```

#### MoveIt! Setup Assistant
Se va a escoger como modelo del robot el fichero URDF: [~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_moveit/two_arm_moveit_description/urdf/ur10_joint_limited_robot.urdf.xacro](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/two_arm_moveit/two_arm_moveit_description/urdf/ur10_joint_limited_robot.urdf.xacro) (podría ser perfectamente [ur10_robot.urdf.xacro](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/two_arm_moveit/two_arm_moveit_description/urdf/ur10_robot.urdf.xacro)).

- Comenzando la configuración del Setup Assistant, hay que decirle donde está el fichero ur10 joint limited robot.urdf.xacro, es decir, el modelo del robot que se quiere configurar:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_1.png  "Cargar el modelo URDF del robot UR10")

- Posteriormente, se le da al boton *Load Files*.
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_2.png  "Cargado el modelo URDF del robot UR10")

- En la pestaña *Self-Collisions*, darle al botón *Generate Collision Matrix*, lo que generará una matriz entre los diferentes componentes del robot que puedan generar autocolisiones durante la planificación de las trayectorias:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_3.png  "Generación de matirz de colisiones")

- En la pestaña *Virtual Joints* está dirigida especialmente para brazos robóticos instalados sobre una base móvil, en este caso no afecta en la configuración, ya que la base es fija, pero se configurará igualmente para configuraciones futuras, hay que crear un joint entre el robot `base_link` y el frame `world`, siendo la configuración la siguiente:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_4.png  "Definiendo Virtual Joint")
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_5.png  "Definido Virtual Joint")

- Una de las pestañas más importantes es definir bien los *Planning groups*, en este caso se tiene dos grupos, el grupo *manipulator* que controlará el brazo del robot y el grupo *gripper* que controlará la pinza:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_6.png  "Manipulator kdl")

- Después hay que darle al botón *Add Kin. Chain*:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_7.png  "Manipulator Kinetic Chain configuración")

- Finalmente se guarda la configuración:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_8.png  "Grupo manipulator configurado")

- Ahora hay que hacerlo para el grupo *Gripper* que controlará la pinza, se pulsa el botón *Add Group*, se rellena con el nombre del grupo y se pone *kdl* como *Kinematic Solver*:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_9.png  "Gripper kdl")

- Tras darle al botón *Add Joints*, hay que buscar por *robotiq_85_left_knucle_joint* y añadirlo a con la flecha *->* y se guarda la configuración:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_10.png  "Gripper Joint configuración")

- Tras guardar, el resultado en la pestaña de *Planning Group* debería ser la siguiente:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_11.png  "Configuración final del Planning Group")

- En la pestaña *Robot Poses* es donde se configura poses fijas de antemano en el robot, se va a configurar la pose *home* el cual reflejará la posición inicial del cobot:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_12.png  "Configurando *home* 1/3")
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_13.png  "Configurando *home* 2/3")
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_14.png  "Configurando *home* 3/3")

- En la pestaña *Robot Poses* se va a configurar la pose *gripper_open*:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_24.png  "Configurando *gripper open*")

- En la pestaña *Robot Poses* se va a configurar la pose *gripper_close*:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_25.png  "Configurando *gripper close*")


- En la pestaña *End Effectors* se va añadir el gripper:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_15.png  "Configurando end effector 1/3")
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_16.png  "Configurando end effector 2/3")
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_17.png  "Configurando end effector 3/3")

- En la pestaña *Passive Joints* representa las articulaciones que no se pueden mover activamente, en el caso del *gripper* definido en el su fichero *URDF* son los joints que imitan el movimiento de otro joint. Los joints definidos como pasivos no se tendrán en cuenta en la planificación, para este caso la configuración es la siguiente:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_18.png  "Configurando Passive Joints")

- En la pestaña *ROS Control*, se añadirá de forma automática, los ficheros generados, serán posteriormente modificado manualmente.
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_19.png  "Configurando ROS Control")
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_20.png  "Configurando ROS Control")

- Hay que rellenar la pestaña *Author Information* para que la configuración pueda terminar:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_21.png  "Configurando Author Information")

- La útima pestaña *Configuration Files*, permite decidir dónde se guardará la configuración de MoveIt!, en este caso en `one_arm_moveit_config` creado previamente, se genera la configuración mediante el botón *Generate Package* y finalmente
 *Exit Setup Assistant* para terminar:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_22.png  "Configurando Author Information")
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_23.png  "Configurando Author Information")


<a name="fase3">
  <h2>
Fase 3: Simulación de un <i>pick & place</i> en Gazebo
  </h2>
</a>

Para poder controlador varios dos o más cobots simultáneamente y con diferentes controladores, lo que implica que pueden ser cobots de diferentes marcas y modelos, se realiza mediante la replicación del nodo de MoveIt! y del robot simulado en Gazebo.

Si se quiere realizar una replicación correcta hay que aplicar el concepto
de *namespace*, se puede ver como fuese un directorio que contiene nodos, topics o incluso otros directorios (namespaces) lo que permite también una organización jerarquizada y ROS permite ejecutar instancias del mismo nodo siempre y cuando estén dentro de diferentes namespaces. Partiendo de lo realizado hasta la Fase 2, se realiza cambios en los paquetes de `two_arm_moveit_gazebo` y `two_arm_moveit_manipulator` que contiene las modificaciones realizadas sobre el paquete configurado por el setup assistant de MoveIt! (`two_arm_moveit_config)`. Se va a dividir el proceso de la configuración en dos, configuración realizada en Gazebo y en MoveIt!.

### Configuración realizada en Gazebo
---
Hay que tener en cuenta que se debe tener dos o más cobots en simulación,
por ello se va a explicar qué cambios hay que realizar en los ficheros launch
para permitir la adicción de dos o más cobots en la simulación correctamente.
La idea es crear un fichero externo que replique (lance instancias) de tantos
cobots como se quieran añadir en la simulación. Por ello primero se preparan
los ficheros de Gazebo del paquete one arm moveit gazebo.

◦ Fichero ur10 joint limited.launch: El contenido del fichero
contendrá el Código Fuente 6.17, respecto al fichero original se le ha
añadido dos argumentos, el nombre del robot (robot name) y la
posición inicial (init pose). Estos dos argumentos serán obtenidos
del fichero que incluirá este launch.
<?xml version="1.0"?>
<launch>
<arg name="robot_name"/>
<arg name="init_pose"/>
<include file="$(find one_arm_moveit_gazebo)/launch/ur10.launch">
<arg name="limited" value="true"/>
<!--arg name="gui" value="$(arg gui)"/-->
<arg name="robot_name" value="$(arg robot_name)"/>
<arg name="init_pose" value="$(arg init_pose)"/>
</include>
</launch>
Código Fuente 6.17 : Fase 3: Fichero ur10 joint limited.launch modificado
◦ Fichero ur10.launch: De forma similar al fichero launch anterior
(ur10 joint limited.launch), a este fichero se le ha añadido los
dos argumentos, el nombre del robot (robot name) y la pose y posición
inicial (init pose), que se definirán del fichero que lo incluya. Aparte
de la adicción de los argumentos, se ha eliminado la instanciación
del mundo virtual de Gazebo y la carga del modelo del cobot en el
servidor de parámetros (robot description) como se puede ver en
el Código Fuente 6.18.
<?xml version="1.0"?>
<launch>
<arg name="limited" default="false" doc="If true, limits joint range
[-PI, PI] on all joints." />
<arg name="paused" default="false" doc="Starts gazebo in paused mode" />
<!--arg name="gui" default="true" doc="Starts gazebo gui" /-->
<arg name="robot_name"/>
<arg name="init_pose"/>
<!-- push robot_description to factory and spawn robot in gazebo -->
<node name="spawn_gazebo_model" pkg="gazebo_ros" type="spawn_model"
respawn="false" output="screen" args="-urdf -param /robot_description
-model $(arg robot_name) $(arg init_pose)" />
<include file="$(find one_arm_moveit_gazebo)/launch/controller_utils.launch"/>
<!-- start this controller -->
<rosparam file="$(find one_arm_moveit_gazebo)/controller/arm_controller_ur10.yaml"
command="load"/>
<node name="arm_controller_spawner" pkg="controller_manager"
type="controller_manager" args="spawn arm_controller" respawn="false"
output="screen"/>
<!-- robotiq_85_gripper controller -->
<rosparam file="$(find one_arm_moveit_gazebo)/controller/
gripper_controller_robotiq.yaml" command="load"/>
<node name="gripper_controller_spawner" pkg="controller_manager"
type="spawner" args="gripper" />
<!-- load other controllers -->
<node name="ros_control_controller_manager" pkg="controller_manager"
type="controller_manager" respawn="false" output="screen"

args="load joint_group_position_controller" />
</launch>
Código Fuente 6.18 : Fase 3: Fichero ur10.launch modificado
◦ Fichero controller utils.launch: En este fichero simplemente
hay que comentar el parámetro tf prefix, su valor por defecto es
una cadena vacı́a, pero eso interfiere a la hora de modificar su valor por
defecto. Como se muestra en el Código Fuente 6.19, simplemente hay
que dejar comentar ese trozo de código respecto al su contenido original,
el resto se deja igual.
[...]
<!-- Robot state publisher -->
<node pkg="robot_state_publisher" type="robot_state_publisher"
name="robot_state_publisher" respawn="true" output="screen">
<param name="publish_frequency" type="double" value="50.0" />
<!-- param name="tf_prefix" type="string" value="" / -->
</node>
[...]
Código Fuente 6.19 : Fase 3: Contenido de la parte modificada del Fichero
controller utils.launch
Una
vez
configurado
los
ficheros
del
paquete
de
Gazebo
one arm moveit gazebo, se procede a instanciar varios cobots en este,
para ello se crea dentro del paquete one arm moveit manipulator
(puede ser cualquier otro paquete) un fichero launch llamado
two arm moveit gazebo.launch.
El contenido del fichero es el siguiente Código Fuente 6.20, lo primero que
se aprecia es que carga el modelo del robot en el servidor de parámetros
e instancia el mundo virtual de Gazebo, lo que se eliminó del fichero
ur10.launch, esto debe estar aquı́ porque solo se quiere instanciar una
vez el mundo de Gazebo.
Después aparecen dos grupos, ur10 1 y ur10 2, esta es la forma de definir
los namespaces, la configuración de estos cobots es idéntica excepto por
tres cosas, el valor del parámetro tf prefix que será el prefijo que irá
en las transfromadas, es importante que coincida con el nombre del grupo
(namespace), el nombre (robot name) y su posición inicial (init pose).
Si se quiere añadir más cobots al sistema, simplemente hay que copiar el
contenido de grupo y modificar el contenido adecuadamente. Y las últimas
dos lı́neas de código unen la base de los cobots con el frame world.

<launch>
<param name="/use_sim_time" value="true"/>
<arg name="robot_name"/>
<arg name="init_pose"/>
<arg name="paused" default="false"/>
<arg name="gui" default="true"/>
<arg name="limited" default="true" doc="If true, limits joint range
[-PI, PI] on all joints." />
<!-- send robot urdf to param server -->
<include file="$(find one_arm_moveit_description)/launch/ur10_upload.launch">
<arg name="limited" value="$(arg limited)"/>
</include>
<include file="$(find gazebo_ros)/launch/empty_world.launch">
<!--arg name="world_name" default="worlds/empty.world"/-->
<arg name="verbose" value="true"/>
<arg name="world_name" default="$(find one_arm_moveit_gazebo)/world/
multiarm_bot.world"/>
<arg name="paused" value="$(arg paused)"/>
<!--arg name="gui" value="$(arg gui)"/-->
<arg name="gui" value="$(arg gui)"/>
</include>
<group ns="ur10_1">
<param name="tf_prefix" value="ur10_1" />
<include file="$(find one_arm_moveit_gazebo)/launch/
ur10_joint_limited.launch">
<arg name="init_pose" value="-x 0.6 -y -0.6 -z 1.1"/>
<arg name="robot_name" value="ur10_1"/>
</include>
</group>
<group ns="ur10_2">
<param name="tf_prefix" value="ur10_2" />
<include file="$(find one_arm_moveit_gazebo)/launch/
ur10_joint_limited.launch">
<arg name="robot_name" value="ur10_2"/>
<arg name="init_pose" value="-x 0.6 -y 1.38 -z 1.1"/>
</include>
</group>
<node pkg="tf" type="static_transform_publisher"
name="world_frames_connection_1" args="0 0 0 0 0 0
/world /ur10_1/world 100"/>
<node pkg="tf" type="static_transform_publisher"
name="world_frames_connection_2" args="0 0 0 0 0 0
/world /ur10_2/world 100"/>
</launch>
Código Fuente 6.20 : Fase 3: Contenido del Fichero two arm moveit gazebo.launch
Con esto, Gazebo ya está configurado, se procede a lanzar el fichero
two arm moveit gazebo para comprobar que se han realizado las
modificaciones adecuadamente. En la Figura 6.14 se ve que está simulando
dos cobots correctamente. Hay más resultados en el Anexo C, en la
Subsección C.2.1 está el árbol de transformadas, en donde se puede ver
cómo se han definido los namespaces correctamente y están representadas
ambos cobots y finalmente está la representación de los nodos y los topics en la Subsección C.2.2, lo más importante de aquı́ es la aparición de dos
grandes recuadros que contienen a cada uno de los cobots y sus controladores
sin tener que configurarlo de nuevo, finalmente se puede apreciar que
ambos cobots están conectados al simulador Gazebo, la configuración se
ha realizado correctamente.

---
Lo primero que hay que hacer en esta fase es configurar Gazebo y los
controladores para que pueda simular adecuadamente los movimientos del cobot. Se crea el paquete *one_arm_moveit_gazebo*, que contendrá toda la configuración relacionada con Gazebo, entre ellos los controladores. Una vez creada el paquete, hay que configurar los controladores que están almacenados en el directorio *controller*, aunque todos los controladores pueden estar definidos en un único fichero por claridad se ha distribuido en tres ficheros.

Los controladores se definen en ficheros con extensión *yaml*, para definir estos controladores hay que darles un nombre y definir el tipo del controlador, los joints dinámicos que se quieren controlar, las restricciones que tiene, el ratio de publicación y otras opciones.

Se procede a explicar brevemente estos controladores:

- Fichero [arm_controller_ur10.yaml](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_gazebo/controller/arm_controller_ur10.yaml): En este fichero se define el controlador para el cobot UR10, aquı́ se define el nombre del controlador `arm_controller`, el tipo de controlador
position `controllers/JointTrajectoryController`, lo que implica la definición del tipo de mensajes y el formateo adecuado de la información necesaria para comunicarse con éste. Después está
el campo `joints`, que es donde se indica qué joints del cobot forma
parte del controlador, todos estos joints son dinámicos. El resto de
campos no se han tocado, pero hay que mantener la consistencia en
cómo se nombran.

- Fichero [gripper_controller_robotiq.yaml](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_gazebo/controller/gripper_controller_robotiq.yaml): En este fichero se define el controlador para la pinza de *Robotiq*, aquı́ se define el nombre del controlador `gripper`, el tipo de controlador `position controllers/JointTrajectoryController` que define el tipo de mensajes y la información necesaria para comunicarse con éste. Después está el campo `joints`, que es donde se indica qué joints del cobot forma parte del controlador en este caso un único `joint_robotiq_85_left_knuckle_joint` porque el resto de joints del controlador imitan los movimientos de este. El resto de campos
no se han tocado, pero hay que mantener la consistencia en cómo se
nombran como en el caso anterior.

- Fichero [joint_state_controller.yaml](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_gazebo/controller/joint_state_controller.yaml): Lo que define este fichero realmente no es un controlador como tal, su función es la de una interfaz que traduce la información de los joints que viene del cobot real y lo traduce a mensajes de tipo `JointState` para después
publicarlo. Es fundamental para el correcto funcionamiento, tanto en
simulación como con el robot real, forma parte del paquete de ROS
*ros_control*.



### Puesta en marcha de Gazebo
Se va a crear el paquete para gazebo, y copiar el contenido de la solución partiendo del la solución *one_arm_moveit* para su posterior modificación:
```{bash}
cd ~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_moveit
catkin_create_pkg two_arm_moveit_gazebo rospy
```
En el directorio creado para gazebo, se copiara del directorio de *one_arm_moveit_gazebo*, las carpetas *controller*, *launch*, *models*, y *world* de *two_arm_no_moveit*.
```{bash}
cd ~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_moveit/two_arm_moveit_gazebo
cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit/one_arm_moveit_gazebo/controller .
cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit/one_arm_moveit_gazebo/launch .
cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit/one_arm_moveit_gazebo/models .
cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_gazebo/world .
```

Sustituir el directorio *one_arm_moveit_gazebo* por *two_arm_moveit_gazebo* y *one_arm_moveit_description* por *two_arm_moveit_description* en los ficheros.

#### Modificación de los ficheros de gazebo

Modificar el fichero [ur10.launch](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/two_arm_moveit/two_arm_moveit_gazebo/launch/ur10.launch) en preparación para lanzar varios robots y comentar la etiqueta **<param name="tf_prefix" type="string" value="" />** en el fichero *controller_utils.launch*.

Modificar el fichero [ur10_joint_limited.launch](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/two_arm_moveit/two_arm_moveit_gazebo/launch/ur10_joint_limited.launch)

Y modificar el fichero [controller_utils.launch](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/two_arm_moveit/two_arm_moveit_gazebo/launch/controller_utils.launch)

Se compila:
```{bash}
cd ~/MultiCobot-UR10-Gripper
catkin_make
```


### Configuración realizada en MoveIt!
---

Los cambios realizados para la configuración de MoveIt! son muy
pequeñas, básicamente hay que agrupar el código de lo que se habı́a
implementado para un único cobot bajo un namespace, adecuar el remap
con el namespacing y posteriormente replicar el proceso tantas veces
como cobots se esté simulando. El Código Fuente 6.21 si se compara
con El Código Fuente 6.11 de la solución desarrollada para un único
cobot, esencialmente se ha agrupado todo en un namespace y se ha
añadido el nombre del namespace como prefijo en los nombres de los
topics del remap para la correcta comunicación con los controladores.

<launch>
<arg name="sim" default="false" />
<arg name="debug" default="false" />
<group ns="ur10_1">
<rosparam command="load" file="$(find one_arm_moveit_manipulator)/config
/joint_names.yaml"/>
<include file="$(find one_arm_moveit_manipulator)/launch/
planning_context_ur10_1.launch">
<arg name="load_robot_description" value="true"/>
</include>
<include file="$(find one_arm_moveit_manipulator)/launch/move_group.launch">
<arg name="debug" default="$(arg debug)" />
<arg name="publish_monitored_planning_scene" value="true"/>
<!--arg name="info" value="true"/-->
</include>
<!-- Remap follow_joint_trajectory -->
<remap from="/ur10_1/follow_joint_trajectory"
to="/ur10_1/arm_controller/follow_joint_trajectory"/>
<include file="$(find one_arm_moveit_config)/launch/moveit_rviz.launch">
<arg name="config" value="true"/>
<arg name="debug" default="false"/>
</include>
</group>
<group ns="ur10_2">
<rosparam command="load" file="$(find one_arm_moveit_manipulator)/config/
joint_names.yaml"/>
<include file="$(find one_arm_moveit_manipulator)/launch/
planning_context.launch">
<arg name="load_robot_description" value="true"/>
</include>
<include file="$(find one_arm_moveit_manipulator)/launch/move_group.launch">
<arg name="debug" default="$(arg debug)" />
<arg name="publish_monitored_planning_scene" value="true"/>
<!--arg name="info" value="true"/-->
</include>
<!-- Remap follow_joint_trajectory -->
<remap from="/ur10_2/follow_joint_trajectory"
to="/ur10_2/arm_controller/follow_joint_trajectory"/>
<include file="$(find one_arm_moveit_config)/launch/moveit_rviz.launch">
<arg name="config" value="true"/>
<arg name="debug" default="false"/>
</include>
</group>
</launch>
Código
Fuente
6.21 :
Fase
two arm moveit execution.launch
3:
Contenido
del
Fichero
Se procede a comprobar si la comunicación entre Gazebo y las réplicas de
MoveIt! se comunican correctamente. Se puede ver dos grandes agrupaciones,
en donde cada cobot se está comunicando con su move group asignado,
la comunicación con los controladores y el camino de las transformadas y
valores de los joints tiene un único origen que es el nodo de Gazebo, esto es muy importante para evitar movimientos extraños dependiendo de la
frecuencia en que se produzcan esas interferencias.
El camino parte del nodo de gazebo es el único que publica a los topics
/joint states de ambos namespaces, el nodo move group está suscrito
a este topic también, después llega al nodo robot state publisher que
realiza las transformadas y se las envı́a por el topic /tf el cual el nodo
move group también está suscrito, esto es importante porque move group
utiliza la información que proviene de ambos para realizar la planificación
de las trayectorias.
Las imágenes obtenidas de la herramienta rqt graph tienen ya muchos
nodos y las lı́neas de comunicación no se aprecian bien, en la
Subsección C.2.3 del Anexo C están las imágenes, pero se recomienda
reproducirlo (ver Anexo F) y obtener la gráfica con la herramienta para
verlo en detalle.


---



### Conexión entre Gazebo y MoveIt!

Se va a modificar los siguientes ficheros teniendo como base sus ficheros originales *demo.launch*, *move_group.launch*, *trajectory_execution.launch.xml*  y *ur10_moveit_controller_manager.launch.xml* y se añadirá los controladores creando dos ficheros *controllers.yaml* y *joint_names.yaml*:

```{bash}
cd ~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_moveit
catkin_create_pkg two_arm_moveit_manipulator rospy

cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit/one_arm_moveit_manipulator/config .
cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit/one_arm_moveit_manipulator/launch .
cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit/one_arm_moveit_manipulator/scripts .
```
Sustituir el directorio *one_arm_moveit_gazebo* por *two_arm_moveit_gazebo* y *one_arm_moveit_description* por *two_arm_moveit_description* en los ficheros y renombrar los ficheros *one_arm_moveit* por *two_arm_moveit*


Se va a proceder a añadir los controladores para su interacción con Gazebo de dos robots:

Para ello se crea un fichero *launch* que lanzará la configuración de los dos robots con diferente *namespaces*.
```{bash}
touch launch/two_arm_moveit_gazebo.launch
```

Fichero: [~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_moveit/two_arm_moveit_manipulator/launch/two_arm_moveit_gazebo.launch](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/two_arm_moveit/two_arm_moveit_manipulator/launch/two_arm_moveit_gazebo.launch)


Los nodos de tf son muy importantes porque MoveIt! tomo el frame */world* como refrencia como referencia a la hora de realizar el planing. 

Y no es posible modificarlo con la API de python, hay que crear un nodo que realize las transformaciones necesarias como en la solución sin MoveIt! o añadir el frame */world* como se ha realizado que será la raíz del resto de los frames, como aparece enla imagen.
![ ](/doc/imgs_md/two_arm_moveit_frames.pdf  "Robots frames")

Fichero: [~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_moveit/two_arm_moveit_manipulator/launch/two_arm_moveit_execution.launch](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/two_arm_moveit/two_arm_moveit_manipulator/launch/two_arm_moveit_execution.launch)


Finalmente, se realiza una prueba:
```{bash}
cd ~/MultiCobot-UR10-Gripper
catkin_make clean
catkin_make
rospack profile 

# Terminal 1
roslaunch two_arm_moveit_manipulator two_arm_moveit_gazebo.launch

# Terminal 2
roslaunch two_arm_moveit_manipulator two_arm_moveit_execution.launch
```

![ ](/doc/imgs_md/two_arm_moveit_gazebo.png  "Example gazebo+rviz+moveit!")

Y la gráfica de los nodos y los topis, despues de las modificaciones, se puede apreciar cómo ahorá el nodo *move_group* tiene comunicación con los controladores. En la imagen no se aprecia, pero hay dos nodos *move_group*s con el mismo contenido pero diferente namespaces, se recomienda generar el proyecto y obtener le gráfica mediante la herramienta *rqt_graph*.

![ ](/doc/imgs_md/two_arm_moveit_graph_changes.png  "rqt_graph representación de los nodos y los topics")

### Pick and Place
Como en las soluciones anteriores, se procederáa realizar unas pruebas muy sencillas. Para ello primero hay que crear los scripts necesarios para controlar el brazo del robot y el gripper correctamente y posterioremente, se realizará los mismos movimientos que en las soluciones anteriormente propuestas.
---
1
2
3
4
def main():
moveit_commander.roscpp_initializer.roscpp_initialize(sys.argv)
rospy.init_node(’ur10_1_arm_moveit’,
anonymous=True)
5
6
7
8
9
PLANNING_GROUP_GRIPPER = "gripper"
PLANNING_GROUP_ARM = "manipulator"
PLANNING_NS = "/ur10_1/"
REFERENCE_FRAME = "/ur10_1/world"
10
11
12
13
14
15
16
## Instantiate a RobotCommander object. This object is an interface to
## the robot as a whole.
robot = moveit_commander.RobotCommander(
"%srobot_description"%PLANNING_NS,
ns="/ur10_1/"
)
17
18
19
20
21
22
arm = moveit_commander.move_group.MoveGroupCommander(
PLANNING_GROUP_ARM,
"%srobot_description"%PLANNING_NS,
ns="/ur10_1/"
)
23
24
25
26
27
28
gripper = moveit_commander.move_group.MoveGroupCommander(
PLANNING_GROUP_GRIPPER,
"%srobot_description"%PLANNING_NS,
ns="/ur10_1/"
)
29
30
31
32
33
34
35
36
## We create this DisplayTrajectory publisher which is used below to publish
## trajectories for RVIZ to visualize.
display_trajectory_publisher = rospy.Publisher(
’/move_group/display_planned_path’,
moveit_msgs.msg.DisplayTrajectory,
queue_size=10
)
37
38
rospy.sleep(2)
39
40
41
42
43
44
45
46
arm.set_num_planning_attempts(15)
arm.set_planning_time(5)
arm.allow_looking(True)
arm.allow_replanning(True)
arm.set_pose_reference_frame(REFERENCE_FRAME)
arm.set_goal_position_tolerance(0.001)
arm.set_goal_orientation_tolerance(0.001)
47
48
49
50
gripper.set_num_planning_attempts(15)
gripper.allow_replanning(True)
gripper.allow_looking(True)
51
52
pick_place(arm, gripper)
53
54
55
## When finished shut down moveit_commander.
moveit_commander.roscpp_shutdown()
Código Fuente 6.22 : Fase 3: Parte del contenido del Fichero two arm moveit 1.py
Como se puede ver en el Código Fuente 6.22, se ha definido la configuración
en la función main, pero no es necesario que sea aquı́, es más es preferible
modularlo y pasarle la configuración por parámetros, pero para esta explicación
es suficiente.
• Lı́nea 2: Lo primero es inicializar moveit commander que es una API
sobre la interfaz desarrollada en C++, lo que es definido como Wrappers,
este provee la mayorı́a de las funcionalidades que provee la interfaz de
la versión para C++ pero no están todas las funcionalidades de MoveIt!.
Es necesario porque entre sus funcionalidades permite calcular trayectorias
cartesianas que es la funcionalidad que principalmente se requiere.
• Lı́nea 3: Inicializa el nodo al que se le ha nombrado ur10 1 arm moveit.
• Lı́neas 6-9 : Se definen las constantes para facilitar la configuración, las dos
primeras lı́neas (6 y 7) definen los nombres que se pusieron a los grupos
de planificación, esto se puede comprobar en el Anexo B, en este caso
se les llamó gripper para la pinza y manipulator para el brazo del cobot
UR10. Las lı́neas 8 y 9 definen la información necesaria para configurar el
planificador, PLANNING NS contiene el nombre del namespace donde está el
nodo move group con el que se quiere comunicar y REFERENCE FRAME es
el link que se tomará como referencia para realizar los cálculos de trayectoria
con respecto al end-effector (el end-effector es ee link), en este caso
darı́a igual que sea /ur10 1/world pero lo correcto serı́a tomar el link
/ur10 1/base link como referencia.
• Lı́nea
13 :
Inicializa
el
RobotCommander,
como
en
el
código
indica, se utiliza para controlar el robot, hay que pasarle como
parámetros el namespace y qué robot description debe utilizar
para definir el robot, porque en el momento hay tres descripciones,
que son el de Gazebo (robot description) y los otros dos
definidos
en
el
fichero
planning context.launch
que
son
instanciadas dentro de sus corresponientes namespaces por ello se tiene
/ur10 1/robot description y /ur10 2/robot description.
• Lı́nea 18 y 24 : Se crea una interfaz para un conjunto de Joints, en este caso
la interfaz arm para el conjunto de joints dinámicos del brazo del cobot
UR10 y la interfaz gripper para el joint que controla la pinza. A través
de estas interfaces se realizarán las planificaciones de las trayectorias y su
ejecución (es posible realizarlo con la interfaz de robot, ya que contiene a
estas dos interfaces, pero es más claro y cómodo realizarlo de esta manera.
• Lı́nea
32 :
Como
dice
display trajectory publisher
en
realiza
el
publicaciones
comentario,
al
topic
/move group/display planned path y el cual Rviz se suscribe
para visualizar las trayectorias, no es necesario, pero para depuración es
recomendable.
• Lı́nea 38 : Simplemente espera dos segundos, se asegura que las instancias
anteriores se han cargado correctamente en el sistema, hay que tener en
cuenta alguno de ellos instancian nodos y si el ordenador sobre el que se
está lazando es lento puede provocar una situación de no deseada.
• Lı́neas 40-50 : Aquı́ se está configurando algunas opciones del planificador
que se va a utilizar, por defecto es RTT, pero se puede modificar. Los más
importantes son los últimos en las lı́neas 44, 45 y 46. En la lı́nea 44 se
define el link de referencia que se utilizará para realizar las planificaciones,
la lı́nea 45 y 45 definen el margen de error aceptable del resultado obtenido
del planificador para la posición y la orientación, hay que tener cuidado
porque cuanto más pequeño es el error que se defina más tiempo tardará
el planificador en dar una respuesta, está definidos para permitir errores de
milı́metros. El mismo proceso para la interfaz de la pinza (gripper).
• Lı́nea 52 : aquı́ se lanza la tarea de pick & place.
• Lı́nea 55 : Una vez que termine la tarea pick & place finaliza.

Tras la descripción detallada del código, para que el script controlase otro cobot
que esté en otro namespace es tan sencillo como cambiarle el valor de la variable
PLANNING NS por la del nombre del namespace donde esté definido el cobot
objetivo (el código completo está en Github, ver Anexo F).
Si se ejecuta el pick & place con ambos brazos, se aprecia que realizan la
tarea simultáneamente y con un poco de retraso entre ellos debido a que uno
empieza un poco más tarde que otro, se puede realizar pruebas a lanzar los
scripts a destiempo y comprobar que efectivamente se mueven simultáneamente.
En el siguiente Capı́tulo 8 se mostrará unas imágenes que dan una idea de cómo
funcionan durante la simulación.
---

```{bash}
cd scripts
touch two_arm_moveit_1.py
touch two_arm_moveit_2.py
```

Fichero [~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_moveit/two_arm_moveit_manipulator/scripts/two_arm_moveit_1.py](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/two_arm_moveit/two_arm_moveit_manipulator/scripts/two_arm_moveit_1.py)

Fichero [~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_moveit/two_arm_moveit_manipulator/scripts/two_arm_moveit_2.py](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/two_arm_moveit/two_arm_moveit_manipulator/scripts/two_arm_moveit_2.py)

Para ejecutar la prueba:
```{bash}
# Terminal 1
roslaunch two_arm_moveit_manipulator two_arm_moveit_gazebo.launch

# Terminal 2
roslaunch two_arm_moveit_manipulator two_arm_moveit_execution.launch

# Terminal 3
rosrun two_arm_moveit_manipulator two_arm_moveit_1.py 

# Terminal 4
rosrun two_arm_moveit_manipulator two_arm_moveit_2.py
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
