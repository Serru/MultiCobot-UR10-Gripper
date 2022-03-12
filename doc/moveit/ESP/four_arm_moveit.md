<!--- Para cuatro robots opción B--->
# Instalación y configuración para cuatro robots UR10 replicando `MoveIt!`

**Español** | [English](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/moveit/ENG/four_arm_moveit.md)

![image](/doc/imgs_md/Diseno-moveit-general-cuatro-cobots-leap-motion.png  "Cargado el modelo URDF del robot UR10")

Se va a realizar la solución para cuatro robots esta vez, de la misma manera que se ha realizado para uno, pero modificando el contenido de los ficheros adaptándolo para su simulación con cuatro robots.

## Requisito previo
- Realizar correctamente la instalación de la [configuración base del sistema](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/setup-doc/proyect_setup.md).
- Implementar la [solución para un robot sin el planificador de `MoveIt!`](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/no_moveit/ESP/one_arm_no_moveit.md).

## Índice
- [Configuración inicial: Configuración para un robot](#setup-inicial)
	- [Fase 1: Configuración del *URDF*](#fase1)
	- [Fase 2: Configuración de `MoveIt!`](#fase2)
	- [Fase 3: Simulación de un `pick & place` en `Gazebo`](#fase3)
	- [Ejecución de las pruebas](#pruebas)
- [Modificaciones: Sistema multirobot compuesto de cuatro robots](#modificaciones)
	- [Configuración realizada en `Gazebo` para cuatro cobots](#modificaciones1)
	- [Configuración realizada en `MoveIt!` para cuatro cobots](#modificaciones2)
	- [Ejecución de las pruebas](#pruebas2)
	
<a name="setup-inicial">
  <h2>
Configuración inicial: Configuración para un robot
  </h2>
</a>

En esta sección se realiza una replicación de la configuración para un único robot, del cual se toma como base y así explicar posteriormente las modificaciones realizadas para poder controlar cuatro robots.

### :warning: Contenido de los ficheros
No copiar y pegar el contenido de los ficheros ciegamente en esta sección. Durante la explicación para la configuración para un robot, la información es idéntica a lo que se obtiene del paquete **one_arm_moveit**, pero hay que modificar el contenido para se ajuste al nuevo paquete, en este caso **four_arm_moveit**.

Hay dos opciones:
- Si se ha realizado la solución para un robot [one_arm_moveit](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/moveit/ESP/one_arm_moveit.md), se puede continuar sobre lo implementado, pero en la sección de *modificaciones* hay que cambiar **four_arm_moveit** por **one_arm_moveit**.
- Si no se ha realizado la solución para un robot [one_arm_moveit](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/moveit/ESP/one_arm_moveit.md), se puede seguir los pasos que se presentan en esta sección, pero hay que tener en cuenta que el contenido de los ficheros en esta fase de *configuración inicial* están enlazadas a los ficheros de **one_arm_moveit** y por ello al realizar la copia de este hay que reemplazar las referencias al paquete **one_arm_moveit** por **four_arm_moveit**

Si no se tiene en cuenta esta información, el paquete no compilará correctamente y el propio compilador obligará a que se elija entre una de las opciones sugeridas previamente.

<a name="fase1">
  <h3>
Fase 1: Configuración del URDF
  </h3>
</a>

#### :book: Descripción del fichero *URDF*
El fichero *URDF* (United Robotics Description Format) modela el cobot utilizando el formato *XML*el cual será utilizado por las diferentes aplicaciones que *ROS* necesite, pero principalmente para realizar una simulación del robot modelado.

El fichero está construido en forma de árbol, en donde hay tres etiquetas principales: `<robot>`, `<link>` y `<joint>`. Para explicarlo bien, se puede tomar como referencia el brazo del cuerpo humano. Si lo que se quiere modelar es el brazo de una persona, la etiqueta `<robot>` representaría al brazo en su conjunto. Este brazo está compuesto de varios huesos (húmero, cúbito y radio) que son representados por las etiquetas `<link>` y por una articulación que une esos huesos (codo) que es representado por la etiqueta `<joint>`. 

Además, como en los huesos, estas etiquetas pueden ir con información adicional contenida en ellas que den información del tamaño, geometría, inercia, orientación, etc. Finalmente, el modelado de un robot se puede unir a otro modelo y formar uno más complejo, que podría ser representado con la adición de la mano al brazo, con la muñeca como articulación que conectan ambos. Hay que tener en cuenta que las etiquetas `<joint>` conecta las etiquetas `<link>` a través de una relación padre-hijo.

Dicho esto, se realiza una representación de los componentes del robot:
 
 ![image](/doc/imgs_md/urdf-robot.png  "Representación del fichero URDF")

En la imagen se representa el contenido del [fichero *URDF*](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_description/urdf/ur10_robot.urdf.xacro) que modela el robot junto a la pinza, se puede ver cómo se conecta el componente del brazo UR10 robot con el link `world`, representando `world`(color amarillo) y la base del brazo del UR10 `base_link` (color verde) situado justo encima, además el joint `world_joint` es la esfera de color amarillo situado entre ambos links. De la misma manera, se tiene el componente de la pinza `robotiq_85_gripper`, está conectado al brazo del UR10 (`ur10_robot`), en donde la esfera que representa el joint `robotiq_85_base_joint` que une ambos componentes (color morado), uniendo el link `robotiq_85_base_link` de la pinza con el link `ee_link` del brazo de UR10.


#### :computer: Creación del directorio para la solución
```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot
mkdir four_arm_moveit
```

#### :computer: Configuración del directorio descripción
Se crea un nuevo paquete y se copia los directorios del proyecto *one_arm_no_moveit* para su posterior modificación.

```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_moveit
catkin_create_pkg four_arm_moveit_description rospy
```

En el directorio creado para *description*, se copiará del directorio de *one_arm_no_moveit_description*, las carpetas *launch* y *urdf*.

```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_moveit/four_arm_moveit_description
cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_description/launch .
cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_description/urdf .
```

#### :computer: Modificación de los ficheros de *description*

- [~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_moveit/four_arm_moveit_description/launch/ur10_upload.launch](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_description/launch/ur10_upload.launch)

- [~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_moveit/four_arm_moveit_description/urdf/ur10_robot.urdf.xacro](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_description/urdf/ur10_robot.urdf.xacro)

- [~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_moveit/four_arm_moveit_description/urdf/ur10_joint_limited_robot.urdf.xacro](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_description/urdf/ur10_joint_limited_robot.urdf.xacro)

Se compila:
```bash
cd ~/MultiCobot-UR10-Gripper
catkin_make
```

<a name="fase2">
  <h3>
Fase 2: Configuración de MoveIt!
  </h3>
</a>

Antes de realizar la configuración con el `Setup Assitant` hay que tener el `URDF` bien definido previamente, con ese hecho se lanza el asistente de configuración con el siguiente comando en la terminal:

```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_moveit/
mkdir four_arm_moveit_config
roslaunch moveit_setup_assistant setup_assistant.launch
```

#### :computer:  `MoveIt!` Setup Assistant
Se va a escoger como modelo del robot el fichero *URDF*: [~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_moveit/four_arm_moveit_description/urdf/ur10_joint_limited_robot.urdf.xacro](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_description/urdf/ur10_joint_limited_robot.urdf.xacro) (podría ser perfectamente [ur10_robot.urdf.xacro](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_description/urdf/ur10_robot.urdf.xacro)).

- Comenzando la configuración del Setup Assistant, hay que decirle donde está el fichero `UR10_joint_limited_robot.urdf.xacro`, es decir, el modelo del robot que se quiere configurar:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_1.png  "Cargar el modelo URDF del robot UR10")

- Posteriormente, se le da al botón *Load Files*.
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_2.png  "Cargado el modelo URDF del robot UR10")

- En la pestaña *Self-Collisions*, darle al botón *Generate Collision Matrix*, lo que generará una matriz entre los diferentes componentes del robot que puedan generar autocolisiones durante la planificación de las trayectorias:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_3.png  "Generación de matriz de colisiones")

- En la pestaña *Virtual Joints* está dirigida especialmente para brazos robóticos instalados sobre una base móvil, en este caso no afecta en la configuración, ya que la base es fija, pero se configurará igualmente para configuraciones futuras, hay que crear un joint entre el robot `base_link` y el *frame* `world`, siendo la configuración la siguiente:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_4.png  "Definiendo Virtual Joint")
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_5.png  "Definido Virtual Joint")

- Una de las pestañas más importantes es definir bien los *Planning groups*, en este caso se tiene dos grupos, el grupo *manipulator* que controlará el brazo del robot y el grupo *gripper* que controlará la pinza:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_6.png  "Manipulator kdl")

- Después hay que darle al botón *Add Kin. Chain*:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_7.png  "Manipulator Kinetic Chain configuración")

- Finalmente, se guarda la configuración:
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


- En la pestaña *End Effectors* se va a añadir la pinza:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_15.png  "Configurando end effector 1/3")
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_16.png  "Configurando end effector 2/3")
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_17.png  "Configurando end effector 3/3")

- En la pestaña *Passive Joints* representa las articulaciones que no se pueden mover activamente, en el caso del *gripper* definido en el su fichero *URDF* son los `joints` que imitan el movimiento de otro joint. Los `joints` definidos como pasivos no se tendrán en cuenta en la planificación, para este caso la configuración es la siguiente:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_18.png  "Configurando Passive Joints")

- En la pestaña *ROS Control*, se añadirá de forma automática, los ficheros generados, serán posteriormente modificados manualmente.
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_19.png  "Configurando ROS Control")
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_20.png  "Configurando ROS Control")

- Hay que rellenar la pestaña *Author Information* para que la configuración pueda terminar:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_21.png  "Configurando Author Information")

- La última pestaña *Configuration Files*, permite decidir dónde se guardará la configuración de `MoveIt!`, en este caso en `one_arm_moveit_config` creado previamente, se genera la configuración mediante el botón *Generate Package* y finalmente
 *Exit Setup Assistant* para terminar:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_22.png  "Configurando Author Information")
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_23.png  "Configurando Author Information")


<a name="fase3">
  <h3>
Fase 3: Simulación de un <i>pick & place</i> en Gazebo
  </h3>
</a>

Esta fase tiene se divide en dos etapas

#### :book: Conexión entre `Gazebo` y `MoveIt!`

Lo primero que hay que hacer en esta fase es configurar `Gazebo` y los controladores para que pueda simular adecuadamente los movimientos del cobot. Se crea el paquete *four_arm_moveit_gazebo*, que contendrá toda la configuración relacionada con `Gazebo`, entre ellos los controladores. Una vez creada el paquete, hay que configurar los controladores que están almacenados en el directorio *controller*, aunque todos los controladores pueden estar definidos en un único fichero por claridad se ha distribuido en tres ficheros.

Los controladores se definen en ficheros con extensión *yaml*, para definir estos controladores hay que darles un nombre y definir el tipo del controlador, los `joints` dinámicos que se quieren controlar, las restricciones que tiene, el ratio de publicación y otras opciones.

Se procede a explicar brevemente estos controladores:

- Fichero [arm_controller_ur10.yaml](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_gazebo/controller/arm_controller_ur10.yaml): En este fichero se define el controlador para el cobot UR10, aquí se define el nombre del controlador `arm_controller`, el tipo de controlador posición `controllers/JointTrajectoryController`, lo que implica la definición del tipo de mensajes y el formateo adecuado de la información necesaria para comunicarse con este. Después está el campo `joints`, que es donde se indica qué `joints` del cobot forma parte del controlador, todos estos `joints` son dinámicos. El resto de campos no se han tocado, pero hay que mantener la consistencia en cómo se nombran.

- Fichero [gripper_controller_robotiq.yaml](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_gazebo/controller/gripper_controller_robotiq.yaml): En este fichero se define el controlador para la pinza de *Robotiq*, aquí se define el nombre del controlador `gripper`, el tipo de controlador `position controllers/JointTrajectoryController` que define el tipo de mensajes y la información necesaria para comunicarse con este. Después está el campo `joints`, que es donde se indica qué `joints` del cobot forma parte del controlador en este caso un único `joint_robotiq_85_left_knuckle_joint` porque el resto de `joints` del controlador imitan los movimientos de este. El resto de campos no se han tocado, pero hay que mantener la consistencia en cómo se nombran como en el caso anterior.

- Fichero [joint_state_controller.yaml](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_gazebo/controller/joint_state_controller.yaml): Lo que define este fichero realmente no es un controlador como tal, su función es la de una interfaz que traduce la información de los `joints` que viene del cobot real y lo traduce a mensajes de tipo `JointState` para después publicarlo. Es fundamental para el correcto funcionamiento, tanto en simulación como con el robot real, forma parte del paquete de *ROS* *ros_control*.

##### :computer: Puesta en marcha de `Gazebo`

Se va a crear el paquete para `Gazebo`, y copiar el contenido de la solución partiendo de la solución *one_arm_no_moveit* para su posterior modificación:
```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_moveit
catkin_create_pkg four_arm_moveit_gazebo rospy
```
En el directorio creado para `Gazebo`, se copiará del directorio de *one_arm_no_moveit_gazebo*, las carpetas *controller*, *launch*, *models*, y *world*.
```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_moveit/four_arm_moveit_gazebo
cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/controller .
cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/launch .
cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/models .
cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/world .
```

##### :computer: Modificación de los ficheros de `Gazebo`
Se crean los ficheros con el siguiente contenido:

- [~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_moveit/four_arm_moveit_gazebo/launch/ur10_joint_limited.launch](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_gazebo/launch/ur10_joint_limited.launch)

- [~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_moveit/four_arm_moveit_gazebo/launch/ur10.launch](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_gazebo/launch/ur10.launch)

Se compila:
```bash
cd ~/MultiCobot-UR10-Gripper
catkin_make
```

Si se deja la configuración como está, no es posible la comunicación entre `Gazebo` y `MoveIt!`. Al lanzar el fichero *demo.launch* también lanza *Rviz*  lo que permite la visualización de los datos y se aprecia que funciona correctamente, pero con la configuración automatizada, no es posible observar su funcionamiento en el simulador de `Gazebo`.

Esto es debido a que no hay comunicación entre `MoveIt!` y `Gazebo`, con lo que se procede a solucionar esta comunicación.

Por conveniencia, para su posible evaluación en el robot real, se mantendrá el lanzamiento `Gazebo` y `MoveIt!` separados.

Para comenzar y entender mejor el paquete de `MoveIt!`, se va a separar el contenido de los paquetes en `MoveIt!` y `Gazebo`:

`Gazebo`:
- launch:
	- gazebo.launch
	- ros_controllers.launch
- config:
	- **ros_controllers.yaml**

No es necesario el fichero *gazebo.launch* que provee `MoveIt!`, ya que se ha configurado previamente el paquete de `Gazebo` que se usará. La configuración es muy similar si se da una ojeada a los ficheros involucrados.

`MoveIt!`:
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
		
		
Se puede apreciar en el esquema de los directorios cómo están relacionados los ficheros. Entre ellos los que se van a modificar manualmente están señalados en negrita.

El fichero de configuración de los controladores es el mismo tanto para `Gazebo` como para `MoveIt!`, esto implica que para conectar los controladores adecuadamente, su configuración debe ser la misma que la se ha realizado para `Gazebo` previamente y eso está configurado en la sección [Puesta en marcha en `Gazebo`](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/no_moveit/ESP/one_arm_no_moveit.md).

Por tanto, se puede añadir los controladores (ficheros .yaml) de `Gazebo` en el paquete de `MoveIt!` para conectar correctamente los controladores del robot (manipulator y gripper).

Si se lanza `Gazebo` y `MoveIt!` sin realizar ninguna modificación, se obtiene la siguiente gráfica de nodos y *topic*s:

- Terminal 1
```bash
roslaunch four_arm_moveit_config demo.launch
```

- Terminal 2
```bash
roslaunch four_arm_moveit_gazebo ur10_joint_limited.launch
```

![ ](/doc/imgs_md/one_arm_moveit_graph_no_changes.png  "Esquema sin cambios")
		
Se ve que `Gazebo` carga correctamente los controladores `arm_controller` y `gripper`, pero no hay comunicación entre el nodo `move_group` y los controladores del nodo `gazebo`, el único punto en común es el *topic* `/joint_states`. Esto implica que si se planifica y ejecuta trayectorias con el plugin *Motion Planning* de `MoveIt!`, no serán representados en `Gazebo` pero sı́ en *Rviz*. Este no es el resultado que se busca, por ello se procede a modificar la configuración de `MoveIt!` para que pueda comunicarse con los controladores cargados en `Gazebo`.

Por tanto, hay que modificar los siguientes ficheros. Los ficheros modificados estarán en un paquete diferente con la intención de facilitar la compresión de las modificaciones realizadas sobre la configuración.

Para ello se va a modificar los siguientes ficheros teniendo como base sus ficheros originales *demo.launch*, *move_group.launch*, *trajectory_execution.launch.xml*  y *ur10_moveit_controller_manager.launch.xml* y se añadirá los controladores creando dos ficheros *controllers.yaml* y *joint_names.yaml*:

```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_moveit
catkin_create_pkg four_arm_moveit_manipulator rospy
mkdir config
touch config/controllers.yaml
touch config/joint_names.yaml
mkdir launch
touch launch/four_arm_moveit_execution.launch
touch launch/move_group.launch
touch launch/trajectory_execution.launch.xml
touch launch/ur10_moveit_controller_manager.launch.xml
mkdir scripts
```
##### :computer: Se va a proceder a añadir los controladores para su interacción con `Gazebo`

- Fichero [~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_moveit/four_arm_moveit_manipulator/config/controllers.yaml](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_manipulator/config/controllers.yaml): La definición de los controladores de *ROS* en `MoveIt!` es similar al que se realizó para `Gazebo`, el nombre de los controladores debe coincidir con los nombres de los controladores descritos en `Gazebo`, se define el servidor de acciones `follow_joint_trajectory`, el tipo debe ser `FollowJointTrajectory` para que el tipo mensaje enviado entre ellos sean compatibles y finalmente los nombres de los `joints` involucrados deben ser idénticos también.

- Otro fichero a modificar es [~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_moveit/four_arm_moveit_manipulator/config/joint_names.yaml](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_manipulator/config/joint_names.yaml): Este fichero define el nombre de los `joints` del controlador del cobot, se almacenará como parámetro del servidor y será utilizado como parte de la configuración de `MoveIt!`.

De los ficheros en el directorio *launch*, se va a modificar el fichero *four_arm_moveit_execution* que es el punto de entrada para usar el paquete de `MoveIt!` y *Rviz*  que tiene como base el fichero *demo.launch*:

- Fichero [~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_moveit/four_arm_moveit_manipulator/launch/four_arm_moveit_execution.launch](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_manipulator/launch/one_arm_moveit_execution.launch).


Después retocar el lanzador de los controladores y `MoveIt!`:

- Fichero [~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_moveit/four_arm_moveit_manipulator/launch/move_group.launch](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_manipulator/launch/move_group.launch).


Fichero [~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_moveit/four_arm_moveit_manipulator/launch/trajectory_execution.launch.xml](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_manipulator/launch/trajectory_execution.launch.xml): Configura la comunicación con un robot real, en este caso `Gazebo` es el que simula el robot, pero `MoveIt!` no es consciente de eso y lo trata como un robot real.


Fichero [~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_moveit/four_arm_moveit_manipulator/launch/ur10_moveit_controller_manager.launch.xml](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_manipulator/launch/ur10_moveit_controller_manager.launch.xml): Carga los controladores definidos para `MoveIt!`.


##### :computer: Finalmente, se realiza una prueba:
```bash
cd ~/MultiCobot-UR10-Gripper
catkin_make
```

- Terminal 1
```bash
roslaunch four_arm_moveit_gazebo ur10_joint_limited.launch
```

- Terminal 2
```bash
roslaunch four_arm_moveit_manipulator four_arm_moveit_execution.launch
```

![ ](/doc/imgs_md/one_arm_moveit_26.png  "Example gazebo+rviz+moveit! (1/2)")
		
![ ](/doc/imgs_md/one_arm_moveit_27.png  "Example gazebo+rviz+moveit! (2/2)")

Y la gráfica de los nodos y los *topic*s, después de las modificaciones, se puede apreciar cómo ahora el nodo *move_group* tiene comunicación con los controladores y es *Gazebo* el que está a la escucha de lo que se publica para ejecutar los movimientos deseados. Estos movimientos modifican el estado actual del robot que es publicado al *topic* `/jont_states` y esa información es transmitida al nodo `robot_state_publisher` y al nodo `move_group`. El nodo `move_group` puede volver a calcular una nueva trayectoria con la información que le llega de los *topic*s `/tf` y `/jont_states`.

![ ](/doc/imgs_md/one_arm_moveit_graph_changes.png  "rqt_graph representación de los nodos y los topics")

##### :computer: Pick and Place

Para realizar el script de *pick & place* en Python, se utiliza la interfaz de Python `moveit_commander` para comunicarse con el nodo `move_group` y sus servicios y acciones. No se va a entrar en detalle porque para el control de un único robot no es muy problemático y hay una buena documentación, por ello se describirá el script en detalle para la solución con dos o más cobots.

Se procede a realizar unas pruebas muy sencillas. Para ello primero hay que crear los scripts necesarios para controlar el brazo del robot y la pinza correctamente y después se realizará los movimientos para que el robot coja un cubo de la mesa y lo deje en la cesta.

```bash
cd scripts
touch four_arm_moveit.py
```

Fichero [~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_moveit/four_arm_moveit_manipulator/scripts/four_arm_moveit.py](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_manipulator/scripts/one_arm_moveit.py)

<a name="pruebas">
  <h2>
Ejecución de las pruebas
  </h2>
</a>

```bash
# Terminal 1
roslaunch four_arm_moveit_gazebo ur10_joint_limited.launch

# Terminal 2
roslaunch four_arm_moveit_manipulator four_arm_moveit_execution.launch

# Terminal 3
rosrun four_arm_moveit_manipulator four_arm_moveit.py 
```
<a name="modificaciones">
  <h2>
Modificaciones: Sistema multirobot compuesto de cuatro robots
  </h2>
</a>

Para poder controlador cuatro cobots simultáneamente y con diferentes controladores, lo que implica que pueden ser cobots de diferentes marcas y modelos, se realiza mediante la replicación del nodo de `MoveIt!` y del robot simulado en `Gazebo`.

Si se quiere realizar una replicación correcta hay que aplicar el concepto de *namespace*, se puede ver como si fuese un *directorio* que contiene nodos, *topic*s o incluso otros directorios (*namespace*s) lo que permite también una organización jerarquizada y *ROS* permite ejecutar instancias del mismo nodo, siempre y cuando estén dentro de diferentes *namespace*s. Partiendo de lo realizado hasta ahora, se realizará cambios en los paquetes de `four_arm_moveit_gazebo` y `four_arm_moveit_manipulator` que contendrán las modificaciones realizadas sobre el paquete de `MoveIt!` (`four_arm_moveit_config)`, el cual fue previamente configurado por el setup assistant. 

Se va a dividir el proceso de la configuración en dos; configuración realizada en `Gazebo` para dos cobots y configuración en `MoveIt!` para dos cobots.

<a name="modificaciones1">
  <h3>
Configuración realizada en Gazebo para cuatro cobots
  </h3>
</a>

Para tener cuatro cobots en simulación, se va a explicar qué cambios hay que realizar en los ficheros *launch* para permitir la adicción de más cobots en la simulación correctamente.

La idea es crear un fichero externo que replique (lance instancias) de tantos cobots como se quieran añadir en la simulación. Por ello primero se preparan los ficheros de `Gazebo` del paquete `four_arm_moveit_gazebo`.

- Fichero [ur10_joint_limited.launch](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/four_arm_moveit/four_arm_moveit_gazebo/launch/ur10_joint_limited.launch): Respecto al fichero original se le ha añadido dos argumentos, el nombre del robot (`robot_name`) y la posición inicial (`init_pose`). Estos dos argumentos serán obtenidos del fichero que incluirá este *launch*.

- Fichero [ur10.launch](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/four_arm_moveit/four_arm_moveit_gazebo/launch/ur10.launch): De forma similar al fichero *launch* anterior (`ur10_joint_limited.launch`), a este fichero se le ha añadido los dos argumentos, el nombre del robot (`robot_name`) y la posición inicial (`init_pose`), que se definirán del fichero que lo incluya. Aparte de la adicción de los argumentos, se ha eliminado la instanciación del mundo virtual de `Gazebo` y la carga del modelo del cobot en el servidor de parámetros (`robot_description`).

- Fichero [controller_utils.launch]https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/four_arm_moveit/four_arm_moveit_gazebo/launch/controller_utils.launch): En este fichero simplemente hay que comentar el parámetro `tf_prefix`, su valor por defecto es una cadena vacía, pero eso interfiere a la hora de modificar su valor por defecto.

Se compila:
```bash
cd ~/MultiCobot-UR10-Gripper
catkin_make
```


##### :computer: Replicación de Cobots en `Gazebo`

Una vez configurado los ficheros del paquete de `Gazebo` `four_arm_moveit_gazebo`, se procede a instanciar varios cobots en este, para ello se crea dentro del paquete `four_arm_moveit_manipulator` (puede ser cualquier otro paquete) un fichero *launch* llamado `four_arm_moveit_gazebo.launch` que contiene lo siguiente:

```xml
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
Lo primero que se aprecia en el contenido del fichero ([four_arm_moveit_gazebo.launch](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/four_arm_moveit/four_arm_moveit_manipulator/launch/four_arm_moveit_gazebo.launch)) es que carga el modelo del robot en el servidor de parámetros e instancia el mundo virtual de `Gazebo`, lo que se eliminó del fichero [ur10.launch](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/four_arm_moveit/four_arm_moveit_gazebo/launch/ur10.launch), esto debe estar aquí porque solo se quiere instanciar una vez el mundo de `Gazebo`.

Después aparecen cuatro grupos, `ur10_1`, `ur10_2`, `ur10_3` y `ur10_4`, esta es la forma de definir los *namespace*s, la configuración de estos cobots es idéntica excepto por tres cosas, el valor del parámetro `tf_prefix` que será el prefijo que irá en las transformadas, es importante que coincida con el nombre del grupo (*namespace*), el nombre (`robot_name`) y su posición inicial (`init_pose`).

Si se quiere añadir más cobots al sistema, simplemente hay que copiar el contenido de *grupo* y modificar el contenido adecuadamente. Y tener en cuenta las últimas cuatro líneas de código unen la base de los cobots con el frame `world`.

<a name="modificaciones2">
  <h3>
Configuración realizada en MoveIt! para dos cobots
  </h3>
</a>

Los cambios realizados para la configuración de `MoveIt!` son muy pequeñas, básicamente hay que agrupar el código de lo que se había implementado para un *único cobot* bajo un *namespace*, adecuar el *remap* con el namespacing y posteriormente replicar el proceso tantas veces como cobots se esté simulando. El contenido del fichero [one_arm_moveit_execution.launch](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_manipulator/launch/one_arm_moveit_execution.launch) implementado para un único cobot y el contenido del fichero [four_arm_moveit_execution.launch](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/four_arm_moveit/four_arm_moveit_manipulator/launch/four_arm_moveit_execution.launch), si se comparan, esencialmente se ha agrupado todo en un *namespace* y se ha añadido el nombre del *namespace* como prefijo en los nombres de los *topic*s del remap para la correcta comunicación con los controladores.

Contenido del fichero `four_arm_moveit_execution.launch`:

```xml
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

Se realiza una prueba de lo implementado hasta el momento:

```bash
cd ~/MultiCobot-UR10-Gripper
catkin_make clean
catkin_make
rospack profile 
```

- Terminal 1
```bash
roslaunch four_arm_moveit_manipulator four_arm_moveit_gazebo.launch
```

- Terminal 2
```bash
roslaunch four_arm_moveit_manipulator four_arm_moveit_execution.launch
```

##### :computer: Pick & Place
Como en las soluciones anteriores, se procederá realizar una prueba muy sencilla. Para ello primero hay que crear los scripts necesarios para controlar los brazos de los robots y las pinzas correctamente y después, se realizará los mismos movimientos que en las soluciones anteriormente propuestas.

El siguiente trozo de código, corresponde a la configuración para poder comunicarse con la API del nodo `move_group`cuando este está en un *namespace*:

```python
[...]

1 def main():
2     moveit_commander.roscpp_initializer.roscpp_initialize(sys.argv)
3     rospy.init_node('ur10_1_arm_moveit',
4                   anonymous=True)
5
6     PLANNING_GROUP_GRIPPER = "gripper"
7     PLANNING_GROUP_ARM = "manipulator"
8     PLANNING_NS = "/ur10_1/"
9     REFERENCE_FRAME = "/ur10_1/world"
10 
11     ## Instantiate a RobotCommander object.  This object is an interface to
12     ## the robot as a whole.
13     robot = moveit_commander.RobotCommander(
14 		"%srobot_description"%PLANNING_NS, 
15 		ns="/ur10_1/"
16 	 )
17 
18     arm = moveit_commander.move_group.MoveGroupCommander(
19 		PLANNING_GROUP_ARM,
20 		"%srobot_description"%PLANNING_NS, 
21 		ns="/ur10_1/"
22 	 )
23
24     gripper = moveit_commander.move_group.MoveGroupCommander(
25 		PLANNING_GROUP_GRIPPER, 
26 		"%srobot_description"%PLANNING_NS, 
27 		ns="/ur10_1/"
28 	)
29
30     ## We create this DisplayTrajectory publisher which is used below to publish
31     ## trajectories for RVIZ to visualize.
32     display_trajectory_publisher = rospy.Publisher(
33 		'/move_group/display_planned_path',
34 		moveit_msgs.msg.DisplayTrajectory, 
35		queue_size=10
36 	 )
37
38     rospy.sleep(2)
39 
40     arm.set_num_planning_attempts(15)
41     arm.set_planning_time(5)
42     arm.allow_looking(True)
43     arm.allow_replanning(True)
44     arm.set_pose_reference_frame(REFERENCE_FRAME)
45     arm.set_goal_position_tolerance(0.001)
46     arm.set_goal_orientation_tolerance(0.001)
47 
48     gripper.set_num_planning_attempts(15)
49     gripper.allow_replanning(True)
50     gripper.allow_looking(True)
51
52     pick_place(arm, gripper)
53
54     ## When finished shut down moveit_commander.
55     moveit_commander.roscpp_shutdown()
```

Se ha definido la configuración en la función `main`, pero no es necesario que sea aquí, es más es preferible modularlo y pasarle la configuración por parámetros, pero para esta explicación es suficiente.

- **Línea 2:** Lo primero es inicializar `moveit_commander` que es una API sobre la interfaz desarrollada en `C++`, lo que es definido como *Wrappers*, este provee la mayoría de las funcionalidades que provee la interfaz de la versión para `C++`, pero no están todas las funcionalidades de *`MoveIt!`*. Es necesario porque entre sus funcionalidades permite calcular trayectorias cartesianas que es la funcionalidad que principalmente se requiere.
- **Línea 3:** Inicializa el nodo al que se le ha nombrado `ur10_1_arm` moveit.
- **Líneas 6-9:** Se definen las constantes para facilitar la configuración, las dos primeras líneas (6 y 7) definen los nombres que se pusieron a los grupos de planificación, en este caso se les llamó `gripper` para la pinza y `manipulator` para el brazo del cobot UR10. Las líneas 8 y 9 definen la información necesaria para configurar el planificador, `PLANNING_NS` contiene el nombre del *namespace* donde está el nodo `move_group` con el que se quiere comunicar y `REFERENCE_FRAME` es el link que se tomará como referencia para realizar los cálculos de trayectoria con respecto al *end-effector* (`ee_link`), en este caso daría igual que sea `/ur10_1/world`, pero lo correcto sería tomar el link `/ur10_1/base_link` como referencia.
- **Línea 13:** Inicializa el `RobotCommander`, como en el código indica, se utiliza para controlar el robot, hay que pasarle como parámetros el *namespace* y qué  `robot_description` debe utilizar para definir el robot, porque en el momento hay tres descripciones, que son el de `Gazebo` (`robot_description`) y los otros dos definidos en el fichero `planning_context.launch` que son instanciadas dentro de sus correspondientes *namespaces* por ello se tiene `/ur10_1/robot_description` y `/ur10_2/robot_description`.
- **Líneas 18 y 24:** Se crea una interfaz para un conjunto de Joints, en este caso la interfaz `arm` para el conjunto de `joints` dinámicos del brazo del cobot UR10 y la interfaz `gripper` para el joint que controla la pinza. A través de estas interfaces se realizarán las planificaciones de las trayectorias y su ejecución (es posible realizarlo con la interfaz de robot, ya que contiene a estas dos interfaces, pero es más claro y cómodo realizarlo de esta manera.
- **Línea 32:** Como dice en el comentario, `display_trajectory_publisher` realiza publicaciones al *topic* `/move_group/display_planned_path`, el cual *Rviz*  está suscrito para visualizar las trayectorias, no es necesario, pero para depuración es recomendable.
- **Línea 38:** Simplemente espera dos segundos, se asegura que las instancias anteriores se han cargado correctamente en el sistema, hay que tener en cuenta alguno de ellos instancian nodos y si el ordenador sobre el que se está lazando es lento puede provocar una situación de no deseada.
- **Líneas 40-50:** Aquí se está configurando algunas opciones del planificador que se va a utilizar, por defecto es RTT, pero se puede modificar. Los más importantes son los últimos en las líneas 44, 45 y 46. En la línea 44 se define el link de referencia que se utilizará para realizar las planificaciones, la línea 45 y 45 definen el margen de error aceptable del resultado obtenido del planificador para la posición y la orientación, hay que tener cuidado porque cuanto más pequeño es el error que se defina más tiempo tardará el planificador en dar una respuesta, está definidos para permitir errores de milímetros. El mismo proceso para la interfaz de la pinza (`gripper`).
- **Línea 52:** Aquí se lanza la tarea de *pick & place*.
- **Línea 55:** Una vez que termine la tarea *pick & place* finaliza.

Tras la descripción detallada del código, para que el script controlase otro cobot que esté en otro *namespace* es tan sencillo como cambiarle el valor de la variable `PLANNING_NS` por la del nombre del *namespace* donde esté definido el cobot objetivo. Si se ejecuta el *pick & place* con ambos brazos, se aprecia que realizan la tarea simultáneamente y con un poco de retraso entre ellos debido a que uno empieza un poco más tarde que otro, se puede realizar pruebas a lanzar los scripts a destiempo y comprobar que efectivamente se mueven simultáneamente.

##### :computer: Creación de los scripts que realizarán la tarea de *Pick & place*

```bash
cd scripts
touch four_arm_moveit_1.py
touch four_arm_moveit_2.py
touch four_arm_moveit_3.py
touch four_arm_moveit_4.py
```

- Fichero [~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_moveit/four_arm_moveit_manipulator/scripts/four_arm_moveit_1.py](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/four_arm_moveit/four_arm_moveit_manipulator/scripts/four_arm_moveit_1.py)

- Fichero [~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_moveit/four_arm_moveit_manipulator/scripts/four_arm_moveit_2.py](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/four_arm_moveit/four_arm_moveit_manipulator/scripts/four_arm_moveit_2.py)

- Fichero [~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_moveit/four_arm_moveit_manipulator/scripts/four_arm_moveit_3.py](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/four_arm_moveit/four_arm_moveit_manipulator/scripts/four_arm_moveit_3.py)

- Fichero [~/MultiCobot-UR10-Gripper/src/multirobot/four_arm_moveit/four_arm_moveit_manipulator/scripts/four_arm_moveit_4.py](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/four_arm_moveit/four_arm_moveit_manipulator/scripts/four_arm_moveit_4.py)


<a name="pruebas2">
  <h2>
Ejecución de las pruebas
  </h2>
</a>

Para ejecutar la prueba:

- Terminal 1
```bash
roslaunch four_arm_moveit_manipulator four_arm_moveit_gazebo.launch
```

- Terminal 2
```bash
roslaunch four_arm_moveit_manipulator four_arm_moveit_execution.launch
```

- Terminal 3
```bash
rosrun four_arm_moveit_manipulator four_arm_moveit_1.py 
```

- Terminal 4
```bash
rosrun four_arm_moveit_manipulator four_arm_moveit_2.py
```

- Terminal 5
```bash
rosrun four_arm_moveit_manipulator four_arm_moveit_3.py
```

- Terminal 6
```bash
rosrun four_arm_moveit_manipulator four_arm_moveit_4.py
```

----

En la terminal de `Gazebo`, al lanzar todos los comandos para realizar las pruebas se obtiene el siguiente warning:

![ ](/doc/imgs_md/four_arm_move_it_gazebo_warning.png  "Warning por falta de recursos")

Esto es debido a que el sistema avisa de que los recursos son insuficientes para realizar la simulación fluidamente, lo que puede acarrear problemas en su velocidad de simulación que puede afectar al resultado de este.

#### :book: Información del resultado final

- Resultado visual en `Gazebo`
![image](/doc/imgs_md/four-arm-moveit-gazebo.png  "Resultado en Gazebo")

- Esquema de los nodos y *topic*s del sistema
![image](/doc/imgs_md/four-arm-moveit-graph.png  "Nodos y topics del sistema")

Se procede a comprobar si la comunicación entre `Gazebo` y las réplicas de `MoveIt!` se comunican correctamente. Se puede ver cuatro grandes agrupaciones, en donde cada cobot se está comunicando con su `move_group` asignado, la comunicación con los controladores y el camino de las transformadas y valores de los `joints` tiene un único origen que es el nodo de `Gazebo`, esto es muy importante para evitar movimientos extraños dependiendo de la frecuencia en que se produzcan esas interferencias.

El camino parte del nodo de `Gazebo` es el único que publica a los *topic*s
`/joint_states` de ambos *namespaces*, el nodo `move_group` está suscrito a este *topic* también, después llega al nodo `robot_state_publisher` que realiza las transformadas y se las envía por el *topic* `/tf` el cual el nodo `move_group` también está suscrito, esto es importante porque `move_group` utiliza la información que proviene de ambos para realizar la planificación de las trayectorias.

- Árbol de las transformadas del modelo del robot
![image](/doc/imgs_md/four-arm-moveit-tree.png  "Árbol de transformadas")

---

<div>
  <p align="left">
    <button name="button">
                <a rel="license" href="https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/moveit-intro.md">Anterior</a>
    </button>
  </p>
</div>