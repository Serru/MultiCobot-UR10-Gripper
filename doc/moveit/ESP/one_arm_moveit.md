<!--- Para un robot  opción B--->
# Instalación y configuración para un único robot UR10 con `MoveIt!`

**Español** | [English](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/moveit/ENG/one_arm_moveit.md)

![image](/doc/imgs_md/Diseno-moveit-general-un-cobot-leap-motion.png  "Cargado el modelo URDF del robot UR10")

## Requisito previo
- Realizar correctamente la instalación de la [configuración base del sistema](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/setup-doc/proyect_setup.md).
- Implementar la [solución para un robot sin el planificador de `MoveIt!`](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/no_moveit/ESP/one_arm_no_moveit.md).

## Índice
- [Fase 1: Configuración del *URDF*](#fase1)
- [Fase 2: Configuración de `MoveIt!`](#fase2)
- [Fase 3: Simulación de un `pick & place` en `Gazebo`](#fase3)
- [Ejecución de las pruebas](#pruebas)


<a name="fase1">
  <h2>
Fase 1: Configuración del URDF
  </h2>
</a>

### :book: Descripción del fichero *URDF*
El fichero *URDF* (United Robotics Description Format) modela el cobot utilizando el formato *XML*el cual será utilizado por las diferentes aplicaciones que *ROS* necesite, pero principalmente para realizar una simulación del robot modelado.

El fichero está construido en forma de árbol, en donde hay tres etiquetas principales: `<robot>`, `<link>` y `<joint>`. Para explicarlo bien, se puede tomar como referencia el brazo del cuerpo humano. Si lo que se quiere modelar es el brazo de una persona, la etiqueta `<robot>` representa al brazo en su conjunto. Este brazo está compuesto de varios huesos (húmero, cúbito y radio) que son representados por las etiquetas `<link>` y por una articulación que une esos huesos (codo) que es representado por la etiqueta `<joint>`. 

Además, como en los huesos, estas etiquetas pueden ir con información adicional contenida en ellas que den información del tamaño, geometría, inercia, orientación, etc. Finalmente, el modelado de un robot se puede unir a otro modelo y formar uno más complejo, que podría ser representado con la adición de la mano al brazo, con la muñeca como articulación que conectan ambos. Hay que tener en cuenta que las etiquetas `<joint>` conecta las etiquetas `<link>` a través de una relación padre-hijo.

Dicho esto, se realiza una representación de los componentes del robot:
 
 ![image](/doc/imgs_md/urdf-robot.png  "Representación del fichero URDF")

En la imagen se representa el contenido del [fichero *URDF*](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_description/urdf/ur10_robot.urdf.xacro) que modela el robot junto a la pinza, se puede ver cómo se conecta el componente del brazo UR10 robot con el link `world`, representando `world`(color amarillo) y la base del brazo del UR10 `base_link` (color verde) situado justo encima, además el joint `world_joint` es la esfera de color amarillo situado entre ambos links. De la misma manera, se tiene el componente de la pinza `robotiq_85_gripper`, está conectado al brazo del UR10 (`ur10_robot`), en donde la esfera que representa el joint `robotiq_85_base_joint` que une ambos componentes (color morado), uniendo el link `robotiq_85_base_link` de la pinza con el link `ee_link` del brazo de UR10.


#### :computer: Creación del directorio para la solución
```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot
mkdir one_arm_moveit
```

#### :computer: Configuración del directorio descripción
Se crea un nuevo paquete y se copia los directorios del proyecto *one_arm_no_moveit* para su posterior modificación.

```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit
catkin_create_pkg one_arm_moveit_description rospy
```

En el directorio creado para *description*, se copiará del directorio de *one_arm_no_moveit_description*, las carpetas *launch* y *urdf*.

```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit/one_arm_moveit_description
cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_description/launch .
cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_description/urdf .
```

#### :computer: Modificación de los ficheros de *description*

- [~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit/one_arm_moveit_description/launch/ur10_upload.launch](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_description/launch/ur10_upload.launch)

- [~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit/one_arm_moveit_description/urdf/ur10_robot.urdf.xacro](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_description/urdf/ur10_robot.urdf.xacro)

- [~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit/one_arm_moveit_description/urdf/ur10_joint_limited_robot.urdf.xacro](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_description/urdf/ur10_joint_limited_robot.urdf.xacro)

Se compila:

```bash
cd ~/MultiCobot-UR10-Gripper
catkin_make
```

<a name="fase2">
  <h2>
Fase 2: Configuración de MoveIt!
  </h2>
</a>

### Configuración de `MoveIt!`

Antes de realizar la configuración con el `Setup Assitant` hay que tener el `URDF` bien definido previamente, con ese hecho se lanza el asistente de configuración con el siguiente comando en la terminal:

```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit/
mkdir one_arm_moveit_config
roslaunch moveit_setup_assistant setup_assistant.launch
```

#### :computer: `MoveIt!` Setup Assistant
Se va a escoger como modelo del robot el fichero *URDF*: [~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit/one_arm_moveit_description/urdf/ur10_joint_limited_robot.urdf.xacro](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_description/urdf/ur10_joint_limited_robot.urdf.xacro) (podría ser perfectamente [ur10_robot.urdf.xacro](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_description/urdf/ur10_robot.urdf.xacro)).

- Comenzando la configuración del Setup Assistant, hay que decirle donde está el fichero `UR10_joint_limited_robot.urdf.xacro`, es decir, el modelo del robot que se quiere configurar:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_1.png  "Cargar el modelo URDF del robot UR10")

- Posteriormente, se le da al botón *Load Files*.
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_2.png  "Cargado el modelo URDF del robot UR10")

- En la pestaña *Self-Collisions*, darle al botón *Generate Collision Matrix*, lo que generará una matriz entre los diferentes componentes del robot que puedan generar autocolisiones durante la planificación de las trayectorias:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_3.png  "Generación de matriz de colisiones")

- En la pestaña *Virtual Joints* está dirigida especialmente para brazos robóticos instalados sobre una base móvil, en este caso no afecta en la configuración, ya que la base es fija, pero se configurará igualmente para configuraciones futuras, hay que crear un joint entre el robot `base_link` y el frame `world`, siendo la configuración la siguiente:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_4.png  "Definiendo Virtual Joint")
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_5.png  "Definido Virtual Joint")

- Una de las pestañas más importantes es definir bien los *Planning groups*, en este caso se tiene dos grupos, el grupo *manipulator* que controlará el brazo del robot y el grupo *gripper* que controlará la pinza:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_6.png  "Manipulator kdl")

- Después hay que darle al botón *Add Kin. Chain*:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_7.png  "Manipulator Kinetic Chain configuración")

- Finalmente, se guarda la configuración:
![ ](/doc/imgs_md/one_arm_moveit_setup_assistant_8.png  "Grupo manipulator configurado")

- Ahora hay que hacerlo para el grupo *gripper* que controlará la pinza, se pulsa el botón *Add Group*, se rellena con el nombre del grupo y se pone *kdl* como *Kinematic Solver*:
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
  <h2>
Fase 3: Simulación de un <i>pick & place</i> en `Gazebo`
  </h2>
</a>

Esta fase tiene se divide en dos etapas

#### :book: Conexión entre `Gazebo` y `MoveIt!`

Lo primero que hay que hacer en esta fase es configurar `Gazebo` y los controladores para que pueda simular adecuadamente los movimientos del cobot. Se crea el paquete *one_arm_moveit_gazebo*, que contendrá toda la configuración relacionada con `Gazebo`, entre ellos los controladores. Una vez creada el paquete, hay que configurar los controladores que están almacenados en el directorio *controller*, aunque todos los controladores pueden estar definidos en un único fichero por claridad se ha distribuido en tres ficheros.

Los controladores se definen en ficheros con extensión *yaml*, para definir estos controladores hay que darles un nombre y definir el tipo del controlador, los `joints` dinámicos que se quieren controlar, las restricciones que tiene, el ratio de publicación y otras opciones.

Se procede a explicar brevemente estos controladores:

- Fichero [arm_controller_ur10.yaml](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_gazebo/controller/arm_controller_ur10.yaml): En este fichero se define el controlador para el cobot UR10, aquí se define el nombre del controlador `arm_controller`, el tipo de controlador posición `controllers/JointTrajectoryController`, lo que implica la definición del tipo de mensajes y el formateo adecuado de la información necesaria para comunicarse con este. Después está el campo `joints`, que es donde se indica qué `joints` del cobot forma parte del controlador, todos estos `joints` son dinámicos. El resto de campos no se han tocado, pero hay que mantener la consistencia en cómo se nombran.

- Fichero [gripper_controller_robotiq.yaml](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_gazebo/controller/gripper_controller_robotiq.yaml): En este fichero se define el controlador para la pinza de *Robotiq*, aquí se define el nombre del controlador `gripper`, el tipo de controlador `position controllers/JointTrajectoryController` que define el tipo de mensajes y la información necesaria para comunicarse con este. Después está el campo `joints`, que es donde se indica qué `joints` del cobot forma parte del controlador en este caso un único `joint_robotiq_85_left_knuckle_joint` porque el resto de `joints` del controlador imitan los movimientos de este. El resto de campos no se han tocado, pero hay que mantener la consistencia en cómo se nombran como en el caso anterior.

- Fichero [joint_state_controller.yaml](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_gazebo/controller/joint_state_controller.yaml): Lo que define este fichero realmente no es un controlador como tal, su función es la de una interfaz que traduce la información de los `joints` que viene del cobot real y lo traduce a mensajes de tipo `JointState` para después publicarlo. Es fundamental para el correcto funcionamiento, tanto en simulación como con el robot real, forma parte del paquete de *ROS* *ros_control*.

#### :computer: Puesta en marcha de `Gazebo`

Se va a crear el paquete para `Gazebo`, y copiar el contenido de la solución partiendo de la solución *one_arm_no_moveit* para su posterior modificación:

```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit
catkin_create_pkg one_arm_moveit_gazebo rospy
```

En el directorio creado para `Gazebo`, se copiará del directorio de *one_arm_no_moveit_gazebo*, las carpetas *controller*, *launch*, *models*, y *world*.

```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit/one_arm_moveit_gazebo
cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/controller .
cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/launch .
cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/models .
cp -r ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/world .
```

#### :computer: Modificación de los ficheros de `Gazebo`
Se crean los ficheros con el siguiente contenido:

- [~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit/one_arm_moveit_gazebo/launch/ur10_joint_limited.launch](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_gazebo/launch/ur10_joint_limited.launch)

- [~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit/one_arm_moveit_gazebo/launch/ur10.launch](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_gazebo/launch/ur10.launch)

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
roslaunch one_arm_moveit_config demo.launch
```

- Terminal 2
```bash
roslaunch one_arm_moveit_gazebo ur10_joint_limited.launch
```

![ ](/doc/imgs_md/one_arm_moveit_graph_no_changes.png  "Esquema sin cambios")
    
Se ve que `Gazebo` carga correctamente los controladores `arm_controller` y `gripper`, pero no hay comunicación entre el nodo `move_group` y los controladores del nodo `gazebo`, el único punto en común es el *topic* `/joint_states`. Esto implica que si se planifica y ejecuta trayectorias con el plugin *Motion Planning* de `MoveIt!`, no serán representados en `Gazebo` pero sı́ en *Rviz*. Este no es el resultado que se busca, por ello se procede a modificar la configuración de `MoveIt!` para que pueda comunicarse con los controladores cargados en `Gazebo`.

Por tanto, hay que modificar los siguientes ficheros. Los ficheros modificados estarán en un paquete diferente con la intención de facilitar la compresión de las modificaciones realizadas sobre la configuración.

Para ello se va a modificar los siguientes ficheros teniendo como base sus ficheros originales *demo.launch*, *move_group.launch*, *trajectory_execution.launch.xml*  y *ur10_moveit_controller_manager.launch.xml* y se añadirá los controladores creando dos ficheros *controllers.yaml* y *joint_names.yaml*:

```bash
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

#### :computer: Se va a proceder a añadir los controladores para su interacción con `Gazebo`

- Fichero [~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit/one_arm_moveit_manipulator/config/controllers.yaml](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_manipulator/config/controllers.yaml): La definición de los controladores de *ROS* en `MoveIt!` es similar al que se realizó para `Gazebo`, el nombre de los controladores debe coincidir con los nombres de los controladores descritos en `Gazebo`, se define el servidor de acciones `follow_joint_trajectory`, el tipo debe ser `FollowJointTrajectory` para que el tipo mensaje enviado entre ellos sean compatibles y finalmente los nombres de los `joints` involucrados deben ser idénticos también.

- Otro fichero a modificar es [~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit/one_arm_moveit_manipulator/config/joint_names.yaml](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_manipulator/config/joint_names.yaml): Este fichero define el nombre de los `joints` del controlador del cobot, se almacenará como parámetro del servidor y será utilizado como parte de la configuración de `MoveIt!`.

De los ficheros en el directorio *launch*, se va a modificar el fichero *one_arm_moveit_execution* que es el punto de entrada para usar el paquete de `MoveIt!` y *Rviz*  que tiene como base el fichero *demo.launch*:

- Fichero [~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit/one_arm_moveit_manipulator/launch/one_arm_moveit_execution.launch](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_manipulator/launch/one_arm_moveit_execution.launch).


Después retocar el lanzador de los controladores y `MoveIt!`:

- Fichero [~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit/one_arm_moveit_manipulator/launch/move_group.launch](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_manipulator/launch/move_group.launch).


- Fichero [~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit/one_arm_moveit_manipulator/launch/trajectory_execution.launch.xml](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_manipulator/launch/trajectory_execution.launch.xml): Configura la comunicación con un robot real, en este caso `Gazebo` es el que simula el robot, pero `MoveIt!` no es consciente de eso y lo trata como un robot real.


- Fichero [~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit/one_arm_moveit_manipulator/launch/ur10_moveit_controller_manager.launch.xml](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_manipulator/launch/ur10_moveit_controller_manager.launch.xml): Carga los controladores definidos para `MoveIt!`.


#### :computer: Finalmente, se realiza una prueba:

```bash
cd ~/MultiCobot-UR10-Gripper
catkin_make
```

- Terminal 1
```bash
roslaunch one_arm_moveit_gazebo ur10_joint_limited.launch
```

- Terminal 2
```bash
roslaunch one_arm_moveit_manipulator one_arm_moveit_execution.launch
```

![ ](/doc/imgs_md/one_arm_moveit_26.png  "Example gazebo+rviz+moveit! (1/2)")
    
![ ](/doc/imgs_md/one_arm_moveit_27.png  "Example gazebo+rviz+moveit! (2/2)")

Y la gráfica de los nodos y los *topic*s, después de las modificaciones, se puede apreciar cómo ahora el nodo *move_group* tiene comunicación con los controladores y es *Gazebo* el que está a la escucha de lo que se publica para ejecutar los movimientos deseados. Estos movimientos modifican el estado actual del robot que es publicado al *topic* `/jont_states` y esa información es transmitida al nodo `robot_state_publisher` y al nodo `move_group`. El nodo `move_group` puede volver a calcular una nueva trayectoria con la información que le llega de los *topic*s `/tf` y `/jont_states`.

![ ](/doc/imgs_md/one_arm_moveit_graph_changes.png  "rqt_graph representación de los nodos y los topics")

#### :computer: Pick and Place

Para realizar el script de *pick & place* en Python, se utiliza la interfaz de Python `moveit_commander` para comunicarse con el nodo `move_group` y sus servicios y acciones. No se va a entrar en detalle porque para el control de un único robot no es muy problemático y hay una buena documentación, por ello se describirá el script en detalle para la solución con dos o más cobots.

Se procede a realizar unas pruebas muy sencillas. Para ello primero hay que crear los scripts necesarios para controlar el brazo del robot y la pinza correctamente y después se realizará los movimientos para que el robot coja un cubo de la mesa y lo deje en la cesta.

```bash
cd scripts
touch one_arm_moveit.py
```

Fichero [~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_moveit/one_arm_moveit_manipulator/scripts/one_arm_moveit.py](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_manipulator/scripts/one_arm_moveit.py)

<a name="pruebas">
  <h2>
Ejecución de las pruebas
  </h2>
</a>

- Terminal 1
```bash
roslaunch one_arm_moveit_gazebo ur10_joint_limited.launch
```

- Terminal 2
```bash
roslaunch one_arm_moveit_manipulator one_arm_moveit_execution.launch
```

- Terminal 3
```bash
rosrun one_arm_moveit_manipulator one_arm_moveit.py 
```

#### :book: Información del resultado final

- Resultado visual en `Gazebo`
![image](/doc/imgs_md/one-arm-moveit-gazebo.png  "Resultado en Gazebo")

- Esquema de los nodos y *topic*s del sistema
![image](/doc/imgs_md/one-arm-moveit-graph.png  "Nodos y topics del sistema")

- Árbol de las transformadas del modelo del robot
![image](/doc/imgs_md/one-arm-moveit-tree.png  "Árbol de transformadas")

---

<div>
  <p align="left">
    <button name="button">
                <a rel="license" href="https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/moveit-intro.md">Anterior</a>
    </button>
  </p>
</div>