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

### Configuración realizada en MoveIt!
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
