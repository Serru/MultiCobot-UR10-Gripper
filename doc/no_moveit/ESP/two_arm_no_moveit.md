<!--- Para dos robots opción A--->
# Instalación y configuración para dos robots UR10 sin `MoveIt!`

**Español** | [English](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/no_moveit/ENG/two_arm_no_moveit.md)

![image](/doc/imgs_md/Diseno-no-moveit-general-dos-cobots-leap-motion.png  "Cargado el modelo URDF del robot UR10")

Se va a realizar la solución para dos robots esta vez, de la misma manera que se ha realizado para uno, pero modificando el contenido de los ficheros adaptándolo para su simulación con dos robots.

Las fases que se ven en el esquema son de orientación. Se pueden hacer en el orden que se prefiera, se ha dividido el esquema en fases para mantener un orden y conocer sobre qué elemento del esquema se está trabajando. En este caso se comenzará por la fase 1, seguido de la fase 2 y finalmente se termina con la fase 3. Hay que tener en cuenta que puede existir configuraciones en una fase que pertenece realmente a otra fase, cuando esto suceda se señalará adecuadamente.

## Requisito previo
- Realizar correctamente la instalación de la [configuración base del sistema](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/setup-doc/proyect_setup.md).

## Índice
- [Configuración inicial: Configuración para un robot](#setup-inicial)
  - [Fase 1: Configuración del simulador de `Gazebo`](#fase1)
  - [Fase 2: Configuración del *URDF*](#fase2)
  - [Fase 3: Implementación de un planificador propio que realiza un `pick & place`](#fase3)
  - [Ejecución de las pruebas](#pruebas)
- [Modificaciones: Sistema multirobot compuesto de dos robots](#modificaciones)
  - [Fase 1: Configuración del simulador de `Gazebo`](#modificaciones1)
  - [Fase 2: Configuración del *URDF*](#modificaciones2)
  - [Fase 3: Implementación de un planificador propio que realiza un `pick & place`](#modificaciones3)
  - [Ejecución de las pruebas](#pruebas2)


<a name="setup-inicial">
  <h2>
Configuración inicial: Configuración para un robot
  </h2>
</a>

En esta sección se realiza una replicación de la configuración para un único robot, del cual se toma como base y así explicar posteriormente las modificaciones realizadas para poder controlar dos robots.

### :warning: Contenido de los ficheros
No copiar y pegar el contenido de los ficheros ciegamente en esta sección. Durante la explicación para la configuración para un robot, la información es idéntica a lo que se obtiene del paquete **one_arm_no_moveit**, pero hay que modificar el contenido para se ajuste al nuevo paquete, en este caso **two_arm_no_moveit**.

Hay dos opciones:
- Si se ha realizado la solución para un robot [one_arm_no_moveit](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/no_moveit/ESP/one_arm_no_moveit.md), se puede continuar sobre lo implementado, pero en la sección de *modificaciones* hay que cambiar **two_arm_no_moveit** por **one_arm_no_moveit**.
- Si no se ha realizado la solución para un robot [one_arm_no_moveit](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/no_moveit/ESP/one_arm_no_moveit.md), se puede seguir los pasos que se presentan en esta sección, pero hay que tener en cuenta que el contenido de los ficheros en esta fase de *configuración inicial* están enlazadas a los ficheros de **one_arm_no_moveit** y por ello al realizar la copia de este hay que reemplazar las referencias al paquete **one_arm_no_moveit** por **two_arm_no_moveit**

Si no se tiene en cuenta esta información, el paquete no compilará correctamente y el propio compilador obligará a que se elija entre una de las opciones sugeridas previamente.

<a name="fase1">
  <h3>
Fase 1: Configuración del simulador de Gazebo
  </h3>
</a>

#### :book: Configuración de `Gazebo`

Lo primero que hay que hacer es configurar `Gazebo` y los controladores para que pueda simular adecuadamente los movimientos del cobot. Se crea el paquete `one_arm_no_ moveit_gazebo`, que contendrá toda la configuración relacionada con `Gazebo`, entre ellos los controladores. 

Una vez creada el paquete, hay que configurar los controladores que están almacenados en el directorio `controller`. Los controladores se definen en ficheros con extensión *yaml*, para definir estos controladores hay que darles un nombre y definir el tipo del controlador, los `joints` dinámicos que se quieren controlar, las restricciones que tiene, el ratio de publicación y otras opciones.

A continuación se presenta el contenido de los ficheros de configuración de los controladores, todos estos controladores, en general, siguen la estructura mencionada. La definición de los controladores pueden ser contenidas en un único fichero, lo importante es que en `Gazebo` los cargue correctamente, se procede a explicar brevemente estos controladores:

- Fichero [arm_controller_ur10.yaml](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/controller/arm_controller_ur10.yaml): En este fichero se define el controlador para el cobot UR10, aquı́ se define el nombre del controlador `arm_controller`, el tipo de controlador posición `controllers/JointTrajectoryController`, lo que implica la definición del tipo de mensajes y el formateo adecuado de la información necesaria para comunicarse con este. Después está el campo `joints`, que es donde se indica qué `joints` del cobot forma parte del controlador, todos estos `joints` son dinámicos. El resto de campos no se han tocado, pero hay que mantener la consistencia en cómo se nombran.

- Fichero [joint_state_controller.yaml](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/controller/joint_state_controller.yaml): Lo que define este fichero realmente no es un controlador como tal, su función es la de una interfaz que traduce la información de los `joints` que viene del cobot real y lo traduce a mensajes de tipo `JointState` para después publicarlo. Es fundamental para el correcto funcionamiento, tanto en simulación como con el robot real, forma parte del paquete de *ROS* *ros_control*.


##### :computer: Creación del directorio para la solución
```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot
mkdir two_arm_no_moveit
```

##### :computer: Puesta en marcha de `Gazebo`
Actualmente, desde el directorio creado del proyecto no se puede lanzar `Gazebo` con el robot, y es el primer paso lanzar `Gazebo` con el robot UR10.

Por imponer cierto orden en la estructura del proyecto a implementar, se van a separar el uso de las diferentes herramientas y sus ficheros siempre que se pueda. Esto permitirá una mejor compresión de cómo está estructurado y facilitará posteriormente su depuración.

Por ello, se creará un paquete, en vez de un directorio que contendrá todo lo relacionado con `Gazebo`:

```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit
catkin_create_pkg two_arm_no_moveit_gazebo rospy
```

En el directorio creado para `Gazebo`, se copiará del directorio de *ur_gazebo*, las carpetas *controller* y *launch*.

```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_gazebo
cp -r ~/MultiCobot-UR10-Gripper/src/universal_robot/ur_gazebo/controller .
cp -r ~/MultiCobot-UR10-Gripper/src/universal_robot/ur_gazebo/launch .
```

Se compila:

```bash
cd ~/MultiCobot-UR10-Gripper
catkin_make
```

Con esto ya se puede lanzar `Gazebo` desde el directorio del proyecto.

```bash
roslaunch two_arm_no_moveit_gazebo ur10.launch
```

##### Configuración del mundo de `Gazebo`

Ahora se procede a crear el mundo con el robot y un entorno sobre el que podrá realizar simples tareas, este proyecto se enfoca en las tareas que pueda realizar el robot, por tanto, no es necesario que el mundo sea muy detallado.

Para ello lo primero es crear el directorio *world*, en donde se guardará los `world`s que se creen:

```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_gazebo
mkdir world
```

Se utilizará el fichero `world`, de otro [repositorio](https://github.com/Infinity8sailor/multiple_arm_setup/tree/main/multiple_ur_description/) que provee de un escenario muy simple que permitirá la comprobación posterior de tareas en el robot.

```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_gazebo
git clone https://github.com/Infinity8sailor/multiple_arm_setup.git
cp -r multiple_arm_setup/multiple_ur_description/models/ .
cp -r multiple_arm_setup/multiple_ur_description/world/ .
sudo rm -r mutiple_arm_setup
```

Para que añadir el `world`con el robot en `Gazebo`, hay que modificar el fichero [ur10.launch](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/launch/ur10.launch) en el directorio *launch* :

```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_gazebo/launch
nano ur10.launch
```

E incluir el mundo en el *launch*, añadiendo el argumento `world`y reemplazando el valor de *default* en el argumento *world_name*:

```xml
 <arg name="world" default="$(find two_arm_no_moveit_gazebo)/world/multiarm_bot.world" />

  <!-- startup simulated world -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" default="$(arg world)"/>
    <arg name="paused" value="$(arg paused)"/>
    <arg name="gui" value="$(arg gui)"/>
  </include>
```

Se procede a lanzar `Gazebo` con el nuevo mundo:

```bash
roslaunch two_arm_no_moveit_gazebo ur10.launch
```

Al lanzarlo, muestra dos errores:

```bash
Error [parser.cc:581] Unable to find uri[model://dropbox]
```

Este error no se puede solucionar, a no ser que se cree el modelo de cero, ya que `Gazebo` no lo provee o se ha eliminado. Por tanto, en el fichero [multiarm_bot.world](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/world/multiarm_bot.world) se comenta el objeto *dropbox*:

```xml
<!--include>
       <pose frame=''>0.5 0.0 0.3 0 0 0</pose> 
       <uri>model://dropbox</uri>
       <name>DropBox</name>
</include-->
```

---

```bash
[ERROR] [1639740881.902412642, 0.057000000]: GazeboRosControlPlugin missing <legacyModeNS> while using DefaultRobotHWSim, defaults to true.
This setting assumes you have an old package with an old implementation of DefaultRobotHWSim, where the robotNamespace is disregarded and absolute paths are used instead.
If you do not want to fix this issue in an old package just set <legacyModeNS> to true.
```

Para eliminar este error, se procede a realizar el cambio desde el fichero de [~/MultiCobot-UR10-Gripper/src/universal_robot/ur_descriptiom/urdf/common.gazebo.xacro](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/universal_robot/ur_description/urdf/common.gazebo.xacro), hay que añadir *<legacyModeNS>* al fichero:

```xml
<plugin name="ros_control" filename="libgazebo_ros_control.so">
      <!--robotNamespace>/</robotNamespace-->
      <!--robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType-->
      <legacyModeNS>true</legacyModeNS>
</plugin>
```

##### :computer: Instalación de `Gazebo` 9 [Opcional]
Puede surgir un *warning*, que puede ser un problema el ignorarlo a la hora de simular el comportamiento de brazo en el futuro, por lo que se procede a resolverlo.

```bash
[ WARN] [1639745803.729749460, 0.061000000]: The default_robot_hw_sim plugin is using the Joint::SetPosition method without preserving the link velocity.
[ WARN] [1639745803.729772883, 0.061000000]: As a result, gravity will not be simulated correctly for your model.
[ WARN] [1639745803.729786659, 0.061000000]: Please set gazebo_pid parameters, switch to the VelocityJointInterface or EffortJointInterface, or upgrade to Gazebo 9.

```

Para ello, se ha decidido instalar `Gazebo` 9 en el entorno de *ROS Kinetic Kame*.

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

El resultado tras instalar `Gazebo` 9 es similar a la siguiente.
![setup stage](/doc/imgs_md/one-arm-no-moveit-gazebo-setup.png  "Gazebo9-world-setup")

<a name="fase2">
  <h3>
Fase 2: Configuración del URDF
  </h3>
</a>

#### :book: Descripción del fichero *URDF*
El fichero *URDF* (United Robotics Description Format) modela el cobot utilizando el formato *XML* el cual será utilizado por las diferentes aplicaciones que *ROS* necesite, pero principalmente para realizar una simulación del robot modelado.

El fichero está construido en forma de árbol, en donde hay tres etiquetas principales: `<robot>`, `<link>` y `<joint>`. Para explicarlo bien, se puede tomar como referencia el brazo del cuerpo humano. Si lo que se quiere modelar es el brazo de una persona, la etiqueta `<robot>` representarı́a al brazo en su conjunto. Este brazo está compuesto de varios huesos (húmero, cúbito y radio) que son representados por las etiquetas `<link>` y por una articulación que une esos huesos (codo) que es representado por la etiqueta `<joint>`. 

Además, como en los huesos, estas etiquetas pueden ir con información adicional contenida en ellas que den información del tamaño, geometrı́a, inercia, orientación, etc. Finalmente, el modelado de un robot se puede unir a otro modelo y formar uno más complejo, que podrı́a ser representado con la adición de la mano al brazo, con la muñeca como articulación que conectan ambos. Hay que tener en cuenta que las etiquetas `<joint>` conecta las etiquetas `<link>` a través de una relación padre-hijo.

Dicho esto, se realiza una representación de los componentes del robot:
 
 ![image](/doc/imgs_md/urdf-robot.png  "Representación del fichero URDF")

En la imagen se representa el contenido del [fichero *URDF*](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_description/urdf/ur10_robot.urdf.xacro) que modela el robot junto a la pinza, se puede ver cómo se conecta el componente del brazo UR10 robot con el link `world`, representando `world`(color amarillo) y la base del brazo del UR10 `base_link` (color verde) situado justo encima, además el joint `world_joint` es la esfera de color amarillo situado entre ambos links. De la misma manera, se tiene el componente de la pinza `robotiq_85_gripper`, está conectado al brazo del UR10 (`ur10_robot`), en donde la esfera que representa el joint `robotiq_85_base_joint` que une ambos componentes (color morado), uniendo el link `robotiq_85_base_link` de la pinza con el link `ee_link` del brazo de UR10.


##### :computer: :warning: Agregación del robotiq_2f_85_gripper al robot UR10 [No ha sido posible, problemas con `Gazebo`]
Para agregar correctamente la pinza hay que entender primero el funcionamiento de este. Las instrucciones de instalación está en [aquí](https://github.com/Danfoa/robotiq_2finger_grippers).
 
 Tomando la siguiente imagen para entender el funcionamiento del controlador de la pinza:
 ![esquema del gripper](/doc/imgs_md/robotiq_2f_85_gripper.png  "controlador del gripper")

Se puede apreciar 2 nodos, uno hace de cliente y otro de servidor. El *servidor* es que se encarga de enviar las órdenes a la pinza y feedback al cliente y el *cliente* envía órdenes al servidor.

Si comparamos la configuración del controlador con la del UR10, se aprecia que el concepto es diferente, ya que no existe un fichero *yaml* que cargue con la información necesaria para controlar la pinza. En vez de eso, es el servidor que crea los *topic*s */command_robotiq_action* y */robotiq_controller/follow_joint_trajectory*, ambos son de tipo *action*.

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

Sabiendo esto, para controlar la pinza, hay un ejemplo en el fichero [robotiq_2f_action_client_example.py](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/robotiq_2finger_grippers/robotiq_2f_gripper_control/scripts/robotiq_2f_action_client_example.py) y puede funcionar con una pinza simulada o la pinza real. Teniendo esto en cuenta, a la hora de agregar la pinza al robot UR10, no es necesario el fichero *yaml* que cargaría el controlador del driver, pero hay que lanzar correctamente el nodo que hará de servidor y si se mira el código de servidor y del cliente, hay que tener especial cuidado con la configuración de los *topic*s y *namespace*s.

El fichero que carga el robot UR10 es [~/MultiCobot-UR10-Gripper/src/universal_robot/ur_description/launch/ur10_upload.launch](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/universal_robot/ur_description/launch/ur10_upload.launch), por tanto, se va a copiar lo necesario del paquete *ur_description* y modificarlo para añadir la pinza al robot y también modificar el fichero [ur10.launch](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/universal_robot/ur_gazebo/launch/ur10.launch) que carga el *URDF* del paquete *ur_gazebo*:

```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit
catkin_create_pkg two_arm_no_moveit_description rospy
cd two_arm_no_moveit_description
mkdir launch
cp ~/MultiCobot-UR10-Gripper/src/universal_robot/ur_description/launch/ur10_upload.launch launch/
mkdir urdf
cp ~/MultiCobot-UR10-Gripper/src/universal_robot/ur_description/urdf/ur10_robot.urdf.xacro udrf/
cp ~/MultiCobot-UR10-Gripper/src/universal_robot/ur_description/urdf/ur10_joint_limited_robot.urdf.xacro udrf/
```

No se sabe la razón exacta de porque no es posible simular correctamente la pinza, ya que `Gazebo` provee el siguiente error:

```bash
[ERROR] [1640030309.858708751, 1040.204000000]: This robot has a joint named "finger_joint" which is not in the gazebo model.
[FATAL] [1640030309.858805517, 1040.204000000]: Could not initialize robot simulation interface
```

Para la simulación con `Gazebo` no funciona, pero sí en la herramienta de *Rosviz*. Esto indica que el paquete de *ROS* para la pinza funciona correctamente, pero la configuración del robot simulado en `Gazebo` (robot real) no es la adecuada. Seguramente habría que tratarlos como elementos independientes que funcionan físicamente como un conjunto, que no es el enfoque que se está dando para esta simulación en `Gazebo`.

![rviz-robotiq-2f-85-gripper](/doc/imgs_md/robotiq_2f_85_gripper_rviz.png  "rviz-robotiq-2f-85-gripper")

Como en el robot real, la pinza es un elemento independiente del brazo robótico puede que sea necesario este repositorio, pero para realizar las simulaciones se va a utilizar otro repositorio que sí es compatible con el simulador `Gazebo`.


##### :computer: Agregación del robotiq_85_gripper al robot UR10
Se a proceder a agregar la pinza al robot UR10, no hay una referencia clara de cómo hacerlo adecuadamente. Para ello se empezará mirando cómo está estructurado el paquete para tener una idea de cómo adaptarlo al proyecto en cuestión.

```bash
LICENSE             robotiq_85_description  robotiq_85_moveit_config  si_utils
README.md           robotiq_85_driver       robotiq_85_msgs
robotiq_85_bringup  robotiq_85_gripper      robotiq_85_simulation
```

A primera vista los paquetes que nos interesan son el *robotiq_85_description*, *robotiq_85_bringup* y *robotiq_85_simulation*, el resto son recursos para su uso con `MoveIt!` o scripts, que posteriormente se aplicarán para el control de la pinza en simulación.

Teniendo esto en cuenta se va a proceder a la incorporación de la pinza en el robot UR10.

Tomando como ejemplo el fichero [~/MultiCobot-UR10-Gripper/src/robotiq_85_gripper/robotiq_85_description/urdf/robotiq_85_gripper.xacro](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/robotiq_85_gripper/robotiq_85_description/urdf/robotiq_85_gripper.xacro), se procede a añadir la pinza al robot, para ello hay que modificar los ficheros:

- [~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_description/urdf/ur10_robot.urdf.xacro](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_description/urdf/ur10_robot.urdf.xacro)
- [~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_description/urdf/ur10_joint_limited_robot.urdf.xacro](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_description/urdf/ur10_joint_limited_robot.urdf.xacro).

Y con esto, se tiene la pinza en el robot UR10, se puede apreciar en la imagen la pinza.
![ ](/doc/imgs_md/ur10_con_gripper_85.png  "ur10 con gripper")

---

##### :computer: Agregación del controlador de la pinza (Gazebo, fase 1)

Ahora, faltan los controladores para mandarle órdenes a la pinza, eso se puede comprobar obteniendo la lista de *topic*s activos y se comprobará que no hay ningún controlador para la pinza:

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

Se puede apreciar, que están cargados los controladores del robot UR10 (**/arm_controller**) pero no existe ningún *topic* para el control de la pinza. Para ello hay que añadirlos y cargarlos en `Gazebo` correctamente de la siguiente manera. 

Los ficheros que contienen la información para controlar la pinza se encuentran en los directorios [~/MultiCobot-UR10-Gripper/src/robotiq_85_gripper/robotiq_85_simulation/robotiq_85_gazebo/controller](https://github.com/Serru/MultiCobot-UR10-Gripper/tree/main/src/robotiq_85_gripper/robotiq_85_simulation/robotiq_85_gazebo/controller) y  [~/MultiCobot-UR10-Gripper/src/robotiq_85_gripper/robotiq_85_simulation/robotiq_85_gazebo/launch](https://github.com/Serru/MultiCobot-UR10-Gripper/tree/main/src/robotiq_85_gripper/robotiq_85_simulation/robotiq_85_gazebo/launch) que estarían relacionadas respectivamente con los directorios *controller* y *launch* del directorio [two_arm_no_moveit_gazebo](https://github.com/Serru/MultiCobot-UR10-Gripper/tree/main/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo).

Al final, se han realizado las siguientes modificaciones:

```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_gazebo/controller
cp ~/MultiCobot-UR10-Gripper/src/robotiq_85_gripper/robotiq_85_simulation/robotiq_85_gazebo/controller/gripper_controller_robotiq.yaml .
```

Y se ha agregado al final del fichero [~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_gazebo/launch/ur10.launch](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/launch/ur10.launch) el controlador de la pinza:

```xml
<!-- robotiq_85_gripper controller -->
  <rosparam file="$(find two_arm_no_moveit_gazebo)/controller/gripper_controller_robotiq.yaml" command="load"/> 
  <node name="gripper_controller_spawner" pkg="controller_manager" type="spawner" args="gripper" />
```

Se lanza `Gazebo` de nuevo (*roslaunch two_arm_no_moveit_gazebo ur10.launch*) y con el comando *rostopic list*, se aprecia que los controladores de la pinza aparecen correctamente (**/gripper**):

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









<a name="fase3">
  <h2>
Fase 3: Implementación de un planificador propio que realiza un <i>pick & place</i>
  </h2>
</a>

#### :book: Creación del planificador y nodos auxiliares

Se seguirá siempre que se pueda la política de *ROS* en separar las tareas en nodos, para facilitar su reutilización futura.

![image](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/imgs_md/Diseno-planificador-fase-2.png "Esquema del diseño del planificador propio")

En la imagen, está el esquema del diseño de la arquitectura del planificador que se ha implementado. Se puede apreciar dos divisiones `Gazebo` (son los nodos de color rojo y los *topic*s de color naranja) y el planificador (son los nodos de color azul y *topic*s de color verde), se ha dividido ası́ para facilitar la explicación.

El grupo de nodos y *topic*s del grupo de `Gazebo`, no se han tenido que tocar y se ha tomado ventaja de su existencia para obtener la información necesaria que el planificador necesita para realizar su tarea. Este grupo se comunica con el grupo del planificador mediante tres *topic*s `/tf` que contiene información de las transformadas del robot y los *topic*s `/gripper/command` y `/arm_controller/command` que reciben la información necesaria para realizar movimientos en el robot.

Teniendo conocimiento de esos nodos y *topic*s del grupo de `Gazebo`, la solución diseñada para el planificador es comunicarse con ellos para obtener la información que necesita. El nodo `ur10_robot_pose` obtiene la información de las transformadas y le envı́a la posición del *end-effector* al nodo `robot_manipulator`, que realiza dos funciones principales, la primera es el control de la pinza y la segunda es planificar la trayectoria del brazo. Los nodos `cmd_gripper_value_pub` y `cmd_ik_trajectory_pub` obtienen las órdenes del nodo `robot_manipulator` y se lo envı́a a los *topic* de los controladores de `Gazebo` directamente (`/gripper/command` y `/arm_controller/command`).

Una vez explicado el funcionamiento general de la solución se va a explicar lo que hace cada nodo con más detalle:

- [ur10_robot_pose](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/scripts/ur10_robot_pose.py): Este script crea un nodo que únicamente publica la posición del *end-effector* con cierta frecuencia (10 Hz) mediante la librerı́a tf de *ROS* es decir, obtiene la posición del *end-effector* de la información del *topic* `/tf` que es publicada en el *topic* `/robot_pose`. La posición del *end-effector* es obtenida con la diferencia entre las posiciones del `/base_link` y `/ee_link` (es el link que conecta con la pinza). Realmente no es necesario este nodo, el nodo `robot_manipulator` podrı́a gestionarlo de propio, aunque sea redundante, pero muy útil durante la depuración y el entendimiento de cómo funciona el planificador.
- [pub_gripper_cmd](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/scripts/pub_gripper_cmd.py): El script crea un nodo que únicamente recibe y transmite las órdenes recibidas por el *topic* `/pub_gripper_control` al *topic* del controlador de la pinza `/gripper/command`. Aunque la funcionalidad es sencilla, este nodo permite cambios del valor que se le envı́a al controlador en cualquier momento, porque no realiza ninguna comprobación de que se haya realizado el movimiento correctamente, esto es un comportamiento deseado. Y si no recibe ninguna orden sigue publicando la orden anterior manteniendo el valor de la pinza y si recibe una nueva orden, desecha inmediatamente el valor anterior permitiendo realizar cambios durante la ejecución del un movimiento.
- [pub_ik_trajectory](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/scripts/pub_ik_trajectory.py): Funciona de la misma manera que lo descrito para el nodo `cmd_gripper_value_pub`, este script instancia el nodo `cmd_ik_trajectory_pub`. Recibe las órdenes del *topic* `/pub_ik_trajectory` al cual está suscrito y los transmite al *topic* del controlador del brazo `/arm_controller/command`. También permite realizar cambios durante la ejecución de una trayectoria, lo que permite cambios bruscos de dirección sin tener que esperar a que termine de llegar a la posición previamente designada.
- [robot_manipulator](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_manipulator/scripts/robot_manipulator.py): Este script instancia el nodo `robot_manipulator` que obtiene la posición actual del *end-effector* del *topic* `/robot_pose`, esta información es necesaria para poder obtener los valores de cada uno de las articulaciones necesarias para llegar a la posición en cartesiano que se desea. Una vez obtenido el valor que deben tener las articulaciones se publican por el *topic* `/pub_ik_trajectory`. Se ha tenido que implementar un [script que hace la función de librerı́a](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_manipulator/scripts/kinematics_utils.py), esta librerı́a está compuesta de varias funciones para realizar los cálculos necesarios en la obtención de la cinemática directa (forward kinematic) y la cinemática inversa (inverse kinematic). Para el caso de la pinza, como solamente hay que controlar el valor de una articulación, no es necesario ningún cálculo, simplemente se le pasa el valor deseado. Durante la implementación hay que tener en cuenta el tipo de los mensajes que deben recibir los controladores para construirlos adecuadamente o no realizarán ningún movimiento o realizará movimientos no deseados. Esta solución pueda dar problemas debido a la aparición de singularidades, esto es debido a que pueden existir divisiones por cero durante el cálculo de las ecuaciones, principalmente debido cuando dos articulaciones están alineadas, para evitarlo, en este caso, simplemente se ha limitado el entorno de trabajo del robot.

---

##### :computer: robot_pose_publisher script
Se va a implementar un nodo que publique la posición del end effector `ee_link` en todo momento, mediante la librería *tf*.

Como se considera más una herramienta que el script que realmente da las órdenes al robot, este script se implementará en el directorio de `Gazebo`.

```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_gazebo/scripts
touch ur10_robot_pose.py
chmod +x ur10_robot_pose.py
```

- Ver el contenido del fichero [ur10_robot_pose.py](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/scripts/ur10_robot_pose.py).

Este nodo calcula la posición del `ee_link` respecto a la posición del `base_link` mediante la librería *tf* y lo publica en el *topic* `\robot_pose`. Esto permitirá obtener rápidamente la posición actual de la pinza.

- Automatización del lanzamiento del nodo, hay que añadir lo siguiente al fichero [~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_gazebo/launch/controller_utils.launch](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/launch/controller_utils.launch).

```xml
  <!-- get the robot position [Own Script]-->
  <node name="ur10_robot_pose" pkg="two_arm_no_moveit_gazebo" type="ur10_robot_pose.py" respawn="true" />
```

---

##### :computer: pub_gripper_cmd script
Permite el control de la pinza, para ello se creará un nodo que escuche de un *topic* (`/pub_gripper_control`) del que obtendrá el valor que enviará al controlador mediante el *topic* `/gripper/command`.

Este nodo es un nodo de apoyo, por ello estará junto a los scripts de `Gazebo`, cerca de los ficheros de los controladores:

```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_gazebo/scripts
touch pub_gripper_cmd.py
chmod +x pub_gripper_cmd.py
```

- Ver el contenido del fichero [pub_gripper_cmd.py](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/scripts/pub_gripper_cmd.py).

- Automatización del lanzamiento del nodo, hay que añadir lo siguiente al fichero [~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_gazebo/launch/controller_utils.launch](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/launch/controller_utils.launch).

```xml
<!-- send the gripper commands [Own Script]-->
  <node name="cmd_gripper_value_pub" pkg="two_arm_no_moveit_gazebo" type="pub_gripper_cmd.py" respawn="true" />
```

---

##### :computer: pub_ik_trajectory script
Se va a implementar un nodo que reciba comandos por el *topic* `/pub_ik_trajectory` y se los irá mandando repetidamente a los controladores del robot, la posición a la que debe ir se irá modificando al recibir nuevas órdenes.

Como se considera más una herramienta que el script que realmente da las órdenes al robot, este script se implementará en el directorio de `Gazebo`, igual que en el apartado anterior.

```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_gazebo/scripts
touch pub_ik_trajectory.py
chmod +x pub_ik_trajectory.py
```

- Ver el contenido del fichero [pub_ik_trajectory.py](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/scripts/pub_ik_trajectory.py).


- Automatización del lanzamiento del nodo, hay que añadir lo siguiente al fichero [~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_gazebo/launch/controller_utils.launch](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/launch/controller_utils.launch).

```xml
  <!-- send the arms commands [Own Script]-->
  <node name="cmd_ik_trajectory_pub" pkg="two_arm_no_moveit_gazebo" type="pub_ik_trajectory.py" respawn="true" />
```

---

##### :computer: robot_manipulator script
Se va a implementar un nodo que obtendrá la información obtenida del nodo `robot_pose_publisher` y enviará las trayectorias al nodo `pub_ik_trajectory`.

Para obtener los valores de las articulaciones dado la posición cartesiana que al que se quiere ir, es necesario la implementación la función *Inverse Kinematics* y *Forward Kinematics*. Estas funciones fueron obtenidas y adaptadas de [The Construct](https://www.theconstructsim.com/).

Para ello:

```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_manipulator/scripts
touch kinematics_utils.py
chmod +x kinematics_utils.py
```
- Ver el contenido de la librería [kinematics_utils.py](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_manipulator/scripts/kinematics_utils.py).

Una vez que se tiene la librería implementada, se procede a desarrollar el nodo que será el que realmente envíe las órdenes al robot para que realice las tareas deseadas, en este caso el *pick & place*, en donde el cobot cogerá de un cubo de la mesa y lo dejará en una cesta, así hasta que no queden más cubos en la mesa.

```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_manipulator/scripts
touch robot_manipulator.py
chmod +x robot_manipulator.py
```

Contiene una serie de funciones para enviar correctamente las trayectorias finales así como la obtención de los valores de los `joints` en la posición del robot.

- Ver el contenido de la librería [robot_manipulator.py](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_manipulator/scripts/robot_manipulator.py).


El robot da problemas dependiendo de las coordenadas que se pasen, ya que puede surgir singularidades, es cuando el robot se bloquea por limitaciones del dominio matemático (ej.: si `theta2 = acos(1)`, el valor es indefinido generando un error o divisiones por cero).

Para evitar eso, se puede definir un *workspace* en donde no sufra de estas singularidades.


<a name="pruebas">
  <h3>
Ejecución de las pruebas
  </h3>
</a>

#### :computer: Realización de pruebas en `Gazebo`
Se ha creado un test en donde, el robot agarra tres cubos de madera y los envía a un contenedor.

El fichero con el código del manipulador para realizar la prueba: [~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_manipulator/scripts/robot_manipulator.py](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_manipulator/scripts/robot_manipulator.py)

Comandos para lanzar el test:

- Terminal 1:

```bash
cd ~/MultiCobot-UR10-Gripper/
source devel/setup.bash
roslaunch two_arm_no_moveit_gazebo ur10_joint_limited.launch
```
- Terminal 2:

```bash
cd ~/MultiCobot-UR10-Gripper/
source devel/setup.bash
rosrun two_arm_no_moveit_manipulator robot_manipulator.py
```

#### :book: Problemas durante las pruebas de pick and place
Durante el inicio de las pruebas se ha visto que en la simulación en `Gazebo` no es posible agarrar los objetos sobre la mesa.

Esto es debido a que falta incluir el plugin de `Gazebo` *gazebo_grasp* que está en el paquete *gazebo-pkgs* instalado previamente como recurso.

##### :computer: Gazbebo Grasp plugin
Se va a proceder a realizar los pasos necesarios para cargar el plugin que permita al robot interaccionar con los objetos en simulación.

Lo primero es tener el fichero *gzplugin_grasp_fix.urdf.xacro* (se puede obtenerlo del repositorio de [Jennifer Buehler](https://github-wiki-see.page/m/JenniferBuehler/gazebo-pkgs/wiki/The-Gazebo-grasp-fix-plugin)), se podría guardarlo en el paquete de *universal robots*, pero como es una modificación realizada para este proyecto, se ha decidido en moverlo al directorio [~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_description/urdf](https://github.com/Serru/MultiCobot-UR10-Gripper/tree/main/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_description/urdf).

```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_description/urdf
touch gzplugin_grasp_fix.urdf.xacro
```

- Ver el contenido del fichero [gzplugin_grasp_fix.urdf.xacro](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_description/urdf/gzplugin_grasp_fix.urdf.xacro).

Una vez que se tiene el plugin de `Gazebo`, hay que añadirlo al brazo robótico, también se ha añadido a la pinza el código para aumentar su fricción, ayudando al agarre de los objetos en la simulación.

```bash
nano ~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_description/urdf/ur10_robot.urdf.xacro
```

Añadir al final el plugin:

```xml
  <xacro:include filename="$(find two_arm_no_moveit_description)/urdf/gzplugin_grasp_fix.urdf.xacro"/>

  <xacro:gzplugin_grasp_fix prefix=""/>
```

- Ver el contenido del fichero [ur10_robot.urdf.xacro](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_description/urdf/ur10_robot.urdf.xacro).

De la misma manera, editamos el fichero de la pinza ([~/MultiCobot-UR10-Gripper/src/robotiq_85_gripper/robotiq_85_description/urdf/robotiq_85_gripper.urdf.xacro](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/robotiq_85_gripper/robotiq_85_description/urdf/robotiq_85_gripper.urdf.xacro)):

```bash
nano ~/MultiCobot-UR10-Gripper/src/robotiq_85_gripper/robotiq_85_description/urdf/robotiq_85_gripper.urdf.xacro
```

- Ver el contenido del fichero [robotiq_85_gripper.urdf.xacro](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/robotiq_85_gripper/robotiq_85_description/urdf/robotiq_85_gripper.urdf.xacro).


Y esta es toda la configuración necesaria para que el plugin funcione correctamente, en el caso de que hubiese varios robots, hay que asignar correctamente el plugin para cada pinza, si no esas pinzas no podrá agarrar objetos durante la simulación.

<a name="modificaciones">
  <h2>
Modificaciones: Sistema multirobot compuesto de dos robots
  </h2>
</a>

![image](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/imgs_md/Diseno-planificador-fase-2-dos.png "Esquema del diseño del planificador propio")

Antes de comenzar realizado las modificaciones para añadir dos robots al sistema, en la imagen se aprecia cómo estará estructurada los nodos y la comunicación entre `Gazebo`, los controladores y el planificador.

Se aprecia que realmente es una replicación de lo desarrollado previamente en la [Fase 3](#fase3) para un único robot. Por ello hay que notar que se realizarán cambios para los controladores de `Gazebo` y sus scripts que permiten la comunicación entre este y el planificador, cambios en el modelado del robot (URDF) y cambios en el script que realizará la tarea de *pick & place*.

Hay que tener cuidado a la hora de escalar en esta solución, el nombre de las articulaciones deben ser únicos, esto es debido a la configuración del modelo del robot. Al no estar dentro de un *namespace* que generarı́a automáticamente nombres únicos para las articulaciones, se tiene que realizar los cambios manualmente y esto afecta también a los nombres de los *topic*s, lo que implica su duplicación, también hay que modificar los scripts para que publiquen y se suscriban a los *topic*s correctos.

Esto no implica que el sistema en sı́ sea poco escalable, porque es sencillo hacer que los scripts reciban un argumento que tomen como prefijo lo que generarı́a nombres únicos de manera automática desde los ficheros *launch*, como lo harı́a un *namespace*.

<a name="modificaciones1">
  <h3>
Fase 1: Configuración del simulador de Gazebo
  </h3>
</a>

#### :book: Configuración de `Gazebo`
Sobre esta configuración, hay que redefinir los ficheros que configuran los controladores de `Gazebo`, básicamente es duplicarlos y añadirles el prefijo `ur10_1` y `ur10_2` al nombre de las articulaciones, tanto para la pinza como para el brazo de UR10.

De manera similar hay que añadir los nodos duplicados y adaptados para su comunicación con los controladores de los cobots en `Gazebo` y de los nodos que obtienen las posiciones actuales de cada cobot del *topic* `/tf` deben ser también añadidos al fichero `controller_utils.launch`.

##### :computer: Modificaciones a realizar para simular dos robots en `Gazebo`
Primero, hay que decidir en el *namespace* para cada robot, es decir el nombre de grupo sobre el que agruparemos las configuraciones para cada uno de los robots.

Se comenzará con los controladores:

- Fichero *ur10_1_arm_controller.yaml*

```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_gazebo/controllers
mv arm_controller_ur10.yaml ur10_1_arm_controller.yaml
```

Ver el contenido del fichero [ur10_1_arm_controller.yaml](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_gazebo/controller/ur10_1_arm_controller.yaml) con las modificaciones realizadas.

Se puede apreciar, que simplemente se ha añadido el prefijo *ur10_1_*, esto permitirá diferencia después a que `joints` debe enviar los comandos, lo cual implica la modificación del fichero *URDF* también.

Por ahora se procede a modificar el resto de ficheros así como la adicción del segundo grupo de controladores para el segundo robot que se llevará el prefijo *ur10_2_*.

- Fichero *ur10_1_gripper_controller_robotiq.yaml*

```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_gazebo/controllers
mv gripper_controller_robotiq.yaml ur10_1_gripper_controller_robotiq.yaml
```

Ver el contenido del fichero [ur10_1_gripper_controller_robotiq.yaml](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_gazebo/controller/ur10_1_gripper_controller_robotiq.yaml) con las modificaciones realizadas.

- Fichero *ur10_2_arm_controller.yaml*

```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_gazebo/controllers
cp ur10_1_arm_controller.yaml ur10_2_arm_controller.yaml
```

Ver el contenido del fichero [ur10_2_arm_controller.yaml](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_gazebo/controller/ur10_2_arm_controller.yaml) con las modificaciones realizadas.

- Fichero *ur10_2_gripper_controller_robotiq.yaml*

```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_gazebo/controllers
mv ur10_1_gripper_controller_robotiq.yaml ur10_2_gripper_controller_robotiq.yaml
```

Ver el contenido del fichero [ur10_2_gripper_controller_robotiq.yaml](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_gazebo/controller/ur10_2_gripper_controller_robotiq.yaml) con las modificaciones realizadas.

##### :computer: Modificación del fichero *launch* de `Gazebo`
Hay que modificar ahora el fichero *launch* para lanzar los controladores de ambos robots en `Gazebo`.

```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_gazebo/launch
nano ur10.launch
```

Ver el contenido del fichero [ur10.launch](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_gazebo/launch/ur10.launch) con las modificaciones realizadas.

```bash
cd ~/MultiCobot-UR10-Gripper/<src/multirobot/two_arm_no_moveit/two_arm_no_moveit_gazebo/launch
nano ur10_joint_limited.launch
```

Ver el contenido del fichero [ur10_joint_limited.launch](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_gazebo/launch/ur10_joint_limited.launch) con las modificaciones realizadas.

Falta modificar el fichero que lanza los scripts que se crearon, previamente así como esos ficheros, ya que se han corregido para adaptarse a los *namespace*s correctamente:

- [~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_gazebo/launch/controller_utils.launch](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_gazebo/launch/controller_utils.launch)

- [~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_gazebo/scripts/ur10_1_pub_gripper_cmd.py](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_gazebo/scripts/ur10_1_pub_gripper_cmd.py)

- [~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_gazebo/scripts/ur10_1_pub_ik_trajectory.py](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_gazebo/scripts/ur10_1_pub_ik_trajectory.py)

- [~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_gazebo/scripts/ur10_1_robot_pose.py](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_gazebo/scripts/ur10_1_robot_pose.py)

- [~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_gazebo/scripts/ur10_2_pub_gripper_cmd.py](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_gazebo/scripts/ur10_2_pub_gripper_cmd.py)

- [~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_gazebo/scripts/ur10_2_pub_ik_trajectory.py](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_gazebo/scripts/ur10_2_pub_ik_trajectory.py)

- [~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_gazebo/scripts/ur10_2_robot_pose.py](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_gazebo/scripts/ur10_2_robot_pose.py)


<a name="modificaciones2">
  <h3>
Fase 2: Configuración del URDF
  </h3>
</a>

#### :book: Descripción del fichero *URDF*
 ![image](/doc/imgs_md/solo-dos-urdf-robot.png  "Representación del fichero URDF para dos robots")

Es parecido a lo realizado en la [Fase 1](#fase1) de la *Configuración inicial*, pero con la diferencia de que esta vez son dos cobots UR10 y no uno. En el fichero [URDF](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_description/urdf/ur10_robot.urdf.xacro), se puede ver el cómo se conectan los componentes de los brazos `ur10_robot` con el link `world`, esto se puede apreciar en la imagen, representando `world` (color amarillo) y las bases de los brazos de los UR10s `ur10_1_base_link` y `ur10_2_base_link` (color verde) situado justo encima, además los `joints` `ur10_1_world_joint` y `ur10_2_world_joint` son las esferas de color amarillo situado entre ambos links. De la misma manera, los componentes de las pinzas `robotiq_85_gripper` están conectados a los brazos de los UR10s (`ur10_robot`) que se aprecia en la imagen en donde las esferas que representan los `joints` `ur10_1_robotiq_85_base_joint` y `ur10_2_robotiq_85_base_joint` que unen ambos componentes (son de color morado), uniendo los links `ur10_1_robotiq_85_base_link` y `ur10_2_robotiq_85_base_link` de las pinzas con los links `ur10_1_ee link` y `ur10_2_ee_link` de los brazos de los UR10s.

##### :computer: Configuración del directorio descripción

Hay que modificar los ficheros *.urdf*, *ur10_joint_limited_robot.urdf.xacro* y *ur10_robot.urdf.xacro*:
 
 - Modificando el fichero [~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_description/urdf/ur10_joint_limited_robot.urdf.xacro](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_description/urdf/ur10_joint_limited_robot.urdf.xacro) con:
 
```xml
 [...]

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
```
 
 
 - Modificando el fichero [~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_description/urdf/ur10_robot.urdf.xacro](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_description/urdf/ur10_robot.urdf.xacro) con:

```xml
[...]
  <!-- arm -->
  <xacro:ur10_robot prefix="ur10_1_" joint_limited="false"
    transmission_hw_interface="$(arg transmission_hw_interface)"
  />

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
```

<a name="modificaciones3">
  <h3>
Fase 3: Implementación de un planificador propio que realiza un <i>pick & place</i>
  </h3>
</a>

#### :book: Creación del planificador y nodos auxiliares

El planificador ya está desarrollado previamente en la [Fase 1](#modificaciones1) de esta sección de modificaciones para un sistema multirobot de dos robots, en donde se ha implementado los nodos que forman parte del planificador.

En esta sección lo importante es implementar los scripts que utilizarán la [librería](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_manipulator/scripts/kinematics_utils.py) desarrollada para realizar la cinemática directa para controlar los brazos robóticos.

El cual realizará la tarea de *pick & place*, se implementarán dos scripts que se lanzarán en distintos terminales para que manden órdenes simultáneamente a cada robot.

##### :computer: Modificación del pick and place

En el directorio *two_arm_no_moveit_manipulator* se modificará los siguientes ficheros:

- [~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_manipulator/ur10_1_robot_manipulator.py](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_manipulator/scripts/ur10_1_robot_manipulator.py).

- [~/MultiCobot-UR10-Gripper/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_manipulator/ur10_2_robot_manipulator.py](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_manipulator/scripts/ur10_2_robot_manipulator.py).

Falta arreglar el plugin de `Gazebo` para que pueda agarrar objetos con ambas pinzas:

- Fichero [~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_description/urdf/gzplugin_grasp_fix.urdf.xacro](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_description/urdf/gzplugin_grasp_fix.urdf.xacro).

Y finalmente, adecuar el fichero `world`para que realice las simulaciones en un entorno adecuado:

- Fichero [~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_gazebo/world/multiarm_bot.world](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/two_arm_no_moveit/two_arm_no_moveit_gazebo/world/multiarm_bot.world).

<a name="pruebas2">
  <h3>
Ejecución de las pruebas
  </h3>
</a>

#### :computer: Lanzar las pruebas de simulación

Es necesario 3 terminales:

- Terminal 1:
```bash
 roslaunch two_arm_no_moveit_gazebo ur10_joint_limited.launch
``` 

- Terminal 2:
```bash
 rosrun two_arm_no_moveit_manipulator ur10_1_robot_manipulator.py
```

- Terminal 3:
```bash
 rosrun two_arm_no_moveit_manipulator ur10_2_robot_manipulator.py
```

#### :book: Información del resultado final

- Resultado visual en `Gazebo`
![image](/doc/imgs_md/two-arm-no-moveit-gazebo.png  "Resultado en Gazebo")

- Esquema de los nodos y *topic*s del sistema
![image](/doc/imgs_md/two-arm-no-moveit-graph.png  "Nodos y topics del sistema")

- Árbol de las transformadas del modelo del robot
![image](/doc/imgs_md/two-arm-no-moveit-tree.png  "Árbol de transformadas")

---

<div>
  <p align="left">
    <button name="button">
                <a rel="license" href="https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/no-moveit-intro.md">Anterior</a>
    </button>
  </p>
</div>