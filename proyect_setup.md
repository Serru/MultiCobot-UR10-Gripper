# Replicación del proyecto de TFG: Sistema mutirobot para el transporte colaborativo de objetos
Proceso de instalación y configuración para la reproducción del proyecto de TFG, titulado "*Sistema multirobot para el transporte colaborativo de objetos*". 

*No se garantiza que funcione una vez realizada la reproducción del proyecto, y se puede necesitar realizar cambios adecuados que el proyecto pueda requerir debido a las actualizaciones que puedan tener.
*

Se proveerá de un link al final, que redireccionará al github que contiene el proyecto funcionando.

###Contribuidores
---
* Autor: [Burgh Oliván, Miguel](mailto:647531@unizar.es)
* Director: [López Nicolás, Gonzalo](mailto:gonlopez@unizar.es)

## Instalación y configuración base del proyecto
Se instala las dependencias y los repositorios comunes a todas las soluciones que tiene el proyecto. 
### Requisitos del sistema
* Ubuntu 16.04
* Python 2.7
* ROS Kinetic Kame
### Instalación de ROS y dependecias
```{bash}
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt-get update

sudo apt-get install ros-kinetic-desktop-full


sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential

sudo apt install python-rosdep
sudo rosdep init
rosdep update

echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
source /opt/ros/kinetic/setup.bash
```

Instalación de dependencias:
```{bash}
sudo apt-get install ros-kinetic-$(dependencia_name)
```
Si la depencia es *moveit_ros_planning*, entonces *dependencia_name* será *moveit-ros-planning*, se sigue este patrón para instalar todas las dependencias.

Dependencias:
- moveit_ros_planning 
- moveit_kinematics
- moveit_simple_controller_manager
- position_controllers
- moveit_fake_controller_manager
- gazebo_ros_control
- joint_trajectory_controller
- moveit_ros_visualization
- moveit_planners_ompl
- joint_state_controller
- effort_controllers
- ros_controllers
- industrial_msgs

### Creación del directorio de trabajo
```{bash}
mkdir -p ~/ tfg_multirobot/src
cd  ~/tfg_multirobot/src
```

### Instalación de repositorios
Instalación de repositorios de terceros que se van a utilizar total o parcialmente. 
#### Universal Robots
Este repositorio de Universal Robots se utilizará como base para realizar las modificaciones necesarias que el proyecto requiera. Los ficheros modificados irán debidamente detallados.

Clonar el repositorio:
```{bash}
cd ~/tfg_multirobot/src
git clone -b kinetic-devel https://github.com/ros-industrial/universal_robot.git
```

Instalación de dependencias que puedan faltar para ROS Kinetic Kame:
```{bash}
cd ~/tfg_multirobot
rosdep update
rosdep install --rosdistro kinetic --ignore-src --from-paths src
```

Compilación:
```{bash}
cd ~/tfg_multirobot
catkin_make
```

#### Robotiq_2finger_grippers
Este repositorio de Robotiq se utilizará para añadir el gripper que se utilizará al robot y los controladores necesarios para su funcionamiento.

Clonar el repositorio:
```{bash}
cd ~/tfg_multirobot/src
git clone https://github.com/Danfoa/robotiq_2finger_grippers.git
```

Instalación de dependencias que puedan faltar para ROS Kinetic Kame:
```{bash}
cd ~/tfg_multirobot
rosdep update
rosdep install --rosdistro kinetic --ignore-src --from-paths src
```

Compilación:
```{bash}
cd ~/tfg_multirobot
catkin_make
```

#### Robotiq_85_gripper
Este repositorio de Robotiq se utilizará para añadir el gripper que se utilizará al robot y los controladores necesarios para su funcionamiento.

Clonar el repositorio:
```{bash}
cd ~/tfg_multirobot/src
git clone -b develop https://github.com/PickNikRobotics/robotiq_85_gripper.git
```

Instalación de dependencias que puedan faltar para ROS Kinetic Kame:
```{bash}
cd ~/tfg_multirobot
rosdep update
rosdep install --rosdistro kinetic --ignore-src --from-paths src
```
#### ros_control
Este repositorio contiene: "*A set of packages that include controller interfaces, controller managers, transmissions and hardware_interfaces*".

Clonar el repositorio:
```{bash}
cd ~/tfg_multirobot/src
git clone -b kinetic-devel https://github.com/ros-controls/ros_control.git
```

Instalación de dependencias que puedan faltar para ROS Kinetic Kame:
```{bash}
cd ~/tfg_multirobot
rosdep update
rosdep install --rosdistro kinetic --ignore-src --from-paths src
```

Compilación:
```{bash}
cd ~/tfg_multirobot
catkin_make
```


#### ur_modern_driver
Este repositorio  esta obsoleto, pero por compatibilidad con ROS Kinetic Kame hay que utilizarlo. Contiene drivers para los robots de Universal Robots (UR3/UR5/UR10) y es compatible con ros_control.

Clonar el repositorio:
```{bash}
cd ~/tfg_multirobot/src
git clone -b kinetic-devel https://github.com/ros-industrial/ur_modern_driver.git
```

Instalación de dependencias que puedan faltar para ROS Kinetic Kame:
```{bash}
cd ~/tfg_multirobot
rosdep update
rosdep install --rosdistro kinetic --ignore-src --from-paths src
```

Compilación:
```{bash}
cd ~/tfg_multirobot
catkin_make
```

Puede dar error de compilación para ROS Kinetic Kame, para solucionarlo hay que modificar el fichero *ur_hardware_interface.cpp*, para realizarlo rápido, sustituir el contenido del fichero por el contenido que hay en [aquí](https://github.com/iron-ox/ur_modern_driver/blob/883070d0b6c0c32b78bb1ca7155b8f3a1ead416c/src/ur_hardware_interface.cpp) y volverlo a compilar.

#### gazebo-pkgs
Es una colleción de plugings para Gazebo, principalmente nos interesa el pluging que permite agarrar objetos.


Dependencias del paquete:
- **gazebo_ros**
- **eigen_conversions**
- **object_recognition_msgs**
- **roslint**
- **general-message-pkgs**

Instalación de dependencias del paquete:
Ros usa **Gazebo 7.x**
```{bash}
sudo apt-get install -y libgazebo7-dev
```
Clonar el repositorio de **gazebo_ros_pkgs**
```{bash}
cd ~/tfg_multirobot/src
git clone -b kinetic-devel https://github.com/ros-simulation/gazebo_ros_pkgs.git
```

Clonar el repositorio de **eigen_conversions**
```{bash}
cd ~/tfg_multirobot/src
git clone -b indigo-devel https://github.com/ros/geometry.git
```

Clonar el repositorio de **object_recognition_msgs**
```{bash}
cd ~/tfg_multirobot/src
git clone -b master https://github.com/wg-perception/object_recognition_msgs.git
```

Clonar el repositorio de **roslint**
```{bash}
cd ~/tfg_multirobot/src
git clone -b master https://github.com/ros/roslint.git
```

Clonar el repositorio de **general-message-pkgs**
```{bash}
cd ~/tfg_multirobot/src
git clone -b master https://github.com/JenniferBuehler/general-message-pkgs.git
```


Instalación de dependencias que puedan faltar para ROS Kinetic Kame:
```{bash}
cd ~/tfg_multirobot
rosdep update
rosdep check --from-paths . --ignore-src --rosdistro kinetic
rosdep install --from-paths . --ignore-src --rosdistro kinetic -y
```

Compilación:
```{bash}
cd ~/tfg_multirobot
catkin_make
```

---
Clonar el repositorio **gazebo-pkgs**:
```{bash}
cd ~/tfg_multirobot/src
git clone https://github.com/JenniferBuehler/gazebo-pkgs.git
```

Instalación de dependencias que puedan faltar para ROS Kinetic Kame:
```{bash}
cd ~/tfg_multirobot
rosdep update
rosdep install --rosdistro kinetic --ignore-src --from-paths src
```

Compilación:
```{bash}
cd ~/tfg_multirobot
catkin_make
```

---
Durante la compilación puede aparecer un **error** que pide que este compilado con el estandar de c++11:
```{bash}
/usr/include/c++/5/bits/c++0x_warning.h:32:2: error: #error This file requires compiler and library support for the ISO C++ 2011 standard. This support must be enabled with the -std=c++11 or -std=gnu++11 compiler options.
```

Para solventarlo, hay que modificar el make de catkin al compilar el proyecto:
- Simplemente hay que modificar el fichero ~/tfg_multirobot/src/CMakeLists.txt y añadir *add_compile_options(-std=c++11)* al principio del fichero, como se muestra a continuación.
```{bash}
# toplevel CMakeLists.txt for a catkin workspace
# catkin/cmake/toplevel.cmake

cmake_minimum_required(VERSION 3.0.2)

project(Project)

set(CATKIN_TOPLEVEL TRUE)
add_compile_options(-std=c++11)
[...]
```

#### Leap Motion
Driver de ROS para el controlador de Leap Motion

Para la correcta instación y configuración del controlador de Leap Motion en ROS Kinetic Kame, hay que realizar un poco más de trabajo que en los repositorios previos.

Lo primero es reemplazar o crear el fichero que dar servicio al controlador: ***/lib/systemd/system/leapd.service***
```{bash}
# Found by Kevin Cole 2014.11.22 at
# https://github.com/atejeda/leap-fedora-rpm
#
# Remember to:
#
#   ln -s /lib/systemd/system/leapd.service /etc/systemd/system/leapd.service
#   systemctl daemon-reload
#

[Unit]
Description=LeapMotion Daemon
After=syslog.target

[Service]
Type=simple
ExecStart=/usr/sbin/leapd

[Install]
WantedBy=multi-user.target
```
```{bash}
sudo ln -s /lib/systemd/system/leapd.service /etc/systemd/system/leapd.service
sudo systemctl daemon-reload
```


Clonar el repositorio:
```{bash}
cd ~/tfg_multirobot/src
git clone https://github.com/ros-drivers/leap_motion.git
```

En los pasos de instalación dice de mover el directorio LeapSDK al directorio $HOME, pero se va a mantener en el repositorio original y se modificarán los PATHS adecuadamente.

```{bash}
# 64-bit operating system
echo "export PYTHONPATH=$PYTHONPATH:$HOME/tfg_multirobot/src/leap_motion/LeapSDK/lib:$HOME/tfg_multirobot/src/leap_motion/LeapSDK/lib/x64" >> ~/.bashrc
source ~/.bashrc
```

Instalación del paquete de ros de Leap Motion:

```{bash}
sudo apt-get install ros-kinetic-leap-motion
```

Instalación de dependencias que puedan faltar para ROS Kinetic Kame:
```{bash}
cd ~/tfg_multirobot
rosdep update
rosdep install --rosdistro kinetic --ignore-src --from-paths src
```

Compilación:
```{bash}
cd ~/tfg_multirobot
catkin_make
```

Si surgen errores durante el uso de los drivers de Leap Motion, con reiniciar el servicio suele ser suficiente:
```{bash}
sudo service leapd restart
```

### Warnings durante la compilación
Warnings ignorados:
```{bash}
WARNING: Package 'ur_modern_driver' is deprecated (This package has been deprecated. Users of CB3 and e-Series controllers should migrate to ur_robot_driver.)

CMake Warning at /opt/ros/kinetic/share/catkin/cmake/catkin_package.cmake:418 (message):
  catkin_package() include dir
  '/home/miguel/tfg_multirobot/build/gazebo-pkgs/gazebo_grasp_plugin/..'
  should be placed in the devel space instead of the build space
Call Stack (most recent call first):
  /opt/ros/kinetic/share/catkin/cmake/catkin_package.cmake:102 (_catkin_package)
  gazebo-pkgs/gazebo_grasp_plugin/CMakeLists.txt:31 (catkin_package)
  
/home/miguel/tfg_multirobot/src/ros_control/hardware_interface/include/hardware_interface/internal/interface_manager.h:69:85: warning: type qualifiers ignored on function return type [-Wignored-qualifiers]
   static const void callConcatManagers(typename std::vector<T*>& managers, T* result)
```
Warnings resueltos:

**gazebo_version_helpers warning:**
```{bash}
CMake Warning at /opt/ros/kinetic/share/catkin/cmake/catkin_package.cmake:166 (message):
  catkin_package() DEPENDS on 'gazebo' but neither 'gazebo_INCLUDE_DIRS' nor
  'gazebo_LIBRARIES' is defined.
Call Stack (most recent call first):
  /opt/ros/kinetic/share/catkin/cmake/catkin_package.cmake:102 (_catkin_package)
  gazebo-pkgs/gazebo_version_helpers/CMakeLists.txt:26 (catkin_package)
```
Modificar el fichero ~/tfg_multirobot/src/gazebo-pkgs/gazebo_version_helpers/CMakeLists.txt, a partir de la línea 26 por lo siguiente:

```{bash}
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gazebo_version_helpers
  CATKIN_DEPENDS gazebo_ros roscpp
  DEPENDS GAZEBO 
)
```

**gazebo_grasp_plugin warning:**
	
```{bash}
CMake Warning at /opt/ros/kinetic/share/catkin/cmake/catkin_package.cmake:166 (message):
  catkin_package() DEPENDS on 'gazebo' but neither 'gazebo_INCLUDE_DIRS' nor
  'gazebo_LIBRARIES' is defined.
Call Stack (most recent call first):
  /opt/ros/kinetic/share/catkin/cmake/catkin_package.cmake:102 (_catkin_package)
  gazebo-pkgs/gazebo_grasp_plugin/CMakeLists.txt:31 (catkin_package)
[...]
```
Modificar el fichero ~/tfg_multirobot/src/gazebo-pkgs/gazebo_grasp_plugin/CMakeLists.txt, a partir de la línea 31 por lo siguiente:
```{bash}
catkin_package(
  # Binary directory required for proto headers inclusion to work, because install commands don't
  # get executed in devel space. The directory above is required so that an include of
  # <gazebo_grasp_plugin/msgs/grasp_event.pb.h> 
  # also works in devel space like it needs to be in install space.
  # Probably we can find a better solution for this, but until then this
  # fix will be OK.
  INCLUDE_DIRS include ${CMAKE_CURRENT_BINARY_DIR}/..
  LIBRARIES gazebo_grasp_fix gazebo_grasp_msgs
  CATKIN_DEPENDS gazebo_ros geometry_msgs roscpp std_msgs gazebo_version_helpers
  DEPENDS GAZEBO
)
```

**gazebo_grasp_plugin_ros warning:**	
```{bash}
CMake Warning at /opt/ros/kinetic/share/catkin/cmake/catkin_package.cmake:166 (message):
  catkin_package() DEPENDS on 'gazebo' but neither 'gazebo_INCLUDE_DIRS' nor
  'gazebo_LIBRARIES' is defined.
Call Stack (most recent call first):
  /opt/ros/kinetic/share/catkin/cmake/catkin_package.cmake:102 (_catkin_package)
  gazebo-pkgs/gazebo_grasp_plugin_ros/CMakeLists.txt:34 (catkin_package)
[...]
```
Modificar el fichero ~/tfg_multirobot/src/gazebo-pkgs/gazebo_grasp_plugin_ros/CMakeLists.txt, a partir de la línea 34 por lo siguiente:
```{bash}
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gazebo_grasp_plugin_ros
 CATKIN_DEPENDS gazebo_grasp_plugin message_runtime roscpp
 DEPENDS GAZEBO
)
```


### Activación del entorno de trabajo actual
```{bash}
cd ~/tfg_multirobot
source ~/tfg_multirobot/devel/setup.bash
```

### Testeo de la configuración base
Tras realizar previamente toda la instalación y configuración del sistema se puede proceder a realizar pruebas para comprobar su funcionamiento antes de proceder a realizar otras modificaciones. No se va a indicar que pruebas se puede realizar, pero los repositorios de origen tiene indicaciones para ejecutar pequeñas demostracciones que son muy útiles para comprender lo que pueden realizar.

El entorno de trabajo debería quedarde la siguiente manera tras la instalación de todos los repositorios:
```{bash}
miguel@Omen:~/tfg_multirobot/src$ ls
CMakeLists.txt        geometry                  ros_control
gazebo-pkgs           leap_motion               roslint
gazebo_ros_pkgs       object_recognition_msgs   universal_robot
general-message-pkgs  robotiq_2finger_grippers  ur_modern_driver
```
Estos repositorios, serán la base para la implementación del proyecto, es decir, los recursos del sistema.










## Preparación para la implementación del proyecto
Una vez que se tiene todos los recursos necesarios para el sistema instalados correctamente, se procede a crear el paquete que contendrá las diferentes soluciones propuestas con sus ventajas y desventajas.

Se ha decidido organizarlo de esta manera para que los recursos estén compartidos entre las diferentes implementaciones, y las modificaciones que se tengan que realizar se guardarán en sus respectivos directorios que al ser lanzados ejecutarán estos ficheros modificados en vez de los ficheros originales de los que parten.

Por ello se procede primero a crear el directorio que contendra todas las soluciones propuestas:
```{bash}
cd ~/tfg_multirobot/src
mkdir tfg_project
```
Este directorio, será el directorio raíz de las implementaciones

Con esto, la preparación para la reproducción de las diferentes soluciones está terminada.

Añadir, que las modificaciones que sean comunes a todas las soluciones se modificarán sobre los recursos compartidos.








## Pruebas realizadas
Para IK sin moveit:

Lanzar Gazebo 7 con el robot UR10:
```{bash}
miguel@Omen:~/tfg_project/one_arm/src/ur10_no_moveit_config/src$ roslaunch ur_gazebo ur10.launch
```

Lanzar el publicador de posiciones:
```{bash}
miguel@Omen:~/tfg_project/one_arm/src/ur10_no_moveit_config/src$ rosrun ur10_nmoveit_config pub_ik_trajectory.py
```


Lanzar el Leap Motion:
```{bash}
miguel@Omen:~/tfg_project/one_arm/src/ur10_no_moveit_config/src$ rosrun leap_motion sender.py
```
Si Leap motion tiene problemas, reiniciar el servicio:
```{bash}
miguel@Omen:~/tfg_project/one_arm/src/ur10_no_moveit_config/src$ sudo service leapd restart
```
Lanzar el IK sin tener en cuenta Leap Motion
```{bash}
miguel@Omen:~/tfg_project/one_arm/src/ur10_no_moveit_config/src$ rosrun leap_motion compute_fk_ik.py
```
Lanzar el IK teniendo en cuenta Leap Motion
```{bash}
miguel@Omen:~/tfg_project/one_arm/src/ur10_no_moveit_config/src$ rosrun ur10_no_moveit_config lm_ik_compute.py
```

<!--- Para un robot  opción B--->
## Instalación y configuración para un único robot UR10 con MoveIt!
## Instalación y configuración para un único robot UR10 con MoveIt! y Leap Motion


<!--- Para dos robots opción B--->
## Instalación y configuración para dos robot UR10 replicando MoveIt!
## Instalación y configuración para dos robot UR10 replicando MoveIt! y Leap Motion

<!--- Para cuatro robots opción B--->
## Instalación y configuración para cuatro robot UR10 replicando MoveIt!


<!--- Anexos--->
## Anexos
### one_arm_no_moveit
* one_arm_no_moveit_gazebo/launch/ur10.launch
```{xml}
<?xml version="1.0"?>
<launch>
  <arg name="limited" default="false"  doc="If true, limits joint range [-PI, PI] on all joints." />
  <arg name="paused" default="false" doc="Starts gazebo in paused mode" />
  <arg name="gui" default="true" doc="Starts gazebo gui" />
  
  <arg name="world" default="$(find one_arm_no_moveit_gazebo)/world/multiarm_bot.world" />

  <!-- startup simulated world -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" default="$(arg world)"/>
    <arg name="paused" value="$(arg paused)"/>
    <arg name="gui" value="$(arg gui)"/>
  </include>

  <!-- send robot urdf to param server -->
  <include file="$(find ur_description)/launch/ur10_upload.launch">
    <arg name="limited" value="$(arg limited)"/>
  </include>

  <!-- push robot_description to factory and spawn robot in gazebo -->
  <node name="spawn_gazebo_model" pkg="gazebo_ros" type="spawn_model" args="-urdf -param robot_description -model robot -x 0.3 -z 1.2" respawn="false" output="screen" />

  <include file="$(find ur_gazebo)/launch/controller_utils.launch"/>

  <!-- start this controller -->
  <rosparam file="$(find ur_gazebo)/controller/arm_controller_ur10.yaml" command="load"/>
  <node name="arm_controller_spawner" pkg="controller_manager" type="controller_manager" args="spawn arm_controller" respawn="false" output="screen"/>

  <!-- load other controllers -->
  <node name="ros_control_controller_manager" pkg="controller_manager" type="controller_manager" respawn="false" output="screen" args="load joint_group_position_controller" />

</launch>

```

* one_arm_no_moveit_gazebo/world/multiarm_bot.world
```{xml}
<sdf version='1.6'>
  <world name='default'>
    <scene>
      <shadows>0</shadows>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
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
    <model name='coke_can'>
      <link name='link'>
        <inertial>
          <pose frame=''>0 0 0.06 0 -0 0</pose>
          <mass>0.39</mass>
          <inertia>
            <ixx>0.00055575</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.00055575</iyy>
            <iyz>0</iyz>
            <izz>0.0001755</izz>
          </inertia>
        </inertial>
        <collision name='collision'>
          <pose frame=''>0.003937 0.004724 -0.18 0 -0 0</pose>
          <geometry>
            <mesh>
              <uri>model://coke_can/meshes/coke_can.dae</uri>
            </mesh>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1</mu>
                <mu2>1</mu2>
              </ode>
              <torsional>
                <ode/>
              </torsional>
            </friction>
            <contact>
              <ode>
                <kp>1e+07</kp>
                <kd>1</kd>
                <min_depth>0.001</min_depth>
                <max_vel>0.1</max_vel>
              </ode>
            </contact>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='visual'>
          <pose frame=''>0.003937 0.004724 -0.18 0 -0 0</pose>
          <geometry>
            <mesh>
              <uri>model://coke_can/meshes/coke_can.dae</uri>
            </mesh>
          </geometry>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <pose frame=''>1.11651 0.230436 0 0 -0 0</pose>
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
      <pose frame=''>1.38 0 0 0 -0 1.56</pose>
    </model>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type='adiabatic'/>
    <physics name='default_physics' default='0' type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <wind/>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>0</latitude_deg>
      <longitude_deg>0</longitude_deg>
      <elevation>0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>
    <model name='coke_can_0'>
      <link name='link'>
        <inertial>
          <pose frame=''>0 0 0.06 0 -0 0</pose>
          <mass>0.39</mass>
          <inertia>
            <ixx>0.00055575</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.00055575</iyy>
            <iyz>0</iyz>
            <izz>0.0001755</izz>
          </inertia>
        </inertial>
        <collision name='collision'>
          <pose frame=''>0.003937 0.004724 -0.18 0 -0 0</pose>
          <geometry>
            <mesh>
              <uri>model://coke_can/meshes/coke_can.dae</uri>
            </mesh>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1</mu>
                <mu2>1</mu2>
              </ode>
              <torsional>
                <ode/>
              </torsional>
            </friction>
            <contact>
              <ode>
                <kp>1e+07</kp>
                <kd>1</kd>
                <min_depth>0.001</min_depth>
                <max_vel>0.1</max_vel>
              </ode>
            </contact>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='visual'>
          <pose frame=''>0.003937 0.004724 -0.18 0 -0 0</pose>
          <geometry>
            <mesh>
              <uri>model://coke_can/meshes/coke_can.dae</uri>
            </mesh>
          </geometry>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <pose frame=''>1.71 -0.434907 0 0 -0 0</pose>
    </model>
    <model name='coke_can_1'>
      <link name='link'>
        <inertial>
          <pose frame=''>0 0 0.06 0 -0 0</pose>
          <mass>0.39</mass>
          <inertia>
            <ixx>0.00055575</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.00055575</iyy>
            <iyz>0</iyz>
            <izz>0.0001755</izz>
          </inertia>
        </inertial>
        <collision name='collision'>
          <pose frame=''>0.003937 0.004724 -0.18 0 -0 0</pose>
          <geometry>
            <mesh>
              <uri>model://coke_can/meshes/coke_can.dae</uri>
            </mesh>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1</mu>
                <mu2>1</mu2>
              </ode>
              <torsional>
                <ode/>
              </torsional>
            </friction>
            <contact>
              <ode>
                <kp>1e+07</kp>
                <kd>1</kd>
                <min_depth>0.001</min_depth>
                <max_vel>0.1</max_vel>
              </ode>
            </contact>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='visual'>
          <pose frame=''>0.003937 0.004724 -0.18 0 -0 0</pose>
          <geometry>
            <mesh>
              <uri>model://coke_can/meshes/coke_can.dae</uri>
            </mesh>
          </geometry>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <pose frame=''>1.19381 -0.392486 0 0 -0 0</pose>
    </model>
    <state world_name='default'>
      <sim_time>1039 984000000</sim_time>
      <real_time>65 22266385</real_time>
      <wall_time>1639754217 935061519</wall_time>
      <iterations>64277</iterations>
      <model name='coke_can'>
        <pose frame=''>1.37994 0.299909 1.01174 0.005809 -0.039142 -2.19479</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose frame=''>1.37994 0.299909 1.01174 0.005809 -0.039142 -2.19479</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0.017749 -0.011267 -0.004855 0.201232 0.312002 0.016283</acceleration>
          <wrench>0.006922 -0.004394 -0.001893 0 -0 0</wrench>
        </link>
      </model>
      <model name='coke_can_0'>
        <pose frame=''>1.60008 -0.299931 1.012 -0.002474 -0.037348 0.933707</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose frame=''>1.60008 -0.299931 1.012 -0.002474 -0.037348 0.933707</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>-5e-06 -3.3e-05 0 0.000582 -9.1e-05 -2.2e-05</acceleration>
          <wrench>-2e-06 -1.3e-05 0 0 -0 0</wrench>
        </link>
      </model>
      <model name='coke_can_1'>
        <pose frame=''>1.0999 -0.300087 1.01182 -0.005953 -0.046932 -2.19471</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose frame=''>1.0999 -0.300087 1.01182 -0.005953 -0.046932 -2.19471</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>-0.002171 0.000944 -0.000204 -0.016737 -0.038098 -0.002995</acceleration>
          <wrench>-0.000847 0.000368 -8e-05 0 -0 0</wrench>
        </link>
      </model>
      <model name='ground_plane'>
        <pose frame=''>17.0053 7.21717 0 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose frame=''>17.0053 7.21717 0 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <model name='table'>
        <pose frame=''>1.38 0 0 0 -0 1.56</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose frame=''>1.38 0 0 0 -0 1.56</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <light name='sun'>
        <pose frame=''>0 0 10 0 -0 0</pose>
      </light>
    </state>
    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose frame=''>2.25201 -2.09966 1.37782 0 0.160015 2.00227</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
      </camera>
    </gui>
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
            <contact>
              <collide_bitmask>65535</collide_bitmask>
              <ode/>
            </contact>
            <friction>
              <ode>
                <mu>100</mu>
                <mu2>50</mu2>
              </ode>
              <torsional>
                <ode/>
              </torsional>
            </friction>
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
      <pose frame=''>17.0053 7.21717 0 0 -0 0</pose>
    </model>
    <model name='coke_can'>
      <link name='link'>
        <inertial>
          <pose frame=''>0 0 0.06 0 -0 0</pose>
          <mass>0.39</mass>
          <inertia>
            <ixx>0.00055575</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.00055575</iyy>
            <iyz>0</iyz>
            <izz>0.0001755</izz>
          </inertia>
        </inertial>
        <collision name='collision'>
          <pose frame=''>0.003937 0.004724 -0.18 0 -0 0</pose>
          <geometry>
            <mesh>
              <uri>model://coke_can/meshes/coke_can.dae</uri>
            </mesh>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1</mu>
                <mu2>1</mu2>
              </ode>
              <torsional>
                <ode/>
              </torsional>
            </friction>
            <contact>
              <ode>
                <kp>1e+07</kp>
                <kd>1</kd>
                <min_depth>0.001</min_depth>
                <max_vel>0.1</max_vel>
              </ode>
            </contact>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='visual'>
          <pose frame=''>0.003937 0.004724 -0.18 0 -0 0</pose>
          <geometry>
            <mesh>
              <uri>model://coke_can/meshes/coke_can.dae</uri>
            </mesh>
          </geometry>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <pose frame=''>1.11651 0.230436 0 0 -0 0</pose>
    </model>
  </world>
</sdf>


```