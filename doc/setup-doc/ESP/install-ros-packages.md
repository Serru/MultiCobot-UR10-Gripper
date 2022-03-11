# Instalación de los paquetes de *ROS*

**Español** | [English](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/setup-doc/ENG/install-ros-packages.md)

## Creación del directorio de trabajo

```bash
mkdir -p ~/ MultiCobot-UR10-Gripper/src
cd  ~/MultiCobot-UR10-Gripper/src
```

## Atención: Paquetes de terceros de *ROS*
Durante el proceso de instalación de los paquetes de *ROS*, no se ha realizado ninguna modificación directa sobre el contenido. Todas las modificaciones efectuadas sobre estos paquetes han sido para su adaptación a *ROS Kinetic* durante su compilación.

*Se han respetado las licencias de cada paquete instalado, en caso de que haya alguna licencia que debido por su actualización, modificación u otra situación, por favor contacte con los autores para su modificación o eliminación del paquete en este repositorio.*

## Instalación de paquetes de terceros
Instalación de paquetes de terceros que se van a utilizar total o parcialmente.

### [Universal Robots](https://github.com/ros-industrial/universal_robot)
Este paquete de Universal Robots se empleará como base para efectuar las modificaciones necesarias que el proyecto requiera, el paquete provee del modelado de los robots de la marca, como son el UR3, UR5, UR10, etc. así como ejemplos de uso con otros paquetes (MoveIt!) y herramientas (Gazebo). Los ficheros modificados irán debidamente detallados. 

Clonar el repositorio:
```bash
cd ~/MultiCobot-UR10-Gripper/src
git clone -b kinetic-devel https://github.com/ros-industrial/universal_robot.git
```

Instalación de dependencias que puedan faltar para *ROS Kinetic Kame*:
```bash
cd ~/MultiCobot-UR10-Gripper
rosdep update
rosdep install --rosdistro kinetic --ignore-src --from-paths src
```

Compilación:
```bash
cd ~/MultiCobot-UR10-Gripper
catkin_make
```

### [Robotiq_2finger_grippers](https://github.com/Danfoa/robotiq_2finger_grippers)
Este repositorio de Robotiq se usará para añadir la pinza que empleará el robot y los controladores necesarios para su funcionamiento. Este paquete está pensado para la comunicación con la pinza del robot físico porque está conectado al robot mediante USB, y entre las funcionalidades que provee este paquete, está la de comunicarse con la pinza mediante USB y proveer una interfaz de comunicación entre el sistema de *ROS* con la pinza.

Clonar el repositorio:
```bash
cd ~/MultiCobot-UR10-Gripper/src
git clone https://github.com/Danfoa/robotiq_2finger_grippers.git
```

Instalación de dependencias que puedan faltar para *ROS Kinetic Kame*:
```bash
cd ~/MultiCobot-UR10-Gripper
rosdep update
rosdep install --rosdistro kinetic --ignore-src --from-paths src
```

Compilación:
```bash
cd ~/MultiCobot-UR10-Gripper
catkin_make
```

### [Robotiq_85_gripper](https://github.com/PickNikRobotics/robotiq_85_gripper)
Este repositorio de Robotiq se utilizará para añadir la pinza que se utilizará al robot y los controladores necesarios para su funcionamiento. Parecido al paquete anterior, pero está orientado para la simulación de la pinza en `Gazebo`.

Clonar el repositorio:
```bash
cd ~/MultiCobot-UR10-Gripper/src
git clone -b develop https://github.com/PickNikRobotics/robotiq_85_gripper.git
```

Instalación de dependencias que puedan faltar para *ROS Kinetic Kame*:
```bash
cd ~/MultiCobot-UR10-Gripper
rosdep update
rosdep install --rosdistro kinetic --ignore-src --from-paths src
```
### [ros_control](https://github.com/ros-controls/ros_control)
Este repositorio contiene: "*A set of packages that include controller interfaces, controller managers, transmissions and hardware_interfaces*".

Clonar el repositorio:
```bash
cd ~/MultiCobot-UR10-Gripper/src
git clone -b kinetic-devel https://github.com/ros-controls/ros_control.git
```

Instalación de dependencias que puedan faltar para *ROS Kinetic Kame*:
```bash
cd ~/MultiCobot-UR10-Gripper
rosdep update
rosdep install --rosdistro kinetic --ignore-src --from-paths src
```

Compilación:
```bash
cd ~/MultiCobot-UR10-Gripper
catkin_make
```


### [ur_modern_driver](https://github.com/ros-industrial/ur_modern_driver)
Este repositorio está obsoleto, pero por compatibilidad con *ROS Kinetic Kame* hay que usarlo. Contiene drivers para los robots de Universal Robots (UR3/UR5/UR10) y es compatible con *ros_control*.

Clonar el repositorio:
```bash
cd ~/MultiCobot-UR10-Gripper/src
git clone -b kinetic-devel https://github.com/ros-industrial/ur_modern_driver.git
```

Instalación de dependencias que puedan faltar para *ROS Kinetic Kame*:
```bash
cd ~/MultiCobot-UR10-Gripper
rosdep update
rosdep install --rosdistro kinetic --ignore-src --from-paths src
```

Compilación:
```bash
cd ~/MultiCobot-UR10-Gripper
catkin_make
```

Puede dar error de compilación para *ROS Kinetic Kame*, para solucionarlo hay que modificar el fichero *ur_hardware_interface.cpp*, para realizarlo rápido, sustituir el contenido del fichero por el contenido que hay en [aquí](https://github.com/iron-ox/ur_modern_driver/blob/883070d0b6c0c32b78bb1ca7155b8f3a1ead416c/src/ur_hardware_interface.cpp) y volverlo a compilar.

### [gazebo-pkgs](https://github.com/JenniferBuehler/gazebo-pkgs)
Es una colección de plugings para `Gazebo`, principalmente nos interesa el pluging que permite agarrar objetos.


Dependencias del paquete:
- **gazebo_ros**
- **eigen_conversions**
- **object_recognition_msgs**
- **roslint**
- **general-message-pkgs**

Instalación de dependencias del paquete:
Ros usa **Gazebo 7.x**
```bash
sudo apt-get install -y libgazebo7-dev
```
Clonar el repositorio de **[gazebo_ros_pkgs](https://github.com/ros-simulation/gazebo_ros_pkgs)**
```bash
cd ~/MultiCobot-UR10-Gripper/src
git clone -b kinetic-devel https://github.com/ros-simulation/gazebo_ros_pkgs.git
```

Clonar el repositorio de **[eigen_conversions](https://github.com/ros/geometry)**
```bash
cd ~/MultiCobot-UR10-Gripper/src
git clone -b indigo-devel https://github.com/ros/geometry.git
```

Clonar el repositorio de **[object_recognition_msgs](https://github.com/wg-perception/object_recognition_msgs)**
```bash
cd ~/MultiCobot-UR10-Gripper/src
git clone -b master https://github.com/wg-perception/object_recognition_msgs.git
```

Clonar el repositorio de **[roslint](https://github.com/ros/roslint)**
```bash
cd ~/MultiCobot-UR10-Gripper/src
git clone -b master https://github.com/ros/roslint.git
```

Clonar el repositorio de **[general-message-pkgs](https://github.com/JenniferBuehler/general-message-pkgs)**
```bash
cd ~/MultiCobot-UR10-Gripper/src
git clone -b master https://github.com/JenniferBuehler/general-message-pkgs.git
```


Instalación de dependencias que puedan faltar para *ROS Kinetic Kame*:
```bash
cd ~/MultiCobot-UR10-Gripper
rosdep update
rosdep check --from-paths . --ignore-src --rosdistro kinetic
rosdep install --from-paths . --ignore-src --rosdistro kinetic -y
```

Compilación:
```bash
cd ~/MultiCobot-UR10-Gripper
catkin_make
```

---

Clonar el repositorio **[gazebo-pkgs](https://github.com/JenniferBuehler/gazebo-pkgs)**:
```bash
cd ~/MultiCobot-UR10-Gripper/src
git clone https://github.com/JenniferBuehler/gazebo-pkgs.git
```

Instalación de dependencias que puedan faltar para *ROS Kinetic Kame*:
```bash
cd ~/MultiCobot-UR10-Gripper
rosdep update
rosdep install --rosdistro kinetic --ignore-src --from-paths src
```

Compilación:
```bash
cd ~/MultiCobot-UR10-Gripper
catkin_make
```

---

Durante la compilación puede aparecer un **error** que pide que esté compilado con el estándar de c++11:
```bash
/usr/include/c++/5/bits/c++0x_warning.h:32:2: error: #error This file requires compiler and library support for the ISO C++ 2011 standard. This support must be enabled with the -std=c++11 or -std=gnu++11 compiler options.
```

Para solventarlo, hay que modificar el `make` de *catkin* al compilar el proyecto:

- Simplemente, hay que modificar el fichero `~/MultiCobot-UR10-Gripper/src/CMakeLists.txt` y añadir *add_compile_options(-std=c++11)* al principio del fichero, como se muestra a continuación.

```bash
# toplevel CMakeLists.txt for a catkin workspace
# catkin/cmake/toplevel.cmake

cmake_minimum_required(VERSION 3.0.2)

project(Project)

set(CATKIN_TOPLEVEL TRUE)
add_compile_options(-std=c++11)
[...]
```

---

<div>
<p align="left">
<button name="button">
            	<a rel="license" href="https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/setup-doc/proyect_setup.md">Menú</a>
</button>
</p>



<p>
<span style="float:left;">
    <button name="button">
    	<a rel="license" href="https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/setup-doc/ESP/install-ROS.md">Anterior</a>
    	</button> 
    </span> 
    <span style="float:right;">
        <button name="button">
            	<a rel="license" href="https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/setup-doc/ESP/leap-motion.md">Siguiente</a>
            	</button>
    </span>
</p>
</div>