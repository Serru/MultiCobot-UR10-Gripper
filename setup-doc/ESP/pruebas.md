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
