<p align="center">

<img alt="MultiCobot-UR10-Gripper" style="border-width:0" src="https://raw.githubusercontent.com/Serru/MultiCobot-UR10-Gripper/main/doc/imgs_md/proyect-logo.png" />
</p>

# MultiCobot-UR10-Gripper

<p align="center">
Sistema multirobot de robots colaborativos (cobots) <a rel="UR10s" href="https://www.universal-robots.com/products/ur10-robot/">UR10s</a> con pinzas de Robotiq (<a rel="robotiq_85_gripper" href="https://robotiq.com/products/2f85-140-adaptive-robot-gripper">robotiq_85_gripper</a>) que permite la realización de tareas simultáneamente con diferentes tipos de controladores y marcas de cobots, así como el control directo mediante el dispositivo Leap Motion del cobot por una persona.</p>

<p align="center">
  <img alt="Doc" style="border-width:0" src="https://img.shields.io/badge/doc-complete-green?logo=markdown&style=plastic" />
  <a rel="Build Status" href="https://app.travis-ci.com/github/Serru/MultiCobot-UR10-Gripper"><img alt="Build Status" style="border-width:0" src="https://img.shields.io/travis/com/Serru/MultiCobot-UR10-Gripper?logo=travis&style=plastic" />
    </a>
  <a rel="license" href="http://creativecommons.org/licenses/by/4.0/"><img alt="Creative Commons License" style="border-width:0" src="https://img.shields.io/github/license/Serru/MultiCobot-UR10-Gripper?style=plastic" />
  </a>
      <a rel="Python" href="https://github.com/Serru/MultiCobot-UR10-Gripper">
  <img alt="Python 2.7" style="border-width:0" src="https://img.shields.io/badge/Python-14354C?logo=python&logoColor=white&style=plastic" />
  </a>
    <a rel="C++" href="https://github.com/Serru/MultiCobot-UR10-Gripper">
  <img alt="C++" style="border-width:0" src="https://img.shields.io/badge/C%2B%2B-00599C?logo=c%2B%2B&style=plastic" />
  </a>
  <a rel="stars" href="https://github.com/Serru/MultiCobot-UR10-Gripper/stargazers"><img alt="Stars of the repository" style="border-width:0" src="https://img.shields.io/github/stars/Serru/MultiCobot-UR10-Gripper?style=plastic" />
  </a> 
    <a rel="Repository Size" href="https://github.com/Serru/MultiCobot-UR10-Gripper"><img alt="Repository Size" style="border-width:0" src="https://img.shields.io/github/repo-size/Serru/MultiCobot-UR10-Gripper?style=plastic" />
  </a>
  <a rel="ubuntu16.04" href="https://releases.ubuntu.com/16.04/"><img alt="Ubuntu 16.04" style="border-width:0" src="https://img.shields.io/badge/OS-ubuntu%2016.04-important?style=plastic&logo=ubuntu" />
  </a>
  <a rel="ros" href="http://wiki.ros.org/kinetic"><img alt="ROS Kinetic Kame" style="border-width:0" src="https://img.shields.io/badge/ROS-Kinetic%20Kame-important?style=plastic&logo=ros" />
  </a>
</p>

**Español** | [English](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/README.md)



## Sobre este proyecto
Este Trabajo Fin de Grado se enfoca en el desarrollo de un sistema multirobot que pueda realizar tareas de forma cooperativa como es el transporte de objetos. No hay mucha documentación en cómo desarrollar un sistema en donde se controle varios robots simultáneamente en el entorno de ROS, el cual es ampliamente utilizado en la investigación y prototipado para efectuar pruebas antes de su puesta en producción.

Se han diseñado, desarrollado, implementado y evaluado experimentalmente dos soluciones: la primera es mediante el paquete de *ROS* `MoveIt!`,  en donde el trabajo se centra sobre todo en su configuración para permitir el control simultáneo de varios cobots; y la segunda es la creación o utilización de un planificador de terceros que envía las órdenes directamente a los controladores encargados de ejecutar los movimientos de los cobots y cada uno de ellos tienen incorporado una pinza que les permiten hacer diferentes tareas.

También está incorporado al sistema el dispositivo Leap Motion que es capaz de detectar, rastrear y reconocer gestos de las manos del usuario como interfaz para el control simultáneo de hasta dos cobots, permitiendo la manipulación de objetos.

[Leer más...](https://deposita.unizar.es/record/66296?ln=es)



## Requisitos del sistema
- Ubuntu 16.04
- Python 2.7
- ROS Kinetic Kame

## 🚀 Quickstart

- Clonar este repositorio:
```bash
git clone https://github.com/Serru/MultiCobot-UR10-Gripper
```

- Establecer el workspace de catkin:
```bash
cd ~/MultiCobot-UR10-Gripper/src
catkin_init_workspace
```

- Instalar todas las dependencias:
```bash
source /opt/ros/kinetic/setup.bash
cd ~/MultiCobot-UR10-Gripper
rosdep update
rosdep install -r --rosdistro kinetic --ignore-src --from-paths src
```

- Compilar el repositorio
```bash
catkin_make
rospack profile
source devel/setup.bash
```

- Lanzar Gazebo con dos robots ejecutando un simple Pick & Place

Terminal 1:
```bash
roslaunch two_arm_no_moveit_gazebo ur10_joint_limited.launch
``` 

Terminal 2:
```bash
rosrun two_arm_no_moveit_manipulator ur10_1_robot_manipulator.py
```

Terminal 3:
```bash
rosrun two_arm_no_moveit_manipulator ur10_2_robot_manipulator.py
```


## Documentación
- [Configuración del sistema base](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/setup-doc/proyect_setup.md)

### Configuración en el simulador de `Gazebo`
- [Diseño y desarrollo para uno, dos y cuatro robots](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/design.md)
- [Diseño e integración del dispositivo Leap Motion en el sistema para el control de uno y dos robots](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/design-lm.md)

### Configuración en el robot físico
- [Configuración para un UR10 con pinza mediante un planificador propio en el robot físico](https://github.com/Serru/MultiCobot-UR10-Gripper-Campero)

## Video con los resultados 
Aquí se deja un video con los resultados obtenidos de las simulaciones efectuadas en `Gazebo`. El vídeo contiene dos y cuatro robots haciendo un *pick & place* sin intervención humana, después mediante el dispositivo Leap Motion se muestra el control de dos cobots por una persona y finalmente el resultado desarrollado se ha probado en el robot físico Campero.

<p>
<a href="https://drive.google.com/file/d/1oqVyre4vlfHqH9SrQuyXH00GcmwIuP97/view?usp=sharing" title="Link Title">
	<img src="https://raw.githubusercontent.com/Serru/MultiCobot-UR10-Gripper/main/doc/imgs_md/img-fondo-video.png" alt="Resultados del proyecto." />
</a>
</p>


## Ayuda y asistencia
Desgraciadamente, este repositorio no está mantenido activamente. El objetivo principal es la divulgación de lo aprendido a la comunidad, que puedan necesitar del conocimiento y contenido de este repositorio para el desarrollo de su proyecto o investigación.

No se garantiza una respuesta, pero puedes contactar con los autores con la información contenida en la sección de [Autores](#autores).

## Licencia

<p align="left">
  <a href="https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/LICENSE">
    <img src="https://licensebuttons.net/l/by/4.0/88x31.png" alt="Este repositorio está publicado bajo la licencia de Creative Commons Attribution 4.0 International." />
  </a>
  </br>
  </br>
Este repositorio está publicado bajo la licencia de <a href="https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/LICENSE">Creative Commons Attribution 4.0 International</a>.
</p>

## Autores
* [Burgh Oliván, Miguel](https://github.com/Serru) - *Autor del Trabajo de Fin de Grado titulado **Sistema multirobot para el transporte colaborativo de objetos**.*
* [López Nicolás, Gonzalo](https://i3a.unizar.es/es/investigadores/gonzalo-lopez-nicolas) - *Director del Trabajo de Fin de Grado titulado **Sistema multirobot para el transporte colaborativo de objetos**.*

La memoria del Trabajo Fin de Grado, se puede encontrar en el [repositorio](https://deposita.unizar.es/record/66296?ln=es) de TFGs de la [Universidad de Zaragoza](http://www.unizar.es/).

## Agradecimientos

Este trabajo está enmarcado en el grupo de investigación de [RoPeRT](https://i3a.unizar.es/es/grupos-de-investigacion/ropert) del [i3A](https://i3a.unizar.es), de la [Universidad de Zaragoza](http://www.unizar.es/).

![image](https://www.unizar.es/sites/default/files/i3a.png)
---
El trabajo desarrollado ha sido evaluado y validado experimentalmente, mostrando un correcto funcionamiento en el robot físico [Campero](http://commandia.unizar.es/wp-content/uploads/camperoRobot.jpg). Por ello este Trabajo Fin de Grado entra dentro de las actividades del proyecto [COMMANDIA (2019)](http://commandia.unizar.es/), cofinanciado por el [Programa Interreg Sudoe](https://www.interreg-sudoe.eu/inicio) y por el [Fondo Europeo de Desarrollo Regional (FEDER)](https://ec.europa.eu/regional_policy/es/funding/erdf/).

![image](http://commandia.unizar.es/wp-content/uploads/cropped-logoCommandia-1.png)

## Reconocimiento

Por favor, cita esta publicación si el contenido de este repositorio te ha sido útil:

BibTeX: 
```
@InProceedings{10.1007/978-3-031-21065-5_34,
author="Burgh-Oliv{\'a}n, Miguel
and Arag{\"u}{\'e}s, Rosario
and L{\'o}pez-Nicol{\'a}s, Gonzalo",
editor="Tardioli, Danilo
and Matell{\'a}n, Vicente
and Heredia, Guillermo
and Silva, Manuel F.
and Marques, Lino",
title="ROS-Based Multirobot System for Collaborative Interaction",
booktitle="ROBOT2022: Fifth Iberian Robotics Conference",
year="2023",
publisher="Springer International Publishing",
address="Cham",
pages="411--422",
abstract="This paper presents the design and implementation of a collaborative multi-robot system based on ROS. The goal is to manipulate objects with multiple cobots simultaneously by following commands given by a user via gestures. Two methods have been designed, developed, implemented and experimentally evaluated: The first one is based on the ROS package called MoveIt! and focuses mainly on configuration to allow simultaneous control of different cobots. The second method involves the development of a third-party motion planner that sends commands directly to the controllers responsible for executing the cobots' movements. The Leap Motion, a device that can be used for gesture recognition, is also integrated into the system to enable user interaction in object manipulation. The system has been tested in simulation using Gazebo and evaluated in a real UR10 robot. The main contribution of the proposed architecture is that it solves the problem of controlling multiple robots simultaneously in ROS. In particular, our approach allows simultaneous execution of tasks with different types of controllers, brands and models, as well as direct control of the robots by using the Leap Motion device.",
isbn="978-3-031-21065-5"
}
```
