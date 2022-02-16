<p align="center">
<img alt="MultiCobot-UR10-Gripper imagen" style="border-width:0" src="https://raw.githubusercontent.com/Serru/MultiCobot-UR10-Gripper/main/imgs_md/proyect-logo.png" />
  </a>

# MultiCobot-UR10-Gripper
<p align="center">
Sistema multirobot para el transporte de colaborativo de objetos, desarrollo del proyecto de fin de grado en la Universidad de Zaragoza.</p>

<p align="center">
  <a rel="license" href="http://creativecommons.org/licenses/by/2.0/"><img alt="Creative Commons License" style="border-width:0" src="https://i.creativecommons.org/l/by/2.0/80x15.png" />
  </a>
  
</p>



**English** | [Español](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/README.md)



## Sobre este proyecto
Este Trabajo Fin de Grado se enfoca en el desarrollo de un sistema multirobot que pueda realizar tareas de forma cooperativa como es el transporte de objetos. No hay mucha documentación en cómo desarrollar un sistema en donde se controle varios robots simultáneamente de forma correcta en el entorno de ROS, el cual es ampliamente utilizado en la investigación y prototipado para realizar pruebas antes de su puesta en producción.

Se han diseñado, desarrollado, implementado y evaluado experimentalmente dos soluciones: la primera es mediante el paquete de ROS MoveIt!, en donde el trabajo se centra sobre todo en su configuración para permitir el control simultáneo de varios
cobots; y la segunda es la creación o utilización de un planificador de terceros que envı́a las órdenes directamente a los controladores encargados de realizar los movimientos de los cobots y cada uno de ellos tienen incorporado una pinza que les permiten realizar
diferentes tareas. 

También está incorporado al sistema el dispositivo Leap Motion que es capaz de detectar, rastrear y reconocer gestos de las manos del usuario como interfaz para el control simultáneo de hasta dos cobots, permitiendo la manipulación de objetos.

[Leer más...](https://deposita.unizar.es/record/66296?ln=es)



## Requisitos del sistema:
- Ubuntu 16.04
- Python 2.7
- ROS Kinetic Kame




## Documentación
- [Setup del sistema](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/proyect_setup.md)
### Con el paquete de MoveIt!
- [Configuración para un UR10 con pinza mediante el paquete MoveIt!](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/moveit/one_arm_moveit.md)
- [Configuración para dos UR10s con pinzas mediante el paquete MoveIt!](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/moveit/two_arm_moveit.md)
- [Configuración para cuatro UR10s con pinzas mediante el paquete MoveIt!](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/moveit/four_arm_moveit.md)
### Sin el paquete de MoveIt!
- [Configuración para un UR10 con pinza mediante un planificador propio](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/no_moveit/one_arm_no_moveit.md)
- [Configuración para dos UR10s con pinzas mediante un planificador propio](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/no_moveit/two_arm_no_moveit.md)
- [Configuración para cuatro UR10s con pinzas mediante un planificador propio](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/no_moveit/four_arm_no_moveit.md)
### Configuración en el robot físico
- [Configuración para un UR10 con pinza mediante un planificador propio en el robot físico](https://github.com/Serru/MultiCobot-UR10-Gripper-Campero)

## Enlances a los videos con los resultados 
Aquí se deja un video con los resultados obtenidos de las simulaciones realizadas en Gazebo. El vídeo contiene dos y cuatro robots realizando un *pick & place* sin intervención humana, después mediante el dispositivo Leap Motion se muestra el control de dos cobots por una persona y finalmente el resultado desarrollado se ha probado en el robot físico Campero.

<p>
<a href="https://drive.google.com/file/d/1oqVyre4vlfHqH9SrQuyXH00GcmwIuP97/view?usp=sharing" title="Link Title">
	<img src="https://raw.githubusercontent.com/Serru/MultiCobot-UR10-Gripper/main/imgs_md/img-fondo-video.png" alt="Resultados del proyecto." />
</a>
</p>


## Ayuda y asistencia
Desgraciadamente, este repositorio no está mantenido activamente. El objetivo principal es la divulgación de lo aprendido a la comunidad, que puedan necesitar del conocimiento y contenido de este repositorio para el desarrollo de su proyecto o investigación.

No se garantiza una respuesta, pero puedes contactar con los autores con la información contenida en la sección de "Autores".

## Licencia

<p align="left">
  <a href="https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/LICENSE">
    <img src="https://licensebuttons.net/l/by/3.0/88x31.png" alt="Este repositorio está publicado bajo la licencia de CC0 1.0 Universal." />
  </a>
  </br>
  </br>
Este repositorio está publicado bajo la licencia de <a href="https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/LICENSE">Creative Commons Attribution 2.0 Generic License</a>.
</p>

## Autores
* [Burgh Oliván, Miguel](https://github.com/Serru) - *Autor del Trabajo de Fin de Grado titulado **Sistema multirobot para el transporte colaborativo de objetos** .*
* [López Nicolás, Gonzalo](https://i3a.unizar.es/es/investigadores/gonzalo-lopez-nicolas) - *Director del Trabajo de Fin de Grado titulado **Sistema multirobot para el transporte colaborativo de objetos** .*

La memoria del Trabajo Fin de Grado, se puede encontrar en el [repositorio](https://deposita.unizar.es/record/66296?ln=es) de TFGs de la [Universidad de Zaragoza](http://www.unizar.es/).

## Agradecimientos

Este trabajo está enmarcado en el grupo de investigación de [RoPeRT](https://i3a.unizar.es/es/grupos-de-investigacion/ropert) del [i3A](https://i3a.unizar.es), de la [Universidad de Zaragoza](http://www.unizar.es/).

![image](https://www.unizar.es/sites/default/files/i3a.png)
---
El trabajo desarrollado ha sido evaluado y validado experimentalmente, mostrando un correcto funcionamiento en el robot físico [Campero](http://commandia.unizar.es/wp-content/uploads/camperoRobot.jpg). Por ello este Trabajo Fin de Grado entra dentro de las actividades del proyecto [COMMANDIA (2019)](http://commandia.unizar.es/), cofinanciado por el [Programa Interreg Sudoe](https://www.interreg-sudoe.eu/inicio) y por el [Fondo Europeo de Desarrollo Regional (FEDER)](https://ec.europa.eu/regional_policy/es/funding/erdf/).

![image](http://commandia.unizar.es/wp-content/uploads/cropped-logoCommandia-1.png)

## Reconocimiento

Porfavor, cita esta publicación si el contenido de este repositorio te ha sido útil:

BibTeX: 
```
@article{
    BurghOliván:66296,
    author = "Burgh Oliván, Miguel Yankan and López Nicolás, Gonzalo",
    title = "{Sistema multirobot para el transporte colaborativo de objetos}",
    year  = "2022",
}
```
