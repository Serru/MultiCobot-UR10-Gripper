# Diseño del sistema multirobot

Tras realizar numerosas pruebas en el entorno de ROS para crear un sistema multirobot se obtiene que las soluciones deben salir de las combinaciones entre el fichero URDF que define el modelo del robot y los paquetes de ROS que en el esquema está representado por el paquete de `MoveIt!` que su función principal es la de planificador.

![image](/doc/imgs_md/Diseno-General-focus.png  "Herramientas y Drivers de ROS en el diseño")

Las variaciones posibles son:

- *Fichero URDF*: describe el modelo del robot, en este modelo se puede integrar varios robots, objetos o lo que se quiera modelar.
- *Paquetes de ROS*: son los paquetes de ROS, pueden ser propios o instalados de terceros sobre el que se realizarán modificaciones para adaptarlos a la solución a desarrollar.

Las soluciones que se proponen giran en torno a las modificaciones y combinaciones entre el modelado del robot (`URDF`) y los `paquetes de ROS` que se utilicen, por organización las soluciones propuestas se van a dividir en las soluciones que utilicen el paquete  `MoveIt!` y las que no. Hay que tener en cuenta que el entorno de trabajo es complejo y hay muchos elementos que interaccionan o tienen dependencias entre sı́, por lo que durante el desarrollo de los diseños propuestos pueden surgir problemas que no tienen solución o que el coste de corregirlos es muy alto.

## Desarrollo y análisis de las soluciones propuestas
- [Mediante el paquete `MoveIt!`](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/moveit-intro.md)
- [Mediante un planificador propio o de terceros (sin el paquete de `MoveIt!`)](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/no-moveit-intro.md)

