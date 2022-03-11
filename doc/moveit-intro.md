# Soluciones propuestas utilizando el paquete [`MoveIt!`](https://github.com/ros-planning/moveit)

**Español** | [English](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/moveit-intro-eng.md)

## Índice
- [Modelando varios robots en el fichero *URDF*](#modelado)
- [Replicando el nodo de `MoveIt!`](#replicacion)

<a name="modelado">
  <h2>
  Modelando varios robots en el fichero <i>URDF</i>
  </h2>
</a>

![image](/doc/imgs_md/Diseno-moveit-general-dos-cobots-leap-motion-urdf.png  "Sistema multirobot mediante el modelado del fichero URDF")

En el diseño del esquema, el componente principal que realiza las funciones de planificador es el paquete `MoveIt!`, se basa en la adición de robots en el fichero *URDF* y el planificador de `MoveIt!` lo reconoce como un único robot, el cual obtiene los valores articulares con el planificador de movimiento de sus `end effectors` (API del nodo `move_group`) y se envía esos valores obtenidos como una única trayectoria a los controladores para que ejecute el movimiento deseado.

**Las ventajas de esta propuesta son:**

- La potencia de las funciones que ofrece `MoveIt!` para cada robot como en su conjunto.
- Una sencilla configuración, la mayoría, se crea de forma automática con su `MoveIt Setup Assistant`.
- Flexibilidad a la hora de adaptarse o la incorporación de futuros elementos que puedan necesitar las funcionalidades de `MoveIt!`, como pueden ser dispositivos que se utilicen para desarrollo de trabajo de visión por computador, *deep learning*, navegación, etc.

**Las desventajas son:**

- Coste en tiempo a la hora de obtener la planificación de las trayectorias, ya que previamente debe obtener las trayectorias de cada `end effector` para después combinarlas como si fuese el movimiento de un único robot.
- Existe único grupo que gestiona los movimientos de los cobots, lo que implica que habría que reconfigurarlo cada vez que se añadiese un robot al sistema, lo que afecta a la escalabilidad del sistema.
- No permite el control de diferentes modelos y marcas de cobots simultáneamente, ya que podrían necesitar diferentes controladores. 

Esta solución tiene una ventaja muy importante y es que es consciente del movimiento de todos los robots y durante la planificación de las trayectorias puede evitarlos para llegar a la posición deseada. Pero la arquitectura del diseño del paquete de `MoveIt!` no permite el control de varios robots con diferentes controladores. 

Si se quiere realizar el control de varios robots mediante este diseño, se podría conseguirlo mediante la secuenciación de los movimientos de cada cobot y no la simultaneidad que es lo que se busca.

En caso de forzarlo (lanzando dos scripts que envíen instrucciones para que planifique las trayectorias deseadas para cada controlador) el nodo de `MoveIt!`, `move_group`, envía un mensaje diciendo que la instrucción ha sido *PREEMTED*, es decir que el Scheduler del sistema lo ha retrasado como trabajo futuro.

Para sobrepasar este problema, habría que hacerle pensar al nodo `move_group` que maneja un único controlador. Para conseguir esto, hay que crear mínimo tres grupos de manipuladores durante la configuración con el asistente de `MoveIt!`, uno para cada brazo junto a su `end effector` y otro grupo que contiene a los dos grupos creados previamente. 

Cuando se tiene esta configuración, habría que realizar el plan de trayectorias de cada brazo sin ejecutarlo y añadir los valores de las articulaciones de cada cobot para esa trayectoria adecuadamente en el grupo que contiene a ambos cobots para ejecutarlo posteriormente.

Esta propuesta está ya implementada por [*TEAM O2AC para la World Robot Summit 2020 Assembly Challenge*](https://github.com/o2ac/o2ac-ur/)


<a name="replicacion">
  <h2>
  Replicando el nodo de <a href="https://github.com/ros-planning/moveit"><i>MoveIt!</i></a>
  </h2>
</a>

![image](/doc/imgs_md/Diseno-moveit-general-dos-cobots-leap-motion.png  "Sistema multirobot mediante replicación de nodos")

En el diseño del esquema, el componente principal que lleva a cabo las funciones de planificador es el paquete `MoveIt!`. Este diseño se basa en la replicación de los componentes que el planificador necesita para su funcionamiento (el nodo principal es `move_group`). Cada replicación es un planificador asignado para cada uno de los cobots que están delimitados dentro de su propio espacio o `namespace`.

En la descripción del robot, es únicamente necesario la descripción de un cobot y el número de robots que puede controlar simultáneamente está directamente relacionado con el número de replicaciones del planificador con diferentes `namespaces`. Los números en la imagen enumera el orden de las fases durante su desarrollo.

**Las ventajas de esta propuesta son:**

- La escalabilidad del sistema y del número de cobots.
- La potencia de las funciones que ofrece `MoveIt!` para cada robot
individualmente.
- Una sencilla configuración, la mayoría se crea de forma automática con su `MoveIt Setup Assistant`.
- Flexibilidad a la hora de adaptarse o la incorporación de futuros elementos que puedan necesitar las funcionalidades de `MoveIt!`, como pueden ser dispositivos que se utilicen para el desarrollo de trabajo de visión por computador, *deep learning*, navegación, etc.
- Permite el control de diferentes modelos y marcas de cobots simultáneamente.
- Hay una comunidad detrás del paquete de `MoveIt!` muy activa que da soporte a sus usuarios.


**Las desventajas son:**

- Poca documentación con respecto a la correcta configuración para el control de varios robots simultáneamente.
- Limitación de las funcionalidades del paquete `MoveIt!` a cada robot individualmente, pero no en conjunto, por ejemplo: la funcionalidad
planificar trayectorias evitando colisiones entre los robots no funcionaría adecuadamente porque tiene no conocimiento de ellos.
- Pérdida de eficiencia conforme se escala: al escalar el sistema y aumentar el número de cobots que controla, se replica también las funcionalidades que en ese momento ocupan un recurso, pero no hacen ningún trabajo.
- Complicado realizar cambios en el código fuente de las funcionalidades del paquete de `MoveIt!`.

### Desarrollo e implementación de la solución y sus pruebas paso a paso (fases 1, 2 y 3)
- [Un UR10 con pinza mediante el paquete `MoveIt!`](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/moveit/ESP/one_arm_moveit.md)
- [Dos UR10s con pinzas mediante el paquete `MoveIt!`](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/moveit/ESP/two_arm_moveit.md)
- [Cuatro UR10s con pinzas mediante el paquete `MoveIt!`](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/moveit/ESP/four_arm_moveit.md)

---

<div>
<p align="left">
<button name="button">
              <a rel="license" href="https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/design.md">Menú</a>
</button>
</p>
</div>