# Soluciones propuestas utilizando un planificador propio o de terceros (sin el paquete de `MoveIt!`)

**Español** | [English](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/no-moveit-intro-eng.md)

![image](/doc/imgs_md/Diseno-no-moveit-general-dos-cobots-leap-motion.png  "Cargado el modelo URDF del robot UR10")


En el diseño del esquema, el componente principal que realiza las funciones de planificador debe ser implementado de cero, el planificador se comunica directamente con los controladores necesarios para realizar los movimientos del cobot, no es necesario que esté contenido dentro de un *namespace*. 

Es necesaria la descripción de todos los cobots que se quiera controlar simultáneamente en el fichero *URDF* que define el modelo. Los números en la imagen listan el orden de las fases por las que pasa esta solución.

**Las ventajas de esta propuesta son:**

- La escalabilidad del sistema y del número de cobots es más compleja.
- Es muy eficiente en comparación con las soluciones anteriores.
- Es sencillo efectuar cambios en el código fuente implementado.
- Permite el control de diferentes modelos y marcas de cobots simultáneamente.

**Las desventajas son:**

- Poca adaptabilidad a la integración en otro proyecto, al ser una solución a medida.
- No tiene capacidad para planificar trayectorias evitando colisiones con otro cobots ni consigo mismo porque son funcionalidades proporcionadas por `MoveIt!`.
- Hay que implementar la funcionalidad para realizar movimientos cartesianos, es un requisito dada la futura incorporación de *Leap Motion* al sistema.
- La configuración puede ser tediosa (controladores, *topic*s, tratamiento de los mensajes, interacción entre lo que se ha creado con lo creado por terceros, etc.), es necesario cierta familiaridad con el entorno.

#### Desarrollo e implementación de la solución y sus pruebas paso a paso (fases 1, 2 y 3)
- [Un UR10 con pinza mediante un planificador propio](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/no_moveit/ESP/one_arm_no_moveit.md)
- [Dos UR10s con pinzas mediante un planificador propio](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/no_moveit/ESP/two_arm_no_moveit.md)
- [Cuatro UR10s con pinzas mediante un planificador propio](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/no_moveit/ESP/four_arm_no_moveit.md)

---

<div>
<p align="left">
<button name="button">
              <a rel="license" href="https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/design.md">Menú</a>
</button>
</p>
</div>