# Un UR10 con pinza mediante un planificador propio y Leap Motion

**Español** | [English](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/no_moveit/ENG/one_arm_no_moveit_lm.md)

![image](/doc/imgs_md/Diseno-no-moveit-general-un-cobot-leap-motion.png  "Cargado el modelo URDF del robot UR10")

Esta sección la continuación de las etapas 1, 2 y 3, en donde se realiza la integración de *Leap Motion* en el sistema, es decir las fases 4 y 5. En las fases anteriores se controlaba el robot enviando las trayectorias previamente definidas para llevar a cabo el *pick & place*, pero ahora será mediante *Leap Motion* el cual es controlado por una persona el que envíe los comandos al robot.

Para ello se va a modificar parte del repositorio de *Leap Motion* directamente y adaptarlo para que controle adecuadamente el robot simulado. La idea es idéntica a lo presentado anteriormente, pero tiene como entrada de datos la información que *Leap Motion* provee. Hay que tratar esta información adecuadamente con la API que provee y adecuarlo para su uso.

## :book: Implementación del manipulador con Leap Motion

Para realizar la implementación hay que tener varias cosas en cuenta:

- Las referencias de la posición del robot que se obtiene de script *ur10_robot_pose.py*
- El entorno de trabajo del *robot*
- El entorno de trabajo de *Leap Motion*

Como se desea que el robot siga los movimientos que ejecuta *Leap Motion*, se va a implementar un controlador que su espacio de trabajo viene contenido dentro del espacio de trabajo del robot.

### :computer: Creación del paquete
```bash
cd ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit
catkin_create_pkg one_arm_no_moveit_leap_motion rospy
cd one_arm_no_moveit_leap_motion
mkdir scripts
cd scripts
cp ~/MultiCobot-UR10-Gripper/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_manipulator/scripts/kinematics_utils.py .
touch lm_robot_manipulator.py
touch sender.py
touch leap_interface.py
```

- Contenido del fichero [lm_robot_manipulator.py](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_leap_motion/scripts/lm_robot_manipulator.py).

- Contenido del fichero [sender.py](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_leap_motion/scripts/sender.py).

- Contenido del fichero [leap_interface.py](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_no_moveit/one_arm_no_moveit_leap_motion/scripts/leap_interface.py).

### :computer: Se ha modificado del repositorio original

Se han creado los ficheros: *leapcobotright.msg* y *leapcobotleft.msg* en el directorio *msg* del repositorio [Leap Motion](https://github.com/Serru/MultiCobot-UR10-Gripper/tree/main/src/leap_motion):

- Contenido del fichero [leapcobotright.msg](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/leap_motion/msg/leapcobotright.msg).

- Contenido del fichero [leapcobotleft.msg](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/leap_motion/msg/leapcobotleft.msg).

- Y el fichero [CMakeLists.txt](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/leap_motion/CMakeLists.txt) añadiendo los ficheros de mensajes recién creados para su compilación:
```bash
[...]
## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Arm.msg
  Bone.msg
  Finger.msg
  Gesture.msg
  Hand.msg
  Human.msg

  # For backwards compatibility with the old driver files
  leap.msg
  leapros.msg
  leapcobotright.msg
  leapcobotleft.msg
)
[...]
```

## :computer: Pruebas en el simulador *Gazebo*
Para efectuar las pruebas, se necesitarán al menos 3 terminales, aunque se pueden reducir para lanzarlos automáticamente en los ficheros *launch*, pero para visualizar mejor la información que se envía y para su depuración se ha dejado así.

- Terminal 1:
```bash
cd ~/MultiCobot-UR10-Gripper
source devel/setup.bash
roslaunch one_arm_no_moveit_gazebo ur10_joint_limited.launch
```

- Terminal 2:
```bash
cd ~/MultiCobot-UR10-Gripper
source devel/setup.bash
rosrun one_arm_no_moveit_leap_motion sender.py
```

- Terminal 3:
```bash
cd ~/MultiCobot-UR10-Gripper
source devel/setup.bash
rosrun one_arm_no_moveit_leap_motion lm_robot_manipulator.py 
```

- Terminal 4 (en caso de fallo en el dispositivo de *Leap Motion*):
```bash
sudo service leapd restart
```

---

<div>
  <p align="left">
    <button name="button">
                <a rel="license" href="https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/design-lm.md">Anterior</a>
    </button>
  </p>
</div>