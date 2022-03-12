# Diseño e integración de Leap Motion

**Español** | [English](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/design-lm-eng.md)

![image](/doc/imgs_md/Diseno-moveit-general-un-cobot-leap-motion.png  "Diseño e integración de Leap Motion")

Se va a explicar cómo integrar *Leap Motion* al sistema desarrollado hasta el momento, con el objetivo de controlar hasta dos robots simultáneamente para las diferentes soluciones propuestas en este repositorio. En la imagen del esquema del diseño, esta sección representan las etapas 4 y 5.

## Requisito previo
- Realizar correctamente la instalación de la [configuración base del sistema](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/setup-doc/proyect_setup.md).
- [Implementación de una de las soluciones propuestas](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/design.md) (corresponden a las fases 1, 2 y 3 del esquema).

## Índice
- [Fase 4: Diseño de la interfaz de *Leap Motion*](#fase4)
	- [Funcionamiento General](#lm1)
	- [Sistemas de coordenadas](#lm2)
	- [Workspace de *Leap Motion*](#lm3)
	- [Formas de control](#lm4)
	- [Identificación de Gestos](#lm5)
	- [Calibración de la velocidad de movimiento](#lm6)
	- [Creación de los `msgs` que transmitirán la información](#lm7)
	- [Obtención y publicación de datos por el *topic*](#lm8)
- [Fase 5: Integración de *Leap Motion* en el sistema](#fase5)

<a name="fase4">
  <h2>
Fase 4: Diseño de la interfaz de <i>Leap Motion</i>
  </h2>
</a>

<a name="lm1">
  <h3>
Funcionamiento General
  </h3>
</a>

![image](/doc/imgs_md/Diseno-leap-motion.png  "Esquema del funcionamiento de Leap Motion")

Utilizando las librerías de *Leap Motion* se identifican los gestos y se obtiene los datos necesarios para el control de la pinza, los movimientos del cobot y las orientaciones del *end-effector*. En la imagen muestra un esquema del funcionamiento general, se creará el fichero `leap_interface.py`, empleando la librería `Leap` de *Leap Motion* que estará a la escucha de eventos (`frames` de *Leap Motion*) y en cada evento obtendrá de ellos los datos que se necesitan y los almacenará en un `objeto` que luego es accedido por el nodo `sender` mediante la interfaz creada para acceder a ese objeto.


El nodo `sender` es el que obtiene la información, los almacena adecuadamente en un mensaje y los publica por el *topic* `leapmotion/data1`, a este *topic* estará suscrito el nodo `UR10_lm_arm_1`, que con la información obtenida del *topic*, envía las órdenes al cobot. Finalmente, el nodo `UR10_lm_arm_1` es básicamente el *script* que realiza el *pick & place*, pero la introducción de datos se obtiene del *topic* `leapmotion/data_1` en vez de ser introducidos manualmente.

En el esquema de la imagen, se ve que hay dos *topic*s que sale del nodo `sender`, esto es debido a que se ha tenido en cuenta durante el diseño que *Leap Motion* puede identificar hasta dos manos, por esta razón se han separado los datos de la mano derecha (`right.msg`) y de la izquierda (`left.msg`) en el esquema porque la información es enviada a través de diferentes *topic*s, no se ha metido toda la información en un único mensaje porque esto permite jugar con el ratio de las publicaciones, da más claridad y es más sencilla la depuración.

<a name="lm2">
  <h3>
Sistemas de coordenadas
  </h3>
</a>

![image](/doc/imgs_md/distintintos-sistemas-referencia.png  "Distintos sistemas de coordenadas de referencia")

Hay que tener en cuenta durante el diseño que las coordenadas de referencia de *ROS* y las que utiliza *Leap Motion* son distintas (como se muestra en la imagen), por ello durante la implementación hay que adaptaras adecuadamente.

<a name="lm3">
  <h3>
Workspace de <i>Leap Motion</i>
  </h3>
</a>

También hay que tener en cuenta que la zona de trabajo de *Leap Motion* bastante pequeña en comparación con la del cobot UR10, por ello según que tareas se quiera realizar hay que tenerlo en cuenta, pero realizar un simple *pick & place* como es en este caso, no hay problema.

<a name="lm4">
  <h3>
Formas de control
  </h3>
</a>

Los datos que se obtienen del *Leap Motion*, permiten implementar de manera sencilla dos formas de control:

- **Joystick:** Este tipo de control tiene una zona muerta (*death zone*), que toma un origen como referencia y en esa zona muerta no se ejecutará ningún movimiento, en el momento en que se sale de esa zona muerta, se va incrementando/decrementando el valor en esa coordenada dependiendo de la distancia a la que esté del origen de referencia. Esto debe ser calibrado para no ejecutar movimientos bruscos.

- **Imitación:** Esta es la solución que se ha escogido porque es más intuitivo a la hora de efectuar movimientos, consiste en tener el origen de referencia de *Leap Motion* y el origen de referencia del *end-effector* del cobot mapeado, es decir, que las coordenadas que se tomen de referencia para *Leap Motion* estará relacionada con la posición inicial del robot UR10. Esto permite al cobot imitar los
movimientos de la mano, igualmente hay que calibrarlo adecuadamente para evitar movimientos bruscos.

<a name="lm5">
  <h3>
Identificación de Gestos
  </h3>
</a>

Durante la identificación de los gestos hay que tener en cuenta que si los gestos que se utilizan son muy similares, *Leap Motion* puede realizar falsos positivos durante la identificación de los gestos, así como partes ocultas de la mano al moverse puede hacerle pensar que ha reconocido un gesto que no se ha realizado.

Se ha implementado cuatro tipos de gestos que son los que se muestran en la imagen, el *puño* indica que hay parar de enviar instrucciones, el *gesto de la pinza* es para controlar la pinza del cobot, el *gesto thumb up* indica que está preparado y el *gesto de rock* indica que toma la posición actual de la mano como origen de referencias. Se ha preparado para la implementación para el control de la orientación del *end-effector*, pero dado que da problemas de identificación con algunos de los gestos se decidió que es mejor tener una orientación fija.

![image](/doc/imgs_md/gestos-leap-motion.png  "Definición de gestos para Leap Motion")


En el fichero [leap_interface.py](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/src/multirobot/one_arm_moveit/one_arm_moveit_leap_motion/scripts/leap_interface.py) es donde se define los gestos, se va a analizar la parte del código que identifica un gesto como ejemplo. El trozo de código fuente que se muestra a continuación trata de comprobar cada vez que *Leap Motion* envía un `frame` si se ha realizado el gesto thumb up, por ello cada vez que se recibe un `frame`, comprueba si es de la mano derecha o izquierda, después se comprueba lo cerrada que esté la mano comprobando si el valor del atributo `grab strength`, en caso de ser mayor que lo definido identifica que la mano está cerrada. Sabiendo que la mano está cerrada se quiere saber si el pulgar está extendido o no y eso lo obtiene revisando si el atributo `thumb_finger.extended()` es igual a `1` y esta es la manera de identificar gestos con *Leap Motion*.

```python
def on_frame(self, controller):
        frame = controller.frame()

        for hand in frame.hands:
            handType = "Left hand" if hand.is_left else "Right hand"

            if handType == "Right hand":
                if hand.grab_strength > self.GRAB_STRENGTH_THRESHOLD:
                    thumb_finger = hand.fingers.finger_type(0)
                    for _ in thumb_finger:
                        if len(thumb_finger.extended()) == 0:
                            self.right_hand_fist = True
                            self.right_hand_thumb_up = False
                        elif len(thumb_finger.extended()) == 1:
                            self.right_hand_thumb_up = True
                            self.right_hand_fist = False
            else:
                if hand.grab_strength > self.GRAB_STRENGTH_THRESHOLD:
                    thumb_finger = hand.fingers.finger_type(0)
                    for _ in thumb_finger:
                        if len(thumb_finger.extended()) == 0:
                            self.left_hand_fist = True
                            self.left_hand_thumb_up = False
                        elif len(thumb_finger.extended()) == 1:
                            self.left_hand_fist = False
                            self.left_hand_thumb_up = True
```

<a name="lm6">
  <h3>
Calibración de la velocidad de movimiento
  </h3>
</a>

Para evitar movimientos bruscos, se ha utilizado la velocidad del *end-effector* como limitante, para ello se ha cogido como velocidad máxima *0.05 rad/s* y la distancia se basa en el mayor error entre los valores de los *joints* actuales y de los *joints* de la posición futura, realizando una división se obtiene el tiempo que debe durar ese movimiento.

<a name="lm7">
  <h3>
Creación de los msgs que transmitirán la información
  </h3>
</a>

Se han creado dos tipos de mensajes para su implementación, una para identificar los movimientos y gestos de la mano derecha y la otra para la mano izquierda que también se emplearán para definir el tipo de dato que se transmitirá por ese *topic*. Estos ficheros con extensión `msg` tienen que ser compilados dentro de un directorio llamado *msg* para ser integrados en el sistema *ROS* o no lo encontrará. Concretamente, el contenido del siguiente código fuente es para la mano derecha, para la mano izquierda sería igual pero sustituyendo `right` por `left`. 

```bash
Header header

# Right hand information
bool is_right_hand                      # Right hand detected
geometry_msgs/Point right_hand_palmpos  # Palm's position
bool right_hand_fist                    # Fist gesture recognize
bool right_hand_thumb_up                # Thumb up gesture recognize
bool right_hand_pinch                   # Pinch gesture recognize
float32 right_hand_pinch_value			# Pinch gesture value
bool right_hand_origin_frame            # Reference frame set
bool right_hand_set_origin_frame_detected # Detect gesture
float32 right_hand_rotate_value         # Values between [-1..0..1] rads
float32 right_hand_turn_value           # Values between [-1..0..1] rads
float32 right_hand_swing_value          # Values between [-1..0..1] rads
```

<a name="lm8">
  <h3>
Obtención y publicación de datos por el topic
  </h3>
</a>

Para la obtención de los datos de *Leap Motion*, se utiliza la interfaz (por ejemplo `li.get_is_right_hand()`) definida en el fichero `leap_interface.py` que permite acceder al *objeto* que almacena los datos que interesan como se muestra a continuación en el código fuente y una vez que el mensaje esté formado, se envía por el *topic* `leapmotion/data` con el tipo de mensaje `leapcobotright`.

```python
pub_ros_right   = rospy.Publisher('leapmotion/data', leapcobotright, queue_size=1)

while not rospy.is_shutdown():
        right_hand_palm_pos_    = li.get_right_hand_palmpos()   # Palm's position

        # Right hand information
        msg_right = leapcobotright()
        msg_right.is_right_hand = li.get_is_right_hand()                     # Right hand detected
        msg_right.right_hand_palmpos.x = right_hand_palm_pos_[0]
        msg_right.right_hand_palmpos.y = right_hand_palm_pos_[1]
        msg_right.right_hand_palmpos.z = right_hand_palm_pos_[2]

        msg_right.right_hand_fist = li.get_right_hand_fist()                 # Fist gesture recognize
        msg_right.right_hand_thumb_up = li.get_right_hand_thumb_up()         # Thumb up gesture recognize
[...]
        pub_ros_right.publish(msg_right)
        rospy.sleep(rospy.get_param(PARAMNAME_FREQ_ENTIRE, FREQUENCY_ROSTOPIC_DEFAULT))

```

<a name="fase5">
  <h2>
Fase 5: Integración de <i>Leap Motion</i> en el sistema
  </h2>
</a>

Para la integración de *Leap Motion* en el sistema de *ROS* y formar parte de la solución diseñada es muy sencillo, simplemente en el *script* que controle los movimientos del cobot este debe estar suscrito al *topic* `leapmotion/data` (en el caso para dos cobots, uno de los scripts se suscribirá al *topic* que envíe información de la mano derecha y otro, información de la mano izquierda) y emplee esa de entrada de datos adecuadamente para gestionar los movimientos del cobot.


En la imagen se muestra la arquitectura del sistema mediante nodos y *topic*s, se puede apreciar la incorporación del *Leap Motion* en el sistema. En la parte inferior de la imagen, se aprecia el nodo *sender* (`one_arm_no_moveit_lm_pub`), el *topic* que publica los datos que se obtiene del dispositivo de *Leap Motion* (`/leapmotion/data`) y el *script* que procesa la información para enviar órdenes al robot es el nodo `ur10_dual_moveit`.

![image](/doc/imgs_md/one-arm-moveit-rqt-graph-gazebo-moveit-fase-4-detalle.png  "Integración de Leap Motion en el sistema")



### Incorporación del dispositivo Leap Motion al sistema `sin` el paquete de `MoveIt!`
- [Un UR10 con pinza mediante un planificador propio y Leap Motion](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/no_moveit/ESP/one_arm_no_moveit_lm.md)
- [Dos UR10s con pinzas mediante un planificador propio y Leap Motion](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/no_moveit/ESP/two_arm_no_moveit_lm.md)


### Incorporación del dispositivo Leap Motion al sistema `con` el paquete de `MoveIt!`
- [Un UR10 con pinza mediante el paquete `MoveIt!` y Leap Motion](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/moveit/ESP/one_arm_moveit_lm.md)
- [Dos UR10s con pinzas mediante el paquete `MoveIt!` y Leap Motion](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/moveit/ESP/two_arm_moveit_lm.md)


---

<div>
<p align="left">
<button name="button">
            	<a rel="license" href="https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/README_ESP.md">Inicio</a>
</button>
</p>
</div>