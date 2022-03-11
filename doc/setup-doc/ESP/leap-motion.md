# Instalación y configuración de *Leap Motion*

**Español** | [English](https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/setup-doc/ENG/leap-motion.md)

## Configuración de *Leap Motion*
Driver de *ROS* para el controlador de *Leap Motion*

Para la correcta instalación y configuración del controlador de *Leap Motion* en *ROS Kinetic Kame*, hay que realizar un poco más de trabajo que en los repositorios previos.

Lo primero es reemplazar o crear el fichero que da servicio al controlador: ***/lib/systemd/system/leapd.service***

```bash
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

Se crea el acceso directo al fichero creado, que se guardará en */etc/systemd/system/* como *leapd.service*:

```bash
sudo ln -s /lib/systemd/system/leapd.service /etc/systemd/system/leapd.service
sudo systemctl daemon-reload
```
## Instalación de *Leap Motion*

Clonar el repositorio:
```bash
cd ~/tfg_multirobot/src
git clone https://github.com/ros-drivers/leap_motion.git
```

En los pasos de instalación dice de mover el directorio `LeapSDK` al directorio `$HOME`, pero se va a mantener en el repositorio original y se modificarán los `PATHS` adecuadamente.

```bash
# 64-bit operating system
echo "export PYTHONPATH=$PYTHONPATH:$HOME/tfg_multirobot/src/leap_motion/LeapSDK/lib:$HOME/tfg_multirobot/src/leap_motion/LeapSDK/lib/x64" >> ~/.bashrc
source ~/.bashrc
```

Instalación del paquete de *ROS* de *Leap Motion*:

```bash
sudo apt-get install ros-kinetic-leap-motion
```

Instalación de dependencias que puedan faltar para *ROS Kinetic Kame*:
```bash
cd ~/tfg_multirobot
rosdep update
rosdep install --rosdistro kinetic --ignore-src --from-paths src
```

Compilación:
```bash
cd ~/tfg_multirobot
catkin_make
```

Si surgen errores durante el uso de los *drivers* de *Leap Motion*, con reiniciar el servicio suele ser suficiente:
```bash
sudo service leapd restart
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
    	<a rel="license" href="https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/setup-doc/ESP/install-ros-packages.md">Anterior</a>
    	</button> 
    </span> 
    <span style="float:right;">
        <button name="button">
            	<a rel="license" href="https://github.com/Serru/MultiCobot-UR10-Gripper/blob/main/doc/setup-doc/ESP/pruebas.md">Siguiente</a>
            	</button>
    </span>
</p>
</div>