---
title: "Leap Motion's setup"
permalink: /docs/lm-setup/
excerpt: "Proper installation and configuration of the Leap Motion driver in ROS Kinetic Kame requires a bit more work than in the previous repositories."
toc: true
---

## *Leap Motion* Settings
*ROS* Driver for *Leap Motion* Controller 

Proper installation and configuration of the *Leap Motion* driver in *ROS Kinetic Kame* requires a bit more work than in the previous repositories. 

First, you need to replace or create the file that serves the driver: ***/lib/systemd/system/leapd.service***

```bash
# Founded by Kevin Cole 2014.11.22 at
# https://github.com/atejeda/leap-fedora-rpm
#
# Remember to:
#
# ln -s /lib/systemd/system/leapd.service /etc/systemd/system/leapd.service
# systemctl daemon-reload
# 
[Units]
Description=LeapMotion Daemon
After=syslog.target 
[Service]
type=simple
ExecStart=/usr/sbin/leapd 
[Install]
WantedBy=multi-user.target
``` 

A direct access to the created file is created, which is stored in */etc/systemd/system/* as *leapd.service*:

```bash
sudo ln -s /lib/systemd/system/leapd.service /etc/systemd/system/leapd.service
sudo systemctl daemon-reload
```
## Installation of *Leap Motion* 

Clone the repository:
```bash
cd ~/tfg_multirobot/src
git clone https://github.com/ros-drivers/leap_motion.git
``` 

In the installation steps it says to move the `LeapSDK` directory to the `$HOME` directory, but it will stay in the original repository and change the `PATHS` accordingly. 

```bash
# 64-bit operating system
echo "export PYTHONPATH=$PYTHONPATH:$HOME/tfg_multirobot/src/leap_motion/LeapSDK/lib:$HOME/tfg_multirobot/src/leap_motion/LeapSDK/lib/x64" &gt; &gt; ~/.bashrc
source ~/.bashrc
``` 

Installing the *Leap Motion* *ROS* package: 

```bash
sudo apt-get install ros-kinetic-leap-motion
``` 

Install all missing dependencies for *ROS Kinetic Kame*:

```bash
cd ~/tfg_multirobot
rosdep update
rosdep install --rosdistro kinetic --ignore-src --from-paths src
``` 

Compile:
```bash
cd ~/tfg_multirobot
catkin_make
``` 

If errors occur when using the *Leap Motion* *driver*, it is usually sufficient to restart the service:

```bash
sudo service leapd restart
``` 