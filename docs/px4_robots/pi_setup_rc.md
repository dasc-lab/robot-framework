---
layout: default
title:  "Ruichang's Pi Setup"
date:   2022-05-09
math: katex
has_children: true
nav_order: 9
---

## flash pi sd card
use imager and install Ubuntu Server 20.04.4 LTS
​
edit the network-config file in system-boot add the following code:
```
wifis:
  wlan0:
    dhcp4: true
    optional: true
    access-points:
      "swarm-5G": {}
```
​
## boot the pi with sd card
​
Warning:
​
During your first boot a tool called cloud-init is doing configuration. WAIT for it to finish before trying to log in. It typically takes less than 2 minutes but there is a break between the log-in prompt and cloud-init completing. If you interrupt the process you have to start again. You’ll know it’s done when it outputs some more lines after the log-in prompt has appeared.
​
login with ubuntu and password ubuntu
​
​
## install docker into pi
​
https://docs.docker.com/engine/install/ubuntu/
​
## enable uart port on pi
over pi terminal
```
sudo nano /boot/firmware/usercfg.txt
```
add the following line
```
dtoverlay=uart4
dtoverlay=uart5
```
reboot the pi
​
## start ros2-pi bridge docker image:
only need to run this command once.
```
sudo docker run -it --privileged --net=host --name=ros2_px4_bridge chenrc98/ros2-px4-pi:version1.1 bash
```
To went back to the docker image, type:
```
sudo start ros2_px4_bridge
sudo exec -it ros2_px4_bridge bash
```
## uart connection between px4 and pi
​
uart4 TXD 8(pin 24), RXD 9(pin 21), CTS 10(pin 19), RTS 11(23)
​
uart5 TXD 12 RXD 13
​
## start px4 bridge
```
cd ~/px4_ros_com_ros2
source install/setup.bash
micrortps_agent -d /dev/ttyAMA1 -b 921600
```
​
## start mavlink-router
```
mavlink-routerd
```
the config file is in ` /etc/mavlink-router/main.conf`
​
​
## custom firmware for PX4
​
In PX4 firmware folder, navigate to boards/cubepilot/cubeorange create file rtps.px4board by
```
touch rtps.px4board
```
and add following line:
```
CONFIG_MODULES_MICRORTPS_BRIDGE=y
```
build the firmware by
```
make cubepilot_cubeorange_rtps
```
QGC parameters
```
RTPS_CONFIG -> TELEM 2
```
​
​
https://www.ardusimple.com/send-ntrip-corrections-to-ardupilot-with-missionplanner-qgroundcontrol-and-mavproxy/