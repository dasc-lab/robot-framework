---
layout: default
title:  "6. Rover Setup"
date:   2022-05-09
math: katex
parent: Px4 Robot
nav_order: 6
---

## Pixhawk Setup
Flash fmu-v3 as the firmware onto a cube black
set the vehicle frame to generic ground rover


## Pi Setup

NOTE: to ssh into pi using ubuntu 20.04 server, you need to first run ```sudo apt-get install libnss-mdns```

The setup is the same as the small quad setup, but being connected to drone-5G instead of swarm-5G. 

NOTE: drone-5G does not have internet, so when setting up, use swarm-5G, and when testing go into /etc/netplan/50-cloud-init.yaml and edit it back to drone-5G. 

There should be a voltage converter connected to the roboclaw to connect to the pi power, and there should be a power connector to connect to the pixhawk cube black.

# TX2 Setup

## Wiring
Connect the black wire to pin 1 (GND), which is closest to the edge of the board. Connect the yellow to pin 4 (RX) and the green wire to pin 5 (TX). The other end goes into TELEM 2 of the cube black. 

To wire to the roboclaw, 

## Firmware 
Use the firmware px3_fmu-v3_default.px4

## Software 

pull the docker container for the rover by using the command 
```
sudo docker run -it --privileged --net=host --name=ros2_px4_bridge chenrc98/ros2-px4-pi:rover1.4
```

In the container change folders and source
```
cd px4_ros_com/
source install/setup.bash
```

To start the gps for the rover, run the command
```
micrortps_agent -d /dev/ttyTHS2 -b 921600
```

This part in the “microRTPS_transport.cpp” is deleted in this version
'''
        serial_ctl.flags |= ASYNC_LOW_LATENCY;

		if (ioctl(_uart_fd, TIOCSSERIAL, &serial_ctl) < 0) {
			int errno_bkp = errno;
			printf("\033[0;31m[ micrortps_transport ]\tError while trying to write serial port latency: %d\033[0m\n", errno);
			close();
			return -errno_bkp;
		}
'''