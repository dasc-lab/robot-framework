---
layout: default
title:  "6. Rover Setup"
date:   2022-05-09
math: katex
parent: Px4 Robot
nav_order: 6
---

## Pixhawk Firmware Setup
Flash the custom firmware px3_fmu-v3_default.px4 as the firmware onto a cube black
set the vehicle frame to generic ground rover. This firmware can be built from the PX4 repo by using
```
cd boards
cd px4
cd fmu-v3
make px4_fmu-v3_default upload
```


## Pi Setup

NOTE: to ssh into pi using ubuntu 20.04 server, you need to first run ```sudo apt-get install libnss-mdns```

The setup is the same as the small quad setup, but being connected to drone-5G instead of swarm-5G. 

NOTE: drone-5G does not have internet, so when setting up, use swarm-5G, and when testing go into /etc/netplan/50-cloud-init.yaml and edit it back to drone-5G. 

There should be a voltage converter connected to the roboclaw to connect to the pi power, and there should be a power connector to connect to the pixhawk cube black.

# TX2 Setup

## Firmware
Make sure jetpack 4.6 is installed (can be installed using DASC 1) using the nvidia jetpack installer on one of the computeres.

## Wiring
Connect the black wire to pin 1 (GND), which is closest to the edge of the board. Connect the yellow to pin 4 (RX) and the green wire to pin 5 (TX). The other end goes into TELEM 2 of the cube black. 

To wire to the Pixhawk side the white wire goes into the top of 1, the purple goes to the bottom of two and the grey goes into the bottom of 3. The roboclaw side, purple goes into s2, and the grey goes into s1 below it, while white goes into left side of s1.  

## Remoting into the Rover
Since most of the software setup is done on the rover directly, you can remote into the rover using
```
ssh rover[number]@rover[number].local
```
or
``` 
ssh rover[number]@ubuntu.local (rover2)
```

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
micrortps_agent -d /dev/ttyTHS2 -b 921600 -n "namespace"
```

The namespace is set on the vehicle itself for later use

This part in the “microRTPS_transport.cpp” is deleted in this version
```
        serial_ctl.flags |= ASYNC_LOW_LATENCY;

		if (ioctl(_uart_fd, TIOCSSERIAL, &serial_ctl) < 0) {
			int errno_bkp = errno;
			printf("\033[0;31m[ micrortps_transport ]\tError while trying to write serial port latency: %d\033[0m\n", errno);
			close();
			return -errno_bkp;
		}
```

# Operation

## Indoor
When using the rover indoor, we need to convert the vicon messages into gps messages for the pixhawk using vicon_px4_bridge, and PX4 into ros commands using ros2_vicon_receiver. 

These libraries can both be pulled from a docker container into the operating laptop
```
sudo docker pull chenrc98/vicon_px4_ros2_bridge
```
once inside 


## Outdoor
Outdoor operation does not need any additional setup, and you can directly start using the robot framework