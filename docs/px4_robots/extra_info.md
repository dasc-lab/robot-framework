---
layout: default
title:  "Extra info"
date:   2022-05-09
math: katex
parent: Px4 Robot
nav_order: 10
---
# EXTRA INFO (Not needed for setup)

## Indoor
When using the rover indoor, we need to convert the vicon messages into gps messages for the pixhawk using vicon_px4_bridge, and PX4 into ros commands using ros2_vicon_receiver. 

These libraries can both be pulled from a docker container into the operating laptop
```
sudo docker pull chenrc98/vicon_px4_ros2_bridge
```
once inside 


## Outdoor
Outdoor operation does not need any additional setup, and you can directly start using the robot framework

## Pi Setup

NOTE: to ssh into pi using ubuntu 20.04 server, you need to first run ```sudo apt-get install libnss-mdns```

The setup is the same as the small quad setup, but being connected to drone-5G instead of swarm-5G. 

NOTE: drone-5G does not have internet, so when setting up, use swarm-5G, and when testing go into /etc/netplan/50-cloud-init.yaml and edit it back to drone-5G. 

There should be a voltage converter connected to the roboclaw to connect to the pi power, and there should be a power connector to connect to the pixhawk cube black.

## micro_rtps for rover
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