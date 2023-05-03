---
layout: default
title:  "Extra Info"
date:   2022-05-09
math: katex
has_children: true
has_toc: true
nav_order: 2
---
# EXTRA INFO (Not needed for setup)

## Docker + ROS guide:
Here is an awesome article:
	https://roboticseabass.com/2021/04/21/docker-and-ros/

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

## Troubleshooting

### [RTPS_TRANSPORT_SHM Error] Failed to create segment
See https://github.com/eProsima/Fast-DDS/issues/1606
See https://github.com/eProsima/Fast-DDS/issues/2790
the fix is to run
```
fastdds shm clean
```

### Can't ssh unless computer is logged in
The problem could be that the computer cant access the wifi network unless the user is logged in. We can disable this as follows:

Navigate to `/etc/NetworkManager/system-connections/`. There you will find a file with the same name as your wireless network. This file contains your wifi credentials and settings. Edit it, find the line with permission=, and remove everything after the = sign (or the whole line).

Restart and you can connect before login.

This was necessary on the Xavier NX. 

## converting git submodules
When making a parent repo a git repo, and making all the nested folders into submodules:
1) from the root of the parent repo run
```
git submodule add <new-url> <path/to/nested/repo>
```

2) run 
```
git submodule update
```

3) go to the nested repo folder
```
git remote set-url <new-url>
```

4) add, commit and push changes

5) go back to parent repo, add, commit and push changes. 


Little trick:
from the parent repo:
```
export REPO="<nested/repo/path">
git submodule add $(cd $REPO && git remote get-url origin) $REPO
```
