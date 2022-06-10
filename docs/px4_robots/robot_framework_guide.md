---
layout: default
title:  "8. Using Robot Framework"
date:   2022-05-09
math: katex
parent: Px4 Robot
nav_order: 8
---


# Ground Robot

## Container 
Use the container vicon_px4_ros2_bridge:veriosn1.1 (px4_vicon_bridge_display in DASC 2) on the laptop you plan to use. if using indoor make sure to launch the file bridge_launch.py, and afterwards run dasc_robot. These are located in px3_ros_com_ros2. Any changes made to any of the files in px4_ros_com_ros2 requires a colcon build afterwards.

## bridge_launch.py

Before launching bridge_launch.py check the launch file to see if there are nodes for each agent you're planning to use has been added and is using the proper namespace set on the vehicle itself and on vicon's computer. Make sure the Vicon computer is connected to swarm-5G or the appropriate network, and that the IP address specified in the hostname variable matches the vicon computer's ip address. Then launch with the command

```
ros2 launch vicon_px4_bridge bridge_launch.py
```

## dasc_robot

Make sure to run ```colcon build``` after whatever edits you make to the framework. To run the dascRobot node use
```
ros2 run dasc_robot dascRobot
```
