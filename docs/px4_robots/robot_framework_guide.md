---
layout: default
title:  "8. Using Robot Framework for Rover3"
date:   2022-05-09
math: katex
parent: Px4 Robot
nav_order: 8
---

This page explains on how to get started with using rover3 that is completely set up.

# Step 1: Power-up and connect to rover
Power up the rover by connecting the LiPo battery (voltage should be more than 10.7 ideally). Use DASC2 laptop and boot into it's Ubuntu 20 (the default one when you power up the lapotp so nothing to change there). Make sure the laptop is connected to drone-5G-5.2 Wifi (this is the big router). On DASC2, open a terminal and do
```
ssh rover3@rover3.local
```
When prompted for password, use 'hello123'.

Note: In case you need to open multiple terminals on laptop and or jetson, I recommend using tmux. tmux allows you to split a single terminal into multiple panes each of which acts as an independent terminal and you won't have to ssh again from another terminal. You can see some guides on using tmux or open multiple terminals directlt if need for subsequent steps.

# Step 2: Connect to QGroundControl(QGC)
This rover's pixhawk has the telemetry module attached to it. Just connect the other telemetry module to laptop with the usb cable and open QGC(on desktop) to connect to pixhawk. QGC will only be used for monitoring usually. The parameter values and other setup has already been done and it doesn't play any necessary role in doing the experiment but it is still good to keep it open. It can show errors like 'high accelerometer or magnetometer bias' that show up once in a while and prevent arming the robot. In this particular case, you just need to do sensor calibration again using QGC.

# Step 2: Start docker contaimners on Jetson and Laptop
We only work inside dockers. 
- Laptop: open a terminal and do 
```
 sudo docker start px4_vicon_bridge_display
```
This starts a docker container. This step needs to be done only once. Now to run and get inside the docker environment do
```
sudo docker exec -it px4_vicon_bridge_display bash
```
This will get you inside docker environment. The default location is root. You can go to home by just doing `cd` or `cd ~`. Not needed though.
- Jetson: in the terminal that you started with ssh, do
```
sudo docker start px4_ros_bridge
```
then do 
 ```
 sudo docker run -it px4_ros_bridge bash
 ```

# Step 3: start micrortps agent on Jetson
micrortps is the ros program that communicates with pixhawk and let us recieve and send data to and from Jetson. To start this run the following command on Jetson
```
micrortps_agent -d /dev/ttyTHS2 -b 921600 "rover3"
```
`/dev/ttyTHS2` is the UART port on Jetson to which telem2 of pixhawk has been connected. `921600` is the baud rate and `rover3` is the namespace we want to use.

I have made as alias in `~/.bashrc` so instead on writing the above long command everytime you start Jetson, you can also just write
```
bridge
```

# Step 4: start vicon to px4 node on Laptop
Inside the docker container on laptop, start the vicon node with following command
```
ros2 launch vicon_px4_bridge bridge_launch.py
```
Check that after doing this that the px4's location and yaw angle is same as vicon position and angle. The simplest way to do so is to check the yaw/heading angle on dashboard in Qgc home screen. It should change properly as you rotate rover around. Otherwise, you can do ros2 topic echo for vicon topic and local_position topic and see that they match.

# Step 5: 
Start running the rover around. The whole framework is in the robot-framework folder in src of ros workspace. You can edit any file by opening it in the terminal directly or use vs code to acces the docker containerd and all its files.

The whole framework and a sample code is inside a single file right now whose location is
```

```
In order to run this node, do
```
ros2 run dasc_robot dascRobot
```

Note: the folder name is robot-framework but the ros package name is dasc_robot. Hence the above command.

The cuurent code makes the rover go to location (x=1,y=0) from wherever it is. The only thing you need to modify is the `main()` function according to your application. Hence, I will go over it below:








<!-- # Old(Incomplete) documentation from here on. 
# Framework
Make sure to make some position/velocity commands before commanding to arm

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
-->
