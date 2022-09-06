---
layout: default
title:  "6. Rover Setup"
date:   2022-05-09
math: katex
parent: Px4 Robot
nav_order: 6
---

# Necessary Hardware

In order to properly run, the rover needs to have the roboclaw, 4 motors, a Pixhawk, and either a Raspberry Pi or a Jetson TX2. If using the Jetson, make sure to also inlude the multi-usb extention, the USB to TTL converter, and the wifi antennas. Make sure to mount the Pixhawk or Pi if using them. The TX2 is already in the rover and is in the bottom. A radio control module is optional, and can be connected to the Pixhawk if RC wants to be used.

# Step 1: Wiring

## 1.1: TX2
Connect the black wire to pin 1 (GND), which is closest to the edge of the board. Connect the yellow to pin 4 (RX) and the green wire to pin 5 (TX). The other end goes into TELEM 2 of the cube black. Attach the USB Hub to the USB Port of the TX2. In the current setup, the Hub is mounted in the front under the lip using a spacer to allow the lid to fit and double sided tape. Plug in the USB to TTL converter into the hub on the USB side and to TELEM 1 with the same type of cable used for TELEM 2.

## 1.2: Pixhawk
To wire to the Pixhawk side the white wire goes into the top of 1, the purple goes to the bottom of two and the grey goes into the bottom of 4. Reference PX4 documentation (https://dev.px4.io/master/en/airframes/airframe_reference.html) to see  The roboclaw side, purple goes into s2 pin, and the grey goes into s1 below it, while white goes into leftmost pin of s1.

# Step 2: Pixhawk Setup

## 2.1: Firmware
Flash the custom firmware px3_fmu-v3_default.px4 as the firmware onto a cube black
set the vehicle frame to generic ground rover. This firmware can be built from the PX4 repo by using
```
cd boards
cd px4
cd fmu-v3
make px4_fmu-v3_default upload
```
## 2.2: QGC Setup
- go to Airframe tab, and select `Generic Ground Vehicle` or whatever the appropriate airframe is
- connect a transmitter, and run the radio setup
- go to Flight Modes, assign the appropriate radio channels to arm, kill switch and mode selection
- go to Safety and check all the settings

## 2.3: QGC Parameters
- Set `MAV_SYS_ID` to be a different value from other Rovers (match number with name of rover)
- Check off Hold and Offboard for COM_RCL_EXCEPT 
- in `EKF2_AID_MASK`: Check `GPS`, `vision position fusion`, and `vision yaw fusion` if outdoor, and **only** `vision position fusion` and `vision yaw fusion` if indoor
	- NOTE: Code for vision position fusion is different whether or note use GPS is checked
- Set `EKF2_HGT_MODE` to GPS if doing outdoor testing and Vision if doing indoor testing
- Set `EKF2_GPS_P_NOISE` to 0.2 m
- Set `EKF2_GPS_V_NOISE` to 0.15 m/s
- set `MAV_0_RATE` to 0 B/s (i.e. unlimited data sending rate)
- set `COM_RC_IN_MODE` set to joystick only
- set `MAV_0_RADIO_CTL` to disable
- set `SER_TEL1_BAUD` to 115200 8N1
- set `RTPS_CONFIG` to TELEM 2
- set `SER_TEL2_BAUD` to 921600 (might need to reboot vehicle to see this option)

## 2.3.2 New QGC Parameters for Rovers
- Set `PWM_MAIN_REV4` to `Enabled`
This is needed to reverse the polarity of PWM output to reflect the changes in new firmware on Roboclaw.
- Set `GND_THROTTLE_MAX` to desired value. current is `50%`

Next, since the rover position control code was modified, the following parameters need to be changed when tuning the position and velocity control modes. The values mentioned here were obtained after tunin(might need more tuning!)g. To get the feedforward terms, the follwing observations were made in manual mode for max throttle: Maximum linear speed: 1.3 m/s(0.75 m in 1 s). Maximum angular velocity: 3.925 rad/s (360 degrees in 1.6 seconds)
- Set `ANG_SPEED_P` to `1.0` <`0.005`>
- Set `ANG_SPEED_I` to `60`
- Set `ANG_SPEED_D` to `0`
- Set `ROVER_K_POS` to `3`
- Set `ROVER_OMG_FF` to `0.254`
- Set `ROVER_VEL_FF` to `0.750`
- Set `GND_SPEED_P` to `1.0?`
- Set `GND_SPEED_I` to `1.0?`
- Set `GND_SPEED_D` to `1.0?`
- Set `GND_SPEED_IMAX` to `1.0?`


## 2.4 PID Tuning

# Step 3: Jetson setup

## 3.1: Firmware
Use the NVIDIA Jetson SDK Manager to install Jetpack 4.6.2. The necessary settings are installed on DASC 1's Ubuntu 18 installation. You will need to create an NVIDIA Developer account using your email to use it. Plug in the Jetson to the DASC via microUSB with the power off or battery disconnected. Hold the REC button and power on the Jetson, holding on the REC button for a while during startup. The SDK Manager should detect the Jetson. Follow the steps on the manager, making sure not to redownload the SDK if it's already on DASC 1, as it takes up a lot of space. 

## 3.2: Setting up Remoting into the Rover
Since most of the software setup is done on the rover directly, you can remote into the rover using
```
ssh rover[number]@rover[number].local
```
or
``` 
ssh rover[number]@ubuntu.local (rover2)
```

To set this up for further rovers, you need to change the profile name and the system name for the rovers.

To change the system name, follow the instructions in (https://www.tecmint.com/set-hostname-permanently-in-linux/), using a command line command and some editing of host files.

To change the hostname to $HOSTNAME (when you ssh, the command you run is `ssh username@hostname.local`):

set the file `/etc/hostname` to 
```
$HOSTNAME
```
then set the first line of `/etc/hosts` to 
```
127.0.0.1 $HOSTNAME
```
Then reboot the computer.

To change the profile name, a temporary user needs to be created, follow the instructions here (https://askubuntu.com/questions/34074/how-do-i-change-my-username).

## 3.3: Docker Setup 
Follow the official instructions from the docker website to install Docker and its necessary components (https://docs.docker.com/engine/install/ubuntu/).

pull the docker container for the rover by using the command 
```
sudo docker run -it --privileged --net=host --name=ros2_px4_bridge chenrc98/ros2-px4-pi:rover1.4
```

or 

```
sudo docker run -it --privileged --net=host --name=ros2_px4_bridge sidpra/ros2-px4-pi:rover1.5
```

This should result in a running container that with full device and network permissions. use ```docker ps``` to check if it is running.

```
sudo docker start ros2_px4_bridge
```
then do 
 ```
 sudo docker exec -it ros2_px4_bridge bash
 ```

To enter the container.

In the container, use 
```
sudo nano ~/.bashrc
```
and add the lines
```
source /opt/ros/galactic/setup.bash
source ~/px4_ros_com/install/setup.bash
```

This will allow users to use ```source ~/.bashrc``` instead of sourcing everything individually.

change into the src folder of px4_ros_com:

```
cd \px4_ros_com\src
```

and clone the robot-framework repo

```
git clone https://github.com/dasc-lab/robot-framework.git
```
and cd into the robot-framework folder and switch to the ground robot branch 
```
git checkout ground_robot
```

change back to the px4_ros_com folder and run 
```
colcon build
```
