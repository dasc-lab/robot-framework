---
layout: default
title:  "Vision Drone Complete Guide"
date:   2023-01-13
math: katex
parent: Vision Drone
nav_order: 0
---

# Vision Drone Complete Guide
This is a complete guide about building and operating the vision drone from scrach.

# Frame and Hardware Assembly

## ESC Assembly
Prepare the following items:


![](./imgs/VisionDrone2.jpeg)


Solder the capacitor, power cable into the ESCs. **Pay attention to the front and back of the ESC.**


![](imgs/VisionDrone3.jpeg)


Prepare the following items:


![](imgs/VisionDrone4.jpeg)


Solder the power cable.


![](imgs/VisionDrone5.jpeg)


Install the ESC into the frame with M2.5*20 screw. **Pay attention to the front and back of the ESC.**


![](imgs/VisionDrone6.jpeg)


Install arms.


![](imgs/VisionDrone7.jpeg)


Install motors.


![](imgs/VisionDrone8.jpeg)


Solder motor wire to ESC.


![](imgs/VisionDrone9.jpeg)


## Pix32 Preparation
**Before assembly: Upload custom firmware to Pix32. Calibrate the Acc, Gyro and compass.**


Insert the nut into the Pix32 stands. **Set iron to 200 degrees**
![](imgs/VisionDrone1.jpeg)


Prepare the following items:
![](imgs/VisionDrone10.jpeg)


**Due different motor ordering with 4 in 1 ESC and PX4, we need to remap the channel.**
![](imgs/VisionDrone0.png)


ESC wiring table:

| ESC   | CUR | NC | 4 | 3 | 2 | 1 | + | - |
| Pix32 | I   | NC | 4 | 2 | 1 | 3 | V | G |


Solder it: **Do not reference to the image, use the wiring table**
![](imgs/VisionDrone11.jpeg)


Install the pix32 support with **M2*12**:
![](imgs/VisionDrone12.jpeg)


Connect the cable:
![](imgs/VisionDrone13.jpeg)


Install into the frame:
![](imgs/VisionDrone14.jpeg)


## Battery stands and Xavier Assembly
Install the battery stands, **replace bottom M3 strew with M3*16**:
![](imgs/VisionDrone16.jpeg)


Install Xavier support:
![](imgs/VisionDrone17.jpeg)


Install Xavier:
![](imgs/VisionDrone18.jpeg)


## Xavier setup guide
Install WiFi module:
![](imgs/VisionDrone20.jpeg)


Install Xavier module:
![](imgs/VisionDrone21.jpeg)


# PX4 Configuration Guide
## PX4 Parameter
- Set `MAV_SYS_ID` to be a different value from other Drone (match number with name of drone)
- Set `Hold` and `Offboard` for `COM_RCL_EXCEPT`
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

## Setup Dshot
- set `SYS_USE_IO` to `IO_PWM_DISABLE`
- set `DSHOT_CONFIG` to `DShot600`
- Verify motor order using Mavlink console `dshot beep1 -m [Motor number]`
- Verify motor spinning direction
- Use following command to reverse motor `dshot reverse -m [Motor number]`, `dshot save -m [Motor number]`





