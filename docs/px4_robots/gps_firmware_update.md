---
layout: default
title:  "GPS Firmware Update"
date:   2022-05-09
math: katex
parent: Px4 Robot
nav_order: 7
---

## GPS Firmware Update

- Needs to be done on Windows
- Open ArduPilot mission planner software
- Install latest firmware of copter to the cube (firmware update for gps needs to be done through an ArduPilot flashed cube)
- Switch COM to COM9 Cube Orange SLCan
- Go to setup optional hardware go to DroneCAN tab
- Click SLCAN Mode CAN1
    - Plug in the here3 gps to the cube orange and press ok
- Once com_hex_here shows up, go to menu and click update
- Follow the instructions of the here3 GPS wiki 
- GO to receiver -> Connection -> network connection -> 
- Open creates a configuration window 
    - In NAV5 settings change the min SV elevation to 13 deg
    - C/n0 set to min 45
- Change Pass through back to zero 
