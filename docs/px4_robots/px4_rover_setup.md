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

## TX2 Setup