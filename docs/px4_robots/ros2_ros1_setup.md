---
layout: default
title:  "Ros2-Ros1 setup"
date:   2022-05-09
math: katex
has_children: true
nav_order: 14
---

# Pulling the container
Ros2 has a repository for bridging ros1 topics to ros2 and vice versa. This repo requires being able to run Ros2, and has numerous steps to implementing, so with docker and docker compose, it is far easier to just run the bridge container, which can be pulled with

```
sudo docker pull ros:galactic-ros1-bridge
```

# Running the bridge
Start the container with docker run, and once inside the container, run the command 
```
ros2 run ros1_bridge dynamic_bridge --bridge-all-1to2-topics  --bridge-all-2to1-topics
```

# How to implement in a docker-compose file

Just include this chunk of code in your docker compose file to have a properly working bridge container:

```
bridge:
    image: ros:galactic-ros1-bridge
    environment:
      # - "ROS_HOSTNAME=bridge"
      - "ROS_MASTER_URI=http://127.0.0.1:11311"
    command: ros2 run ros1_bridge dynamic_bridge --bridge-all-1to2-topics  --bridge-all-2to1-topics
    depends_on:
      - ros1
    network_mode: "host
```