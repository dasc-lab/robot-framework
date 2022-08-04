---
layout: default
title:  "Laptop Setup"
date:   2022-05-09
math: katex
has_children: true
nav_order: 12
---

# Step 1: Pulling Docker Container

On the laptop you are using to interface with the rovers, run

```
docker run -it --privileged --net=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --name=px4_vicon_bridge_display chenrc98/vicon_px4_ros2_bridge:version1.1
```

That's all that's needed from the laptop side
