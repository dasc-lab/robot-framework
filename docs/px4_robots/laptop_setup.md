---
layout: default
title:  "Laptop Setup"
date:   2022-05-09
math: katex
has_children: true
nav_order: 12
---

## Step 1: Pulling Docker Container

On the laptop you are using to interface with the rovers, run

```
docker run -it --privileged --net=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --name=px4_vicon_bridge_display chenrc98/vicon_px4_ros2_bridge:version1.1
```

That's all that's needed from the laptop side


## Optional:

If you want to avoid typing the ssh password everytime you log into a robot, you can run the following steps to save the password:

```
ssh-keygen -t ed25519
```
complete the prompts

```
ssh-copy-id -i ~/.ssh/id_25519.pub ubuntu@drone7.local
```
complete the prompts. 

Now you can simply ssh into a quad, and not need to type in the password.

More details: https://superuser.com/questions/8077/how-do-i-set-up-ssh-so-i-dont-have-to-type-my-password
