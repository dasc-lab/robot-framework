---
layout: default
title:  "Raspi Setup"
date:   2022-05-09
math: katex
parent: Px4 Robot
nav_order: 3
---
# Raspi Setup

This page describes how to setup a robot for experiments. 

1. Use Raspi Imager, and flash an sd card with  `Ubuntu Server 20.04.4 LTS`

2. On the sd card, modify the `network-config` file in `system-boot` with the following:
    ```
    wifis:
      wlan0:
        dhcp4: true
        optional: true
        access-points:
          "swarm-5G": {}
    ```
or whatever the name of the wifi. The `{}` indicates that there is no password for this wifi network. 

3. Boot the pi with sd card 

    *Warning:* During your first boot a tool called `cloud-init` is doing configuration. **WAIT for it to finish** before trying to log in. It typically takes less than 2 minutes but there is a break between the log-in prompt and cloud-init completing. If you interrupt the process you have to start again. You’ll know it’s done when it outputs some more lines after the log-in prompt has appeared.

4. Login with username `ubuntu` and password `ubuntu`

5. Enable uart: modify the `/boot/firware/usercfg.txt` and add the following lines:
```
dtoverlay=uart4
dtoverlay=uart5
```

6. Change the hostname: modify the file `/etc/hostname` to 
```
new_hostname
```
and change the first line of `/etc/hosts` to 
```
127.0.0.1 new_hostname
```
In the future, to ssh into this computer, you will run `ssh ubuntu@new_hostname.local`

7. Reboot pi


8. Install docker: see https://docs.docker.com/engine/install/ubuntu/ 

9. Post-install steps for docker: https://docs.docker.com/engine/install/linux-postinstall/

10. Clone `robot-jumpstart`: [`https://github.com/dasc-lab/robot-jumpstart`](https://github.com/dasc-lab/robot-jumpstart)

Follow the instructions in the `readme.md`.

