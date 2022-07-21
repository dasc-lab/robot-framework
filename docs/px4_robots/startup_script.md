---
layout: default
title:  "Startup Script Tutorial"
date:   2022-05-09
math: katex
has_children: true
nav_order: 13
---

This tutorial will go over how to make the startup script for the 

# Step 1: The Script 
In /usr/local/bin, create a shell script named mav_startup.sh, and inside it put:

```
#! /bin/bash
#
sudo docker start ros2_px4_bridge
sudo docker exec -i ros2_px4_bridge bash -c 'mavlink-routerd'

```
The first line will start the docker, and the second one will execute the mavlink connection inside the docker

# Step 2: Making a Service File
In /etc/systemd/system create a file called mav_startup.service and inside it put:

```
[Unit]
After=network.target
[Service]
ExecStart=/usr/local/bin/mav_startup.sh

[Install]
WantedBy=multi-user.target
```

The ```After=network.target``` tag makes it so that the service runs after network services, which is normally the last one. The ```ExecStart=/usr/local/bin/mav_startup.sh``` tells the service to execute the script we created in step 1. ```WantedBy=multi-user.target``` Allows for the widest permissions.

# Step 3: Enabling Service 

After creating modify the file permissions with
```
sudo chmod 744 /usr/local/bin/mav_startup.sh
sudo chmod 664 /etc/systemd/system/mav_startup.service
```

Then reload systemctl daemon and enable the service to run on startup with 
```
sudo systemctl daemon-reload
sudo systemctl enable mav_startup.service
```

The script should now run on startup!

If you want to test the service you can start it with 
```
sudo systemctl start mav_startup.service
```
and stop it with
```
sudo systemctl stop mav_startup.service
```

# Troubleshooting

If the service fails to run, you can use
```
systemctl status mav_startup.service
```
to see what the status of the execution was
```
systemctl --failed
```
also may help.

# Removing the Service

To remove the service first stop it, then disable it before removing the files using
```
systemctl stop mav_startup.service
systemctl disable mav_startup.service
```

Then remove the service and script files with 
```
rm /etc/systemd/system/mav_startup.service
rm /usr/lib/systemd/system/mav_startup.service
```

Reset systemctl with
```
systemctl daemon-reload
systemctl reset-failed
```