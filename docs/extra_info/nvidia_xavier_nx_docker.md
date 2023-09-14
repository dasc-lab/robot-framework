---
layout: default
title:  "Nvidia Xavier NX + Docker + Realsense + ROS2 Setup"
date:   2023-06-24
math: katex
parent: Extra Info
---

# Nvidia Xavier NX + Docker + Realsense + ROS2 Setup

It was not straightforward to get good performance out of the xavier NX + Realsense. Here is a summary of some findings:

Using these tricks I was able to get upto about 50fps depth images on the xavier. 

## Check this out:
See https://gist.github.com/andrewssobral/ae77483b8fa147cce98b5b92f1a5ae17 for an absolutely incredible list of tips and tricks for the xavier.

## Xavier Tuning
- Make sure you setup an NVME SSD on the Xavier
- ensure you have a good wifi connection
- Max out the jetson:
    - `sudo nvpmodel -m 0` or `sudo nvpmodel -m 5`: this sets the power mode of the xavier.  See the comparison of the modes here: https://docs.nvidia.com/jetson/archives/r34.1/DeveloperGuide/text/SD/PlatformPowerAndPerformance/JetsonXavierNxSeriesAndJetsonAgxXavierSeries.html#supported-modes-and-power-efficiency (essentially 0 is good for GPU stuff and 5 is good for CPU stuff)
    - `sudo jetson_clocks`: this forces each core to operate at their max frequency all the time (this consumes more power, but probably keeps the xavier ready for compute intense ops)
- Install `jtop` and monitor usage. in particular, make sure that when you expect the GPU to be used, it does infact get used. 
- The GUI on the xavier must be somewhat expensive for the xavier to keep and update. Try going headless.  
- Increase the max memory size on ros2 buffers: `sudo sysctl -w net.core.rmem_max=2147483647`

## Docker Tuning
- Use nvidia provided docker setup: https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common (see example usage in https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox). You can add another Dockerfile layer to add stuff you want.
- I suspect nvidia isaac team has figured out magic settings to make the realsense work better than if you just directly used a docker container. Perhaps this is just as simple as enabling GPU, but I suspect theyve also tweaked udev settings or maybe some ros dds settings. havent looked into it much, except for accepting that the nvidia docker magically works better.
- check that `nvcc -V` gives outputs that indicate that CUDA is available on the system

## Realsense Tuning
- set the image resolution desired: `depth_module.profile` and `rgb_color.profile` (e.g. to `640x480x15`)
- use the `decimation filter` in the realsense to reduce the resolution of the pointcloud
- I didnt find the other filters to work well.
- set `clip_distance` to something appropriate like 8.0m
- set the `qos` settings on all the fields you care about, usually to `SENSOR_DATA`
- set `initial_reset` to `True` (not sure why but it seems to be some sort of internal hardware reset)
- disable all the unused streams
- make sure `image_transport` is installed in the docker env. this gives access to a compressed image  that can be used when communicating over the wifi
- running `ros2 run realsense2_camera realsense2_camera_node` produces images with greater fps than `ros2 launch realsense2_camera rs_camera.launch.py`. not sure why, but maybe this is a hint: https://github.com/IntelRealSense/realsense-ros/issues/2507#issuecomment-1411214372

## Networking + Wifi Tuning
- some have suggested that cycloneDDS works better than FastDDS over wifi.
- apparently `netload` from `apt-get install netdiag` can help you see how much load there is on the network
- use `ComposableNodes` to reduce the inter-process memory usage: see
    - (to see what they are) 
      - https://docs.ros.org/en/iron/Concepts/Intermediate/About-Composition.html
      - https://docs.ros.org/en/galactic/Tutorials/Intermediate/Composition.html
      - https://roscon.ros.org/2019/talks/roscon2019_composablenodes.pdf
    - (to see how to launch them) https://docs.ros.org/en/galactic/How-To-Guides/Launching-composable-nodes.html

## General
- dont forget to use CMake optimizations like `-O3`! this can be a really signficant bump in performance essentially for free (compilation time increases though)
    - in any `CMakeLists.txt` you can add the lines
    ```
    if(NOT CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE "Release") or "Debug" if you want to lower compile time but increased runtime
    endif()
    ```
    - generally you are supposed to do this from command line, i.e., run `cmake -DCMAKE_BUILD_TYPE=Release ..` but whos got time for that?
- in the future I want to look into cross-compilation methods since the Xavier is really (really) slow at compiling code

## Boosting Xavier Speed
The power modes on the xavier NX are designed with power efficiency in mind. But we want performance. We want to unlock all the cores at their max clock speeds. To do this, we just need to define one more power mode. 

Modify `/etc/nvpmodel.conf`.  Copy one of the power modes, give it a new id and name, and change whatever settings you want. in the case of the xavier NX 16GB, I settled on

```
< POWER_MODEL ID=9 NAME=MODE_MAX_ALL >
CPU_ONLINE CORE_0 1
CPU_ONLINE CORE_1 1
CPU_ONLINE CORE_2 1
CPU_ONLINE CORE_3 1
CPU_ONLINE CORE_4 1
CPU_ONLINE CORE_5 1
TPC_POWER_GATING TPC_PG_MASK 1
GPU_POWER_CONTROL_ENABLE GPU_PWR_CNTL_EN on
CPU_DENVER_0 MIN_FREQ 1190400
CPU_DENVER_0 MAX_FREQ 1907200
CPU_DENVER_1 MIN_FREQ 1190400
CPU_DENVER_1 MAX_FREQ 1907200
CPU_DENVER_2 MIN_FREQ 1190400
CPU_DENVER_2 MAX_FREQ 1907200
GPU MIN_FREQ 0
GPU MAX_FREQ 1109250000
GPU_POWER_CONTROL_DISABLE GPU_PWR_CNTL_DIS auto
EMC MAX_FREQ 1866000000
NAFLL_DLA MAX_FREQ 1100800000
NAFLL_DLA_FALCON MAX_FREQ 640000000
DLA0_CORE MAX_FREQ 1100800000
DLA1_CORE MAX_FREQ 1100800000
DLA0_FALCON MAX_FREQ 640000000
DLA1_FALCON MAX_FREQ 640000000
NAFLL_PVA_VPS MAX_FREQ 819200000
NAFLL_PVA_CORE MAX_FREQ 601600000
PVA0_VPS0 MAX_FREQ 819200000
PVA0_VPS1 MAX_FREQ 819200000
PVA1_VPS0 MAX_FREQ 819200000
PVA1_VPS1 MAX_FREQ 819200000
PVA0_AXI MAX_FREQ 601600000
PVA1_AXI MAX_FREQ 601600000
CVNAS MAX_FREQ 576000000
NVDEC MAX_FREQ 793600000
NVDEC1 MAX_FREQ 793600000
NVENC MAX_FREQ 729600000
NVENC1 MAX_FREQ 729600000
NVJPG MAX_FREQ 460800000
SE1 MAX_FREQ 704000000
SE2 MAX_FREQ 704000000
SE3 MAX_FREQ 704000000
SE4 MAX_FREQ 704000000
VDDIN_OC_LIMIT WARN 4000
VDDIN_OC_LIMIT CRIT 5000
```

where mainly I changed it so all cores are active:
```
CPU_ONLINE CORE_i 1
```
and the clock speed is maximized.
```
CPU_DENVER_i MIN_FREQ 1190400
CPU_DENVER_i MAX_FREQ 1907200
```
and the GPU max frequency is also maximized:
```
GPU MIN_FREQ 0
GPU MAX_FREQ 1109250000
```

To use this power mode, in the cmd line type
```
sudo nvpmodel -m 9
```
or change the last line of the `/etc/nvpmodel.conf` to make `9` the default.

To check that the power mode is set correctly, type
```
nvpmodel -p --verbose
```
or
```
nvpmodel -p --verbose | grep POWER_MODEL
```
to list just the power modes. 

Also dont forget about jetson clocks and the fan modes. 

References:
- https://forums.developer.nvidia.com/t/cpus-usage-problem-solved/65993
- https://developer.ridgerun.com/wiki/index.php/Xavier/JetPack_5.0.2/Performance_Tuning/Maximizing_Performance
- https://cdn.alliedvision.com/fileadmin/content/documents/products/software/software/embedded/Optimizing-Performance-Jetson_appnote.pdf
- https://docs.nvidia.com/jetson/archives/l4t-archived/l4t-3261/index.html#page/Tegra%20Linux%20Driver%20Package%20Development%20Guide/power_management_jetson_xavier.html#wwpID0E0XO0HA


## Troubleshooting:
- https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/docs/troubleshooting.md#realsense-driver-doesnt-work-with-ros2-humble
- the intel realsense-ros github page has a ton of issues and many people suggesting fixes