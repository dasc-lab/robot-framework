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
        set(CMAKE_BUILD_TYPE "Release") or "Debug" if you want to lower compile time and increase runtime
    endif()
    ```
    - generally you are supposed to do this from command line, i.e., run `cmake -DCMAKE_BUILD_TYPE=Release ..` but whos got time for that?
- in the future I want to look into cross-compilation methods since the Xavier is really (really) slow at compiling code


## Troubleshooting:
- https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/docs/troubleshooting.md#realsense-driver-doesnt-work-with-ros2-humble
- the intel realsense-ros github page has a ton of issues and many people suggesting fixes