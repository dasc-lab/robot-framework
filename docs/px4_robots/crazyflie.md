- Connect crazy Radio PA to laptop
  - 2 for 8 quads
  - 


- crazyswarm/ros_ws/src/crazyswarm/launch/allCrazyflies.yaml
  -  provide all info for each crazyflie
  -  channel for number (id) of the radio attached

- crazyswarm/ros_ws/src/crazyswarm/scripts:
  - ```python3 chooser.py  ```
  - select/deslect quads

  - crazyswarm/ros_ws/src/crazyswarm/launch/hover_swarm.launch
  - motion capture hostname: 192.168.0.149 (IP address of mocap PC)4



Test vicon only
- Set mocap IP address in crazyswarm/ros_ws/src/crazyswarm/launch/mocap_helper.launch

Kill chooser


For experiment:
- roslaunch crazyswarm hover_swarm.launch
- python3 src/crazyswarm/scripts/hellow_world.py (M shape)
- python3 src/crazyswarm/scripts/land_all.py
- 
    
 
Control LED lights

Robot_Mode.py
scripts/pycrazyswarm/crazyflie.py




For sim
- python3 led_colors.py --sim
