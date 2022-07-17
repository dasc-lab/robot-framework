---
layout: default
title:  "8. Using Robot Framework for Rover3"
date:   2022-05-09
math: katex
parent: Px4 Robot
nav_order: 8
---

This page explains on how to get started with using rover3 that is completely set up.

NOTE: Boot the laptop into Ubuntu 20. The instructions have been tested for DASC2 at the time of writing this. DASC1 is yet to be tested.

# Step 1: Power-up and connect to rover
Power up the rover by connecting the LiPo battery (voltage should be more than 10.7 ideally). Use DASC2 laptop and boot into it's Ubuntu 20 (the default one when you power up the lapotp so nothing to change there). Make sure the laptop is connected to drone-5G-5.2 Wifi (this is the big router). On DASC2, open a terminal and do
```
ssh rover3@rover3.local
```
When prompted for password, use 'hello123'.

Note: In case you need to open multiple terminals on laptop and or jetson, I recommend using tmux. tmux allows you to split a single terminal into multiple panes each of which acts as an independent terminal and you won't have to ssh again from another terminal. You can see some guides on using tmux or open multiple terminals directlt if need for subsequent steps.

# Step 2: Start docker containers on Jetson and Laptop
We only work inside dockers. 
- Laptop: open a terminal and do 
```
 sudo docker start px4_vicon_bridge_display
```
This starts a docker container. This step needs to be done only once. Now to run and get inside the docker environment do this on every terminal that you want docker environment on
```
sudo docker exec -it px4_vicon_bridge_display bash
```
This will get you inside the docker environment. The default location is root. You can go to home by just doing `cd` or `cd ~`. Not needed though.
- Jetson: in the terminal that you started with ssh, do
```
sudo docker start ros2_px4_bridge
```
then do 
 ```
 sudo docker exec -it ros2_px4_bridge bash
 ```
 **Note:** If using tmux, start tmux after getting inside the docker environment.


# Step 3: Connect to QGroundControl(QGC)
open QGC(double click on QGC icon inside the QGC folder on desktop on the laptop) to connect to pixhawk. QGC will only be used for monitoring usually. The parameter values and other setup has already been done and it doesn't play any necessary role in doing the experiment but it is still good to keep it open. It can show errors like 'high accelerometer or magnetometer bias' that show up once in a while and prevent arming the robot. In this particular case, you just need to do sensor calibration again using QGC.

## If using Telemetry Module
This rover's pixhawk has the telemetry module attached to it. Just connect the other telemetry module to laptop with the usb cable. None of the rovers use telemetry module as of now.

## If using mavlink-routerd
mavlink-router is a program that runs on Jetson, connects to TELEM1 port of pixhawk and redirects data to the laptop so that QGC can run. The setup includes editing a config file to mention the UART port on Jetson where telem1 of pixhawk is connected and the IP address of the laptop you are working on.
```
nano /etc/mavlink-router/main.conf
```
- For `Device`, use `/dev/ttyUSB0
- For `Baud`, use `115200`
- For `Port`, use `14550`
- For `Address`, use `IP Address of Laptop`
Save the file and exit. Now to start this program write the following in one terminal
```
mavlink-routerd
```

# Step 4: start micrortps agent on Jetson
micrortps is the ros program that communicates with pixhawk and let us recieve and send data to and from Jetson. To start this, run the following command on Jetson
```
micrortps_agent -d /dev/ttyTHS2 -b 921600 -n "rover3"
```
`/dev/ttyTHS2` is the UART port on Jetson to which telem2 of pixhawk has been connected. `921600` is the baud rate and `rover3` is the namespace we want to use.

I have made an alias in `~/.bashrc` so instead on writing the above long command everytime you start Jetson, you can also just write
```
bridge
```

The output is supposed to look like this
```
root@rover3:/# bridge
--- MicroRTPS Agent ---
[   micrortps_agent   ]	Starting link...
[   micrortps_agent   ]	UART transport: device: /dev/ttyTHS2; baudrate: 921600; poll: 1ms; flow_control: No
---   Subscribers   ---
- Timesync subscriber started
- OffboardControlMode subscriber started
- VehicleLocalPositionSetpoint subscriber started
- VehicleCommand subscriber started
- VehicleAttitudeSetpoint subscriber started
- VehicleRatesSetpoint subscriber started
- VehicleOdometry subscriber started
- SensorGps subscriber started
- ManualControlSetpoint subscriber started
- GpsInjectData subscriber started
- TrajectorySetpoint subscriber started
- VehicleVisualOdometry subscriber started
-----------------------

----   Publishers  ----
- Timesync publishers started
- SensorCombined publisher started
- VehicleAttitude publisher started
- VehicleLocalPosition publisher started
- TimesyncStatus publisher started
- VehicleStatus publisher started
- BatteryStatus publisher started
-----------------------

```
**Note:** verify that the data is coming from pixhawk. Sometimes the output is not as displayed above and in that case, you need to stop and run bridge again until you see the complete output as above. It is also possible that the above output is displayed but there is no data (should not happen anymore but what if cables got loose? ). A quick check is to print the sensor data and see if it is there with folloiwing command:
```
ros2 topic echo /rover3/fmu/sensor_combined/out
```
Do this from Jetson docker and form laptop dokcer. in ROS2, any two machines on same network automatically share data with no extra setup! Therefore, even though this ros node was started on Jetson, the data should be visible from laptop too. If it is not, then there are issues. However, it just works usually and you won't face this issue.

# Step 5: start vicon to px4 node on Laptop
Inside the docker container on laptop, start the vicon node with following command
```
ros2 launch vicon_px4_bridge bridge_launch.py 
```
Before running the above command, check the launch file first. You can edit the file in vs code (it can edit files inside docker containers! on both the laptop, opening vs code should open the docker conatiner by default) or use nano
```
nano ~/px4_ros_com_ros2/src/vicon_px4_bridge/launch/bridge_launch.py
```
and then add all the robots you want to get data from vicon for. The launch files in both the laptop only have rover2 and 3 added. For other rovers, you need to add them to the launch file.

**Note**: make sure that rover object exists in the Vicon software! if not, then add markers to the rover and make an object first -> save object ->shared. Also, the IP address in this launch file should be that of the vicon computer. It is set to 192.168.2.136.

Check after doing this that the px4's location and yaw angle is same as vicon position and angle. The simplest way to do so is to check the yaw/heading angle on dashboard in Qgc home screen. It should change properly as you rotate rover around. Otherwise, you can do ros2 topic echo for vicon topic`(/vicon/rover3/rover3)` and local_position topic`(/rover3/fmu/vehicle_local_position/out
)` and see that they match.

**Note**: vicon/ros2 follows ENU (east-north-up) convention for defining x,y,z whereas PX4 follows NED convention. Therefore, vicon and px4 yaw angles re off by 90 degrees. The x,y,z are also not the same. They have the following relationship: `x_NED=y_ENU, y_NED=x_ENU, z_NED=-z_ENU`. Therefore, when comparing the above ros messages, take care of comparing the right components against each other. Even when sending waypoints (keep reading below on how to do that), you send the waypoints in ENU frame and the code converts them to NED behind the scenes.

# Step 6: Send Waypoints

**Note**: The whole framework is in a single file DASCRobots.cpp (anf header file DASCRobots.hpp). This was  built as an library in CMakeLists.txt of the package. This library was then linked to the example_control.cpp that is run below. For other langiages like Pythin and Julia, it suffices to build a wrapper around this library.

Start running the rover around. The whole framework is in the robot-framework folder in src of ros workspace. You can edit any file by opening it in the terminal directly or use vs code to acces the docker container and all its files.

The whole framework and a sample code is inside a single file right now whose location is
```
/root/px4_ros_com_ros2/src/robot-framework/src/DASCAerialRobot.cpp
```
In order to run this node, do
```
ros2 run dasc_robot example_control
```

Note: the folder name is robot-framework but the ros package name is dasc_robot. Hence the above command.

The current code makes the rover go to a location (x=1,y=0) from wherever it is. The only thing you need to modify is the `main()` function according to your application. Hence, I will go over it below:

```
int main(int argc, char* argv[]) {
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
    auto rover3 = std::make_shared<DASCAerialRobot>("rover3", 1);
    
    rover3->init();
    
    rclcpp::executors::MultiThreadedExecutor server_exec;
    server_exec.add_node(rover3);
    
    auto server_spin_exec = [&server_exec]() {
        server_exec.spin();
    };
    std::thread server_exec_thread(server_spin_exec);
    std::cout << "Node init" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    std::cout << "Node start" << std::endl;
```

The above are the steps you need to do for initializing a robot. For multiple robots, you need to write the sames lines again for them. For example, if you have another rover2, then the code would look like this
```
int main(int argc, char* argv[]) {
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
    auto rover3 = std::make_shared<DASCAerialRobot>("rover3", 1);
    auto rover2 = std::make_shared<DASCAerialRobot>("rover2", 1);
    
    rover3->init();
    rover2->init();
    
    rclcpp::executors::MultiThreadedExecutor server_exec;
    server_exec.add_node(rover3);
    server_exec.add_node(rover2)
    
    auto server_spin_exec = [&server_exec]() {
        server_exec.spin();
    };
    std::thread server_exec_thread(server_spin_exec);
    std::cout << "Node init" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    std::cout << "Node start" << std::endl;
```

Now moving on to next few lines of this example code
```
rover3->setCmdMode(DASCRobot::ControlMode::kPositionMode);

double vx = 2.0;
double wz = 3.14/3;
    
for (int i = 0; i < 100; i++) {
        // rover3->cmdWorldPosition(rad * cos(theta), rad * sin(theta), height, 0, 0);
        rover3->cmdWorldPosition(1.0,0,0,0,0);  
        // rover3->cmdLocalVelocity(0,vx,0.0,0.0,wz);  
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
        rover3->cmdOffboardMode();
        rover3->arm();
    
    std::cout << "Arm" << std::endl;
```
Here, we first set the command mode. `kPositionMode` is the position control mode. You can demand desired x,y from it. It does not care about yaw at the moment. If you want to use the velocity mode, it will be `kVelocityMode` in which you need to set linear x velocity and yaw_rat. Other fields will be ignored (but better put them to 0). 
After setting the mode, we first start sending some offboard commands (desired position) in a loop and then set the rover to use offboard mode.
Note: in PX4, automatic mode is called 'offboard mode' as the setpoints are received from outside, like an offboard computer (Jetson/Pi/laptop). Finally we arm it.

We send some offboard commands before actually setting it to offboard mode because if px4 does not detect any offboard waypoint commands when it is switch to offboard mode, it will reject the request to switch to offboard mode. 

Now moving on to next part of code.
```
    while(rclcpp::ok()) {
        rover3->cmdWorldPosition(1.0,0,0,0,0);  
        // rover3->cmdLocalVelocity(0.0,vx,0.0,0.0,wz);  
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    server_exec_thread.join();
    rclcpp::shutdown();
	return 0;
}
```
Here, we just keep sending the commands in a loop that runs until you close the node from terminal (by pressing Ctrl+C for example).
**Note:** for position control, the message has syntax, (x,y,z,yaw,yaw_rate) and for velocity control the syntax is (vx,vy,vz,yaw,yaw_rate). As explained above, for rovers, only x,y and vx,yaw_rate will be used. others should be 0.

**Note**: If you want to stop the rover, just kill the node by pressing Ctrl+C. It will move back to manual control mode.







<!-- # Old(Incomplete) documentation from here on. 
# Framework
Make sure to make some position/velocity commands before commanding to arm

# Ground Robot

## Container 
Use the container vicon_px4_ros2_bridge:veriosn1.1 (px4_vicon_bridge_display in DASC 2) on the laptop you plan to use. if using indoor make sure to launch the file bridge_launch.py, and afterwards run dasc_robot. These are located in px3_ros_com_ros2. Any changes made to any of the files in px4_ros_com_ros2 requires a colcon build afterwards.

## bridge_launch.py

Before launching bridge_launch.py check the launch file to see if there are nodes for each agent you're planning to use has been added and is using the proper namespace set on the vehicle itself and on vicon's computer. Make sure the Vicon computer is connected to swarm-5G or the appropriate network, and that the IP address specified in the hostname variable matches the vicon computer's ip address. Then launch with the command

```
ros2 launch vicon_px4_bridge bridge_launch.py
```

## dasc_robot

Make sure to run ```colcon build``` after whatever edits you make to the framework. To run the dascRobot node use
```
ros2 run dasc_robot dascRobot
```
-->
