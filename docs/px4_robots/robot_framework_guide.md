---
layout: default
title:  "8. Using Robot Framework for Rover3"
date:   2022-05-09
math: katex
parent: Px4 Robot
nav_order: 8
---

This page explains on how to get started with using rover3 that is completely set up.

# TLDR:

- [on ground station] start QGC
- (optional) turn on radio transmitter
- [on ground station] start the docker container that has ros workspace, vicon, robot-framework
- [on ground station] `tmux`
- [on ground station] `ros2 launch vicon_px4_bridge bridge_launch.py`
- [on drone pi] start docker containing ros that has ros workspace, rtps-bridge
- [on drone_pi] `tmux`
- [on drone pi] `mavlink-routerd`
- [on drone pi] `bridge`
- check QGC that the roll pitch yaw are correct
- (optional) check that drone can be armed from radio transmitter
- [ground station] run robot-framework: `ros2 run dasc_robot example_control`



NOTE: Boot the laptop into Ubuntu 20. The instructions have been tested for DASC2 at the time of writing this. DASC1 is yet to be tested.

# Step 1: Power-up and connect to rover
Power up the rover by connecting the LiPo battery (voltage should be more than 10.7 ideally). Use DASC2 laptop and boot into it's Ubuntu 20 (the default one when you power up the lapotp so nothing to change there). Make sure the laptop is connected to drone-5G-5.2 Wifi (this is the big router). On DASC2, open a terminal and do
```
ssh rover3@rover3.local
```
When prompted for password, use 'hello123'.

Note: In case you need to open multiple terminals on laptop and or jetson, I recommend using tmux. tmux allows you to split a single terminal into multiple panes each of which acts as an independent terminal and you won't have to ssh again from another terminal. You can see some guides on using tmux or open multiple terminals directlt if need for subsequent steps.

Note: I made an alias in bashrc file, so you can just type `rover3` and it will implement the above ssh command

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

# Step 4: Do some ROS settings for Discovery Server
This section is based on https://fast-dds.docs.eprosima.com/en/latest/fastdds/ros2/discovery_server/ros2_discovery_server.html#daemon-s-related-commands.


On laptop, in one of the terminals, do
```
fastdds discovery --server-id 0
```
This will start the discovery server (ros2 by default is completely decentralized in which each computer runs its own process to find other computers thereby sharing lots of data. A discovery server will make this search process centralized and speed up things. In fact, if you have more than 2 robots, the decentralized version will make your wifi freeze. So this is an important step for multi-robot experiment).

Next, just once on Laptop and every other system (RPi/Jetson), we need to export address to a config file and restart ros2 daemon. Make sure a dds configuration xml file exists inside the ros2 workspace folder (can be anywhere but has been placed here on most systems). Open the file and change the IP address to that of the laptop on all Jetsons/Rpis/Laptops. If the file does not exist, then make a new file `super_client_configuration_file.xml` and add the lines from [following link to it](https://fast-dds.docs.eprosima.com/en/latest/_downloads/9f9e92b14612364b742c8ecde24b3d24/super_client_configuration_file.xml). The file name can be anything. Just make sure the IP address is correct. Now export this file as follows
```
export FASTRTPS_DEFAULT_PROFILES_FILE=super_client_configuration_file.xml
```
and restart the ros2 daemon
```
ros2 daemon stop
ros2 daemon start
```

Now every terminal on laptop anfd Jetson needs to run these two commands before running anything else
```
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER=LAPTOP_IP_ADDRESS:11811
```

It has been made part of `.bashrc` of all Jetsons and therefore need not be run. But you still need to check the IP address. If `.bashrc` does not contain these lines at the end yet, then just run them yourself.

**Note:** The above commands cannot be made part of `.bashrc` of the laptop as the discovery server needs to be run before exporting these commands.



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

# Step 6: Send Waypoints in C++

**Note**: The whole framework is in a single file DASCRobots.cpp (anf header file DASCRobots.hpp). This was  built as an library in CMakeLists.txt of the package. This library was then linked to the example_control.cpp that is run below. For other langiages like Pythin and Julia, it suffices to build a wrapper around this library.

Start running the rover around. The whole framework is in the robot-framework folder in src of ros workspace. You can edit any file by opening it in the terminal directly or use vs code to acces the docker container and all its files.

The whole framework and a sample code is inside a single file right now whose location is
```
/root/px4_ros_com_ros2/src/robot-framework/src/DASCARobots.cpp
```
In order to run this node, do
```
ros2 run dasc_robot example_control
```

Note: the folder name is robot-framework but the ros package name is dasc_robot. Hence the above command.

Note: there's also a multi robot example code that can be run with the following command. Have a look at the complete code [here](https://github.com/dasc-lab/robot-framework/blob/master/src/example_control_multi_rover.cpp)
```
ros2 run dasc_robot example_control_multi_rover
```

The current code makes the rover go to a location (x=1,y=0) from wherever it is. The only thing you need to modify is the `main()` function according to your application. Hence, I will go over it below:

```
int main(int argc, char* argv[]) {
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
    auto rover3 = std::make_shared<DASCAerialRobot>("rover3", 3);
    
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

The above are the steps you need to do for initializing a robot. 

**Note**: In `std::make_shared<DASCAerialRobot>("rover3", 3);`, `rover3` is the name and `3` is the ID.  

For multiple robots, you need to write the sames lines again for them. For example, if you have another rover2, then the code would look like this
```
int main(int argc, char* argv[]) {
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
    auto rover3 = std::make_shared<DASCAerialRobot>("rover3", 3);
    auto rover2 = std::make_shared<DASCAerialRobot>("rover2", 2);
    
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



# Step 6: Send Waypoints in Python

A Python wrapper implements the same C++ based framework that was discussed before. The way it works is that a .so file is generated by robot-framework and this is read by a Python code (no need to know this to use the code!). The Python code is made part of another ros package `robot-framework-py` [on Github](https://github.com/dasc-lab/robot-framework-py). The framework files are inside the `dasc_robots` folder and need not be touched. The user files will all go to `dasc_robot_py` folder that follows the standard format for [writing Python nodes in ROS2](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html). As a summary, every Python code needs to have a folder like `dasc_robot_py` where user files go. This folder should also contain `__init__.py` file that can be empty (This is standard way of making packages in Python which is what ROS2 is doing here). Then in root foldet, you will also see `setup.py` and `setup.cfg`. In both the files, check that the name of package is mentioned properly, in this case `dasc_robot_py`. All the user files that you will write to control your robot will have to be declared in `setup.py` so that ROS can recognize it. The following line
```
'test_run = dasc_robot_py.test_run:main',
```
declared a node name `test_run` and its location is 'asc_robot_py/test_run.py'. The `main` comes because we want to start with the main function defined in that `.py` file.

Since the python wrapper is a python package that we will import, we need to export path to this package (I didn't do a system installation like with pip). The following line might be written in `.bashrc`. If not, then add it or do this on every terminal you want to run the python node from
```
export PYTHONPATH=$PYTHONPATH:path_to_dasc_robot_folder
```


Now, in order to run this code, you need to do
```
ros2 run dasc_robot_py test_run
```
Here is the code

```
from dasc_robots.robot import Robot
from dasc_robots.ros_functions import *
import rclpy
from rclpy.node import Node
import numpy as np

import threading

from std_msgs.msg import String

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('minimal_publisher')

    # Spin in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()
    rate = node.create_rate(30)

    publisher = node.create_publisher(String, 'topic', 10)    
    msg = String()
    
    ######################## Wrapper based functions
    ros_init("test_run")

    robot3 = Robot("rover3", 3)
    robot7 = Robot("rover7", 7)
    print("Robot Initialized")

    robot3.init()
    robot7.init()

    robots = [robot3, robot7]

    robot3.set_command_mode( 'velocity' )
    robot7.set_command_mode( 'velocity' )

    vx = 0.0
    wz = 2.0
    for i in range(100):
        robot3.command_velocity( np.array([0,vx,0,0,wz]) )
        robot7.command_velocity( np.array([0,vx,0,0,wz]) )
        rate.sleep()
        
    robot3.cmd_offboard_mode()
    robot3.cmd_offboard_mode()
    robot3.arm()

    robot7.cmd_offboard_mode()
    robot7.cmd_offboard_mode()
    robot7.arm()

    threads = start_ros_nodes(robots)
    #####################################
    i = 0
    while rclpy.ok():
        msg.data = 'Hello World: %d' % i
        i += 1
        # node.get_logger().info('Publishing: "%s"' % msg.data)
        publisher.publish(msg)
        robot3.command_velocity( np.array([0,vx,0,0,wz]) )
        robot7.command_velocity( np.array([0,vx,0,0,wz]) )
        rate.sleep()

    print("hello")

    node.destroy_node()
    rclpy.shutdown() 
```
Here is a breakdown of how the code works:
```
from dasc_robots.robot import Robot
from dasc_robots.ros_functions import *
import rclpy
from rclpy.node import Node
import numpy as np
import threading
from std_msgs.msg import String
```
Here we import required packages. `dasc_robots.robot` and `dasc_robots.ros_functions` are from the Python wrapper in `dasc_robots` folder. `dasc_robots.robot` defines the robot class that can be used to make a robot object, get robot data such as position, velocity, and sensor data, and finally send control commands. `dasc_robots.ros_functions` defines some basic ROS2 functions. Using a warpper means that C++ works under the hood. Therefore several basic ROS commands still need to be implemented in C++ and hence the wrapper for them. Feel free to open `robot.py` and `ros_functions.py` in [dasc_robots](https://github.com/dasc-lab/robot-framework-py/tree/main/dasc_robots) folder to see a description of all the function available. The `threading` module above is imported to run your own code at a fixed rate. Keep reading to see how it is used.

```
def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('minimal_publisher')

    # Spin in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()
    rate = node.create_rate(30)

    publisher = node.create_publisher(String, 'topic', 10)    
    msg = String()
```
We start with main function and initialize ROS2 in Pyhton with `rclpy.init(args=args)`. Then we declare out Python file as a node with node = rclpy.create_node('minimal_publisher'). You can choose the name here.(These are standard steps to be followed when writing a python node. However, compared to [basic guide](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)) no need to make a class just to write your control/planning code. The c++ based framework makes classes for robots. We will implement our planning/control code in simple `for/while` loop below.

Now, to run our loops at fixed frequency, we make a thread `threading.Thread` and set the rate in Hz with `rate = node.create_rate(30)`. The `publisher` and `msg` above are just examples to create a simple publisher and publish a data that you want.

```

    ######################## Wrapper based functions
    ros_init("test_run")

    robot3 = Robot("rover3", 3)
    robot7 = Robot("rover7", 7)
    print("Robot Initialized")

    robot3.init()
    robot7.init()

    robot3.set_command_mode( 'velocity' )
    robot7.set_command_mode( 'velocity' )
    
    robots = [robot3, robot7]
    
    threads = start_ros_nodes(robots)

    vx = 0.0
    wz = 2.0
    for i in range(100):
        robot3.command_velocity( np.array([0,vx,0,0,wz]) )
        robot7.command_velocity( np.array([0,vx,0,0,wz]) )
        rate.sleep()
        
    robot3.cmd_offboard_mode()
    robot3.arm()

    robot7.cmd_offboard_mode()
    robot7.arm()

    
    #####################################
```

This block makes use of wrapper functions. `ros_init` initializes ROS in c++ (rclpy.init we did before doesn't do this). `robot3 = Robot("rover3", 3)` makes the robot object. `robot3.init()` does some initialization (the sequence of commands here are same as the C++ code.). `robot3.set_command_mode( 'velocity' )` sets the control mode to velocity control. You can also choose `position`. See `robot.py`'s description of this function for the modes that can be choosen. `threads = start_ros_nodes(robots)` is the equivalent of c++ code that runs a separate thread for each robot. In the next few lines, like C++ code, we first send some velocity commands and then we switch robot to `offboard` mode (in PX4 offboard mode is the name given to automatic control when waypoints are sent from a computer. The robot will start operating autonomously the moment it is switched to offboard mode. We send some waypoints beforehand as it needs some data to already exist in its buffer when it swicthes to offboard). Finally we `arm` the robots to give it permission to move.

```
    i = 0
    while rclpy.ok():
        msg.data = 'Hello World: %d' % i
        i += 1
        # node.get_logger().info('Publishing: "%s"' % msg.data)
        publisher.publish(msg)
        robot3.command_velocity( np.array([0,vx,0,0,wz]) )
        robot7.command_velocity( np.array([0,vx,0,0,wz]) )
        rate.sleep()

    print("hello")

    node.destroy_node()
    rclpy.shutdown() 
```
Here we start our main `while` loop, publish some string as an example, and also keep sending velocity commands to robot. All the other processing like getting data from robot, implementing your control/planning algorithm can also happen inside this loop. Note that you need to send velocity/position commands at a minimum frequency (5/10 Hz.. not sure). So if your code is slower than that, you can either write 2 python files, one does computation and publishes results, and one sends velocity commands at high frequency based on whetever your last computation result from first file was. Or you can make a class in the way it is made in [Python ROS2 guide](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html) and run both these process in the same python file as two different process running at their own frequencies. Many ways to do in ROS!















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

