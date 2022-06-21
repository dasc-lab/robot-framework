#include "DASCAerialRobot.hpp"
#include <thread>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include "frame_transforms.h"
#include <cmath>


int main(int argc, char* argv[]) {
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
    auto rover3 = std::make_shared<DASCAerialRobot>("rover2", 1);
    // auto drone2 = std::make_shared<DASCAerialRobot>("drone2", 2);
    // auto drone3 = std::make_shared<DASCAerialRobot>("drone3", 3);
    // auto drone4 = std::make_shared<DASCAerialRobot>("drone4", 4);
    // auto drone5 = std::make_shared<DASCAerialRobot>("drone5", 5);
    
    rover3->init();
    // drone2->init();
    // drone3->init();
    // drone4->init();
    // drone5->init();
    
    rclcpp::executors::MultiThreadedExecutor server_exec;
    server_exec.add_node(rover3);
    // server_exec.add_node(drone2);
    // server_exec.add_node(drone3);
    // server_exec.add_node(drone4);
    // server_exec.add_node(drone5);
    
    auto server_spin_exec = [&server_exec]() {
        server_exec.spin();
    };
    std::thread server_exec_thread(server_spin_exec);
    std::cout << "Node init" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(20000));
    std::cout << "Node start" << std::endl;
    // double rad = 2.0;
    // double theta = 0.0;
    // double height = 0.0;

    rover3->setCmdMode(DASCRobot::ControlMode::kPositionMode);
    // drone2->setCmdMode(DASCRobot::ControlMode::kPositionMode);
    // drone3->setCmdMode(DASCRobot::ControlMode::kPositionMode);
    // drone4->setCmdMode(DASCRobot::ControlMode::kPositionMode);
    // drone5->setCmdMode(DASCRobot::ControlMode::kPositionMode);
    
    rover3->setGPSGlobalOrigin(42.2944313, -83.71044393888889, 275.0);
    // drone2->setGPSGlobalOrigin(42.2944313, -83.71044393888889, 275.0);
    // drone3->setGPSGlobalOrigin(42.2944313, -83.71044393888889, 275.0);
    // drone4->setGPSGlobalOrigin(42.2944313, -83.71044393888889, 275.0);
    // drone5->setGPSGlobalOrigin(42.2944313, -83.71044393888889, 275.0);
    
    // drone2->setGPSGlobalOrigin(47.3977419, 8.5455950, 488.105);
    // drone3->setGPSGlobalOrigin(47.3977419, 8.5455950, 488.105);
    // drone4->setGPSGlobalOrigin(47.3977419, 8.5455950, 488.105);
    // for (int i = 0; i < 100; i++) {
    //     rover2->cmdWorldPosition(rad * cos(theta), rad * sin(theta), height, 0, 0);
    //     // drone2->cmdWorldPosition(rad * cos(theta + M_PI_2), rad * sin(theta + M_PI_2), height, 0, 0);
    //     // drone3->cmdWorldPosition(rad * cos(theta + M_PI_2 * 2.0), rad * sin(theta + M_PI_2 * 2.0), height, 0, 0);
    //     // drone4->cmdWorldPosition(rad * cos(theta + M_PI_2 * 3.0), rad * sin(theta + M_PI_2 * 3.0), height, 0, 0);
    //     // drone5->cmdWorldPosition(0, 0, height + 0.5, 0, 0);
    //     // drone3->cmdWorldPosition(x - 1, y + 1, z, 0.0);
    //     // drone4->cmdWorldPosition(x - 1, y - 1, z, 0.0);
    //     std::this_thread::sleep_for(std::chrono::milliseconds(50));
    //}

    // NEED TO SEND A SHORT COMMAND BEFORE ARMING 
    for (int i = 0; i < 100; i++) {
        rover3->cmdWorldPosition(0.5,0,0,0,0);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
        rover3->cmdOffboardMode();
        rover3->arm();
        // drone2->cmdOffboardMode();
        // drone2->arm();
        // drone3->cmdOffboardMode();
        // drone3->arm();
        // drone4->cmdOffboardMode();
        // drone4->arm();
        // drone5->cmdOffboardMode();
        // drone5->arm();
    std::cout << "Arm" << std::endl;
    // for(int i = 0; i < 10; i++) {
    //     std::array<double, 4> quat;
    //     rover2->getBodyQuaternion(quat, true);
    //     std::cout << quat[0] << quat[1] << quat[2] << quat[3] << std::endl;
    // }
    for (int i = 0; i < 200; i++) {
        rover3->cmdWorldPosition(0.5,0.5,0,0,0);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::cout << "Position sent" << std::endl;

    // for (int i = 0; i < 20; i++) {
    //     rover2->cmdWorldVelocity(1,0,0,0,0);
    //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // }
    // for (int i = 0; i < 200; i++) {
    //     rover2->cmdWorldVelocity(0,0,0,0,2);
    //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // }
    // std::cout << "Velocity sent" << std::endl;

    // for(int i = 0; i < 10; i++) {
    //     std::array<double, 4> quat;
    //     drone1->getBodyQuaternion(quat, true);
    //     std::cout << quat[0] << quat[1] << quat[2] << quat[3] << std::endl;
    // }
    // for (int i = 0; i < 200; i++) {
    //     drone1->cmdWorldPosition(x + 1, y - 1, z, 0.0);
    //     drone2->cmdWorldPosition(x + 1, y + 1, z, 0.0);
    //     drone3->cmdWorldPosition(x - 1, y + 1, z, 0.0);
    //     drone4->cmdWorldPosition(x - 1, y - 1, z, 0.0);
    //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // }
    // std::this_thread::sleep_for(std::chrono::milliseconds(20000));
    // for (int i = 0; i < 200; i++) {
    //     rover2->cmdWorldPosition(rad * cos(theta), rad * sin(theta), height, 0, 0);
    //     // drone2->cmdWorldPosition(rad * cos(theta + M_PI_2), rad * sin(theta + M_PI_2), height, 0, 0);
    //     // drone3->cmdWorldPosition(rad * cos(theta + M_PI_2 * 2.0), rad * sin(theta + M_PI_2 * 2.0), height, 0, 0);
    //     // drone4->cmdWorldPosition(rad * cos(theta + M_PI_2 * 3.0), rad * sin(theta + M_PI_2 * 3.0), height, 0, 0);
    //     // drone5->cmdWorldPosition(0, 0, height + 0.5, 0, 0);
    //     std::this_thread::sleep_for(std::chrono::milliseconds(50));
    // }
    // while(rclcpp::ok()) {
    //     theta += 0.00785;
    //     if (theta > M_PI * 2.0) {
    //         theta = 0.0;
    //     }
    //     rover2->cmdWorldPosition(rad * cos(theta), rad * sin(theta), height, 0, 0);
    //     // drone2->cmdWorldPosition(rad * cos(theta + M_PI_2), rad * sin(theta + M_PI_2), height, 0, 0);
    //     // drone3->cmdWorldPosition(rad * cos(theta + M_PI_2 * 2.0), rad * sin(theta + M_PI_2 * 2.0), height, 0, 0);
    //     // drone4->cmdWorldPosition(rad * cos(theta + M_PI_2 * 3.0), rad * sin(theta + M_PI_2 * 3.0), height, 0, 0);
    //     // drone5->cmdWorldPosition(0, 0, height + 0.5, 0, 0);
    //     // drone1->cmdWorldPosition(x + 1, y - 1, z, 0.0);
    //     // drone2->cmdWorldPosition(x + 1, y + 1, z, 0.0);
    //     // drone3->cmdWorldPosition(x - 1, y + 1, z, 0.0);
    //     // drone4->cmdWorldPosition(x - 1, y - 1, z, 0.0);
    //     std::this_thread::sleep_for(std::chrono::milliseconds(50));
    // }

    server_exec_thread.join();
    rclcpp::shutdown();
	return 0;
}