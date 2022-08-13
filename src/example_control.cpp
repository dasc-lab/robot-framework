#include <thread>
#include "DASCRobots.hpp"


int main(int argc, char* argv[]) {
	std::cout << "Starting offboard 77 control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
    auto rover = std::make_shared<DASCRobot>("rover3", 3);
    
    rover->init();
    
    rclcpp::executors::MultiThreadedExecutor server_exec;
    server_exec.add_node(rover);
    
    auto server_spin_exec = [&server_exec]() {
        server_exec.spin();
    };
    std::thread server_exec_thread(server_spin_exec);
    std::cout << "Node init" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    std::cout << "Node start" << std::endl;

    rover->setCmdMode(DASCRobot::ControlMode::kVelocityMode);

    while(rclcpp::ok()) {
        std::array<double, 3> pos = rover->getWorldPosition();
        std::cout << "x: " << pos[0] << "\t y: " << pos[1] << "\t z: " << pos[2] << std::endl;
        std::array<double, 4> quat;
        rover->getBodyQuaternion(quat, false);
        std::array<double, 3> acc = rover->getWorldAcceleration();
        std::cout << quat[0] << "\t" << quat[1] << "\t" << quat[2] << "\t" << quat[3] << std::endl;
        std::cout << "Acc: " << acc[0] << "\t" << acc[1] << "\t" << acc[2] << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // double vx = 0.0;
    // double wz = 1.57/3.0;//-3.925;//3.14/3;  
    // // 11:29: +ve clockwise for now

    // for (int i = 0; i < 100; i++) {
    //     // rover->cmdWorldPosition(rad * cos(theta), rad * sin(theta), height, 0, 0);
    //     // rover->cmdWorldPosition(0.0,0.0,0,0,0);  
    //     rover->cmdLocalVelocity(0.0,vx,0.0,0.0,wz);  
    //     std::this_thread::sleep_for(std::chrono::milliseconds(50));
    // }
    //     rover->cmdOffboardMode();
    //     rover->arm();
    
    // std::cout << "Arm" << std::endl;


    // while(rclcpp::ok()) {
    //     // std::array<double, 4> quat;
    //     // rover->getBodyQuaternion(quat, false);
    //     // std::array<double, 3> acc = rover->getBodyAcceleration();
    //     // std::cout << quat[0] << "\t" << quat[1] << "\t" << quat[2] << "\t" << quat[3] << std::endl;
    //     // std::cout << "Acc: " << acc[0] << "\t" << acc[1] << "\t" << acc[2] << std::endl;
    //     // rover->cmdWorldPosition(0.0,0.0,0,0,0);  
    //     rover->cmdLocalVelocity(0.0,vx,0.0,0.0,wz);  
    //     std::this_thread::sleep_for(std::chrono::milliseconds(50));
    // }

    server_exec_thread.join();
    rclcpp::shutdown();
	return 0;
}
