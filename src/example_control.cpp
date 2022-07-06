#include <thread>
#include "DASCRobots.hpp"


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

    rover3->setCmdMode(DASCRobot::ControlMode::kPositionMode);

    double vx = 2.0;
    double wz = 3.14/3;

    for (int i = 0; i < 100; i++) {
        // rover3->cmdWorldPosition(rad * cos(theta), rad * sin(theta), height, 0, 0);
        rover3->cmdWorldPosition(1.0,0,0,0,0);  
        // rover3->cmdLocalVelocity(0.0,vx,0.0,0.0,wz);  
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
        rover3->cmdOffboardMode();
        rover3->arm();
    
    std::cout << "Arm" << std::endl;

    while(rclcpp::ok()) {
        rover3->cmdWorldPosition(1.0,0,0,0,0);  
        // rover3->cmdLocalVelocity(0.0,vx,0.0,0.0,wz);  
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    server_exec_thread.join();
    rclcpp::shutdown();
	return 0;
}