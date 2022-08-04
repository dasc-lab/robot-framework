#include <thread>
#include "DASCRobots.hpp"

// std::thread server_exec_thread;

int main(int argc, char* argv[]) {
	std::cout << "Starting multi robot offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
    auto rover3 = std::make_shared<DASCRobot>("rover3", 3);
    auto rover4 = std::make_shared<DASCRobot>("rover4", 4);
    auto rover7 = std::make_shared<DASCRobot>("rover7", 7);
    
    rover3->init();
    rover4->init();
    rover7->init();
    
    rclcpp::executors::MultiThreadedExecutor server_exec;
    server_exec.add_node(rover3);
    server_exec.add_node(rover4);
    server_exec.add_node(rover7);
    
    auto server_spin_exec = [&server_exec]() {
        server_exec.spin();
    };
    std::thread server_exec_thread(server_spin_exec);
    // server_exec_thread = std::thread( server_spin_exec );
    std::cout << "Node init" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    std::cout << "Node start" << std::endl;

    rover3->setCmdMode(DASCRobot::ControlMode::kVelocityMode);
    rover4->setCmdMode(DASCRobot::ControlMode::kVelocityMode);
    rover7->setCmdMode(DASCRobot::ControlMode::kVelocityMode);

    double vx = 1.0;
    double wz = 1.57/3.0;//-3.925;//3.14/3;  
    // 11:29: +ve clockwise for now

    for (int i = 0; i < 100; i++) {
        // rover->cmdWorldPosition(rad * cos(theta), rad * sin(theta), height, 0, 0);
        // rover->cmdWorldPosition(-1.0,1.0,0,0,0);  
        rover3->cmdLocalVelocity(0.0,vx,0.0,0.0,wz);  
        rover4->cmdLocalVelocity(0.0,vx,0.0,0.0,wz);  
        rover7->cmdLocalVelocity(0.0,vx,0.0,0.0,wz);  
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
        rover3->cmdOffboardMode();
        rover3->arm();
        rover4->cmdOffboardMode();
        rover4->arm();
        rover7->cmdOffboardMode();
        rover7->arm();
    
    std::cout << "Arm" << std::endl;

    while(rclcpp::ok()) {
        // rover->cmdWorldPosition(-1.0,1.0,0,0,0);  
        rover3->cmdLocalVelocity(0.0,vx,0.0,0.0,wz);  
        rover4->cmdLocalVelocity(0.0,vx,0.0,0.0,wz);  
        rover7->cmdLocalVelocity(0.0,vx,0.0,0.0,wz);  
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    server_exec_thread.join();
    rclcpp::shutdown();
	return 0;
}
