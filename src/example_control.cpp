#include <thread>
#include "DASCRobots.hpp"


int main(int argc, char* argv[]) {
	std::cout << "Starting offboard control node..." << std::endl;
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

    //rover->setCmdMode(DASCRobot::ControlMode::kVelocityMode);
    rover->setCmdMode(DASCRobot::ControlMode::kPositionMode);

    //double vx = 0.0;
    //double wz = 1.57/3.0;//-3.925;//3.14/3;  
    // 11:29: +ve clockwise for now

    for (int i = 0; i < 100; i++) {
        // rover->cmdWorldPosition(rad * cos(theta), rad * sin(theta), height, 0, 0);
        //rover->cmdWorldPosition(0.0, 0.0, 1.0, 0, 0);  
        //rover->cmdLocalVelocity(0.0,0, 1.0, 0.0, 0.0);  
	    rover->cmdWorldPosition(0, 0, 0.50, 0, 0.0);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
        rover->cmdOffboardMode();
        rover->arm();
    
    std::cout << "Arm" << std::endl;
    std::cout << "in loop" << std::endl;

    while(rclcpp::ok()) {
        rover->cmdWorldPosition(0.0, 0.0, 0.5, 0, 0);  
        // rover->cmdLocalVelocity(0.0,vx,0.0,0.0,wz);  
        //rover->cmdLocalVelocity(0.0,0, 1.0, 0.0, 0.0);  
        
	rover->cmdWorldPosition(0, 0, 0.5, 0, 0.0);

	std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    server_exec_thread.join();
    rclcpp::shutdown();
    return 0;
}
