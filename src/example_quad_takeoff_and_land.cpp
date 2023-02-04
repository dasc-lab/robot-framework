#include <thread>
#include "DASCRobots.hpp"


int main(int argc, char* argv[]) {
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
    auto quad = std::make_shared<DASCRobot>("drone1", 1);
    
    quad->init();
    
    rclcpp::executors::MultiThreadedExecutor server_exec;
    server_exec.add_node(quad);
    
    auto server_spin_exec = [&server_exec]() {
        server_exec.spin();
    };

    std::thread server_exec_thread(server_spin_exec);
    std::cout << "Node init" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    
    std::cout << "Node start" << std::endl;

    quad->setCmdMode(DASCRobot::ControlMode::kPositionMode);

    for (int i = 0; i < 100; i++) {
	quad->cmdWorldPosition(0, 0, 0.50, 0, 0.0);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    quad->cmdOffboardMode();
    quad->arm();
    
    std::cout << "Arm" << std::endl;
    std::cout << "in loop" << std::endl;

    auto start = quad->get_current_timestamp_us(); 

    while(rclcpp::ok()) {

	auto now = quad->get_current_timestamp_us();
	auto elapsed = now - start;
	// RCLCPP_INFO(quad->get_logger(), "TIME: %.1f s",  ((float)elapsed) * 1e-6);

	std::array<double, 3> pos = quad->getWorldPosition();
	RCLCPP_INFO(quad->get_logger(), "POS: %.1f, %.1f, %.1f",  pos[0], pos[1], pos[2]);

	if (elapsed <= 15*1000*1000) {
            // takeoff is for 15 seconds
	    quad->cmdWorldPosition(0.0, 0.0, 1.0, 0, 0);
	}
	else if ( elapsed <= 30*1000*1000) {
            quad->cmdWorldPosition(0.0, 0.0, -0.2, 0, 0);
	}
	else {
		break;
	}
	std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // disarm
    quad->disarm();
    std::cout << "after disarm" << std::endl;

    server_exec_thread.join();
    rclcpp::shutdown();
    return 0;
}
