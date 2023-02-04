#include <thread>
#include "DASCRobots.hpp"


int main(int argc, char* argv[]) {
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);

    auto quad = std::make_shared<DASCRobot>("visquad1", 1);
    
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

    auto start = quad->get_current_timestamp_us(); 

    while(rclcpp::ok()) {

	auto now = quad->get_current_timestamp_us();
	auto elapsed = now - start;

	std::array<double, 3> pos = quad->getWorldPosition();

	RCLCPP_INFO(quad->get_logger(), "TIME: %.1f POS: %.1f, %.1f, %.1f",  ((float)elapsed) * 1e-6, pos[0], pos[1], pos[2]);

	std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }


    server_exec_thread.join();
    rclcpp::shutdown();
    return 0;
}
