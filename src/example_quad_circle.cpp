#include <thread>
#include "DASCRobots.hpp"
#include "trajectory.hpp"

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

    for (int i = 0; i < 100; i++) {
	quad->cmdWorldPosition(0, 0, 0.50, 0, 0.0);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    quad->cmdOffboardMode();
    quad->arm();
    
    std::cout << "Arm" << std::endl;
    std::cout << "in loop" << std::endl;

    auto start = quad->get_current_timestamp_us(); 

    double circle_r = 1.0;
    double circle_z = 1.0;
    double circle_T = 10.0;

    traj::CircularTrajectory traj = traj::CircularTrajectory(circle_r, circle_z, circle_T);

    while(rclcpp::ok()) {

	auto now = quad->get_current_timestamp_us();
	double elapsed_s = (double(now - start) * 1e-6);

	// std::array<double, 3> pos = quad->getWorldPosition();
	// RCLCPP_INFO(quad->get_logger(), "Time: %.1f, POS: %.1f, %.1f, %.1f", elapsed_s,  pos[0], pos[1], pos[2]);

	if (elapsed_s <= 5.0) {
            // takeoff is for 5 seconds
	    RCLCPP_INFO_ONCE(quad->get_logger(), "Taking off...");
       	    std::array<double, 3> target_pos = traj.pos(0.0);
	quad->cmdWorldPosition(target_pos[0], target_pos[1], target_pos[2], 0, 0);
	}
	else if ( elapsed_s <= (circle_T + 5.0)) {
	    RCLCPP_INFO_ONCE(quad->get_logger(), "Starting Trajectory...");
	     // circular trajectory
	     std::array<double, 3> target_pos = traj.pos(elapsed_s - 5.0);
	quad->cmdWorldPosition(target_pos[0], target_pos[1], target_pos[2], 0, 0);
	}
	else if ( elapsed_s <= (circle_T + 10.0)) {
	    RCLCPP_INFO_ONCE(quad->get_logger(), "Finished. Hovering...");
	     // hover at end of trajectory 
	     std::array<double, 3> target_pos = traj.pos(circle_T);
	quad->cmdWorldPosition(target_pos[0], target_pos[1], target_pos[2], 0, 0);
	}
	else if ( elapsed_s <= (circle_T + 15.0)) {
     	     // land
	    RCLCPP_INFO_ONCE(quad->get_logger(), "Landing...");
	     std::array<double, 3> target_pos = traj.pos(circle_T);
	     target_pos[2] = 0.0; // force it to the ground
	quad->cmdWorldPosition(target_pos[0], target_pos[1], target_pos[2], 0, 0);
	}
	else {
	     break;
	}
        
	std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    
    
    RCLCPP_INFO_ONCE(quad->get_logger(), "Done. Disarming...");

    // disarm
    quad->disarm();
    std::cout << "after disarm" << std::endl;

    server_exec_thread.join();
    rclcpp::shutdown();
    return 0;
}
