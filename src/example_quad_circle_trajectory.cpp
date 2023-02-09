#include <thread>
#include "DASCRobots.hpp"
#include "trajectory.hpp"

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

    double circle_r = 1.0;
    double circle_z = 1.0;
    double circle_T = 10.0;
    int circle_N = 4; // how many times to repeat the trajectory

    traj::CircularTrajectory traj = traj::CircularTrajectory(circle_r, circle_z, circle_T);
    
    quad->setCmdMode(DASCRobot::ControlMode::kTrajectoryMode);
    quad->cmdOffboardMode();

    while(rclcpp::ok()) {

	auto now = quad->get_current_timestamp_us();
	double elapsed_s = (double(now - start) * 1e-6);

	// std::array<double, 3> pos = quad->getWorldPosition();
	// RCLCPP_INFO(quad->get_logger(), "Time: %.1f, POS: %.1f, %.1f, %.1f", elapsed_s,  pos[0], pos[1], pos[2]);
	


	if (elapsed_s <= 15.0) {
            // takeoff is for 15 seconds
	    RCLCPP_INFO_ONCE(quad->get_logger(), "Taking off...");
	    auto sp = traj.setpoint(0.0);
	    quad->cmdWorldPosition(sp.pos[0], sp.pos[1], sp.pos[2],  sp.yaw, 0);
	}
	else if ( elapsed_s <= (circle_N * circle_T + 15.0)) {
	    RCLCPP_INFO_ONCE(quad->get_logger(), "Starting Trajectory...");
	    // circular trajectory
	    auto sp = traj.setpoint(elapsed_s - 15.0);
	    quad->cmdDiffflatSetpoint(sp);
	}
	else if ( elapsed_s <= (circle_N * circle_T + 20.0)) {
	    RCLCPP_INFO_ONCE(quad->get_logger(), "Finished. Hovering...");
	     // hover at end of trajectory 
	    auto sp = traj.setpoint(circle_T);
	    quad->cmdWorldPosition(sp.pos[0], sp.pos[1], sp.pos[2],  sp.yaw, 0);
	}
	else if ( elapsed_s <= (circle_N * circle_T + 25.0)) {
     	     // land
	    RCLCPP_INFO_ONCE(quad->get_logger(), "Landing...");
	    auto sp = traj.setpoint(circle_T);
	    sp.pos[2] = -0.2;
	    quad->cmdWorldPosition(sp.pos[0], sp.pos[1], sp.pos[2],  sp.yaw, 0);
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
