
#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp>


#include "px4_msgs/msg/vehicle_command.hpp"


class Estop : public rclcpp::Node {

    public:
    Estop() 
        : Node("estop")
    {
        for (int i=0; i < maxN; i++){
            std::string topic_name = std::format("/drone{}/fmu/vehicle_command/in");
            drone_publishers_.push_back(
                    this->create_publisher<px4_msgs::msg::VehicleCommand>(topic_name,10)
                    );
        }
        
        for (int i=0; i < maxN; i++){
            std::string topic_name = std::format("/rover{}/fmu/vehicle_command/in");
            rover_publishers_.push_back(
                    this->create_publisher<px4_msgs::msg::VehicleCommand>(topic_name,10)
                    );
        }

        timer_ = this->create_wall_timer(
            200ms, std::bind(&Estop::timer_callback, this));
    }

    private:

    const int maxN = 25;

    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr> drone_publishers_;
    std::vector<rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr> rover_publishers_;


    void timer_callback() {
       
        auto msg = px4_msgs::msg::VehicleCommand();
        msg.timestamp = 0;
        msg.param1 = 0.0;
        msg.param2 = 21196; // force disarm
        msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;

        // drone stop command
        for (int i=0; i< maxN; i++){
            msg.target_system = i;
            drone_publishers[i]->publish(msg);
        }

        // rover stop command
        for (int i=0; i< maxN; i++){
            msg.target_system = i;
            rover_publishers[i]->publish(msg);
        }

        RCLCPP_WARN(this->get_logger(), "sending e stop to drones with id less than %d", maxN);
    }



}; // class Estop


int main(int argc, char* argv[]) {

  std::cout << "Sending ESTOP" << std::endl;
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<Estop>());

    rclcpp::shutdown();
    return 0;
}
