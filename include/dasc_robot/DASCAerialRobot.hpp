#ifndef DASC_ROBOT_DASCAERIALROBOT_HPP_
#define DASC_ROBOT_DASCAERIALROBOT_HPP_

#include <queue>

#include "DASCRobot.hpp"
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>

using namespace px4_msgs::msg;

class DASCAerialRobot : public DASCRobot {
    public:
        DASCAerialRobot(std::string robot_name);
        bool init();
        bool arm();
        bool disarm();
        std::array<float, 3> getWorldPosition();
        std::array<float, 3> getWorldVelocity();
        std::array<float, 3> getWorldAcceleration();
        std::array<float, 3> getBodyAcceleration();
        std::array<float, 3> getBodyRate();
        std::array<float, 4> getBodyQuaternion();
        bool setCmdMode(DASCRobot::ControlMode mode);
        bool cmdWorldPosition(float x, float y, float z, float yaw);
        bool cmdWorldVelocity(float x, float y, float z, float yaw_rate);
        bool cmdWorldAcceleration(float x, float y, float z, float yaw_acceleration);
        bool cmdAttitude(float q_w, float q_x, float q_y, float q_z, float thrust);
        bool cmdRates(float roll, float pitch, float yaw, float thrust);
    private:
        enum class AerialRobotServerState {
                kInit = 0,
                kNormal,
                kFailSafe,
            };
        bool initialized_;
        ControlMode current_control_mode_;
        AerialRobotServerState server_state_;
        std::string robot_name_;
        const uint64_t timeout_ms_;
        std::atomic<uint64_t> timestamp_;

        std::mutex acc_queue_mutex_;
        std::mutex gyro_queue_mutex_;
        std::mutex quaternion_queue_mutex_;
        std::mutex world_position_queue_mutex_;
        std::mutex world_velocity_queue_mutex_;
        std::mutex world_acceleration_queue_mutex_;

        std::queue <std::array<float, 3>> accelerometer_m_s2_queue_;
        std::queue<std::array<float, 3>> gyro_rad_queue_;
        std::queue<std::array<float, 4>> quaternion_queue_;
        std::queue<std::array<float, 3>> world_position_queue;
        std::queue<std::array<float, 3>> world_velocity_queue;
        std::queue<std::array<float, 3>> world_acceleration_queue;
        

        rclcpp::Subscription<Timesync>::SharedPtr timesync_sub_;
        rclcpp::Subscription<SensorCombined>::SharedPtr sensor_combined_sub_;
        rclcpp::Subscription<VehicleAttitude>::SharedPtr vehicle_attitude_sub_;
        rclcpp::Subscription<VehicleLocalPosition>::SharedPtr vehicle_local_position_sub_;

        rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
        rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
        rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
        rclcpp::Publisher<VehicleAttitudeSetpoint>::SharedPtr vehicle_attitude_publisher_;
        rclcpp::Publisher<VehicleRatesSetpoint>::SharedPtr vehicle_rates_publisher_;

        void timesyncCallback(const Timesync::UniquePtr msg);
        void sensorCombinedCallback(const SensorCombined::UniquePtr msg);
        void vehicleAttitudeCallback(const VehicleAttitude::UniquePtr msg);
        void vehicleLocalPositionCallback(const VehicleLocalPosition::UniquePtr msg);
        void updateState();
};

#endif // DASC_ROBOT_DASCAERIALROBOT_HPP_
