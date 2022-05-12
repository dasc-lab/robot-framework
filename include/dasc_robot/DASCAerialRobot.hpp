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
#include <tf2_ros/transform_broadcaster.h>

using namespace px4_msgs::msg;

class DASCAerialRobot : public DASCRobot {
    public:
        DASCAerialRobot(std::string robot_name, uint8_t id);
        bool init();
        bool arm();
        bool disarm();
        std::array<double, 3> getWorldPosition();
        std::array<double, 3> getWorldVelocity();
        std::array<double, 3> getWorldAcceleration();
        std::array<double, 3> getBodyAcceleration();
        std::array<double, 3> getBodyRate();
        bool getBodyQuaternion(std::array<double, 4>& quat, bool blocking);
        bool setCmdMode(DASCRobot::ControlMode mode);
        bool cmdWorldPosition(double x, double y, double z, double yaw, double yaw_rate);
        bool cmdWorldVelocity(double x, double y, double z, double yaw, double yaw_rate);
        bool cmdWorldAcceleration(double x, double y, double z, double yaw, double yaw_rate);
        bool cmdAttitude(double q_w, double q_x, double q_y, double q_z, double thrust);
        bool cmdRates(double roll, double pitch, double yaw, double thrust);
        bool cmdOffboardMode();
        /**
         * @brief Set the Global GPS coordinates reference for local frame
         * 
         * @param lat Latitude, (degrees)
         * @param lon Longitude, (degrees)
         * @param alt Altitude AMSL, (meters)
         */
        void setGPSGlobalOrigin(double lat, double lon, double alt);
    private:
        enum class AerialRobotServerState {
            kInit = 0,
            kReady,
            kPosition,
            kVelocity,
            kAcceleration,
            kAttitude,
            kRate,
            kControllerTimeout,
            kControllerTimeoutPositionHold,
            kFailSafe,
            kFailSafeLand,
        };
        ControlMode current_control_mode_;
        AerialRobotServerState server_state_;
        AerialRobotServerState last_server_state_;
        std::string robot_name_;
        const uint64_t vel_acc_timeout_ns_;
        const uint64_t att_rate_timeout_ns_;
        unsigned int controllerTimeoutCount_;
        uint8_t robot_id_;
        
        std::atomic<uint64_t> px4_timestamp_;
        std::atomic<uint64_t> last_publish_timestamp_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        std::mutex acc_queue_mutex_;
        std::mutex gyro_queue_mutex_;
        std::mutex quaternion_queue_mutex_;
        std::mutex world_position_queue_mutex_;
        std::mutex world_velocity_queue_mutex_;
        std::mutex world_acceleration_queue_mutex_;
        std::condition_variable quaternion_queue_cv_;

        std::queue<std::array<double, 3>> accelerometer_m_s2_queue_;
        std::queue<std::array<double, 3>> gyro_rad_queue_;
        std::queue<std::array<double, 4>> quaternion_queue_;
        std::queue<std::array<double, 3>> world_position_queue;
        std::queue<std::array<double, 3>> world_velocity_queue;
        std::queue<std::array<double, 3>> world_acceleration_queue;
        
        rclcpp::TimerBase::SharedPtr timer_;

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
        void positionFSMUpdate();
        void velocityFSMUpdate();
        void accelerationFSMUpdate();
        void attitudeFSMUpdate();
        void rateFSMUpdate();
        void controllerTimeoutFSMUpdate();
        void failsafeFSMUpdate();
        void emergencyStop();

        double clampToPi(double yaw);
        std::array<double, 4> ned_to_enu(const std::array<double, 4> &quat);
        std::array<double, 4> enu_to_ned(const std::array<double, 4> &quat);
};

#endif // DASC_ROBOT_DASCAERIALROBOT_HPP_
