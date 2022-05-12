#ifndef DASC_ROBOT_DASCROBOT_HPP_
#define DASC_ROBOT_DASCROBOT_HPP_

#include <rclcpp/rclcpp.hpp>

class DASCRobot : virtual public rclcpp::Node {

    public:
        enum class ControlMode {
        kPositionMode = 0,
        kVelocityMode,
        kAccelerationMode,
        kAttitudeMode,
        kRateMode,
        };

        virtual bool init() = 0;
        virtual bool arm() = 0;
        virtual bool disarm() = 0;
        virtual std::array<double, 3> getWorldPosition() = 0;
        virtual std::array<double, 3> getWorldVelocity() = 0;
        virtual std::array<double, 3> getWorldAcceleration() = 0;
        virtual std::array<double, 3> getBodyAcceleration() = 0;
        virtual std::array<double, 3> getBodyRate() = 0;
        virtual bool getBodyQuaternion(std::array<double, 4>& quat, bool blocking) = 0;
        virtual bool setCmdMode(ControlMode mode) = 0;
        virtual bool cmdWorldPosition(double x, double y, double z, double yaw, double yaw_rate) = 0;
        virtual bool cmdWorldVelocity(double x, double y, double z, double yaw, double yaw_rate) = 0;
        virtual bool cmdWorldAcceleration(double x, double y, double z, double yaw, double yaw_rate) = 0;
        virtual bool cmdAttitude(double q_w, double q_x, double q_y, double q_z, double thrust) = 0;
        virtual bool cmdRates(double roll, double pitch, double yaw, double thrust) = 0;
    private:
        bool initialized_;
        ControlMode current_control_mode_;
};


#endif // DASC_ROBOT_DASCROBOT_HPP_
