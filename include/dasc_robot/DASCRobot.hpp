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
        virtual std::array<float, 3> getWorldPosition() = 0;
        virtual std::array<float, 3> getWorldVelocity() = 0;
        virtual std::array<float, 3> getWorldAcceleration() = 0;
        virtual std::array<float, 3> getBodyAcceleration() = 0;
        virtual std::array<float, 3> getBodyRate() = 0;
        virtual std::array<float, 4> getBodyQuaternion() = 0;
        virtual bool setCmdMode(ControlMode mode) = 0;
        virtual bool cmdWorldPosition(float x, float y, float z, float yaw) = 0;
        virtual bool cmdWorldVelocity(float x, float y, float z, float yaw_rate) = 0;
        virtual bool cmdWorldAcceleration(float x, float y, float z, float yaw_acceleration) = 0;
        virtual bool cmdAttitude(float q_w, float q_x, float q_y, float q_z, float thrust) = 0;
        virtual bool cmdRates(float roll, float pitch, float yaw, float thrust) = 0;
    private:
        bool initialized_;
        ControlMode current_control_mode_;
};


#endif // DASC_ROBOT_DASCROBOT_HPP_
