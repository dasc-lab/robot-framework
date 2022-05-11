#include <thread>
#include "DASCAerialRobot.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

using std::placeholders::_1;
using namespace std::chrono_literals;

DASCAerialRobot::DASCAerialRobot(std::string robot_name, uint8_t id) : 
    Node(robot_name), 
    current_control_mode_(ControlMode::kPositionMode), 
    server_state_(AerialRobotServerState::kInit),
    robot_name_(robot_name),
    pos_vel_acc_timeout_ns_(5e8), // 500ms
    att_rate_timeout_ns_(1e8), // 100ms
    controllerTimeoutCount_(0),
    robot_id_(id) {
}

bool DASCAerialRobot::init() {
    if (!rclcpp::ok()) {
        return false;
    }

    if (this->server_state_ != AerialRobotServerState::kInit) {
        RCLCPP_ERROR(this->get_logger(), "DASC Aerial Robot Server already initialized!");
        return false;
    }

    auto sensor_qos = rclcpp::QoS(
        rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)
    );

    timesync_sub_ = this->create_subscription<px4_msgs::msg::Timesync>(
        "/" + this->robot_name_ + "/fmu/timesync/out", 
        sensor_qos,
        std::bind(&DASCAerialRobot::timesyncCallback, this, _1)
    );

    sensor_combined_sub_ = this->create_subscription<px4_msgs::msg::SensorCombined>(
        this->robot_name_ + "/fmu/sensor_combined/out", 
        sensor_qos,
        std::bind(&DASCAerialRobot::sensorCombinedCallback, this, _1)
    );

    vehicle_attitude_sub_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
        this->robot_name_ + "/fmu/vehicle_attitude/out", 
        sensor_qos,
        std::bind(&DASCAerialRobot::vehicleAttitudeCallback, this, _1)
    );

    vehicle_local_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        this->robot_name_ + "/fmu/vehicle_local_position/out", 
        sensor_qos,
        std::bind(&DASCAerialRobot::vehicleLocalPositionCallback, this, _1)
    );

    offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>(
        this->robot_name_ + "/fmu/offboard_control_mode/in", 
        sensor_qos
    );

    trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>(
        this->robot_name_ + "/fmu/trajectory_setpoint/in", 
        sensor_qos
    );

    vehicle_command_publisher_ = this->create_publisher<VehicleCommand>(
        this->robot_name_ + "/fmu/vehicle_command/in",
        sensor_qos
    );

    vehicle_attitude_publisher_ = this->create_publisher<VehicleAttitudeSetpoint>(
        this->robot_name_ + "/fmu/vehicle_attitude_setpoint/in",
        sensor_qos
    );

    vehicle_rates_publisher_ = this->create_publisher<VehicleRatesSetpoint>(
        this->robot_name_ + "/fmu/vehicle_rates_setpoint/in",
        sensor_qos
    );

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    auto timer_callback = [this]() -> void {
        this->updateState();
    };
    this->timer_ = this->create_wall_timer(100ms, timer_callback);
    this->server_state_ = AerialRobotServerState::kReady;
    RCLCPP_INFO(this->get_logger(), "DASC Aerial Robot Server initialized!");
    return true;
}

bool DASCAerialRobot::arm() {
    if (this->server_state_ == AerialRobotServerState::kInit) {
        RCLCPP_ERROR(this->get_logger(), "Calling Arm with uninitialized server!");
        return false;
    }
    VehicleCommand msg;
    msg.timestamp = px4_timestamp_.load();
    msg.param1 = 1.0;
    msg.param2 = 21196; // set param2 to 21196 to force arm/disarm operation
    msg.command = VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
	msg.target_system = this->robot_id_;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
    this->vehicle_command_publisher_->publish(msg);
    return true;
}

bool DASCAerialRobot::disarm() {
    if (this->server_state_ == AerialRobotServerState::kInit) {
        RCLCPP_ERROR(this->get_logger(), "Calling Disarm with uninitialized server!");
        return false;
    }
    VehicleCommand msg;
    msg.timestamp = px4_timestamp_.load();
    msg.param1 = 0.0;
    msg.command = VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
	msg.target_system = this->robot_id_;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
    this->vehicle_command_publisher_->publish(msg);
    return true;
}

std::array<float, 3> DASCAerialRobot::getWorldPosition() {
    if (this->server_state_ == AerialRobotServerState::kInit) {
        RCLCPP_ERROR(this->get_logger(), "Calling getWorldPosition with uninitialized server!");
        return {NAN, NAN, NAN};
    }
    std::lock_guard<std::mutex> pos_guard(this->world_position_queue_mutex_);
    std::array<float, 3> pos = {NAN, NAN, NAN};
    if (this->world_position_queue.size() > 0) {
        pos = this->world_position_queue.front();
        this->world_velocity_queue.pop();
    }
    return pos;
}

std::array<float, 3> DASCAerialRobot::getWorldVelocity() {
    if (this->server_state_ == AerialRobotServerState::kInit) {
        RCLCPP_ERROR(this->get_logger(), "Calling getWorldVelocity with uninitialized server!");
        return {NAN, NAN, NAN};
    }
    std::lock_guard<std::mutex> vel_guard(this->world_velocity_queue_mutex_);
    std::array<float, 3> vel = {NAN, NAN, NAN};
    if (this->world_velocity_queue.size() > 0) {
        vel = this->world_velocity_queue.front();
        this->world_velocity_queue.pop();
    }
    return vel;
}

std::array<float, 3> DASCAerialRobot::getWorldAcceleration() {
    if (this->server_state_ == AerialRobotServerState::kInit) {
        RCLCPP_ERROR(this->get_logger(), "Calling getWorldAcceleration with uninitialized server!");
        return {NAN, NAN, NAN};
    }
    std::lock_guard<std::mutex> acc_guard(this->world_acceleration_queue_mutex_);
    std::array<float, 3> acc = {NAN, NAN, NAN};
    if (this->world_acceleration_queue.size() > 0) {
        acc = this->world_acceleration_queue.front();
        this->world_acceleration_queue.pop();
    }
    return acc;
}

std::array<float, 3> DASCAerialRobot::getBodyAcceleration() {
    if (this->server_state_ == AerialRobotServerState::kInit) {
        RCLCPP_ERROR(this->get_logger(), "Calling getBodyAcceleration with uninitialized server!");
        return {NAN, NAN, NAN};
    }
    std::lock_guard<std::mutex> acc_guard(this->acc_queue_mutex_);
    std::array<float, 3> acc = {NAN, NAN, NAN};
    if (this->accelerometer_m_s2_queue_.size() > 0) {
        acc = this->accelerometer_m_s2_queue_.front();
        this->accelerometer_m_s2_queue_.pop();
    }
    return acc;
}

std::array<float, 3> DASCAerialRobot::getBodyRate() {
    if (this->server_state_ == AerialRobotServerState::kInit) {
        RCLCPP_ERROR(this->get_logger(), "Calling getBodyRate with uninitialized server!");
        return {NAN, NAN, NAN};
    }
    std::lock_guard<std::mutex> gyro_guard(this->gyro_queue_mutex_);
    std::array<float, 3> gyro = {NAN, NAN, NAN};
    if (this->gyro_rad_queue_.size() > 0) {
        gyro = this->gyro_rad_queue_.front();
        this->gyro_rad_queue_.pop();
    }
    return gyro;
}

bool DASCAerialRobot::getBodyQuaternion(std::array<float, 4>& quat, bool blocking) {
    quat = {NAN, NAN, NAN, NAN};
    if (this->server_state_ == AerialRobotServerState::kInit) {
        RCLCPP_ERROR(this->get_logger(), "Calling getBodyQuaternion with uninitialized server!");
        return false;
    }
    std::unique_lock<std::mutex> quat_ulock(this->quaternion_queue_mutex_);
    if (blocking && this->quaternion_queue_.size() == 0) {
        RCLCPP_INFO(this->get_logger(), "Queue Empty Wait");
        this->quaternion_queue_cv_.wait(quat_ulock);
        RCLCPP_INFO(this->get_logger(), "Wake");
    }
    if (this->quaternion_queue_.size() > 0) {
        RCLCPP_INFO(this->get_logger(), "Queue not empty");
        quat = this->quaternion_queue_.front();
        this->quaternion_queue_.pop();
    }
    else {
        RCLCPP_ERROR(this->get_logger(), "Quaternion queue empty");
        return false;
    }

    return true;
}

bool DASCAerialRobot::setCmdMode(ControlMode mode) {
    if (this->server_state_ == AerialRobotServerState::kInit) {
        RCLCPP_ERROR(this->get_logger(), "Calling setCmdMode with uninitialized server!");
        return false;
    }
    switch (mode)
    {
    case ControlMode::kPositionMode:
        this->server_state_ = AerialRobotServerState::kPosition;
        RCLCPP_INFO(this->get_logger(), "Server mode kPosition");
        break;
    
    case ControlMode::kVelocityMode:
        this->server_state_ = AerialRobotServerState::kVelocity;
        RCLCPP_INFO(this->get_logger(), "Server mode kVelocity");
        break;

    case ControlMode::kAccelerationMode:
        this->server_state_ = AerialRobotServerState::kAcceleration;
        RCLCPP_INFO(this->get_logger(), "Server mode kAcceleration");
        break;

    case ControlMode::kAttitudeMode:
        this->server_state_ = AerialRobotServerState::kAttitude;
        RCLCPP_INFO(this->get_logger(), "Server mode kAttitude");
        break;

    case ControlMode::kRateMode:
        this->server_state_ = AerialRobotServerState::kRate;
        RCLCPP_INFO(this->get_logger(), "Server mode kRate");
        break;

    default:
        RCLCPP_ERROR(this->get_logger(), "Unknow Control Mode %d", mode);
        return false;
    }
    //update current_timestamp
    this->last_publish_timestamp_ = this->get_clock()->now().nanoseconds();
    return true;
}

bool DASCAerialRobot::cmdWorldPosition(float x, float y, float z, float yaw) {
    if (this->server_state_ == AerialRobotServerState::kInit) {
        RCLCPP_ERROR(this->get_logger(), "Calling cmdWorldPosition with uninitialized server!");
        return false;
    }
    if (yaw > M_PI || yaw < -M_PI) {
        RCLCPP_ERROR(this->get_logger(), "cmdWorldPosition yaw: %f out of range [PI, -PI]", yaw);
        return false;
    }
    this->last_publish_timestamp_ = this->get_clock()->now().nanoseconds();
    TrajectorySetpoint msg;
    msg.timestamp = px4_timestamp_.load();
    msg.x = y;
    msg.y = x;
    msg.z = -z;
    msg.yaw = clampToPi(-yaw + M_PI_2);
    this->trajectory_setpoint_publisher_->publish(msg);
    return true;
}

bool DASCAerialRobot::cmdWorldVelocity(float x, float y, float z, float yaw_rate) {
    if (this->server_state_ == AerialRobotServerState::kInit) {
        RCLCPP_ERROR(this->get_logger(), "Calling cmdWorldVelocity with uninitialized server!");
        return false;
    }
    this->last_publish_timestamp_ = this->get_clock()->now().nanoseconds();
    TrajectorySetpoint msg;
    msg.timestamp = px4_timestamp_.load();
    msg.vx = y;
    msg.vy = x;
    msg.vz = -z;
    msg.yawspeed = -yaw_rate;
    this->trajectory_setpoint_publisher_->publish(msg);
    return true;
}

bool DASCAerialRobot::cmdWorldAcceleration(float x, float y, float z, float yaw) {
    if (this->server_state_ == AerialRobotServerState::kInit) {
        RCLCPP_ERROR(this->get_logger(), "Calling cmdWorldAcceleration with uninitialized server!");
        return false;
    }
    this->last_publish_timestamp_ = this->get_clock()->now().nanoseconds();
    TrajectorySetpoint msg;
    msg.timestamp = px4_timestamp_.load();
    msg.acceleration[0] = y;
    msg.acceleration[1] = x;
    msg.acceleration[2] = -z;
    msg.yaw = clampToPi(-yaw + M_PI_2);
    this->trajectory_setpoint_publisher_->publish(msg);
    return true;
}

bool DASCAerialRobot::cmdAttitude(float q_w, float q_x, float q_y, float q_z, float thrust) {
    if (this->server_state_ == AerialRobotServerState::kInit) {
        RCLCPP_ERROR(this->get_logger(), "Calling cmdAttitude with uninitialized server!");
        return false;
    }
    this->last_publish_timestamp_ = this->get_clock()->now().nanoseconds();
    VehicleAttitudeSetpoint msg;
    msg.timestamp = px4_timestamp_.load();
    msg.q_d[0] = q_w;
    msg.q_d[1] = q_y;
    msg.q_d[2] = q_x;
    msg.q_d[3] = -q_z;
    msg.thrust_body[2] = -thrust;
    this->vehicle_attitude_publisher_->publish(msg);
    return true;
}

bool DASCAerialRobot::cmdRates(float roll, float pitch, float yaw, float thrust) {
    if (this->server_state_ == AerialRobotServerState::kInit) {
        RCLCPP_ERROR(this->get_logger(), "Calling cmdRates with uninitialized server!");
        return false;
    }
    this->last_publish_timestamp_ = this->get_clock()->now().nanoseconds();
    VehicleAttitudeSetpoint msg;
    msg.timestamp = px4_timestamp_.load();
    msg.roll_body = roll;
    msg.pitch_body = pitch;
    msg.yaw_body = yaw;
    msg.thrust_body[2] = -thrust;
    this->vehicle_attitude_publisher_->publish(msg);
    return true;
}

bool DASCAerialRobot::cmdOffboardMode() {
    if (server_state_ == AerialRobotServerState::kInit) {
        RCLCPP_ERROR(this->get_logger(), "Calling cmdOffboardMode with uninitialized server!");
        return false;
    }
    VehicleCommand msg;
    msg.timestamp = px4_timestamp_.load();
    msg.command = VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
    msg.param1 = 1;
    msg.param2 = 6; // PX4_CUSTOM_MAIN_MODE_OFFBOARD
    msg.target_system = this->robot_id_;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    this->vehicle_command_publisher_->publish(msg);
    return true;
}

void DASCAerialRobot::timesyncCallback(const px4_msgs::msg::Timesync::UniquePtr msg) {
    this->px4_timestamp_.store(msg->timestamp);
    // RCLCPP_INFO(this->get_logger(), "Time Sync callback");
}

void DASCAerialRobot::sensorCombinedCallback(const SensorCombined::UniquePtr msg) {
    std::array<float, 3> acc, gyro;
    std::lock_guard<std::mutex> acc_guard(this->acc_queue_mutex_);
    std::lock_guard<std::mutex> gyro_guard(this->gyro_queue_mutex_);
    // msg->accelerometer_m_s2 in FRD, convert to ROS
    acc[0] = msg->accelerometer_m_s2[0];
    acc[1] = -msg->accelerometer_m_s2[1];
    acc[2] = -msg->accelerometer_m_s2[2];
    // msg->gyro_rad in FRD, convert to ROS
    gyro[0] = msg->gyro_rad[0];
    gyro[1] = -msg->gyro_rad[1];
    gyro[2] = -msg->gyro_rad[2];
    this->accelerometer_m_s2_queue_.push(acc);
    if (this->accelerometer_m_s2_queue_.size() > 1) {
        this->accelerometer_m_s2_queue_.pop();
    }
    this->gyro_rad_queue_.push(gyro);
    if (this->gyro_rad_queue_.size() > 1) {
        this->gyro_rad_queue_.pop();
    }
    // RCLCPP_INFO(this->get_logger(), "Sensor Combined callback");
}

void DASCAerialRobot::vehicleAttitudeCallback(const VehicleAttitude::UniquePtr msg) {
    std::array<float, 4> quat;
    RCLCPP_INFO(this->get_logger(), "try lock quat");
    std::lock_guard<std::mutex> quat_guard(this->quaternion_queue_mutex_);
    RCLCPP_INFO(this->get_logger(), "lock quat");
    quat[0] = msg->q[0];
    quat[1] = msg->q[1];
    quat[2] = msg->q[2];
    quat[3] = msg->q[3];
    this->quaternion_queue_.push(quat);
    if (this->quaternion_queue_.size() > 1) {
        this->quaternion_queue_.pop();
    }
    this->quaternion_queue_cv_.notify_all();
    RCLCPP_INFO(this->get_logger(), "notify all");
    // RCLCPP_INFO(this->get_logger(), "Vehicle Attitude callback");
}

void DASCAerialRobot::vehicleLocalPositionCallback(const VehicleLocalPosition::UniquePtr msg) {
    std::array<float, 3> pos, vel, acc;
    std::lock_guard<std::mutex> pos_guard(this->world_position_queue_mutex_);
    std::lock_guard<std::mutex> vel_guard(this->world_velocity_queue_mutex_);
    std::lock_guard<std::mutex> acc_guard(this->world_acceleration_queue_mutex_);

    // update position FRD to ROS
    if (msg->xy_valid && msg->z_valid) {
        pos[0] = msg->y;
        pos[1] = msg->x;
        pos[2] = -msg->z;
        this->world_position_queue.push(pos);
        if (this->world_position_queue.size() > 1) {
            this->world_position_queue.pop();
        }
    }
    // update velocity 
    if (msg->v_xy_valid&& msg->v_z_valid) {
        vel[0] = msg->vy;
        vel[1] = msg->vx;
        vel[2] = -msg->vz;
        this->world_velocity_queue.push(vel);
        if (this->world_velocity_queue.size() > 1) {
            this->world_velocity_queue.pop();
        }
    }
    //update acceleration
    acc[0] = msg->ay;
    acc[1] = msg->ax;
    acc[2] = -msg->az;
    this->world_acceleration_queue.push(acc);
    if (this->world_acceleration_queue.size() > 1) {
        this->world_acceleration_queue.pop();
    }

    std::array<float, 4> quat;
    bool success = this->getBodyQuaternion(quat, false);
    if (success) {
        geometry_msgs::msg::TransformStamped transform_msg;
        transform_msg.header.stamp = this->get_clock()->now();
        transform_msg.header.frame_id = "world";
        transform_msg.child_frame_id = this->robot_name_;
        transform_msg.transform.translation.x = pos[0];
        transform_msg.transform.translation.y = pos[1];
        transform_msg.transform.translation.z = pos[2];
        transform_msg.transform.rotation.w = quat[0];
        transform_msg.transform.rotation.x = quat[1];
        transform_msg.transform.rotation.y = quat[2];
        transform_msg.transform.rotation.z = quat[3];

        this->tf_broadcaster_->sendTransform(transform_msg);
    }

    // RCLCPP_INFO(this->get_logger(), "Vehicle Local Position callback");
}

void DASCAerialRobot::updateState() {
    switch (this->server_state_)
    {
    case AerialRobotServerState::kInit:
        RCLCPP_ERROR(this->get_logger(), "Server spinning in uninitialized state");
        return;
    
    case AerialRobotServerState::kReady:
        break;

    case AerialRobotServerState::kPosition:
        this->positionFSMUpdate();
        break;

    case AerialRobotServerState::kVelocity:
        this->velocityFSMUpdate();
        break;

    case AerialRobotServerState::kAcceleration:
        this->accelerationFSMUpdate();
        break;

    case AerialRobotServerState::kAttitude:
        this->attitudeFSMUpdate();
        break;

    case AerialRobotServerState::kRate:
        this->rateFSMUpdate();
        break;

    case AerialRobotServerState::kControllerTimeout:
    case AerialRobotServerState::kControllerTimeoutPositionHold:
        this->controllerTimeoutFSMUpdate();
        break;

    case AerialRobotServerState::kFailSafe:
    case AerialRobotServerState::kFailSafeLand:
        this->failsafeFSMUpdate();
        break;
    
    default:
        RCLCPP_ERROR(this->get_logger(), "Unknow server state %d", this->server_state_);
        return;
    }

    return;
}

void DASCAerialRobot::positionFSMUpdate() {
    rclcpp::Time current_time = this->get_clock()->now();
    if (current_time.nanoseconds() - this->last_publish_timestamp_ > this->pos_vel_acc_timeout_ns_) {
        this->server_state_ = AerialRobotServerState::kControllerTimeout;
        this->last_server_state_ = AerialRobotServerState::kPosition;
        RCLCPP_WARN(this->get_logger(), "Position Timeout, switch to ControllerTimeout state!");
    }
    else {
        OffboardControlMode msg;
        msg.timestamp = px4_timestamp_.load();
        msg.position = true;
        offboard_control_mode_publisher_->publish(msg);
    }
}

void DASCAerialRobot::velocityFSMUpdate() {
    rclcpp::Time current_time = this->get_clock()->now();
    if (current_time.nanoseconds() - this->last_publish_timestamp_ > this->pos_vel_acc_timeout_ns_) {
        this->server_state_ = AerialRobotServerState::kControllerTimeout;
        this->last_server_state_ = AerialRobotServerState::kVelocity;
        RCLCPP_WARN(this->get_logger(), "Velocity Timeout, switch to ControllerTimeout state!");
    }
    else {
        OffboardControlMode msg;
        msg.timestamp = px4_timestamp_.load();
        msg.velocity = true;
        offboard_control_mode_publisher_->publish(msg);
    }
}

void DASCAerialRobot::accelerationFSMUpdate() {
    rclcpp::Time current_time = this->get_clock()->now();
    if (current_time.nanoseconds() - this->last_publish_timestamp_ > this->pos_vel_acc_timeout_ns_) {
        this->server_state_ = AerialRobotServerState::kControllerTimeout;
        this->last_server_state_ = AerialRobotServerState::kAcceleration;
        RCLCPP_WARN(this->get_logger(), "Acceleration Timeout, switch to ControllerTimeout state!");
    }
    else {
        OffboardControlMode msg;
        msg.timestamp = px4_timestamp_.load();
        msg.acceleration = true;
        offboard_control_mode_publisher_->publish(msg);
    }
}

void DASCAerialRobot::attitudeFSMUpdate() {
    rclcpp::Time current_time = this->get_clock()->now();
    if (current_time.nanoseconds() - this->last_publish_timestamp_ > this->att_rate_timeout_ns_) {
        this->server_state_ = AerialRobotServerState::kControllerTimeout;
        this->last_server_state_ = AerialRobotServerState::kAttitude;
        RCLCPP_WARN(this->get_logger(), "Attitude Timeout, switch to ControllerTimeout state!");
    }
    else {
        OffboardControlMode msg;
        msg.timestamp = px4_timestamp_.load();
        msg.attitude = true;
        offboard_control_mode_publisher_->publish(msg);
    }
}

void DASCAerialRobot::rateFSMUpdate() {
    rclcpp::Time current_time = this->get_clock()->now();
    if (current_time.nanoseconds() - this->last_publish_timestamp_ > this->att_rate_timeout_ns_) {
        this->server_state_ = AerialRobotServerState::kControllerTimeout;
        this->last_server_state_ = AerialRobotServerState::kRate;
        RCLCPP_WARN(this->get_logger(), "Rate Timeout, switch to ControllerTimeout state!");
    }
    else {
        OffboardControlMode msg;
        msg.timestamp = px4_timestamp_.load();
        msg.body_rate = true;
        offboard_control_mode_publisher_->publish(msg);
    }
}

void DASCAerialRobot::controllerTimeoutFSMUpdate() {
    if (this->server_state_ == AerialRobotServerState::kControllerTimeout) {
        /**
         * @brief reference file for setting flight mode
         * https://github.com/PX4/PX4-Autopilot/blob/master/src/modules/commander/px4_custom_mode.h
         * https://github.com/PX4/PX4-Autopilot/blob/master/src/modules/commander/Commander.cpp
         */
        VehicleCommand msg;
        msg.timestamp = px4_timestamp_.load();
        msg.command = VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
        msg.param1 = 1;
        msg.param2 = 4; // PX4_CUSTOM_MAIN_MODE_AUTO
        msg.param3 = 3; // PX4_CUSTOM_SUB_MODE_AUTO_LOITER
        msg.target_system = this->robot_id_;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        this->vehicle_command_publisher_->publish(msg);

        controllerTimeoutCount_ = 0;
        this->server_state_ = AerialRobotServerState::kControllerTimeoutPositionHold;
        RCLCPP_INFO(this->get_logger(), "Command Auto Loiter mode!");
    } 
    else if (this->server_state_ == AerialRobotServerState::kControllerTimeoutPositionHold) {
        rclcpp::Time current_time = this->get_clock()->now();
        if (controllerTimeoutCount_++ > 100) { 
            this->server_state_ = AerialRobotServerState::kFailSafe;
        }
        else if ((last_server_state_ == AerialRobotServerState::kPosition || 
                  last_server_state_ == AerialRobotServerState::kVelocity || 
                  last_server_state_ == AerialRobotServerState::kAcceleration) && 
                  current_time.nanoseconds() - last_publish_timestamp_ < pos_vel_acc_timeout_ns_) {
            this->server_state_ = last_server_state_;
        }
        else if ((last_server_state_ == AerialRobotServerState::kAttitude || 
                  last_server_state_ == AerialRobotServerState::kRate) &&
                  current_time.nanoseconds() - last_publish_timestamp_ < att_rate_timeout_ns_) {
            this->server_state_ = last_server_state_;
        }
    }
}

void DASCAerialRobot::failsafeFSMUpdate() {
    if (this->server_state_ == AerialRobotServerState::kFailSafe) {
        VehicleCommand msg;
        msg.timestamp = px4_timestamp_.load();
        msg.command = VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
        msg.param1 = 1;
        msg.param2 = 4; // PX4_CUSTOM_MAIN_MODE_AUTO
        msg.param3 = 6; // PX4_CUSTOM_SUB_MODE_AUTO_LAND
        msg.target_system = this->robot_id_;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        this->vehicle_command_publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Command Vehicle Auto Land");
        this->server_state_ = AerialRobotServerState::kFailSafeLand;
    }
    else if (this->server_state_ == AerialRobotServerState::kFailSafeLand) {
        RCLCPP_INFO(this->get_logger(), "Server state FailSafeLand");
    }
    else {
        RCLCPP_ERROR(this->get_logger(), "Unknow server state %d", this->server_state_);
        return;
    }
}

void DASCAerialRobot::emergencyStop() {
    VehicleCommand msg;
    msg.timestamp = px4_timestamp_.load();
    msg.command = VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
    msg.param1 = VehicleCommand::ARMING_ACTION_DISARM;
    msg.param2 = 21196; // set param2 to 21196 to force arm/disarm operation
    msg.target_system = this->robot_id_;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
    this->vehicle_command_publisher_->publish(msg);
}

void DASCAerialRobot::setGPSGlobalOrigin(float lat, float lon, float alt) {
    /**
     * @brief Reference to following link
     * https://github.com/PX4/PX4-Autopilot/blob/master/msg/vehicle_command.msg
     */
    VehicleCommand msg;
    msg.timestamp = px4_timestamp_.load();
    msg.command = VehicleCommand::VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN;
    msg.param5 = lat;
    msg.param6 = lon;
    msg.param7 = alt;
    msg.target_system = this->robot_id_;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
    this->vehicle_command_publisher_->publish(msg);
}

float DASCAerialRobot::clampToPi(float yaw) {
    if (yaw > M_PI_2) {
        while (yaw > M_PI_2) {
            yaw -= M_PI;
        }
    }
    else {
        while (yaw < -M_PI_2) {
            yaw += M_PI;
        }
    }
    return yaw;
}

int main(int argc, char* argv[]) {
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
    auto drone1 = std::make_shared<DASCAerialRobot>("drone1", 1);
    // auto drone2 = std::make_shared<DASCAerialRobot>("drone2", 2);
    // auto drone3 = std::make_shared<DASCAerialRobot>("drone3", 3);
    // auto drone4 = std::make_shared<DASCAerialRobot>("drone4", 4);
    drone1->init();
    // drone2->init();
    // drone3->init();
    // drone4->init();
    rclcpp::executors::MultiThreadedExecutor server_exec;
    server_exec.add_node(drone1);
    // server_exec.add_node(drone2);
    // server_exec.add_node(drone3);
    // server_exec.add_node(drone4);
    auto server_spin_exec = [&server_exec]() {
        server_exec.spin();
    };
    std::thread server_exec_thread(server_spin_exec);
    std::cout << "Node start" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    float x = 0, y = 0, z = 2;
    drone1->setCmdMode(DASCRobot::ControlMode::kPositionMode);
    // drone2->setCmdMode(DASCRobot::ControlMode::kPositionMode);
    // drone3->setCmdMode(DASCRobot::ControlMode::kPositionMode);
    // drone4->setCmdMode(DASCRobot::ControlMode::kPositionMode);
    drone1->setGPSGlobalOrigin(47.3977419, 8.5455950, 488.105);
    // drone2->setGPSGlobalOrigin(47.3977419, 8.5455950, 488.105);
    // drone3->setGPSGlobalOrigin(47.3977419, 8.5455950, 488.105);
    // drone4->setGPSGlobalOrigin(47.3977419, 8.5455950, 488.105);
    for (int i = 0; i < 20; i++) {
        drone1->cmdWorldPosition(x + 1, y - 1, z, 0.0);
        // drone2->cmdWorldPosition(x + 1, y + 1, z, 0.0);
        // drone3->cmdWorldPosition(x - 1, y + 1, z, 0.0);
        // drone4->cmdWorldPosition(x - 1, y - 1, z, 0.0);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    drone1->cmdOffboardMode();
    drone1->arm();
    // drone2->cmdOffboardMode();
    // drone2->arm();
    // drone3->cmdOffboardMode();
    // drone3->arm();
    // drone4->cmdOffboardMode();
    // drone4->arm();
    std::cout << "Arm" << std::endl;
    for(int i = 0; i < 10; i++) {
        std::array<float, 4> quat;
        drone1->getBodyQuaternion(quat, true);
        std::cout << quat[0] << quat[1] << quat[2] << quat[3] << std::endl;
    }
    // for (int i = 0; i < 200; i++) {
    //     drone1->cmdWorldPosition(x + 1, y - 1, z, 0.0);
    //     // drone2->cmdWorldPosition(x + 1, y + 1, z, 0.0);
    //     // drone3->cmdWorldPosition(x - 1, y + 1, z, 0.0);
    //     // drone4->cmdWorldPosition(x - 1, y - 1, z, 0.0);
    //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // }
    // std::this_thread::sleep_for(std::chrono::milliseconds(20000));

    server_exec_thread.join();
    rclcpp::shutdown();
	return 0;
}