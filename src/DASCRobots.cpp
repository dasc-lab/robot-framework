#include <thread>
#include "DASCRobots.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include "frame_transforms.h"
#include <cmath>

using std::placeholders::_1;
using namespace std::chrono_literals;

DASCRobot::DASCRobot(std::string robot_name, uint8_t id) : 
    Node(robot_name), 
    current_control_mode_(ControlMode::kPositionMode), 
    server_state_(RobotServerState::kInit),
    robot_name_(robot_name),
    vel_acc_timeout_ns_(5e8), // 500ms
    att_rate_timeout_ns_(1e8), // 100ms
    controllerTimeoutCount_(0),
    robot_id_(id) {
}

bool DASCRobot::init() {
    if (!rclcpp::ok()) {
        return false;
    }

    if (this->server_state_ != RobotServerState::kInit) {
        RCLCPP_ERROR(this->get_logger(), "DASC Aerial Robot Server already initialized!");
        return false;
    }

    auto sensor_qos = rclcpp::QoS(
        rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data),
        rmw_qos_profile_sensor_data
    );

    timesync_sub_ = this->create_subscription<px4_msgs::msg::Timesync>(
        "/" + this->robot_name_ + "/fmu/timesync/out", 
        sensor_qos,
        std::bind(&DASCRobot::timesyncCallback, this, _1)
    );

    sensor_combined_sub_ = this->create_subscription<px4_msgs::msg::SensorCombined>(
        this->robot_name_ + "/fmu/sensor_combined/out", 
        sensor_qos,
        std::bind(&DASCRobot::sensorCombinedCallback, this, _1)
    );

    vehicle_attitude_sub_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
        this->robot_name_ + "/fmu/vehicle_attitude/out", 
        sensor_qos,
        std::bind(&DASCRobot::vehicleAttitudeCallback, this, _1)
    );

    vehicle_local_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        this->robot_name_ + "/fmu/vehicle_local_position/out", 
        sensor_qos,
        std::bind(&DASCRobot::vehicleLocalPositionCallback, this, _1)
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

    vehicle_useExternalController_publisher_ = this->create_publisher<ExternalController>(
        this->robot_name_ + "/fmu/external_controller/in",
	sensor_qos
    );

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    auto timer_callback = [this]() -> void {
        this->updateState();
    };
    this->timer_ = this->create_wall_timer(100ms, timer_callback);
    this->server_state_ = RobotServerState::kReady;
    RCLCPP_INFO(this->get_logger(), "DASC Aerial Robot Server initialized!");
    return true;
}

bool DASCRobot::arm() {
    if (this->server_state_ == RobotServerState::kInit) {
        RCLCPP_ERROR(this->get_logger(), "Calling Arm with uninitialized server!");
        return false;
    }
    VehicleCommand msg;
    msg.timestamp = get_current_timestamp_us();
    msg.param1 = 1.0;
    msg.param2 = 0.0; // set param2 to 21196 to force arm/disarm operation
    msg.command = VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
	msg.target_system = this->robot_id_;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
    this->vehicle_command_publisher_->publish(msg);
    return true;
}

bool DASCRobot::disarm() {
    if (this->server_state_ == RobotServerState::kInit) {
        RCLCPP_ERROR(this->get_logger(), "Calling Disarm with uninitialized server!");
        return false;
    }
    VehicleCommand msg;
    msg.timestamp = get_current_timestamp_us();
    msg.param1 = 0.0;
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

std::array<double, 3> DASCRobot::getWorldPosition() {
    std::cout << "in world pos" << std::endl;
    if (this->server_state_ == RobotServerState::kInit) {
        RCLCPP_ERROR(this->get_logger(), "Calling getWorldPosition with uninitialized server!");
        return {NAN, NAN, NAN};
    }
    std::lock_guard<std::mutex> pos_guard(this->world_position_queue_mutex_);
    std::array<double, 3> pos = {NAN, NAN, NAN};
    if (this->world_position_queue.size() > 0) {
        pos = this->world_position_queue.front();
    }
    return pos;
}

bool DASCRobot::getWorldPosition(std::array<double, 3>& pos) {
    pos = getWorldPosition();
    return true;
}

std::array<double, 3> DASCRobot::getWorldVelocity() {
    if (this->server_state_ == RobotServerState::kInit) {
        RCLCPP_ERROR(this->get_logger(), "Calling getWorldVelocity with uninitialized server!");
        return {NAN, NAN, NAN};
    }
    std::lock_guard<std::mutex> vel_guard(this->world_velocity_queue_mutex_);
    std::array<double, 3> vel = {NAN, NAN, NAN};
    if (this->world_velocity_queue.size() > 0) {
        vel = this->world_velocity_queue.front();
    }
    return vel;
}

bool DASCRobot::getWorldVelocity(std::array<double, 3>& vel) {
    vel = getWorldVelocity();
    return true;
}

std::array<double, 3> DASCRobot::getWorldAcceleration() {
    if (this->server_state_ == RobotServerState::kInit) {
        RCLCPP_ERROR(this->get_logger(), "Calling getWorldAcceleration with uninitialized server!");
        return {NAN, NAN, NAN};
    }
    std::lock_guard<std::mutex> acc_guard(this->world_acceleration_queue_mutex_);
    std::array<double, 3> acc = {NAN, NAN, NAN};
    if (this->world_acceleration_queue.size() > 0) {
        acc = this->world_acceleration_queue.front();
    }
    return acc;
}

bool DASCRobot::getWorldAcceleration(std::array<double, 3>& acc) {
    acc = getWorldAcceleration();
    return true;
}

std::array<double, 3> DASCRobot::getBodyAcceleration() {
    if (this->server_state_ == RobotServerState::kInit) {
        RCLCPP_ERROR(this->get_logger(), "Calling getBodyAcceleration with uninitialized server!");
        return {NAN, NAN, NAN};
    }
    std::lock_guard<std::mutex> acc_guard(this->acc_queue_mutex_);
    std::array<double, 3> acc = {NAN, NAN, NAN};
    if (this->accelerometer_m_s2_queue_.size() > 0) {
        acc = this->accelerometer_m_s2_queue_.front();
    }
    return acc;
}

bool DASCRobot::getBodyAcceleration(std::array<double, 3>& acc) {
    acc = getBodyAcceleration();
    return true;
}

std::array<double, 3> DASCRobot::getBodyRate() {
    if (this->server_state_ == RobotServerState::kInit) {
        RCLCPP_ERROR(this->get_logger(), "Calling getBodyRate with uninitialized server!");
        return {NAN, NAN, NAN};
    }
    std::lock_guard<std::mutex> gyro_guard(this->gyro_queue_mutex_);
    std::array<double, 3> gyro = {NAN, NAN, NAN};
    if (this->gyro_rad_queue_.size() > 0) {
        gyro = this->gyro_rad_queue_.front();
    }
    return gyro;
}

bool DASCRobot::getBodyRate(std::array<double, 3>& brate) {
    brate = getBodyRate();
    return true;
}

bool DASCRobot::getBodyQuaternion(std::array<double, 4>& quat, bool blocking) {
    quat = {NAN, NAN, NAN, NAN};

    if (this->server_state_ == RobotServerState::kInit) {
        RCLCPP_ERROR(this->get_logger(), "Calling getBodyQuaternion with uninitialized server!");
        return false;
    }
    std::unique_lock<std::mutex> quat_ulock(this->quaternion_queue_mutex_);
    if (blocking && this->quaternion_queue_.size() == 0) {
        //RCLCPP_INFO(this->get_logger(), "Queue Empty Wait");
        this->quaternion_queue_cv_.wait(quat_ulock);
        //RCLCPP_INFO(this->get_logger(), "Wake");
    }
    if (this->quaternion_queue_.size() > 0) {
        quat = this->quaternion_queue_.front();
    }
    else {
        //RCLCPP_ERROR(this->get_logger(), "Quaternion queue empty");
        return false;
    }

    return true;
}


bool DASCRobot::useExternalController(bool mode){
    if (this->server_state_ == RobotServerState::kInit) {
        RCLCPP_ERROR(this->get_logger(), "Calling useExternalController with uninitialized server!");
        return false;
    }

    ExternalController msg;
    msg.timestamp = get_current_timestamp_us();
    msg.use_geometric_control = mode;

    // RCLCPP_INFO(this->get_logger(), "Setting Geometric Control use to %d", mode);
    vehicle_useExternalController_publisher_->publish(msg);
    
    return true;
}


bool DASCRobot::setCmdMode(ControlMode mode) {
    if (this->server_state_ == RobotServerState::kInit) {
        RCLCPP_ERROR(this->get_logger(), "Calling setCmdMode with uninitialized server!");
        return false;
    }
    switch (mode)
    {
    case ControlMode::kPositionMode:
        this->server_state_ = RobotServerState::kPosition;
        RCLCPP_INFO(this->get_logger(), "Server mode kPosition");
        break;
    
    case ControlMode::kVelocityMode:
        this->server_state_ = RobotServerState::kVelocity;
        RCLCPP_INFO(this->get_logger(), "Server mode kVelocity");
        break;

    case ControlMode::kAccelerationMode:
        this->server_state_ = RobotServerState::kAcceleration;
        RCLCPP_INFO(this->get_logger(), "Server mode kAcceleration");
        break;

    case ControlMode::kAttitudeMode:
        this->server_state_ = RobotServerState::kAttitude;
        RCLCPP_INFO(this->get_logger(), "Server mode kAttitude");
        break;

    case ControlMode::kRateMode:
        this->server_state_ = RobotServerState::kRate;
        RCLCPP_INFO(this->get_logger(), "Server mode kRate");
        break;

    default:
        RCLCPP_ERROR(this->get_logger(), "Unknown Control Mode %d", static_cast<int>(mode));
        return false;
    }
    //update current_timestamp
    this->last_publish_timestamp_ = this->get_clock()->now().nanoseconds();
    return true;
}

bool DASCRobot::cmdWorldPosition(double x, double y, double z, double yaw, double yaw_rate) {
    if (this->server_state_ == RobotServerState::kInit) {
        RCLCPP_ERROR(this->get_logger(), "Calling cmdWorldPosition with uninitialized server!");
        return false;
    }
    if (yaw > M_PI || yaw < -M_PI) {
        RCLCPP_ERROR(this->get_logger(), "cmdWorldPosition yaw: %f out of range [PI, -PI]", yaw);
        return false;
    }
    this->last_publish_timestamp_ = this->get_clock()->now().nanoseconds();
    TrajectorySetpoint msg;
    msg.timestamp = get_current_timestamp_us();
    msg.x = y;
    msg.y = x;
    msg.z = -z;
    msg.yaw = clampToPi(-yaw + M_PI_2);
    msg.yawspeed = -yaw_rate;
    msg.vx = NAN;
    msg.vy = NAN;
    msg.vz = NAN;
    msg.acceleration = {NAN, NAN, NAN};
    msg.jerk = {NAN, NAN, NAN};
    msg.thrust = {NAN, NAN, NAN};
    this->trajectory_setpoint_publisher_->publish(msg);
    return true;
}

bool DASCRobot::cmdWorldVelocity(double x, double y, double z, double yaw, double yaw_rate) {
    // std::cout << "commanding world velocity now" << std::endl;
    if (this->server_state_ == RobotServerState::kInit) {
        RCLCPP_ERROR(this->get_logger(), "Calling cmdWorldVelocity with uninitialized server!");
        return false;
    }
    this->last_publish_timestamp_ = this->get_clock()->now().nanoseconds();
    TrajectorySetpoint msg;
    msg.timestamp = get_current_timestamp_us();
    msg.x = NAN;
    msg.y = NAN;
    msg.z = NAN;
    msg.yaw = clampToPi(-yaw + M_PI_2);;
    msg.yawspeed = -yaw_rate;
    msg.vx = y;
    msg.vy = x;
    msg.vz = -z;
    msg.acceleration = {NAN, NAN, NAN};
    msg.jerk = {NAN, NAN, NAN};
    msg.thrust = {NAN, NAN, NAN};
    this->trajectory_setpoint_publisher_->publish(msg);
    return true;
}

bool DASCRobot::cmdLocalVelocity(double x, double y, double z, double yaw, double yaw_rate) {
    // std::cout << "Commanding velocity now" << std::endl;
    if (this->server_state_ == RobotServerState::kInit) {
        RCLCPP_ERROR(this->get_logger(), "Calling cmdLocalVelocity with uninitialized server!");
        return false;
    }
    this->last_publish_timestamp_ = this->get_clock()->now().nanoseconds();
    TrajectorySetpoint msg;
    msg.timestamp = get_current_timestamp_us();
    msg.x = NAN;
    msg.y = NAN;
    msg.z = NAN;
    msg.yaw = clampToPi(-yaw + M_PI_2);;
    msg.yawspeed = -yaw_rate;
    msg.vx = y;
    msg.vy = x;
    msg.vz = -z;
    msg.acceleration = {NAN, NAN, NAN};
    msg.jerk = {NAN, NAN, NAN};
    msg.thrust = {NAN, NAN, NAN};
    this->trajectory_setpoint_publisher_->publish(msg);
    return true;
}

bool DASCRobot::cmdWorldAcceleration(double x, double y, double z, double yaw, double yaw_rate) {
    if (this->server_state_ == RobotServerState::kInit) {
        RCLCPP_ERROR(this->get_logger(), "Calling cmdWorldAcceleration with uninitialized server!");
        return false;
    }
    this->last_publish_timestamp_ = this->get_clock()->now().nanoseconds();
    TrajectorySetpoint msg;
    msg.timestamp = get_current_timestamp_us();
    msg.x = NAN;
    msg.y = NAN;
    msg.z = NAN;
    msg.yaw = clampToPi(-yaw + M_PI_2);;
    msg.yawspeed = -yaw_rate;
    msg.vx = NAN;
    msg.vy = NAN;
    msg.vz = NAN;
    msg.acceleration[0] = y;
    msg.acceleration[1] = x;
    msg.acceleration[2] = -z;
    msg.jerk = {NAN, NAN, NAN};
    msg.thrust = {NAN, NAN, NAN};
    msg.yaw = clampToPi(-yaw + M_PI_2);
    this->trajectory_setpoint_publisher_->publish(msg);
    return true;
}

bool DASCRobot::cmdAttitude(double q_w, double q_x, double q_y, double q_z, double thrust) {
    if (this->server_state_ == RobotServerState::kInit) {
        RCLCPP_ERROR(this->get_logger(), "Calling cmdAttitude with uninitialized server!");
        return false;
    }
    this->last_publish_timestamp_ = this->get_clock()->now().nanoseconds();
    VehicleAttitudeSetpoint msg;
    msg.timestamp = get_current_timestamp_us();
    msg.roll_body = NAN;
    msg.pitch_body = NAN;
    msg.yaw_body = NAN;
    msg.yaw_sp_move_rate = NAN;
    std::array<double, 4> quat_enu = {q_w, q_x, q_y, q_z};
    std::array<double, 4> quat_ned = enu_to_ned(quat_enu);
    msg.q_d[0] = quat_ned[0];
    msg.q_d[1] = quat_ned[1];
    msg.q_d[2] = quat_ned[2];
    msg.q_d[3] = quat_ned[3];
    msg.thrust_body[2] = -thrust;
    this->vehicle_attitude_publisher_->publish(msg);
    return true;
}

bool DASCRobot::cmdRates(double roll, double pitch, double yaw, double thrust) {
    if (this->server_state_ == RobotServerState::kInit) {
        RCLCPP_ERROR(this->get_logger(), "Calling cmdRates with uninitialized server!");
        return false;
    }
    this->last_publish_timestamp_ = this->get_clock()->now().nanoseconds();
    VehicleRatesSetpoint msg;
    msg.timestamp = get_current_timestamp_us();
    msg.roll = roll;
    msg.pitch = -pitch;
    msg.yaw = -yaw;
    msg.thrust_body[2] = -thrust;
    this->vehicle_rates_publisher_->publish(msg);
    return true;
}

bool DASCRobot::cmdOffboardMode() {
    RCLCPP_INFO(this->get_logger(), "Commanding offboard mode");
    if (server_state_ == RobotServerState::kInit) {
        RCLCPP_ERROR(this->get_logger(), "Calling cmdOffboardMode with uninitialized server!");
        return false;
    }
    VehicleCommand msg;
    msg.timestamp = get_current_timestamp_us();
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

void DASCRobot::timesyncCallback(const px4_msgs::msg::Timesync::UniquePtr msg) {
    this->px4_timestamp_.store(msg->timestamp);
    this->px4_server_timestamp_.store(this->get_clock()->now().nanoseconds());
    // RCLCPP_INFO(this->get_logger(), "store before: %lu", (unsigned long)(msg->timestamp));
    // RCLCPP_INFO(this->get_logger(), "store ns: %lu", 1000 * (unsigned long)(msg->timestamp));
    // RCLCPP_INFO(this->get_logger(), "px4_server_timestamp_ ns: %lu", (unsigned long)(px4_server_timestamp_.load()));
    // RCLCPP_INFO(this->get_logger(), "Time Sync callback");
}

void DASCRobot::sensorCombinedCallback(const SensorCombined::UniquePtr msg) {
    std::array<double, 3> acc, gyro;
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

void DASCRobot::vehicleAttitudeCallback(const VehicleAttitude::UniquePtr msg) {
    std::array<double, 4> quat;
    //RCLCPP_INFO(this->get_logger(), "try lock quat");

    std::lock_guard<std::mutex> quat_guard(this->quaternion_queue_mutex_);
    quat[0] = msg->q[0];
    quat[1] = msg->q[1];
    quat[2] = msg->q[2];
    quat[3] = msg->q[3];
    auto quat_enu = ned_to_enu(quat);
    this->quaternion_queue_.push(quat_enu);
    if (this->quaternion_queue_.size() > 1) {
        this->quaternion_queue_.pop();
    }
    this->quaternion_queue_cv_.notify_all();
    // RCLCPP_INFO(this->get_logger(), "Vehicle Attitude callback");
}

void DASCRobot::vehicleLocalPositionCallback(const VehicleLocalPosition::UniquePtr msg) {
    std::array<double, 3> pos, vel, acc;
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

    std::array<double, 4> quat;
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

void DASCRobot::updateState() {
    // std::cout << "In update State for " << this->robot_id_ << std::endl;
    switch (this->server_state_)
    {
    case RobotServerState::kInit:
        RCLCPP_ERROR(this->get_logger(), "Server spinning in uninitialized state");
        return;
    
    case RobotServerState::kReady:
        break;

    case RobotServerState::kPosition:
        this->positionFSMUpdate();
        break;

    case RobotServerState::kVelocity:
        this->velocityFSMUpdate();
        break;

    case RobotServerState::kAcceleration:
        this->accelerationFSMUpdate();
        break;

    case RobotServerState::kAttitude:
        this->attitudeFSMUpdate();
        break;

    case RobotServerState::kRate:
        this->rateFSMUpdate();
        break;

    case RobotServerState::kControllerTimeout:
    case RobotServerState::kControllerTimeoutPositionHold:
        this->controllerTimeoutFSMUpdate();
        break;

    case RobotServerState::kFailSafe:
    case RobotServerState::kFailSafeLand:
        this->failsafeFSMUpdate();
        break;
    
    default:
        RCLCPP_ERROR(this->get_logger(), "Unknown server state %d", static_cast<int>(this->server_state_));
        return;
    }

    return;
}

void DASCRobot::positionFSMUpdate() {
    OffboardControlMode msg;
    msg.timestamp = get_current_timestamp_us();
    msg.position = true;
    offboard_control_mode_publisher_->publish(msg);
    // std::cout << "Send offboard position command \n"; 
}

void DASCRobot::velocityFSMUpdate() {
    rclcpp::Time current_time = this->get_clock()->now();
    if (current_time.nanoseconds() - this->last_publish_timestamp_ > this->vel_acc_timeout_ns_) {
        this->server_state_ = RobotServerState::kControllerTimeout;
        this->last_server_state_ = RobotServerState::kVelocity;
        RCLCPP_WARN(this->get_logger(), "Velocity Timeout, switch to ControllerTimeout state!");
    }
    else {
        OffboardControlMode msg;
        msg.timestamp = get_current_timestamp_us();
        msg.velocity = true;
        offboard_control_mode_publisher_->publish(msg);
        // std::cout << "Send offboard velocity command \n"; 
    }
}

void DASCRobot::accelerationFSMUpdate() {
    rclcpp::Time current_time = this->get_clock()->now();
    if (current_time.nanoseconds() - this->last_publish_timestamp_ > this->vel_acc_timeout_ns_) {
        this->server_state_ = RobotServerState::kControllerTimeout;
        this->last_server_state_ = RobotServerState::kAcceleration;
        RCLCPP_WARN(this->get_logger(), "Acceleration Timeout, switch to ControllerTimeout state!");
    }
    else {
        OffboardControlMode msg;
        msg.timestamp = get_current_timestamp_us();
        msg.acceleration = true;
        offboard_control_mode_publisher_->publish(msg);
    }
}

void DASCRobot::attitudeFSMUpdate() {
    rclcpp::Time current_time = this->get_clock()->now();
    if (current_time.nanoseconds() - this->last_publish_timestamp_ > this->att_rate_timeout_ns_) {
        this->server_state_ = RobotServerState::kControllerTimeout;
        this->last_server_state_ = RobotServerState::kAttitude;
        RCLCPP_WARN(this->get_logger(), "Attitude Timeout, switch to ControllerTimeout state!");
    }
    else {
        OffboardControlMode msg;
        msg.timestamp = get_current_timestamp_us();
        msg.attitude = true;
        offboard_control_mode_publisher_->publish(msg);
    }
}

void DASCRobot::rateFSMUpdate() {
    rclcpp::Time current_time = this->get_clock()->now();
    if (current_time.nanoseconds() - this->last_publish_timestamp_ > this->att_rate_timeout_ns_) {
        this->server_state_ = RobotServerState::kControllerTimeout;
        this->last_server_state_ = RobotServerState::kRate;
        RCLCPP_WARN(this->get_logger(), "Rate Timeout, switch to ControllerTimeout state!");
    }
    else {
        OffboardControlMode msg;
        msg.timestamp = get_current_timestamp_us();
        msg.body_rate = true;
        offboard_control_mode_publisher_->publish(msg);
    }
}

void DASCRobot::controllerTimeoutFSMUpdate() {
    if (this->server_state_ == RobotServerState::kControllerTimeout) {
        /**
         * @brief reference file for setting flight mode
         * https://github.com/PX4/PX4-Autopilot/blob/master/src/modules/commander/px4_custom_mode.h
         * https://github.com/PX4/PX4-Autopilot/blob/master/src/modules/commander/Commander.cpp
         */
        VehicleCommand msg;
        msg.timestamp = get_current_timestamp_us();
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
        this->server_state_ = RobotServerState::kControllerTimeoutPositionHold;
        RCLCPP_INFO(this->get_logger(), "Command Auto Loiter mode!");
    } 
    else if (this->server_state_ == RobotServerState::kControllerTimeoutPositionHold) {
        rclcpp::Time current_time = this->get_clock()->now();
        if (controllerTimeoutCount_++ > 100) { 
            this->server_state_ = RobotServerState::kFailSafe;
        }
        else if ((last_server_state_ == RobotServerState::kPosition || 
                  last_server_state_ == RobotServerState::kVelocity || 
                  last_server_state_ == RobotServerState::kAcceleration) && 
                  current_time.nanoseconds() - last_publish_timestamp_ < vel_acc_timeout_ns_) {
            this->server_state_ = last_server_state_;
        }
        else if ((last_server_state_ == RobotServerState::kAttitude || 
                  last_server_state_ == RobotServerState::kRate) &&
                  current_time.nanoseconds() - last_publish_timestamp_ < att_rate_timeout_ns_) {
            this->server_state_ = last_server_state_;
        }
    }
}

void DASCRobot::failsafeFSMUpdate() {
    if (this->server_state_ == RobotServerState::kFailSafe) {
        VehicleCommand msg;
        msg.timestamp = get_current_timestamp_us();
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
        this->server_state_ = RobotServerState::kFailSafeLand;
    }
    else if (this->server_state_ == RobotServerState::kFailSafeLand) {
        RCLCPP_INFO(this->get_logger(), "Server state FailSafeLand");
    }
    else {
        RCLCPP_ERROR(this->get_logger(), "Unknow server state %d", static_cast<int>(this->server_state_));
        return;
    }
}

void DASCRobot::emergencyStop() {
    VehicleCommand msg;
    msg.timestamp = get_current_timestamp_us();
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

void DASCRobot::setGPSGlobalOrigin(double lat, double lon, double alt) {
    /**
     * @brief Reference to following link
     * https://github.com/PX4/PX4-Autopilot/blob/master/msg/vehicle_command.msg
     */
    VehicleCommand msg;
    msg.timestamp = get_current_timestamp_us();
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

double DASCRobot::clampToPi(double yaw) {
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

std::array<double, 4> DASCRobot::ned_to_enu(const std::array<double, 4> &quat) {
    Eigen::Quaternion<double> quat_ned = Eigen::Quaternion<double>(quat[0], quat[1], quat[2], quat[3]);
    Eigen::Quaternion<double> quat_enu = px4_ros_com::frame_transforms::utils::quaternion::quaternion_from_euler(M_PI, M_PI, M_PI_2) * quat_ned;
    return {quat_enu.w(), quat_enu.x(), -quat_enu.y(), -quat_enu.z()};
}

std::array<double, 4> DASCRobot::enu_to_ned(const std::array<double, 4> &quat) {
    Eigen::Quaternion<double> quat_enu = Eigen::Quaternion<double>(quat[0], quat[1], quat[2], quat[3]);
    Eigen::Quaternion<double> quat_ned = px4_ros_com::frame_transforms::utils::quaternion::quaternion_from_euler(M_PI, M_PI, M_PI_2) * quat_enu;
    return {quat_ned.w(), quat_ned.x(), -quat_ned.y(), -quat_ned.z()};
}

/*
 Returns timestamp in microseconds

 Arguments 
 px4_sync: DEFAULT is true as most calls need syncing (to send commands or to get data)
           option2 is false when user wants ROS time for contriolling multiple robots and does not care about individual px4 syncing
*/
uint64_t DASCRobot::get_current_timestamp_us(bool px4_sync) {
    if (px4_sync){
        auto delta = (this->get_clock()->now().nanoseconds() - this->px4_server_timestamp_);// this is in ns

        // RCLCPP_INFO(this->get_logger(), "px4: %lu", (unsigned long)px4_timestamp_.load());
        // RCLCPP_INFO(this->get_logger(), "ros: %lu", (unsigned long)(this->get_clock()->now().nanoseconds()));
        // RCLCPP_INFO(this->get_logger(), "delta: %lu", (unsigned long)delta);
        // RCLCPP_INFO(this->get_logger(), "time: %lu", (unsigned long)(px4_timestamp_.load() + delta));

        return px4_timestamp_.load() + delta / 1000;
    }
    else{
        return this->get_clock()->now().nanoseconds() / 1000;
    }
}
