#include "DASCAerialRobot.hpp"

using std::placeholders::_1;

DASCAerialRobot::DASCAerialRobot(std::string robot_name) : 
    Node(robot_name), 
    initialized_(false), 
    current_control_mode_(ControlMode::kPositionMode), 
    server_state_(AerialRobotServerState::kInit),
    robot_name_(robot_name),
    pos_vel_acc_timeout_ms_(500),
    att_rate_timeout_ms_(100) {
}

bool DASCAerialRobot::init() {
    if (!rclcpp::ok()) {
        return false;
    }

    if (this->initialized_) {
        RCLCPP_ERROR(this->get_logger(), "DASC Aerial Robot Server already initialized!");
        return false;
    }

    auto sensor_qos = rclcpp::QoS(
        rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)
    );

    timesync_sub_ = this->create_subscription<px4_msgs::msg::Timesync>(
        this->robot_name_ + "/fmu/timesync/out", 
        sensor_qos,
        std::bind(&DASCAerialRobot::timesyncCallback, this, _1)
    );

    sensor_combined_sub_ = this->create_subscription<px4_msgs::msg::SensorCombined>(
        this->robot_name_ + "/fmu/sensor_combined/out", 
        sensor_qos,
        std::bind(&DASCAerialRobot::sensorCombinedCallback, this, _1)
    );

    vehicle_attitude_sub_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
        this->robot_name_ + "/fmu/timesync/out", 
        sensor_qos,
        std::bind(&DASCAerialRobot::vehicleAttitudeCallback, this, _1)
    );

    vehicle_local_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        this->robot_name_ + "/fmu/timesync/out", 
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
        this->robot_name_ + "/fmu/",
        sensor_qos
    );

    vehicle_rates_publisher_ = this->create_publisher<VehicleRatesSetpoint>(
        this->robot_name_ + "/fmu/",
        sensor_qos
    );
    RCLCPP_INFO(this->get_logger(), "DASC Aerial Robot Server initialized!");
    return true;
}

bool DASCAerialRobot::arm() {
    return true;
}

bool DASCAerialRobot::disarm() {
    return true;
}

std::array<float, 3> DASCAerialRobot::getWorldPosition() {
    std::lock_guard<std::mutex> pos_guard(this->world_position_queue_mutex_);
    return {0.0, 0.0, 0.0};
}

std::array<float, 3> DASCAerialRobot::getWorldVelocity() {
    std::lock_guard<std::mutex> vel_guard(this->world_velocity_queue_mutex_);
    return {0.0, 0.0, 0.0};
}

std::array<float, 3> DASCAerialRobot::getWorldAcceleration() {
    std::lock_guard<std::mutex> acc_guard(this->world_acceleration_queue_mutex_);
    return {0.0, 0.0, 0.0};
}

std::array<float, 3> DASCAerialRobot::getBodyAcceleration() {
    std::lock_guard<std::mutex> acc_guard(this->acc_queue_mutex_);
    return {0.0, 0.0, 0.0};
}

std::array<float, 3> DASCAerialRobot::getBodyRate() {
    std::lock_guard<std::mutex> gyro_guard(this->gyro_queue_mutex_);
    return {0.0, 0.0, 0.0};
}

std::array<float, 4> DASCAerialRobot::getBodyQuaternion() {
    std::lock_guard<std::mutex> quat_guard(this->quaternion_queue_mutex_);
    return {1.0, 0.0, 0.0, 0.0};
}

bool DASCAerialRobot::setCmdMode(ControlMode mode) {
    uint64_t current_timestamp;
    switch (mode)
    {
    case ControlMode::kPositionMode:
        this->server_state_ = AerialRobotServerState::kPosition;
        break;
    
    case ControlMode::kVelocityMode:
        this->server_state_ = AerialRobotServerState::kVelocity;
        break;

    case ControlMode::kAccelerationMode:
        this->server_state_ = AerialRobotServerState::kAcceleration;
        break;

    case ControlMode::kAttitudeMode:
        this->server_state_ = AerialRobotServerState::kAttitude;
        break;

    case ControlMode::kRateMode:
        this->server_state_ = AerialRobotServerState::kRate;
        break;

    default:
        RCLCPP_ERROR(this->get_logger(), "Unknow Control Mode %d", mode);
        return false;
    }
    //update current_timestamp
    this->last_publish_timestamp_ = current_timestamp;
    return true;
}

bool DASCAerialRobot::cmdWorldPosition(float x, float y, float z, float yaw) {
    return true;
}

bool DASCAerialRobot::cmdWorldVelocity(float x, float y, float z, float yaw_rate) {
    return true;
}

bool DASCAerialRobot::cmdWorldAcceleration(float x, float y, float z, float yaw_acceleration) {
    return true;
}

bool DASCAerialRobot::cmdAttitude(float q_w, float q_x, float q_y, float q_z, float thrust) {
    return true;
}

bool DASCAerialRobot::cmdRates(float roll, float pitch, float yaw, float thrust) {
    return true;
}

void DASCAerialRobot::timesyncCallback(const px4_msgs::msg::Timesync::UniquePtr msg) {
    this->timestamp_.store(msg->timestamp);
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
}

void DASCAerialRobot::vehicleAttitudeCallback(const VehicleAttitude::UniquePtr msg) {
    std::array<float, 4> quat;
    std::lock_guard<std::mutex> quat_guard(this->quaternion_queue_mutex_);
    quat[0] = msg->q[0];
    quat[1] = msg->q[1];
    quat[2] = msg->q[2];
    quat[3] = msg->q[3];
    this->quaternion_queue_.push(quat);
    if (this->quaternion_queue_.size() > 1) {
        this->quaternion_queue_.pop();
    }
}

void DASCAerialRobot::vehicleLocalPositionCallback(const VehicleLocalPosition::UniquePtr msg) {
    std::array<float, 3> pos, vel, acc;
    std::lock_guard<std::mutex> pos_guard(this->world_position_queue_mutex_);
    std::lock_guard<std::mutex> vel_guard(this->world_velocity_queue_mutex_);
    std::lock_guard<std::mutex> acc_guard(this->world_acceleration_queue_mutex_);
    
    // update position FRD to ROS
    if (msg->xy_valid && msg->z_valid) {
        pos[0] = msg->x;
        pos[1] = -msg->y;
        pos[2] = -msg->z;
        this->world_position_queue.push(pos);
        if (this->world_position_queue.size() > 1) {
            this->world_position_queue.pop();
        }
    }
    // update velocity 
    if (msg->v_xy_valid&& msg->v_z_valid) {
        vel[0] = msg->vx;
        vel[1] = -msg->vy;
        vel[2] = -msg->vz;
        this->world_velocity_queue.push(vel);
        if (this->world_velocity_queue.size() > 1) {
            this->world_velocity_queue.pop();
        }
    }
    //update acceleration
    acc[0] = msg->ax;
    acc[1] = -msg->ay;
    acc[2] = -msg->az;
    this->world_acceleration_queue.push(acc);
    if (this->world_acceleration_queue.size() > 1) {
        this->world_acceleration_queue.pop();
    }

}

void DASCAerialRobot::updateState() {
    if (this->server_state_ == AerialRobotServerState::kInit) {
        RCLCPP_ERROR(this->get_logger(), "Server spinning in uninitialized state");
        return;
    }
    else if (this->server_state_ == AerialRobotServerState::kPosition) {
        this->positionFSMUpdate();
    }
    else if (this->server_state_ == AerialRobotServerState::kVelocity) {
        this->velocityFSMUpdate();
    }
    else if (this->server_state_ == AerialRobotServerState::kAcceleration) {
        this->accelerationFSMUpdate();
    }
    else if (this->server_state_ == AerialRobotServerState::kAttitude) {
        this->attitudeFSMUpdate();
    }
    else if (this->server_state_ == AerialRobotServerState::kRate) {
        this->rateFSMUpdate();
    }
    else if (this->server_state_ == AerialRobotServerState::kControllerTimeout) {
        this->controllerTimeoutFSMUpdate();
    }
    else if (this->server_state_ == AerialRobotServerState::kFailSafe) {
        this->failsafeFSMUpdate();
    }
    else {
        RCLCPP_ERROR(this->get_logger(), "Unknow server state %d", this->server_state_);
        return;
    }

    return;
}

void DASCAerialRobot::positionFSMUpdate() {
    // che
}

int main(int argc, char* argv[]) {
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<DASCAerialRobot>("drone1"));

	rclcpp::shutdown();
	return 0;
}