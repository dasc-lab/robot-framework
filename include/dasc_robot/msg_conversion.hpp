#pragma once
#ifndef __SETPOINT_MSG_CONVERSION
#define __SETPOINT_MSG_CONVERSION

namespace px4_msgs::msg {
	
double clampToPi(double yaw) {
    return std::atan2(std::sin(yaw), std::cos(yaw));
}

DiffflatSetpoint convert(TrajectorySetpoint traj_msg){

	// needs to convert FRD (for traj_msg) to ENU (for flat_msg)

	DiffflatSetpoint flat_msg;
	flat_msg.timestamp = traj_msg.timestamp;
	flat_msg.pos[0] =  traj_msg.y;
	flat_msg.pos[1] =  traj_msg.x;
	flat_msg.pos[2] = -traj_msg.z;
	flat_msg.vel[0] =  traj_msg.vy;
	flat_msg.vel[1] =  traj_msg.vx;
	flat_msg.vel[2] = -traj_msg.vz;
	flat_msg.acc[0] =  traj_msg.acceleration[1];
	flat_msg.acc[1] =  traj_msg.acceleration[0];
	flat_msg.acc[2] = -traj_msg.acceleration[2];
	flat_msg.jerk[0] =  traj_msg.jerk[1];
	flat_msg.jerk[1] =  traj_msg.jerk[0];
	flat_msg.jerk[2] = -traj_msg.jerk[2];
	flat_msg.snap[0] = NAN;
	flat_msg.snap[1] = NAN;
	flat_msg.snap[2] = NAN;
	flat_msg.yaw = clampToPi(-traj_msg.yaw + M_PI_2); 
	flat_msg.yaw_rate = -traj_msg.yawspeed;
	flat_msg.yaw_accel = 0.0; 

	return flat_msg;
}

TrajectorySetpoint convert(DiffflatSetpoint flat_msg){

	// needs to convert FRD (for traj_msg) to ENU (for flat_msg)

	TrajectorySetpoint traj_msg;
	traj_msg.timestamp = flat_msg.timestamp;
	traj_msg.y               = flat_msg.pos[0];
	traj_msg.x               = flat_msg.pos[1];
	traj_msg.z               = -flat_msg.pos[2];
	traj_msg.vy              = flat_msg.vel[0];
	traj_msg.vx              = flat_msg.vel[1];
	traj_msg.vz              = -flat_msg.vel[2];
	traj_msg.acceleration[1] = flat_msg.acc[0];
	traj_msg.acceleration[0] = flat_msg.acc[1];
	traj_msg.acceleration[2] = -flat_msg.acc[2];
	traj_msg.jerk[1]         =  flat_msg.jerk[0];
	traj_msg.jerk[0]         =  flat_msg.jerk[1];
	traj_msg.jerk[2]         = -flat_msg.jerk[2];
	traj_msg.yaw             = clampToPi(-flat_msg.yaw + M_PI_2);
	traj_msg.yawspeed        = -flat_msg.yaw_rate;
	traj_msg.thrust[0]       = NAN;
	traj_msg.thrust[1]       = NAN;
	traj_msg.thrust[2]       = NAN;

	return traj_msg;
}

} // namespace px4_msgs::msg 

#endif //  __SETPOINT_MSG_CONVERSION
