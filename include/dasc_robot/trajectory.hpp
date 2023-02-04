

#ifndef _TRAJECTORIES
#define _TRAJECTORIES
#endif


#include <cstdlib>
#include <math.h>
#include <px4_msgs/msg/trajectory_setpoint.hpp>

namespace traj {

class CircularTrajectory{

public:
	double r;
	double z;
	double T;

	CircularTrajectory(double radius, double height, double period){
		r = radius;
		z = height;
		T = period;
	}

	TrajectorySetpoint setpoint(double time){
		TrajectorySetpoint sp;

		double w = 2*M_PI / T;

		sp.x = std::sin(w*time + M_PI/2);
		sp.y = std::sin(w*time);
		sp.z = 1.0;
		
		sp.vx = w*std::sin(w*time + M_PI/2 + M_PI/2);
		sp.vy = w*std::sin(w*time + M_PI/2);
		sp.vz = 0.0;

		sp.acceleration[0] = w*w*std::sin(w*time + 2*M_PI/2 + M_PI/2);
		sp.acceleration[1] = w*w*std::sin(w*time + 2*M_PI/2);
		sp.acceleration[2] = 0.0;
		
		sp.jerk[0] = w*w*w*std::sin(w*time + 3*M_PI/2 + M_PI/2);
		sp.jerk[1] = w*w*w*std::sin(w*time + 3*M_PI/2);
		sp.jerk[2] = 0.0;

		sp.yaw = 0.0;
		sp.yawspeed = 0.0;

		return sp;
	}


	std::array<double, 3> pos(double time){

		std::array<double, 3> p;
		p[0] = std::cos(2 * M_PI * time / T);
		p[1] = std::sin(2 * M_PI * time / T);
		p[2] = z;
		return p;
	}

};


class TrefoilTrajectory {

public:
	double sx;
	double sy;
	double sz;
	double T;
	double oz;

	TrefoilTrajectory(double scale_x, double scale_y, double scale_z, double offset_z, double period){
		sx = scale_x;
		sy = scale_y;
		sz = scale_z;
		oz = offset_z;
		T = period;
	}

	TrajectorySetpoint setpoint(double t){
		
		TrajectorySetpoint sp;

		double w = 2.0 * M_PI / T;


		sp.x = sx * (std::sin(w*t) + 2*std::sin(2*w*t) );
		sp.y = sy * (std::cos(w*t) - 2*std::cos(2*w*t) );
		sp.z = oz - sz * std::sin(3*w*t);
		
		sp.vx =  sx * w * (std::cos(w*t) + 4*std::cos(2*w*t) );
		sp.vy = -sy * w * (std::sin(w*t) - 4*std::sin(2*w*t) );
		sp.vz = -sz * w * 3 * std::cos(3*w*t);

		sp.acceleration[0] = - sx * w*w * (std::sin(w*t) + 8 * std::sin(2*w*t));
		sp.acceleration[1] = - sy * w*w * (std::cos(w*t) - 8 * std::cos(2*w*t));
		sp.acceleration[2] =   sz * w*w * 9 * std::sin(3*w*t);
		
		sp.jerk[0] = -sx * w*w*w * (std::cos(w*t) + 16 * std::cos(2*w*t) );
		sp.jerk[1] =  sy * w*w*w * (std::sin(w*t) - 16 * std::sin(2*w*t) );
		sp.jerk[2] =  sz * w*w*w * 27 * std::cos(3*w*t);

		sp.yaw = w*t;
		sp.yawspeed = w;

		return sp;
	}


};


} //namespace traj
