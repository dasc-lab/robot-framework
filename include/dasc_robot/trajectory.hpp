

#ifndef _TRAJECTORIES
#define _TRAJECTORIES
#endif


#include <cstdlib>
#include <math.h>
#include <px4_msgs/msg/diffflat_setpoint.hpp>

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

	DiffflatSetpoint setpoint(double time){
		DiffflatSetpoint sp;

		double w = 2*M_PI / T;

		sp.pos[0] = std::sin(w*time + M_PI/2);
		sp.pos[1] = std::sin(w*time);
		sp.pos[2] = 1.0;
		
		sp.vel[0] = w*std::sin(w*time + M_PI/2 + M_PI/2);
		sp.vel[1] = w*std::sin(w*time + M_PI/2);
		sp.vel[2] = 0.0;

		sp.acc[0] = w*w*std::sin(w*time + 2*M_PI/2 + M_PI/2);
		sp.acc[1] = w*w*std::sin(w*time + 2*M_PI/2);
		sp.acc[2] = 0.0;
		
		sp.jerk[0] = w*w*w*std::sin(w*time + 3*M_PI/2 + M_PI/2);
		sp.jerk[1] = w*w*w*std::sin(w*time + 3*M_PI/2);
		sp.jerk[2] = 0.0;
		
		sp.snap[0] = w*w*w*w*std::sin(w*time + 4*M_PI/2 + M_PI/2);
		sp.snap[1] = w*w*w*w*std::sin(w*time + 4*M_PI/2);
		sp.snap[2] = 0.0;

		sp.yaw = 0.0;
		sp.yaw_rate = 0.0;
		sp.yaw_accel = 0.0;

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

	DiffflatSetpoint setpoint(double t){
		
		DiffflatSetpoint sp;

		double w = 2.0 * M_PI / T;


		sp.pos[0] = sx * (std::sin(w*t) + 2*std::sin(2*w*t) );
		sp.pos[1] = sy * (std::cos(w*t) - 2*std::cos(2*w*t) );
		sp.pos[2] = oz - sz * std::sin(3*w*t);
		
		sp.vel[0] =  sx * w * (std::cos(w*t) + 4*std::cos(2*w*t) );
		sp.vel[1] = -sy * w * (std::sin(w*t) - 4*std::sin(2*w*t) );
		sp.vel[2] = -sz * w * 3 * std::cos(3*w*t);

		sp.acc[0] = - sx * w*w * (std::sin(w*t) + 8 * std::sin(2*w*t));
		sp.acc[1] = - sy * w*w * (std::cos(w*t) - 8 * std::cos(2*w*t));
		sp.acc[2] =   sz * w*w * 9 * std::sin(3*w*t);
		
		sp.jerk[0] = -sx * w*w*w * (std::cos(w*t) + 16 * std::cos(2*w*t) );
		sp.jerk[1] =  sy * w*w*w * (std::sin(w*t) - 16 * std::sin(2*w*t) );
		sp.jerk[2] =  sz * w*w*w * 27 * std::cos(3*w*t);

		sp.snap[0] = NAN;
		sp.snap[1] = NAN;
		sp.snap[2] = NAN;


		sp.yaw = w*t;
		sp.yaw_rate = w;
		sp.yaw_accel = 0;

		return sp;
	}


};


} //namespace traj
