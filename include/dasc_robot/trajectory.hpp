

#ifndef _TRAJECTORIES
#define _TRAJECTORIES
#endif


#include <cstdlib>
#include <math.h>

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

	std::array<double, 3> pos(double time){

		std::array<double, 3> p;
		p[0] = std::cos(2 * M_PI * time / T);
		p[1] = std::sin(2 * M_PI * time / T);
		p[2] = z;

		return p;

	}

};


} //namespace traj
