#ifndef SPIRIT_TRAJECTORY_H
#define SPIRIT_TRAJECTORY_H

#include "eigen_types.h"

class Trajectory{
public:
	Trajectory(){}

	stacked_traj_t getStackedTrajectory()
	{
		stacked_traj_t res;
		for (int i = 0; i < MPC_STEPS; ++i)
		{
			res(i*NUM_STATES+0,0) = phi(i,0);
			res(i*NUM_STATES+1,0) = theta(i,0);
			res(i*NUM_STATES+2,0) = psi(i,0);
			res(i*NUM_STATES+3,0) = xpos(i,0);
			res(i*NUM_STATES+4,0) = ypos(i,0);
			res(i*NUM_STATES+5,0) = zpos(i,0);
			res(i*NUM_STATES+6,0) = phivel(i,0);
			res(i*NUM_STATES+7,0) = thetavel(i,0);
			res(i*NUM_STATES+8,0) = psivel(i,0);
			res(i*NUM_STATES+9,0) = xvel(i,0);
			res(i*NUM_STATES+10,0) = yvel(i,0);
			res(i*NUM_STATES+11,0) = zvel(i,0);
		}
    	return res;
	}

	traj_var_t phi;
	traj_var_t theta;
	traj_var_t psi;
	traj_var_t xpos;
	traj_var_t ypos;
	traj_var_t zpos;
	traj_var_t phivel;
	traj_var_t thetavel;
	traj_var_t psivel;
	traj_var_t xvel;
	traj_var_t yvel;
	traj_var_t zvel;
};

#endif