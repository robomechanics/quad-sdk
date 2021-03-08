#include <qpOASES.hpp>

int main(int argc, char** argv)
{
		USING_NAMESPACE_QPOASES

		//Quadratic prog param
		real_t H[7*7]; //here H = Id
		real_t A[6*7] = {-0.270792, -0.140175, -0.249325, -0.412277, -0.250952, -0.298071, 1.49078e-19, 0.534738, 0.00704892, 0.483388, -0.193107, 0.302895, -0.2471, -2.1684e-17, 0, -0.451464, 0.117068, -0.348254, 0.00703103, 0.00620952, -7.96211e-20, 0, 0.0502231, 0.393264, -0.584145, 0.758854, -0.637889, -0.023751, 0, 0.998738, -0.0197759, 0.766682, 0.632337, 0.769921, 0.00352803, 1, 4.89658e-12, 0.919213, 0.266407, -0.155855, 0.017872, -0.999712}; //the jacobian
		real_t g[7]; //set to 0
		real_t lb[7]; //joints lower velocity limits
		real_t ub[7]; //joints upper velocity limits
		//Position control - no orientation
		real_t lbA[6] = {0.00442514, -0.122699, -0.0725897, 0, 0, 0}; //stylus velocity : ubA = lbA
		real_t ubA[6] = {0.00442514, -0.122699, -0.0725897, 0, 0, 0};; //stylus velocity : ubA = lbA
		int_t nWSR = 1000; //specifies the maximum number of working set recalculation
		real_t cpu_time = 1.0; //maximum allowed CPU time(s) for the whole initialisation
		real_t xOpt[7];

		H[0] = 1.0;
		for(int i=1; i<7*7; ++i )
		{
			if (i%8 == 0)
				H[i] = 1.0;
			else
				H[i] = 0.0;
		}
		for (int i = 0 ; i < 7 ; i++)
		{
			g[i] = 0.0; 	//g = 0
		}

		//joints velocities limits
		lb[0] = -1.5; //s0 joints lower limits
		lb[1] = -1.5; //s1 joints lower limits
		lb[2] = -1.5; //e0 joints lower limits
		lb[3] = -1.5; //e1 joints lower limits
		lb[4] = -4.0; //w0 joints lower limits
		lb[5] = -4.0; //w1 joints lower limits
		lb[6] = -4.0; //w2 joints lower limits
		for (int i = 0 ; i < 7 ; i++)
		{
			ub[i] = -lb[i]; //joints lower limits
		}
		cpu_time = 1.0; //maximum amount of cpu time 
		nWSR = 100; 

		/*//linear and angular velocities
		the first three elements represent the linear velocity (px, py, pz) while
		the next three are the angular velocity (yaw, pitch, roll)*/
		for (int i = 0 ; i < 6 ; i++)
		{
			lbA[i] = 0.0;
			ubA[i] = lbA[i];
		}

		printf("Initialisation of the QP problem");
		SQProblem joints_velocities(7,6); //create qp problem of 7 variables and 6 constraints with the hessian matrix set with the identity
		joints_velocities.init(H,g,A,lb,ub,lbA,ubA, nWSR, &cpu_time);
		if (joints_velocities.isSolved())
			printf( "qp problem solved");
		if (joints_velocities.isInfeasible())
			printf( "qp problem not feasible");
		joints_velocities.getPrimalSolution( xOpt );
		printf( "\nxOpt = [ %e, %e, %e, %e, %e, %e, %e ];  objVal = %e\n\n", xOpt[0],xOpt[1], xOpt[2],xOpt[3], xOpt[4],xOpt[5], xOpt[6], joints_velocities.getObjVal() );


		A[0] = -0.284328;
		A[1] = -0.118524;
		A[2] = -0.190925; 
		A[3] = -0.350569; 
		A[4] = -0.318565; 
		A[5] = -0.229695; 
		A[6] = 3.93023e-19; 
		A[7] = 0.559749 ;
		A[8] = 0.0299793; 
		A[9] = 0.431302; 
		A[10] = -0.0760635; 
		A[11] = 0.234798; 
		A[12] = -0.311727; 
		A[13] = -9.11272e-17; 
		A[14] = 0; 
		A[15] = -0.403937; 
		A[16] = 0.293076;
		A[17] = -0.267321; 
		A[18] = 0.00656818; 
		A[19] = 0.00311694; 
		A[20] = -3.12555e-19; 
		A[21] = 0; 
		A[22] = 0.245215; 
		A[23] = 0.688118; 
		A[24] = -0.611048; 
		A[25] = 0.592551; 
		A[26] = -0.804865; 
		A[27] = -0.0181344; 
		A[28] = 0; 
		A[29] = 0.969469; 
		A[30] = -0.174051; 
		A[31] = 0.38453; 
		A[32] = 0.801775; 
		A[33] = 0.593226; 
		A[34] = 0.003365; 
		A[35] = 1; 
		A[36] = 4.89658e-12;
		A[37] = 0.704415; 
		A[38] = 0.691923; 
		A[39] = 0.0777117; 
		A[40] = 0.0165947; 
		A[41] = -0.99983;

		lbA[0] = -2.4834;
		lbA[1] =  -1.4414;
		lbA[2] =  -1.85119;
		lbA[3] =  0;
		lbA[4] =  0;
		lbA[5] =  0; 

		for (int i = 0 ; i < 6 ; i++)
		{
			ubA[i] = lbA[i];
		}
	
		nWSR = 100; 
		cpu_time = 1.0;
		joints_velocities.init(H,g,A,lb,ub,lbA,ubA, nWSR, &cpu_time);
		joints_velocities.getPrimalSolution( xOpt );
		printf( "\nxOpt = [ %e, %e, %e, %e, %e, %e, %e ];  objVal = %e\n\n", xOpt[0],xOpt[1], xOpt[2],xOpt[3], xOpt[4],xOpt[5], xOpt[6], joints_velocities.getObjVal() );
		return 0;
}
