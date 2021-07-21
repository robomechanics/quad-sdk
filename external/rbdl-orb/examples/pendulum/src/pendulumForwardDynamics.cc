/*====================================================================
   Simple pendulum example
   Copyright (c) 2017 Matthew Millard <matthew.millard@iwr.uni-heidelberg.de>
   Licensed under the zlib license. See LICENSE for more details.
 *///=================================================================


#include <string>
#include <iostream>
#include <stdio.h> 
#include <rbdl/rbdl.h>
#include "csvtools.h"

#include <boost/numeric/odeint/stepper/runge_kutta_cash_karp54.hpp>
#include <boost/numeric/odeint/stepper/controlled_runge_kutta.hpp>
#include <boost/numeric/odeint/integrate/integrate_adaptive.hpp>
#include <boost/numeric/odeint/stepper/generation/make_controlled.hpp>
//using namespace std;
using namespace boost::numeric::odeint;


#ifndef RBDL_BUILD_ADDON_LUAMODEL
    #error "Error: RBDL addon LuaModel not enabled."
#endif

#include <rbdl/addons/luamodel/luamodel.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;



//====================================================================
// Boost stuff
//====================================================================

typedef std::vector< double > state_type;

typedef runge_kutta_cash_karp54< state_type > error_stepper_type;
typedef controlled_runge_kutta< error_stepper_type > controlled_stepper_type;


class rbdlToBoost {

    public:
        rbdlToBoost(Model* model) : model(model) {
            q = VectorNd::Zero(model->dof_count);
            qd = VectorNd::Zero(model->dof_count);
            qdd = VectorNd::Zero(model->dof_count);
            tau = VectorNd::Zero(model->dof_count);

        }

        //3c. Boost uses this 'operator()' function to evaluate the state
        //    derivative of the pendulum.
        void operator() (const state_type &x, 
                         state_type &dxdt, 
                         const double t){

            //3d. Here we split out q (generalized positions) and qd 
            //    (generalized velocities) from the x (state vector)
            //q
            int j = 0;            
            for(int i=0; i<model->dof_count; i++){                
                q[i] = (double)x[j];
                j++;
            }

            //qd
            for(int i=0; i<model->dof_count; i++){                
                qd[i] = (double)x[j];
                j++;
            }

            //3e. Here we set the applied generalized forces to zero
            for(int i=0; i<model->dof_count; i++){                
                tau[i] = 0;
            }

            //3f. RBDL's ForwardDynamics function is used to evaluate
            //    qdd (generalized accelerations)
            ForwardDynamics (*model,q,qd,tau,qdd);

            //3g. Here qd, and qdd are used to populate dxdt 
            //(the state derivative)
            j = 0;
            for(int i = 0; i < model->dof_count; i++){
                dxdt[j] = (double)qd[i];
                j++;
            }            
            for(int i = 0; i < model->dof_count; i++){
                dxdt[j] = (double)qdd[i];
                j++;
            }


        }

    private:
        Model* model;
        VectorNd q, qd, qdd, tau;
};

struct pushBackStateAndTime
{
    std::vector< state_type >& states;
    std::vector< double >& times;

    pushBackStateAndTime( std::vector< state_type > &states , 
                              std::vector< double > &times )
    : states( states ) , times( times ) { }

    void operator()( const state_type &x , double t )
    {
        states.push_back( x );
        times.push_back( t );
    }
};

void f(const state_type &x, state_type &dxdt, const double t);

/* Problem Constants */
int main (int argc, char* argv[]) {
    rbdl_check_api_version (RBDL_API_VERSION);


    //problem specific constants
    int     nPts    = 100;
    double  t0      = 0;
    double  t1      = 3;


    //Integration settings
    double absTolVal   = 1e-10;
    double relTolVal   = 1e-6;

    VectorNd q, qd;

    Model* model  = NULL;
    model         = new Model();

    //3a. The Lua model is read in here, and turned into a series of 
    //    vectors and matricies in model which RBDL uses to evaluate 
    //    dynamics quantities
    if (!Addons::LuaModelReadFromFile ("./../model/pendulum.lua", 
                                       model, false)             ){        
        std::cerr     << "Error loading model ./model/pendulum.lua" 
                    << std::endl;
        abort();
    }

    q       = VectorNd::Zero (model->dof_count);
    qd      = VectorNd::Zero (model->dof_count);

    double t        = 0;             //time
    double ts       = 0;            //scaled time
    double dtsdt    = M_PI/(t1-t0);    //dertivative scaled time 
                                    //w.r.t. time

    printf("DoF: %i\n",model->dof_count);


    printf("Forward Dynamics \n");

    //3b. Here we instantiate a wrapper class which is needed so that 
    //    Boost can evaluate the state derivative of the model.
    rbdlToBoost rbdlModel(model);
    state_type xState(2);
    int steps = 0;
    xState[0] = -M_PI/4.0;
    xState[1] = 0;


    double dt   = (t1-t0)/((double)nPts);    


    double ke, pe = 0;



    std::vector<std::vector< double > > matrixData;
    std::vector<std::vector< double > > matrixErrorData;
    std::vector< double > rowData(model->dof_count+1);
    std::vector< double > rowErrorData(2);

    double a_x = 1.0 , a_dxdt = 1.0;
    controlled_stepper_type  
    controlled_stepper(
        default_error_checker< double , 
                               range_algebra , 
                               default_operations >
        ( absTolVal , relTolVal , a_x , a_dxdt ) );
    

    double tp = 0;
    rowData[0] = 0;
    for(int z=0; z < model->dof_count; z++){
        rowData[z+1] = xState[model->dof_count + z];
    }
    matrixData.push_back(rowData);

    double kepe0 = 0;

    printf("Columns\n");
    printf("      t,         q,       qd,       ke,        pe,   ke+pe-(kepe0)\n");
    for(int i = 0; i <= nPts; i++){

        t = t0 + dt*i;

        //3h. Here we integrate forward in time between a series of nPts from
        //    t0 to t1
        integrate_adaptive( 
            controlled_stepper ,
            rbdlModel , xState , tp , t , (t-tp)/10 );
        tp = t;

        //3i. At each point the state, kinetic (ke), and potential energy (pe) 
        //    is evaluated. In this conservative system the sum of kinetic and
        //    potential energy should be constant. Any error that accumulates
        //    is due to the cumulation of integration error.

        q[0]  = xState[0];
        qd[0] = xState[1];

        pe = Utils::CalcPotentialEnergy(*model, q, true);
        ke = Utils::CalcKineticEnergy(*model, q, qd, true);


        printf("%f, %f, %f, %f, %f, %f\n",
                    t, q[0],qd[0],ke,
                    pe,(ke+pe-kepe0)); 

        rowData[0] = t;
        for(int z=0; z < model->dof_count; z++){
            rowData[z+1] = xState[z];
        }
        matrixData.push_back(rowData);

        if(i==0) kepe0 = (ke+pe);

        rowErrorData[0] = t;
        rowErrorData[1] = (ke + pe) - kepe0;
        matrixErrorData.push_back(rowErrorData);
    }
    printf("Columns\n");
    printf("      t,         q,       qd,       ke,        pe,      ke+pe-(kepe0)\n");

    //3j. Now the data we have accumulated is written to file
    std::string header = "";
    std::string fname   = "../output/meshup.csv";    
    printMatrixToFile(matrixData, header, fname);
    printf("Wrote: ../output/meshup.csv (meshup animation file)\n");

    fname               = "../output/kepe.csv";
    header = "time,systemEnergy,";
    printMatrixToFile(matrixErrorData,header,fname);
    printf("Wrote: ../output/kepe.csv (simulation data)\n");
    delete model;

    return 0;
        
}

