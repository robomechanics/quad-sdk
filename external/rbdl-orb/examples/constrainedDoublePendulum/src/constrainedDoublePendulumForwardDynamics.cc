/*====================================================================
 * Simple pendulum example
 * Copyright (c) 2015 Matthew Millard 
 * <matthew.millard@iwr.uni-heidelberg.de>
 *
 *///=================================================================


#include <string>
#include <iostream>
#include <iomanip>
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
#include <rbdl/addons/luamodel/luatables.h>


using namespace std;
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
        rbdlToBoost(Model& model,std::vector<ConstraintSet>& cs
                        ) : model(model),cs(cs) {
            q = VectorNd::Zero(model.dof_count);
            qd = VectorNd::Zero(model.dof_count);
            qdd = VectorNd::Zero(model.dof_count);
            tau = VectorNd::Zero(model.dof_count);

        }

        void operator() (const state_type &x, 
                         state_type &dxdt, 
                         const double t){

            //q
            int j = 0;
            for(unsigned int i=0; i<model.dof_count; i++){
                q[i] = (double)x[j];
                j++;
            }

            //qd
            for(unsigned int i=0; i<model.dof_count; i++){
                qd[i] = (double)x[j];
                j++;
            }
            //tau = for now. This could call a control
            //               function.
            for(unsigned int i=0; i<model.dof_count; i++){
                tau[i] = 0;
            }

            //2c. A special forward dynamics function needs to be called in
            //    order to compute qdd which simultaneously satisfies the 
            //    constraint set and the equations of motion.
            ForwardDynamicsConstraintsDirect (model, q, qd, tau, cs[0], qdd);


            //populate dxdt
            j = 0;
            for(unsigned int i = 0; i < model.dof_count; i++){
                dxdt[j] = (double)qd[i];
                j++;
            }
            for(unsigned int i = 0; i < model.dof_count; i++){
                dxdt[j] = (double)qdd[i];
                j++;
            }

        }

    private:
        Model& model;
        std::vector< RigidBodyDynamics::ConstraintSet >& cs;
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
    double absTolVal   = 1e-6;
    double relTolVal   = 1e-6;

    VectorNd q, qd, qdd, tau;

    RigidBodyDynamics::Model model;
    string modelFile = "./../model/constrainedDoublePendulum.lua";

    //2a. Some extra work is needed to load in the model, and the 
    //    constraint sets
    std::vector<std::string> constraintSetNames = 
        Addons::LuaModelGetConstraintSetNames(modelFile.c_str());
    std::vector<RigidBodyDynamics::ConstraintSet> constraintSets;

    constraintSets.resize(constraintSetNames.size());
    
    if (! Addons::LuaModelReadFromFileWithConstraints(
                                      modelFile.c_str(),
                                      &model,
                                      constraintSets,
                                      constraintSetNames,
                                      false)            ){
        std::cerr     << "Error loading model" << std::endl;
        abort();
    }


    q       = VectorNd::Zero (model.dof_count);
    qd      = VectorNd::Zero (model.dof_count);
    qdd     = VectorNd::Zero (model.dof_count);
    tau     = VectorNd::Zero (model.dof_count);

    printf("DoF: %i\n",model.dof_count);


    printf("==============================\n");
    printf("1. Forward Dynamics \n");
    printf("==============================\n");    

    // rx0 x1,y1,z1,rx1,ry1,rz1
    //  0,  1, 2, 3,  4,  5,  6,
    q[2] = 1.0;

    rbdlToBoost rbdlModel(model,constraintSets);
    state_type xState(model.dof_count*2);

    int j = 0;
    for(unsigned int i = 0; i< (model.dof_count); ++i){
        xState[j++] = q(i);
    }
    for(unsigned int i = 0; i< (model.dof_count); ++i){
        xState[j++] = qd(i);
    }

    double dt   = (t1-t0)/((double)nPts);    


    double ke, pe = 0;


    std::vector<std::vector< double > > matrixData;
    std::vector<std::vector< double > > matrixPlotData;
    std::vector< double > rowData(model.dof_count+1);
    std::vector< double > rowPlotData(4);

    double a_x = 1.0 , a_dxdt = 1.0;
    controlled_stepper_type  
    controlled_stepper(
        default_error_checker< double , 
                               range_algebra , 
                               default_operations >
        ( absTolVal , relTolVal , a_x , a_dxdt ) );
    

    double tp = 0;
    rowData[0] = 0;
    for(unsigned int z=0; z < model.dof_count; z++){
        rowData[z+1] = xState[model.dof_count + z];
    }
    matrixData.push_back(rowData);
    double t = 0;

    VectorNd constraintPosError = VectorNd::Zero(5);
    VectorNd constraintVelError = VectorNd::Zero(5);

    double constraintPosNorm, constraintVelNorm;

    printf("Columns\n");
    printf("      t,        ke,       pe,    ke+pe ,norm(cons_pos), norm(cons_vel)\n");
    for(int i = 0; i <= nPts; i++){
       

        t = t0 + dt*i;


        integrate_adaptive( 
            controlled_stepper ,
            rbdlModel , xState , tp , t , (t-tp)/10000 );
        tp = t;

        j = 0;
        for(unsigned int k = 0; k < model.dof_count; ++k){
            q[k] = xState[j++];
        }
        for(unsigned int k = 0; k < model.dof_count; ++k){
            qd[k] = xState[j++];
        }

        //Ensure that the constraint set quantites are up to date
        ForwardDynamicsConstraintsDirect (model, q, qd, tau,
          constraintSets[0], qdd);


        //2d. The position-level and velocity level constraint set errors
        //    can be fished out of the vectors in the ConstraintSet struct
        //
        //    Note: In the coming months there will be some functions added so that you
        //          can easily access this information.
        //
        //    Homework: After reading Featherstone & Orin Sec. 3.4, go and have a look
        //              at the fields in the ConstraintSet struct in 
        //              rbdl-orb/include/rbdl/Constraints.h
        //
        //              At the same time, open the doxygen for RBDL and read the sections
        //              on Constraints and ConstraintSets        
        ConstraintSet &ci = constraintSets[0];        
        constraintPosNorm = 0;
        constraintVelNorm = 0;
        for(int k=0; k<5;++k){
          constraintPosError(k) = ci.err(k);
          constraintVelError(k) = ci.errd(k);

          constraintPosNorm += ci.err(k)*ci.err(k);
          constraintVelNorm += ci.errd(k)*ci.errd(k);
        }
        constraintPosNorm = sqrt(constraintPosNorm);
        constraintVelNorm = sqrt(constraintVelNorm);

        pe = Utils::CalcPotentialEnergy(model, q, true);
        ke = Utils::CalcKineticEnergy(model, q, qd, true);


        printf("%f, %f, %f, %f, %f, %f\n",
                    t, ke,pe, ke+pe, constraintPosNorm, constraintVelNorm);

        rowData[0] = t;
        for(unsigned int z=0; z < model.dof_count; z++){
            rowData[z+1] = xState[z];
        }
        matrixData.push_back(rowData);


        rowPlotData[0] = t;
        rowPlotData[1] = constraintPosNorm;
        rowPlotData[2] = constraintVelNorm;
        rowPlotData[3] =(ke+pe);

        matrixPlotData.push_back(rowPlotData);
    }
    printf("Columns\n");
    printf("      t,        ke,       pe,    ke+pe ,norm(cons_pos), norm(cons_vel)\n");


    std::string header = "";
    std::string fname   = "../output/meshup.csv";
    printMatrixToFile(matrixData, header, fname);
    printf("Wrote: ../output/meshup.csv (meshup animation file)\n");

    fname   = "../output/simulationData.csv";
    header = "time,constraintPositionNorm,constraintVelocityNorm,systemEnergy,";
    printMatrixToFile(matrixPlotData, header, fname);
    printf("Wrote: ../output/simulationData.csv (error data)\n");


    return 0;
        
}

