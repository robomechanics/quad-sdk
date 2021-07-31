/*====================================================================
   bouncingBallBenchmark.cc
   Copyright (c) 2019 Matthew Millard <matthew.millard@iwr.uni-heidelberg.de>
   Licensed under the zlib license. See LICENSE for more details.
 *///=================================================================


#include <string>
#include <iostream>
#include <stdio.h> 
#include <rbdl/rbdl.h>
#include <rbdl/addons/luamodel/luamodel.h>
#include <rbdl/addons/geometry/geometry.h>
#include "csvtools.h"

#include "ContactToolkit.h"

#include <boost/numeric/odeint/stepper/runge_kutta_cash_karp54.hpp>
#include <boost/numeric/odeint/stepper/controlled_runge_kutta.hpp>
#include <boost/numeric/odeint/integrate/integrate_adaptive.hpp>
#include <boost/numeric/odeint/stepper/generation/make_controlled.hpp>
//using namespace std;
using namespace boost::numeric::odeint;

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
    rbdlToBoost(Model* model,
                std::string &ballName,
                double ballRadius,
                Vector3d &pointOnPlane, 
                Vector3d &planeNormal,
                double stiffness,
                double exponent,
                double damping,
                double staticFrictionSpeed,
                double staticFrictionCoefficient,
                double dynamicFrictionSpeed,
                double dynamicFrictionCoefficient,
                double viscousFrictionSlope,
                unsigned int numberOfWorkTermsInState
                ):model(model),r(ballRadius),r0P0(pointOnPlane),
                  eN0(planeNormal),k(stiffness),p(exponent),beta(damping),
                  staticFrictionSpeed(staticFrictionSpeed),
                  staticFrictionCoefficient(staticFrictionCoefficient),
                  dynamicFrictionSpeed(dynamicFrictionSpeed),
                  dynamicFrictionCoefficient(dynamicFrictionCoefficient),
                  viscousFrictionSlope(viscousFrictionSlope),
                  numberOfWorkTermsInState(numberOfWorkTermsInState)
    {

        q = VectorNd::Zero(model->dof_count);
        qd = VectorNd::Zero(model->dof_count);
        qdd = VectorNd::Zero(model->dof_count);
        tau = VectorNd::Zero(model->dof_count);
        fext.resize(model->mBodies.size());
        for(unsigned int i=0; i<fext.size();++i){
            fext[i]=SpatialVector::Zero();
        }
        ballId = model->GetBodyId(ballName.c_str());
        fK0n = Vector3dZero;
        tK0n = Vector3dZero;
        fK0t = Vector3dZero;
        tK0t = Vector3dZero;

        //1a. The regularized friction model is created and printed to file
        ContactToolkit::createRegularizedFrictionCoefficientCurve(
                          staticFrictionSpeed, staticFrictionCoefficient,
                          dynamicFrictionSpeed,dynamicFrictionCoefficient,
                          viscousFrictionSlope,"mu",frictionCoefficientCurve);

        frictionCoefficientCurve.printCurveToCSVFile("../output/",
                           "frictionCoefficientCurve",0,dynamicFrictionSpeed*2);
        printf("Wrote: ../output/frictionCoefficentCurve.csv\n");

        //Set the velocity at which the relaxed method is used to compute
        //the direction vector of the tangential velocity of the contact
        //point
        veps = staticFrictionSpeed/100.0;

        //If veps is too small, we really might have problems
        assert(veps > std::sqrt(std::numeric_limits<double>::epsilon()));


    }

    //1b. The state derivative for this model is called by Boost whenever the 
    //    code for the 'operator()' function is called
    void operator() (const state_type &x,
                     state_type &dxdt, 
                     const double t){

        //1c. The state x is split into generalized positions (q), generalized
        //    velocities (qdd). Here tau is set to 0 because we are not 
        //    applying any generlized forces: the ball contact/friction forces
        //    are applied as external forces
        //q
        int j = 0;
        for(unsigned int i=0; i<model->q_size; i++){                
            q[i] = double(x[j]);
            j++;
        }

        //qd
        for(unsigned int i=0; i<model->qdot_size; i++){
            qd[i] = double(x[j]);
            j++;
        }

        //tau = 0
        for(unsigned int i=0; i<model->qdot_size; i++){                
            tau[i] = 0;
        }

        //2a. To evaluate the contact forces we must see if the ball is interpenetrating
        //    the ground. Outside of FEA modelling it is common to allow (non deforming)
        //    geometry to intepenetrate and to use the interpenetration to compute
        //    contact forces. Here we go with the simplest option and map the 
        //    depth of penetration to a contact force.
        r0B0 = CalcBodyToBaseCoordinates(*model,q,ballId,Vector3dZero,true);
        ContactToolkit::calcSpherePlaneContactPointPosition(r0B0,r,eN0,r0K0);

        //if the contact point is in the sphere compute contact data
        z = (r0K0-r0P0).dot(eN0);
        dz=0.; //this is zero until contact is made
        if( z < 0. ){

          //2b. If the sphere is in contact with the plane, we must get the position and
          //velocity of the contact point.

          //Get the point of contact resolved in the coordinates of the ball          
          EB0   = CalcBodyWorldOrientation(*model,q,ballId,true);
          rBKB  = EB0*(r0K0-r0B0);

          //Get the velocity of the point of contact
          v0K0 = CalcPointVelocity(*model,q,qd,ballId,rBKB,true);

          //Evaluate Hunt-Crossley Contact forces
          dz = (v0K0).dot(eN0); //assuming the plane is fixed.

          //2c. Now we can evaluate the contact forces using a Hunt-Crossley contact model
          //    This is a popular contact model because it is numerically well behaved.
          //    See the comments in ContactToolkit.h above the function definition
          //    for calcHuntCrossleyContactForce for further details.          
          ContactToolkit::calcHuntCrossleyContactForce(z,dz,k,p,beta,hcInfo);

          //2d. To apply this contact force as an external force in RBDL, we must first
          //    transform it into a spatial wrench that is resolved in the Root frame.
          //    Here we turn the scalar contact force into a vector, and then evaluate
          //    the torque that this force vector produces about the Root frame  
          fK0n = hcInfo.force*eN0;
          tK0n = VectorCrossMatrix(r0K0)*fK0n;

          //2e. Now we can use the tangential velocity of the contact point of the ball
          //    to evaluate the coefficient of friction, and then the friction forces
          v0K0t = v0K0 - dz*eN0;
          ContactToolkit::calcTangentialVelocityDirection(v0K0t,veps,eT0);

          mu = frictionCoefficientCurve.calcValue(v0K0t.norm());

          //2f. As with the contact model we evaluate the force the friction model applies
          //    to the ball and also the torque it generates about the ROOT frame          
          fK0t = -mu*hcInfo.force*eT0;
          tK0t = VectorCrossMatrix(r0K0)*fK0t;

          //2g. The total wrench generated by the contact and friction models is applied to
          //    the entry in the vector fext that corresponds to the ball.            
          fext[ballId][0] = tK0n[0] + tK0t[0];
          fext[ballId][1] = tK0n[1] + tK0t[1];
          fext[ballId][2] = tK0n[2] + tK0t[2];

          fext[ballId][3] = fK0n[0] + fK0t[0];
          fext[ballId][4] = fK0n[1] + fK0t[1];
          fext[ballId][5] = fK0n[2] + fK0t[2];

        }else{
          //zero the entry of fext associated with the ball.          
          fext[ballId]=SpatialVector::Zero();
          hcInfo.force        = 0.;
          hcInfo.springForce  = 0.;
          hcInfo.dampingForce = 0.;          
          fK0n = Vector3dZero;
          tK0n = Vector3dZero;
          fK0t = Vector3dZero;
          tK0t = Vector3dZero;
        }

        //3a. Now the generalized accelerations of the ball can be computed
        ForwardDynamics(*model,q,qd,tau,qdd,&fext);

        //3b. The state derivative dxdt is now formed using qd, and qdd. The 
        //    derivatives of the  work done by the contact and friction models 
        //    is also stored so that we  can track the system energy of the 
        //    system through the simulation
        j = 0;
        for(unsigned int i = 0; i < model->q_size; i++){
            dxdt[j] = double(qd[i]);
            j++;
        }
        for(unsigned int i = 0; i < model->qdot_size; i++){
            dxdt[j] = double(qdd[i]);
            j++;
        }

        dworkN = hcInfo.force*dz;
        dxdt[j] = dworkN;
        j++;
        dxdt[j] = fK0t.dot(v0K0t);
        j++;
        assert((numberOfWorkTermsInState
                + model->q_size 
                + model->qdot_size) == j);

    }

    /*
      Ascii Vector Notation:
      rBKB
        r: vector
        B: from the origin of the Ball frame
        K: to the contact point K
        B: expressed in the coordinates of the ball frame.
      EB0:
        E: rotation matrix
        B: to Frame B
        0: from Frame 0
      eN0:
        e: unit vector
        N: Normal direction
        0: expressed in the coordinates of the root frame (0)
      fK0
        f: force
        K: At point K
        0: expressed in the coordinates of the root frame
      tK0
        t: torque
        K: At point K
        0: expressed in the coordinates of the root frame

    */

    //Multibody Variables
    Model* model;
    VectorNd q, qd, qdd, tau;    

    //Normal-Contact Model Working Variables
    unsigned int ballId;
    double z, dz; //pentration depth and velocity
    HuntCrossleyContactInfo hcInfo;
    std::vector< SpatialVector > fext;
    double dworkN, dworkT; //Work in the normal and tangential directions
    double mu;      //Friction coefficient
    Matrix3d EB0;   //Orientation of the ball expressed in the Root frame
    Vector3d r0B0;  //Position of the ball
    Vector3d rBKB;  //B : ball.
    Vector3d r0K0;  //K : contact point
    Vector3d v0K0;  //velocity of the contact point
    Vector3d v0K0t; //tangential velocity of the contact point
    Vector3d r0P0;  //Origin of the plane
    Vector3d eN0;   //Normal of the plane
    Vector3d eT0;   //Tangental direction of the plane: in 2d this isn't necessary
                    //here we compute it to show how this is done in a
                    //numerically stable way in 3d.
    Vector3d fK0n, tK0n; //contact force and moment
    Vector3d fK0t, tK0t; //tangential friction force and moment

    //Normal Contact-Model Parameters
    double r; //ball radius
    double k; //stiffness
    double p; //exponential power on the spring compression
    double beta; //damping

    //Friction Model Parameters: see ContactToolkit 
    // createRegularizedFrictionCoefficientCurve for details
    double staticFrictionSpeed;
    double staticFrictionCoefficient;
    double dynamicFrictionSpeed;
    double dynamicFrictionCoefficient;
    double viscousFrictionSlope;
    double veps; //Velocity at which the relaxed method is used to compute
                 //the direction vector of the tangential velocity
    RigidBodyDynamics::Addons::Geometry
        ::SmoothSegmentedFunction frictionCoefficientCurve;

    unsigned int numberOfWorkTermsInState;

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

  RigidBodyDynamics::Model model;

  std::string fileName("../model/ballPlaneContact.lua");
  std::string ballName("Ball");

  //0a. ballPlaneContact.lua model is read in
  if (!Addons::LuaModelReadFromFile(fileName.c_str(),&model)){
    std::cerr << "Error loading LuaModel: " << fileName << std::endl;
    abort();
  }

  //0b. Contact and friction parameters are set (more on these later)
  double radius = 0.5;
  Vector3d pointOnPlane = Vector3d(0.,0.,0.);
  Vector3d planeNormal  = Vector3d(0.,0.,1.);

  //Hunt-Crossley contact terms. See
  // ContactToolkit::calcHuntCrossleyContactForce for details
  double exponent = 2.0; //The spring force will increase with the deflection squared.
  double stiffness = 9.81/pow(0.01,2.); //The ball will settle to 1cm penetration
  double damping = 0.1; //lightly damped  

  //Friction model terms. See
  // ContactToolkit::createRegularizedFrictionCoefficientCurve for details
  double staticFrictionSpeed        = 0.001;
  double staticFrictionCoefficient  = 0.8;
  double dynamicFrictionSpeed       = 0.01;
  double dynamicFrictionCoeffient   = 0.6;
  double viscousFrictionSlope       = 0.1;

  unsigned int numWorkTermsInState = 2; //1 normal work term
                                        //1 friction term


  VectorNd q, qd, x, tau;
  q.resize(model.dof_count);
  qd.resize(model.dof_count);
  tau.resize(model.dof_count);
  x.resize(model.dof_count*2);
  q.setZero();
  qd.setZero();
  x.setZero();
  tau.setZero();

  q[1] = 1.; //ball starts 1m off the ground
  qd[0]= 1.;

  for(unsigned int i=0; i<q.rows();++i){
    x[i] =q[i];
    x[i+q.rows()] = qd[i];
  }

  //0c. An object that Boost can integrate is instantiated:
  rbdlToBoost rbdlModel(&model,
                        ballName,
                        radius,
                        pointOnPlane,
                        planeNormal,
                        stiffness,
                        exponent,
                        damping,
                        staticFrictionSpeed,
                        staticFrictionCoefficient,
                        dynamicFrictionSpeed,
                        dynamicFrictionCoeffient,
                        viscousFrictionSlope,
                        numWorkTermsInState);

    //4a. The model state is initialized
    state_type xState(x.size()+numWorkTermsInState);
    state_type dxState(x.size()+numWorkTermsInState);
    for(unsigned int i=0; i<x.size(); ++i){
      xState[i]   = x[i];
    }

    //4b. This model is integrated forward in time from t0 to t1 and is evaluated
    //    at npts between these time points
    double t;
    double t0 = 0;
    double t1 = 1.0;
    unsigned int npts      = 100;

    double absTolVal = 1e-8;
    double relTolVal = 1e-8;

    double dt = (t1-t0)/(npts-1);
    double ke,pe,w =0;
    unsigned int k=0;

    std::vector<std::vector< double > > matrixData, matrixForceData;
    std::vector<std::vector< double > > matrixErrorData;
    std::vector< double > rowData(model.dof_count+1);
    std::vector< double > rowForceData(10);
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
    for(unsigned int z=0; z < model.dof_count; z++){
        rowData[z+1] = xState[z];
    }
    matrixData.push_back(rowData);

    SpatialVector fA0 = SpatialVector::Zero();
    for(unsigned int i=0; i<rowForceData.size();++i){
      rowForceData[i]=0.;
    }

    matrixForceData.push_back(rowForceData);

    double kepe0 = 0;
    double th,dth;

    //                 ,             ,             ,             ,             ,             ,             ,
    printf("Columns below:\n");
    printf("          t,        theta,   d/dt theta,           ke,           pe,            w, ke+pe-w-kepe0\n");

    for(unsigned int i=0; i<= npts; ++i){
      t = t0 + dt*i;

      integrate_adaptive(
          controlled_stepper ,
          rbdlModel , xState , tp , t , (t-tp)/10 );
      tp = t;

      //4c. At each time point the q's, qd's, and the work done on the ball by the 
      //    contact & friction model is saved

      for(unsigned int j=0; j<x.rows();++j){
        x[j] = xState[j];
      }
      k=0;
      for(unsigned int j=0; j<model.q_size;++j){
        q[j] = xState[k];
        ++k;
      }
      for(unsigned int j=0; j<model.qdot_size;++j){
        qd[j] = xState[k];
        ++k;
      }
      w = 0.;
      for(unsigned int j=0; j<numWorkTermsInState;++j){
        w += xState[k];
        ++k;
      }

      //4d. The q's, qd's, and work terms are used to numerically evaluate the system
      //    energy of the ball less the work done on it: ke+pe-w, where 
      //    ke is kinetic energy, pe is potential energy, and w is work.
      pe = Utils::CalcPotentialEnergy(model,
                                      q,true);

      ke = Utils::CalcKineticEnergy(model,
                                    q,
                                    qd,true);

      rowData[0] = t;
      for(unsigned int z=0; z < model.dof_count; z++){
          rowData[z+1] = xState[z];
      }
      matrixData.push_back(rowData);

    //4e. This is evaluated relative to the system energy of the ball at the 
    //    beginning of the simulation. If the integrator were perfect the sum of 
    //    ke+pe-w-kepe0 would be numerically zero. Because numerical integration is
    //    not perfect this quantity (printed to screen) will drift over time.
    //    You can see this if you adjust the absolute and relative tolerances
    //    (set between 4b and 4c) on the integrator and re-run the simulations.
    //    drift will be larger with looser integration tolerances.      
      if(i==0) kepe0 = (ke+pe-w);

      rowErrorData[0] = t;
      rowErrorData[1] = (ke + pe -w) - kepe0;
      matrixErrorData.push_back(rowErrorData);

      printf("%e, %e, %e, %e, %e, %e, %e\n",
                  t, q[2],qd[2],ke,
                  pe,w,(ke+pe-w-kepe0));

      //Make sure the model state is up to date.
      rbdlModel(xState,dxState,tp);

      //4f. Here we grab quantities that we would like to save
      rowForceData[0] = t;
      //contact point location
      rowForceData[1] = rbdlModel.r0K0[0];
      rowForceData[2] = rbdlModel.r0K0[1];
      rowForceData[3] = rbdlModel.r0K0[2];
      //force at the contact point
      rowForceData[4] = rbdlModel.fK0n[0] + rbdlModel.fK0t[0];
      rowForceData[5] = rbdlModel.fK0n[1] + rbdlModel.fK0t[1];
      rowForceData[6] = rbdlModel.fK0n[2] + rbdlModel.fK0t[2];
      //moment at the contact point
      rowForceData[7] = 0.;//This contact model generates no contact moments
      rowForceData[8] = 0.;//This contact model generates no contact moments
      rowForceData[9] = 0.;//This contact model generates no contact moments

      matrixForceData.push_back(rowForceData);


      bool here=true;

    }
    printf("Columns above:\n");
    printf("          t,        theta,   d/dt theta,           ke,           pe,            w, ke+pe-w-kepe0\n");


    //5a. Finally we write the simulation data to file
    std::cout << std::endl;
    std::string emptyHeader("");
    std::string fileNameOut("../output/animation.csv");
    printMatrixToFile(matrixData,emptyHeader,fileNameOut);
    printf("Wrote: ../output/animation.csv (meshup animation file)\n");
    fileNameOut.assign("../output/animationForces.ff");
    printMatrixToFile(matrixForceData,emptyHeader,fileNameOut);
    printf("Wrote: ../output/animationForces.ff (meshup force file)\n");

    fileNameOut = "../output/kepe.csv";
    std::string header("time,systemEnergy,");
    printMatrixToFile(matrixErrorData,header,fileNameOut);
    printf("Wrote: ../output/kepe.csv (simulation data)\n");



   return 0;
        
}
