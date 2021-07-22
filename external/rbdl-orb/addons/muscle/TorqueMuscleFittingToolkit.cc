#include "TorqueMuscleFittingToolkit.h"

#include "IpIpoptApplication.hpp"

using namespace Ipopt;
static double EPSILON = std::numeric_limits<double>::epsilon();
static double SQRTEPSILON = sqrt(EPSILON);


using namespace RigidBodyDynamics::Math;
using namespace RigidBodyDynamics::Addons::Muscle;
using namespace RigidBodyDynamics::Addons::Geometry;
using namespace std;


void TorqueMuscleFittingToolkit::
      fitTorqueMuscleParameters(
        Millard2016TorqueMuscle const &tqMcl,
        VectorNd const &jointAngle,
        VectorNd const &jointAngularVelocity,
        VectorNd const &jointTorque,
        double activationUpperBound,
        double passiveTorqueAngleMultiplierUpperBound,
        TorqueMuscleParameterFittingData &parametersOfBestFit,
        bool verbose) 
{

  if(tqMcl.mMuscleCurvesAreDirty){
    Millard2016TorqueMuscle* mutableTqMcl =
        const_cast<Millard2016TorqueMuscle* >(&tqMcl);
    mutableTqMcl->updateTorqueMuscleCurves();
  }


  assert(jointAngle.rows() > 1);
  assert(jointAngularVelocity.rows() > 1);
  assert(jointTorque.rows() > 1);

  unsigned int nrows = jointAngle.rows();

  double activationLowerBound = 0.0;
  assert(jointAngularVelocity.rows() == nrows);
  assert(jointTorque.rows() == nrows);
  assert(activationUpperBound > 0 && activationUpperBound <= 1.0);
  assert(activationLowerBound >= 0 && activationLowerBound<activationUpperBound);

  VectorNd jointTorqueSingleSided;
  jointTorqueSingleSided.resize(jointTorque.size());
  for(unsigned int i=0; i<jointTorque.size();++i){
    if(tqMcl.getJointTorqueSign()*jointTorque[i] >= 0){
      jointTorqueSingleSided[i] = jointTorque[i];
    }else{
      jointTorqueSingleSided[i] = 0.0;
    }
  }

  double tiso         = tqMcl.getMaximumActiveIsometricTorque();
  double omegaMax     = tqMcl.getMaximumConcentricJointAngularVelocity();

  double maxConcentricAngularVelocityData = 0;
  for(unsigned int i=0; i<jointAngularVelocity.rows();++i){
      if(fabs(jointAngularVelocity[i])
          > fabs(maxConcentricAngularVelocityData)){
        maxConcentricAngularVelocityData = fabs(jointAngularVelocity[i]);
      }
  }

  //Set ta and tv blending variables to a small finite value so that
  //it is possible to solve for an activation of finite value when
  //calcTorqueMuscleDataFeatures is called
  double taLambda     = sqrt(SQRTEPSILON);
  double tvLambda     = sqrt(SQRTEPSILON);
  if(tqMcl.getActiveTorqueAngleCurveBlendingVariable() > taLambda){
    taLambda = tqMcl.getActiveTorqueAngleCurveBlendingVariable();
  }
  if(tqMcl.getTorqueAngularVelocityCurveBlendingVariable() > tvLambda){
    tvLambda = tqMcl.getTorqueAngularVelocityCurveBlendingVariable();
  }

  double tpLambda     = 0.0;

  double taAngleScaling         = tqMcl.mTaAngleScaling;
  double taAngleAtOneNormTorque = tqMcl.mAngleAtOneNormActiveTorque;
  double tvOmegaMaxScale        = 1.0;
  double tpOffset               = 0.0;

  double objectiveValue         = 0.0;
  double constraintError        = 0.0;

  TorqueMuscleDataFeatures tmf;

  omegaMax = fabs(tqMcl.getMaximumConcentricJointAngularVelocity()
                  *tvOmegaMaxScale);

  //If the optimization run fails, the values from tmf are passed out to
  //the user to help them debug the problem.
  tqMcl.calcTorqueMuscleDataFeatures(
            jointTorqueSingleSided,jointAngle,jointAngularVelocity,
            taLambda,tpLambda,tvLambda,
            taAngleScaling,taAngleAtOneNormTorque,tpOffset,omegaMax,
            tiso,tmf);


    Millard2016TorqueMuscle* mutableTqMcl =
        const_cast<Millard2016TorqueMuscle* >(&tqMcl);

    SmartPtr<FitTorqueMuscleParameters> fittingProblem
        = new FitTorqueMuscleParameters(
                          jointAngle,
                          jointAngularVelocity,
                          jointTorqueSingleSided,
                          activationUpperBound,
                          passiveTorqueAngleMultiplierUpperBound,
                          taLambda,
                          tvLambda,
                          *mutableTqMcl);

    SmartPtr<IpoptApplication> app = new IpoptApplication(false);

    app->RethrowNonIpoptException(true);
    double solnTol = SQRTEPSILON;
    app->Options()->SetNumericValue("tol",solnTol);
    app->Options()->SetStringValue("mu_strategy","adaptive");

    if(!verbose){
      app->Options()->SetIntegerValue("print_level",0);
    }


    ApplicationReturnStatus status;

    objectiveValue = std::numeric_limits<double>::infinity();
    constraintError= std::numeric_limits<double>::infinity();
    bool converged = false;

    double taAngleScaleStart = 1.0;
    double tvOmegaMaxScaleStart =fabs(1.1*maxConcentricAngularVelocityData
                                  /tqMcl.getMaximumConcentricJointAngularVelocity());

    double tisoScale = 1.0;
    bool updateStart = false;

      status = app->Initialize();
      if(status == Solve_Succeeded){
        status = app->OptimizeTNLP(fittingProblem);
        if(status == Solve_Succeeded){
          converged   = true;
          updateStart = true;
          taAngleScaling  = fittingProblem
                            ->getSolutionActiveTorqueAngleAngleScaling();
          tvOmegaMaxScale = fittingProblem
                            ->getSolutionTorqueAngularVelocityOmegaMaxScale();
          tpLambda    = fittingProblem
                        ->getSolutionPassiveTorqueAngleBlendingParameter();
          tpOffset    = fittingProblem
                        ->getSolutionPassiveTorqueAngleCurveOffset();
          tisoScale   = fittingProblem
                        ->getSolutionMaximumActiveIsometricTorqueScale();

          tiso      = tisoScale * tiso;
          omegaMax  = fabs(tvOmegaMaxScale
                           *tqMcl.getMaximumConcentricJointAngularVelocity());

          objectiveValue = fittingProblem->getObjectiveValue();
          constraintError= fittingProblem->getConstraintError().norm();

          tqMcl.calcTorqueMuscleDataFeatures(
                  jointTorqueSingleSided,jointAngle,jointAngularVelocity,
                  taLambda,tpLambda,tvLambda,
                  taAngleScaling,taAngleAtOneNormTorque,tpOffset,omegaMax,
                  tiso,tmf);
          }
    }


    if(converged){
      parametersOfBestFit.fittingConverged = true;
      parametersOfBestFit.isTorqueMuscleActive = !(tmf.isInactive);

      parametersOfBestFit.indexAtMaximumActivation=tmf.indexOfMaxActivation;
      parametersOfBestFit.indexAtMinimumActivation=tmf.indexOfMinActivation;
      parametersOfBestFit.indexAtMaxPassiveTorqueAngleMultiplier
                            =tmf.indexOfMaxPassiveTorqueAngleMultiplier;

      parametersOfBestFit.activeTorqueAngleBlendingVariable   = taLambda;
      parametersOfBestFit.torqueVelocityBlendingVariable      = tvLambda;
      parametersOfBestFit.passiveTorqueAngleBlendingVariable  = tpLambda;
      parametersOfBestFit.passiveTorqueAngleCurveOffset       = tpOffset;
      parametersOfBestFit.maximumActiveIsometricTorque        = tiso;
      parametersOfBestFit.activeTorqueAngleAngleScaling       = taAngleScaling;
      parametersOfBestFit.maximumAngularVelocity              = fabs(omegaMax);

      parametersOfBestFit.summaryDataAtMaximumActivation
            =tmf.summaryAtMaxActivation;
      parametersOfBestFit.summaryDataAtMinimumActivation
            =tmf.summaryAtMinActivation;
      parametersOfBestFit.summaryDataAtMaximumPassiveTorqueAngleMultiplier
            =tmf.summaryAtMaxPassiveTorqueAngleMultiplier;
    }else{
      parametersOfBestFit.fittingConverged = false;
      parametersOfBestFit.isTorqueMuscleActive = !(tmf.isInactive);

      parametersOfBestFit.indexAtMaximumActivation
          =numeric_limits<unsigned int>::signaling_NaN();
      parametersOfBestFit.indexAtMinimumActivation
          =numeric_limits<unsigned int>::signaling_NaN();
      parametersOfBestFit.indexAtMaxPassiveTorqueAngleMultiplier
          =numeric_limits<unsigned int>::signaling_NaN();

      parametersOfBestFit.activeTorqueAngleBlendingVariable
          = numeric_limits<double>::signaling_NaN();
      parametersOfBestFit.passiveTorqueAngleBlendingVariable
          = numeric_limits<double>::signaling_NaN();
      parametersOfBestFit.passiveTorqueAngleCurveOffset
          = numeric_limits<double>::signaling_NaN();
      parametersOfBestFit.torqueVelocityBlendingVariable
          = numeric_limits<double>::signaling_NaN();
      parametersOfBestFit.maximumActiveIsometricTorque
          = numeric_limits<double>::signaling_NaN();
      parametersOfBestFit.activeTorqueAngleAngleScaling
          = numeric_limits<double>::signaling_NaN();
      parametersOfBestFit.maximumAngularVelocity
          = numeric_limits<double>::signaling_NaN();

      parametersOfBestFit.summaryDataAtMaximumActivation
            =tmf.summaryAtMaxActivation;
      parametersOfBestFit.summaryDataAtMinimumActivation
            =tmf.summaryAtMinActivation;
      parametersOfBestFit.summaryDataAtMaximumPassiveTorqueAngleMultiplier
            =tmf.summaryAtMaxPassiveTorqueAngleMultiplier;
    }

}



FitTorqueMuscleParameters::
          FitTorqueMuscleParameters(
            const RigidBodyDynamics::Math::VectorNd &jointAngle,
            const RigidBodyDynamics::Math::VectorNd &jointAngularVelocity,
            const RigidBodyDynamics::Math::VectorNd &jointTorque,
            double maxActivation,
            double maxPassiveTorqueAngleMultiplier,
            double taLambda,
            double tvLambda,
            Millard2016TorqueMuscle &tqMcl):
              mJointAngle(jointAngle),
              mJointAngularVelocity(jointAngularVelocity),
              mJointTorque(jointTorque),
              mTqMcl(tqMcl)
{
  assert(jointTorque.rows() == jointAngle.rows());
  assert(jointTorque.rows() == jointAngularVelocity.rows());

  //Initialize all member variables
  mN = 5;
  mM = 3*jointTorque.rows();

  double angleMin = std::numeric_limits< double >::max();
  double omegaMin = std::numeric_limits< double >::max();
  double angleMax = -std::numeric_limits< double >::max();
  double omegaMax = -std::numeric_limits< double >::max();
  double tauMax   = 0;

  for(unsigned int i=0; i<mJointAngle.rows();++i){
    if(mJointAngle[i] < angleMin){
      angleMin = mJointAngle[i];
    }
    if(mJointAngle[i] > angleMax){
      angleMax = mJointAngle[i];
    }
    if(mJointAngularVelocity[i] < omegaMin){
      omegaMin = mJointAngularVelocity[i];
    }
    if(mJointAngularVelocity[i] > omegaMax){
      omegaMax = mJointAngularVelocity[i];
    }
    if(mJointTorque[i]*mTqMcl.getJointTorqueSign() > tauMax){
      tauMax = mJointTorque[i]*mTqMcl.getJointTorqueSign();
    }
  }

  double tvScale = max(fabs(omegaMin),fabs(omegaMax))
                   /fabs(mTqMcl.getMaximumConcentricJointAngularVelocity());
  double taScale = fabs(angleMax-angleMin)
                   /mTqMcl.getActiveTorqueAngleCurveWidth();

  double tauScale = tauMax/fabs(mTqMcl.getMaximumActiveIsometricTorque());




  mConIdxTauActMaxStart  = 0;
  mConIdxTauActMaxEnd    = jointTorque.rows()-1;
  mConIdxTauActMinStart  = jointTorque.rows();
  mConIdxTauActMinEnd    = 2*jointTorque.rows()-1;
  mConIdxTauPassiveStart = 2*jointTorque.rows();
  mConIdxTauPassiveEnd   = 3*jointTorque.rows()-1;

  mMaxActivation          = maxActivation;
  mMinActivation          = 0.0;
  mMaxTp                  = maxPassiveTorqueAngleMultiplier;
  mTauIso                 = mTqMcl.getMaximumActiveIsometricTorque();
  mTaLambda               = taLambda;
  mTvLambda               = tvLambda;
  mTaAngleAtOneNormTorque = mTqMcl.mAngleAtOneNormActiveTorque;
  mOmegaMax               = mTqMcl.mOmegaMax;

  mTaAngleScaleStart    = max(taScale,
                              mTqMcl.getActiveTorqueAngleCurveAngleScaling());
  mTvOmegaMaxScaleStart = max(tvScale, 1.);
  mTpLambdaStart        = mTqMcl.getPassiveTorqueAngleCurveBlendingVariable();
  mTpAngleOffsetStart   = mTqMcl.getPassiveCurveAngleOffset();
  mTauScalingStart      = max(tauScale, 1.);

  mTaAngleScaleLB     = 1.;
  mTvOmegaMaxScaleLB  = 1.;
  mTpLambdaLB         = 0.;
  mTpAngleOffsetLB    = -M_PI*0.5;
  mTauScalingLB       = 1.0;

  mTaAngleScaleUB     = 2.0e19;
  mTvOmegaMaxScaleUB  = 2.0e19;
  mTpLambdaUB         = 1.0;
  mTpAngleOffsetUB    = M_PI*0.5;
  mTauScalingUB       = 2.0e19;

  mIndexTaAngleScale    = 0;
  mIndexTvOmegaMaxScale = 1;
  mIndexTpLambda        = 2;
  mIndexTpOffset        = 3;
  mIndexTauScaling      = 4;

  //Initialize vectors
  mConstraintErrors.resize(mM);
  mXOffset.resize(mN);

  mWeights.resize(mN);
  for(unsigned int i=0; i<mWeights.rows(); ++i){
    mWeights[i] = 1.0;
  }

  mXOffset.resize(mN);
  mXOffset[0]         = mTqMcl.getActiveTorqueAngleCurveAngleScaling();
  mXOffset[1]         = 1.0;
  mXOffset[2]         = mTqMcl.getPassiveTorqueAngleCurveBlendingVariable();
  mXOffset[3]         = mTqMcl.getPassiveCurveAngleOffset();
  mXOffset[4]         = 1.0;

  for(unsigned int i=0; i< mM; ++i){
    mConstraintErrors[i] = 0.;
  }

  mObjValue = 0.;

}


//Manditory functions of the TNLP interface
bool FitTorqueMuscleParameters::
           get_nlp_info(Ipopt::Index &n,
                        Ipopt::Index &m,
                        Ipopt::Index &nnz_jac_g,
                        Ipopt::Index &nnz_h_lag,
                        Ipopt::TNLP::IndexStyleEnum &index_style)
{
  //Parameters
  //  lambda_A
  //  lambda_V
  //  tisoScaling
  n = mN;

  //Constraints
  //  a_max*(tisoScaled)*fal*fv+fpe - tau_measured >= 0
  m = mM;

  //Constraint Jacobian
  //This is a dense Jacobian
  nnz_jac_g = mM*mN;

  //Hessian
  //This is a symmetric matrix 5x5 matrix, but its densley populated so
  //we must fill in the bottom left triangle.
  nnz_h_lag = 15;

  index_style = Ipopt::TNLP::C_STYLE;

  return true;
}


bool FitTorqueMuscleParameters::
             get_bounds_info(Ipopt::Index n,
                             Ipopt::Number *x_l,
                             Ipopt::Number *x_u,
                             Ipopt::Index m,
                             Ipopt::Number *g_l,
                             Ipopt::Number *g_u)
{
  assert(n==mN);
  assert(m==mM);
  x_l[0] = mTaAngleScaleLB;
  x_l[1] = mTvOmegaMaxScaleLB;
  x_l[2] = mTpLambdaLB;
  x_l[3] = mTpAngleOffsetLB;
  x_l[4] = mTauScalingLB;

  x_u[0] = mTaAngleScaleUB;
  x_u[1] = mTvOmegaMaxScaleUB;
  x_u[2] = mTpLambdaUB;
  x_u[3] = mTpAngleOffsetUB;
  x_u[4] = mTauScalingUB;

  //The optimization routine will only strengthen
  //the muscle to satisfy the constraint - but it
  //will not weaken it.

  double tauActMaxLB = 0;
  double tauActMaxUB = 0;
  double tauActMinLB = 0;
  double tauActMinUB = 0;
  double tauPassiveLB = 0;
  double tauPassiveUB = 0;



  if(mTqMcl.getJointTorqueSign() > 0){
    tauActMaxLB  =  0;
    tauActMaxUB  =  2e19;

    tauActMinLB  = -2e19;
    tauActMinUB  =  0;

    tauPassiveLB = -2e19;
    tauPassiveUB =  0;
  }else{
    tauActMaxLB = -2.e19;
    tauActMaxUB = 0.;

    tauActMinLB = 0;
    tauActMinUB = 2e19;

    tauPassiveLB = -2e19;
    tauPassiveUB =  0;
  }

  for(unsigned int i  = mConIdxTauActMaxStart; i <= mConIdxTauActMaxEnd; ++i){
    g_l[i] = tauActMaxLB;
    g_u[i] = tauActMaxUB;
  }
  for(unsigned int i  = mConIdxTauActMinStart; i <= mConIdxTauActMinEnd; ++i){
    g_l[i] = tauActMinLB;
    g_u[i] = tauActMinUB;
  }
  for(unsigned int i  = mConIdxTauPassiveStart; i <= mConIdxTauPassiveEnd; ++i){
    g_l[i] = tauPassiveLB;
    g_u[i] = tauPassiveUB;
  }

  return true;
}


bool FitTorqueMuscleParameters::
                    get_starting_point(Ipopt::Index n,
                                        bool init_x,
                                        Ipopt::Number *x,
                                        bool init_z,
                                        Ipopt::Number *z_L,
                                        Ipopt::Number *z_U,
                                        Ipopt::Index m,
                                        bool init_lambda,
                                        Ipopt::Number *lambda)
{
  assert(n==mN);

  x[0]        = mTaAngleScaleStart;
  x[1]        = mTvOmegaMaxScaleStart;
  x[2]        = mTpLambdaStart;
  x[3]        = mTpAngleOffsetStart;
  x[4]        = mTauScalingStart;

  init_x      = true;
  init_z      = false;
  init_lambda = false;

  mDtp_Dx.resize(mN);

  return true;
}

bool FitTorqueMuscleParameters::
                    eval_f(Ipopt::Index n,
                            const Ipopt::Number *x,
                            bool new_x,
                            Ipopt::Number &obj_value)
{
  assert(n==mN);

  obj_value = 0.;
  double xUpd = 0.;
  for(unsigned i=0; i<mN; ++i){
    xUpd = x[i]-mXOffset[i];
    obj_value += mWeights[i]*xUpd*xUpd;
  }

  new_x = false;
  return true;
}

bool FitTorqueMuscleParameters::
             eval_grad_f(Ipopt::Index n,
                         const Ipopt::Number *x,
                         bool new_x,
                         Ipopt::Number *grad_f)
{
  assert(n==mN);

  for(unsigned int i=0; i<(unsigned int)n;++i){
    grad_f[i] = 0.;
  }

  double xUpd = 0;
  for(unsigned int i=0; i<(unsigned int)n;++i){
    xUpd = x[i]-mXOffset[i];
    grad_f[i] = 2.0*mWeights[i]*xUpd;
  }

  return true;
}

bool FitTorqueMuscleParameters::
             eval_g(Ipopt::Index n,
                    const Ipopt::Number *x,
                    bool new_x,
                    Ipopt::Index m,
                    Ipopt::Number *g)
{
  assert(n==mN);
  assert(m==mM);

  updOptimizationVariables(x);
  double tauMax     = mTauScaling*mTauIso;
  double omegaMax   = mTvOmegaMaxScale*mOmegaMax;


  unsigned int j = 0;
  for(unsigned int i  = mConIdxTauActMaxStart; i <= mConIdxTauActMaxEnd; ++i){
    mTqMcl.updTorqueMuscleSummary(mMaxActivation,
                                  mJointAngle[j],
                                  mJointAngularVelocity[j],
                                  mTaLambda,
                                  mTpLambda,
                                  mTvLambda,
                                  mTaAngleScale,
                                  mTaAngleAtOneNormTorque,
                                  mTpAngleOffset,
                                  omegaMax,
                                  tauMax,
                                  mTms);

    g[i] = mTms.jointTorque - mJointTorque[j];
    ++j;
  }

  j=0;
  unsigned int k = mConIdxTauPassiveStart;
  for(unsigned int i  = mConIdxTauActMinStart; i <= mConIdxTauActMinEnd; ++i){
    mTqMcl.updTorqueMuscleSummary(mMinActivation,
                                  mJointAngle[j],
                                  mJointAngularVelocity[j],
                                  mTaLambda,
                                  mTpLambda,
                                  mTvLambda,
                                  mTaAngleScale,
                                  mTaAngleAtOneNormTorque,
                                  mTpAngleOffset,
                                  omegaMax,
                                  tauMax,
                                  mTms);

    g[i] = mTms.jointTorque - mJointTorque[j];
    g[k] = mTms.fiberPassiveTorqueAngleMultiplier - mMaxTp;
    ++j;
    ++k;
  }

  return true;
}

bool FitTorqueMuscleParameters::
            eval_jac_g(Ipopt::Index n,
                        const Ipopt::Number *x,
                        bool new_x,
                        Ipopt::Index m,
                        Ipopt::Index nele_jac,
                        Ipopt::Index *iRow,
                        Ipopt::Index *jCol,
                        Ipopt::Number *values)
{
  assert(n==mN);
  assert(m==mM);

  if(values == NULL){

    unsigned int idx = 0;
    for(unsigned int rows = 0; rows < mM; ++rows){
      for(unsigned int cols = 0; cols < mN; ++cols){
        iRow[idx] = rows;
        jCol[idx] = cols;
        ++idx;
      }
    }

  }else{
    updOptimizationVariables(x);
    double tauMax     = mTauScaling*mTauIso;
    double omegaMax   = mTvOmegaMaxScale*mOmegaMax;

    unsigned int idx = 0;
    unsigned int j   = 0;
    for(unsigned int i  = mConIdxTauActMaxStart; i <= mConIdxTauActMaxEnd; ++i){
      mTqMcl.updTorqueMuscleInfo(   mMaxActivation,
                                    mJointAngle[j],
                                    mJointAngularVelocity[j],
                                    mTaLambda,
                                    mTpLambda,
                                    mTvLambda,
                                    mTaAngleScale,
                                    mTaAngleAtOneNormTorque,
                                    mTpAngleOffset,
                                    omegaMax,
                                    tauMax,
                                    mTmi);


      values[idx] = mTmi.DjointTorque_DactiveTorqueAngleAngleScaling;
      ++idx;
      values[idx] = mTmi.DjointTorque_DmaximumAngularVelocity*mOmegaMax;
      ++idx;
      values[idx] = mTmi.DjointTorque_DpassiveTorqueAngleBlendingVariable;
      ++idx;
      values[idx] = mTmi.DjointTorque_DpassiveTorqueAngleCurveAngleOffset;
      ++idx;
      values[idx] = mTmi.jointTorque/mTauScaling;
      ++idx;

      ++j;
    }

    j=0;
    for(unsigned int i  = mConIdxTauActMinStart; i <= mConIdxTauActMinEnd; ++i){
      mTqMcl.updTorqueMuscleInfo(   mMinActivation,
                                    mJointAngle[j],
                                    mJointAngularVelocity[j],
                                    mTaLambda,
                                    mTpLambda,
                                    mTvLambda,
                                    mTaAngleScale,
                                    mTaAngleAtOneNormTorque,
                                    mTpAngleOffset,
                                    omegaMax,
                                    tauMax,
                                    mTmi);


      values[idx] = mTmi.DjointTorque_DactiveTorqueAngleAngleScaling;
      ++idx;
      values[idx] = mTmi.DjointTorque_DmaximumAngularVelocity*mOmegaMax;
      ++idx;
      values[idx] = mTmi.DjointTorque_DpassiveTorqueAngleBlendingVariable;
      ++idx;
      values[idx] = mTmi.DjointTorque_DpassiveTorqueAngleCurveAngleOffset;
      ++idx;
      values[idx] = mTmi.jointTorque/mTauScaling;
      ++idx;

      ++j;
    }


    j=0;
    for(unsigned int i= mConIdxTauPassiveStart; i <= mConIdxTauPassiveEnd; ++i){
      mTqMcl.updTorqueMuscleInfo(0.,
                                 mJointAngle[j],
                                 mJointAngularVelocity[j],
                                 mTaLambda,
                                 mTpLambda,
                                 mTvLambda,
                                 mTaAngleScale,
                                 mTaAngleAtOneNormTorque,
                                 mTpAngleOffset,
                                 omegaMax,
                                 tauMax,
                                 mTmi);

      values[idx] = 0.;
      ++idx;
      values[idx] = 0.;
      ++idx;
      values[idx] = mTmi.DfiberPassiveTorqueAngleMultiplier_DblendingVariable;
      ++idx;
      values[idx] = mTmi.DfiberPassiveTorqueAngleMultiplier_DangleOffset;
      ++idx;
      values[idx] = 0.;
      ++idx;

      ++j;
    }



  }

  return true;
}

void FitTorqueMuscleParameters::
        finalize_solution(Ipopt::SolverReturn status,
                          Ipopt::Index n,
                          const Ipopt::Number *x,
                          const Ipopt::Number *z_L,
                          const Ipopt::Number *z_U,
                          Ipopt::Index m,
                          const Ipopt::Number *g,
                          const Ipopt::Number *lambda,
                          Ipopt:: Number obj_value,
                          const Ipopt::IpoptData *ip_data,
                          Ipopt::IpoptCalculatedQuantities *ip_cq)
{
  assert(n==mN);
  assert(m==mM);

  updOptimizationVariables(x);
  mObjValue = obj_value;
  for(unsigned int i=0; i<mM; ++i){
    mConstraintErrors[i] = g[i];
  }

}

/*
  This is incorrect - the constraint has a non-zero Hessian.
*/
bool FitTorqueMuscleParameters::
              eval_h(Ipopt::Index n,
                      const Ipopt::Number *x,
                      bool new_x,
                      Ipopt::Number obj_factor,
                      Ipopt::Index m,
                      const Ipopt::Number *lambda,
                      bool new_lambda,
                      Ipopt::Index nele_hess,
                      Ipopt::Index *iRow,
                      Ipopt::Index *jCol,
                      Ipopt::Number *values)
{
  assert(n==mN);
  assert(m==mM);

  if(values==NULL){
    unsigned int ele = 0;
    for(unsigned int row = 0; row < mN; ++row){
      for(unsigned int col = 0; col <= row; ++col){
        iRow[ele] = row;
        jCol[ele] = col;
        ++ele;
      }
    }
    assert(ele == (unsigned int)nele_hess);
  }else{
    /*
      [K   ]            [D2tau_DlambdaA2                                         ]
      [  K ] + lambda_i [D2tau_DlambdaADlambdaV D2tau_DlambdaV2                  ]
      [   K]            [D2tau_DlambdaADtauS    D2tau_DlambdaVDtauS  D2tau_DtauS2]

    */

    updOptimizationVariables(x);
    double tauMax     = mTauScaling*mTauIso;
    double omegaMax   = mTvOmegaMaxScale*mOmegaMax;


    for(unsigned int i = 0; i < nele_hess; ++i){
      values[i] = 0.;
    }

    values[0] = obj_factor*2.0*mWeights[0];
    values[2] = obj_factor*2.0*mWeights[1];
    values[5] = obj_factor*2.0*mWeights[2];
    values[9] = obj_factor*2.0*mWeights[3];
    values[14]= obj_factor*2.0*mWeights[4];


    //Add in the components of the Hessian due to the constraints.

    double d2t_ds2  = 0.;
    double d2t_dsdm = 0.;
    double d2t_dm2  = 0.;
    double d2t_dsdp = 0.;
    double d2t_dmdp = 0.;
    double d2t_dp2  = 0.;
    double d2t_dsdo = 0.;
    double d2t_dmdo = 0.;
    double d2t_dpdo = 0.;
    double d2t_do2  = 0.;
    double dt_ds    = 0.;
    double dt_dm    = 0.;
    double dt_dp    = 0.;
    double dt_do    = 0.;

    double d2tp_dp2 = 0.;
    double d2tp_dpdo= 0.;
    double d2tp_do2 = 0.;

    unsigned int j = 0;
    for(unsigned int i  = mConIdxTauActMaxStart; i <= mConIdxTauActMaxEnd; ++i){
      mTqMcl.updTorqueMuscleInfo(   mMaxActivation,
                                    mJointAngle[j],
                                    mJointAngularVelocity[j],
                                    mTaLambda,
                                    mTpLambda,
                                    mTvLambda,
                                    mTaAngleScale,
                                    mTaAngleAtOneNormTorque,
                                    mTpAngleOffset,
                                    omegaMax,
                                    tauMax,
                                    mTmi);

      d2t_ds2  = mTmi.fittingInfo[13];
      d2t_dsdm = mTmi.fittingInfo[14]*mOmegaMax;
      d2t_dm2  = mTmi.fittingInfo[15]*mOmegaMax*mOmegaMax;
      d2t_dsdp = mTmi.fittingInfo[16];
      d2t_dmdp = mTmi.fittingInfo[17]*mOmegaMax;
      d2t_dp2  = mTmi.fittingInfo[18];
      d2t_dsdo = mTmi.fittingInfo[19];
      d2t_dmdo = mTmi.fittingInfo[20]*mOmegaMax;
      d2t_dpdo = mTmi.fittingInfo[21];
      d2t_do2  = mTmi.fittingInfo[22];
      dt_ds    = mTmi.DjointTorque_DactiveTorqueAngleAngleScaling;
      dt_dm    = mTmi.DjointTorque_DmaximumAngularVelocity*mOmegaMax;
      dt_dp    = mTmi.DjointTorque_DpassiveTorqueAngleBlendingVariable;
      dt_do    = mTmi.DjointTorque_DpassiveTorqueAngleCurveAngleOffset;

      values[0]  += lambda[i]*d2t_ds2;
      values[1]  += lambda[i]*d2t_dsdm;
      values[2]  += lambda[i]*d2t_dm2;
      values[3]  += lambda[i]*d2t_dsdp;
      values[4]  += lambda[i]*d2t_dmdp;
      values[5]  += lambda[i]*d2t_dp2;
      values[6]  += lambda[i]*d2t_dsdo;
      values[7]  += lambda[i]*d2t_dmdo;
      values[8]  += lambda[i]*d2t_dpdo;
      values[9]  += lambda[i]*d2t_do2;
      values[10] += lambda[i]*dt_ds/mTauScaling;
      values[11] += lambda[i]*dt_dm/mTauScaling;
      values[12] += lambda[i]*dt_dp/mTauScaling;
      values[13] += lambda[i]*dt_do/mTauScaling;
      values[14] += lambda[i]*0.;
      ++j;
    }

    j=0;
    for(unsigned int i  = mConIdxTauActMinStart; i <= mConIdxTauActMinEnd; ++i){
      mTqMcl.updTorqueMuscleInfo(   mMinActivation,
                                    mJointAngle[j],
                                    mJointAngularVelocity[j],
                                    mTaLambda,
                                    mTpLambda,
                                    mTvLambda,
                                    mTaAngleScale,
                                    mTaAngleAtOneNormTorque,
                                    mTpAngleOffset,
                                    omegaMax,
                                    tauMax,
                                    mTmi);

      d2t_ds2  = mTmi.fittingInfo[13];
      d2t_dsdm = mTmi.fittingInfo[14]*mOmegaMax;
      d2t_dm2  = mTmi.fittingInfo[15]*mOmegaMax*mOmegaMax;
      d2t_dsdp = mTmi.fittingInfo[16];
      d2t_dmdp = mTmi.fittingInfo[17]*mOmegaMax;
      d2t_dp2  = mTmi.fittingInfo[18];
      d2t_dsdo = mTmi.fittingInfo[19];
      d2t_dmdo = mTmi.fittingInfo[20]*mOmegaMax;
      d2t_dpdo = mTmi.fittingInfo[21];
      d2t_do2  = mTmi.fittingInfo[22];
      dt_ds = mTmi.DjointTorque_DactiveTorqueAngleAngleScaling;
      dt_dm = mTmi.DjointTorque_DmaximumAngularVelocity*mOmegaMax;
      dt_dp = mTmi.DjointTorque_DpassiveTorqueAngleBlendingVariable;
      dt_do = mTmi.DjointTorque_DpassiveTorqueAngleCurveAngleOffset;

      values[0]  += lambda[i]*d2t_ds2;
      values[1]  += lambda[i]*d2t_dsdm;
      values[2]  += lambda[i]*d2t_dm2;
      values[3]  += lambda[i]*d2t_dsdp;
      values[4]  += lambda[i]*d2t_dmdp;
      values[5]  += lambda[i]*d2t_dp2;
      values[6]  += lambda[i]*d2t_dsdo;
      values[7]  += lambda[i]*d2t_dmdo;
      values[8]  += lambda[i]*d2t_dpdo;
      values[9]  += lambda[i]*d2t_do2;
      values[10] += lambda[i]*dt_ds/mTauScaling;
      values[11] += lambda[i]*dt_dm/mTauScaling;
      values[12] += lambda[i]*dt_dp/mTauScaling;
      values[13] += lambda[i]*dt_do/mTauScaling;
      values[14] += lambda[i]*0.;

      ++j;
    }

    //Room for improvement: combine this loop with the previous one.
    //But, be sure not to screw up idx.
    j=0;
    for(unsigned int i= mConIdxTauPassiveStart; i <= mConIdxTauPassiveEnd; ++i){
      mTqMcl.updTorqueMuscleInfo(0.,
                                 mJointAngle[j],
                                 mJointAngularVelocity[j],
                                 mTaLambda,
                                 mTpLambda,
                                 mTvLambda,
                                 mTaAngleScale,
                                 mTaAngleAtOneNormTorque,
                                 mTpAngleOffset,
                                 omegaMax,
                                 tauMax,
                                 mTmi);

      d2tp_dp2 = mTmi.fittingInfo[10];
      d2tp_dpdo= mTmi.fittingInfo[11];
      d2tp_do2 = mTmi.fittingInfo[12];

      values[5]  += lambda[i]*d2tp_dp2; //

      values[8]  += lambda[i]*d2tp_dpdo; //
      values[9]  += lambda[i]*d2tp_do2;  //

      ++j;
    }
  }

  return true;
}

double FitTorqueMuscleParameters::
        getSolutionActiveTorqueAngleAngleScaling()
{
  return mTaAngleScale;
}
double FitTorqueMuscleParameters::
        getSolutionTorqueAngularVelocityOmegaMaxScale()
{
  return mTvOmegaMaxScale;
}
double FitTorqueMuscleParameters::
        getSolutionPassiveTorqueAngleBlendingParameter()
{
  return mTpLambda;
}
double FitTorqueMuscleParameters::
        getSolutionPassiveTorqueAngleCurveOffset()
{
  return mTpAngleOffset;
}

double FitTorqueMuscleParameters::
  getSolutionMaximumActiveIsometricTorqueScale()
{
  return mTauScaling;
}

double FitTorqueMuscleParameters::getObjectiveValue()
{
  return mObjValue;
}
RigidBodyDynamics::Math::VectorNd&
  FitTorqueMuscleParameters::getConstraintError()
{
  return mConstraintErrors;
}

void FitTorqueMuscleParameters::
    updOptimizationVariables(const Ipopt::Number *x){

  mTaAngleScale       = x[0];
  mTvOmegaMaxScale    = x[1];
  mTpLambda           = x[2];
  mTpAngleOffset      = x[3];
  mTauScaling         = x[4];

}
