#ifndef TORQUEMUSCLEFITTINGTOOLKIT_H_
#define TORQUEMUSCLEFITTINGTOOLKIT_H_


#include <vector>
#include <rbdl/rbdl_math.h>
#include "IpTNLP.hpp"
#include "Millard2016TorqueMuscle.h"

namespace RigidBodyDynamics {
  namespace Addons {
    namespace Muscle{


class TorqueMuscleFittingToolkit {
  public:


  /**
    \brief This function will adjust the parameters of the
    muscle-torque-generator (MTG) so that the MTG is strong enough and 
    flexible enough that it can produce the measured output torque
    across the vector of measured joint angles and angular
    velocities without exceeding the desired activation upper
    bound, nor the passive forces exceeding the prescribed upper bound. 
    This function requires that IPOPT is installed
    and that cmake has been correctly configured to point at
    IPOPT's install directory. For a detailed description of the method please
    see : M.Millard, A.L.Kleesattel, M.Harant, & K.Mombaur. A reduced muscle 
    model and planar musculoskeletal model fit for the synthesis of whole body 
    movements. Journal of Biomechanics. (under review as of August 2018)

    The method adjusts five parameters:

    -# \f$s^A\f$: the scale of the domain (angle) of the 
                  active-torque-angle-curve. This scaling is applied so that
                  the angle which corresponds to the peak value of the 
                  active-torque-angle-curve remains unchanged.
    -# \f$s^V\f$: the scaling applied to the maximum angular velocity at 
                 which the concentric side of the torque-angular-velocity 
                 curve goes to zero.                  
    -# \f$\lambda^P\f$: the passive-torque-angle-blending-variable
    -# \f$\Delta^P\f$: the passive-torque-angle-offset
    -# \f$s^\tau\f$: the scaling applied to \f$\tau_{o}\f$ the 
        maximum active isometric torque

    These 5 parameters are a subset of the fitting variables that are available,
    and are illustrated below:

    \image html fig_MuscleAddon_BlendableTorqueMuscle.png "Examples of the adjustable characteristic curves"


    To peform the fitting, the user must provide vectors of the
    joint angle, joint angular velocity, and the joint torque
    for the joint that the muscle-torque-generator (MTG) actuates. The sign of
    all of these quantities must be consistent with the multibody
    and the torque muscle. IPOPT solves for the fitting parameters 
    \f$\mathbf{x}\f$ such that the final result is as close as possible to the 
    values the MTG has before the fitting is performed \f$\mathbf{x}_o\f$
    \f[
      (\mathbf{x}-\mathbf{x}_o)^T \mathbf{K} (\mathbf{x}-\mathbf{x}_o)
    \f]
    such that at every point \f$i\f$ in the vector of joint angles, velocities, 
    and torques the following constraints are satisfied
    \f[
      \tau^M(a^{max},\theta_i,\dot{\theta}_i,\mathbf{x})-\tau_i \ge 0
    \f]
    \f[
      \tau^M(0,\theta_i,\dot{\theta}_i,\mathbf{x})-\tau_i \le 0
    \f]
    \f[
      \mathbf{t}_{P}(\theta_i) - \mathbf{t}_{P}^{max} \le 0
    \f]
    \f[
        s^A, s^V, s^{\tau} \ge 1
    \f]
    \f[
        1 \ge \lambda^P \ge 0
    \f]    
    \f[
       \pi/2 \ge \Delta^P \ge -\pi/2.
    \f]
    These constraints ensure that the muscle can deliver the
    requested torque with an activation of \f$a^{max}\f$ or less, such that
    the active forces of the muscle are never fighting the passive forces, 
    and that passive forces that are \f$\mathbf{t}_{P}^{max}\f$ or less. 
    The bound limits on the various optimization variables ensure that the 
    muscle is not made to be weaker or slower than the default settings for
    the muscle. Thus if your subject is weaker than the default settings
    for the muscle, you should first weaken the MTG and then run this function.

    <b>Notes</b>
    -# This routine will return the parameters that
    fit the data, even if they are unreasonable: it is your
    responsibility to have a look at this data to sanity check it
    before using it.
    -# This function will <b>not</b> update the internal
    parameters of the model - it just calculates what they need
    to be and returns them in the
    TorqueMuscleParametersFittingData structure. If you decide
    that these parameters are parameters are suitable, then they
    can be assigned to the model using the function
    Millard2016TorqueMuscle.setFittedParameters. The solve and set functions 
    have been deliberately split up to reflect the inherentely
    iterative, and interactive nature of fitting the strength
    of a musculoskeletal model to data.
    -# Before attempting to fit the strength of the muscles
    to experimental data we recommend that you pre-process the
    data first. Normally this involves filtering the joint angles
    using a zero-phase 2nd order low-pass Butterworth filter with
    a cutoff frequency of 8-10 Hz (for human data). This can be
    achieved in Matlab using the functions 'butter' and
    'filtfilt'. Following this filtering step joint angular
    velocities can be safely calculated numerically using a
    central-difference method. If you fail to filter measured
    joint angles it is likely that the joint angular velocities
    you calculate will be noisy and will result in a terrible
    fit. This function is not magic, and so the rule
    garbage in, garbage out applies.

    @param tqMcl:
      The Millard2016TorqueMuscle object that is to be fitted.
    @param jointAngle (radians)
      A vector of measured joint angles. The sign of this vector
      must be consistent with the multibody model and this
      torque muscle.
    @param jointAngularVelocity (radians/s)
      A vector of joint angular velocities.
      The sign of this vector must be consistent with the
      multibody model and this torque muscle.
    @param jointTorque (Nm)
       A vector of joint torques.  Only the
       values in this vector that have the same torque sign as
       the muscle are considered during the fitting process. This
       means, inherently, that the muscle fitting is done assuming that
       there is no co-contraction. If you want co-contraction to be included
       in the fitting process, you must preprocess this vector so that it just
       contains the (estimated) single-signed torque of the muscle in question.
       The sign of this vector must be consistent with the
       multibody model and this torque muscle.
    @param activationUpperBound (0,1]
       The maximum activation that is permitted for the given
       set of joint angles, joint angular velocities, and
       joint torques.
    @param passiveTorqueAngleMultiplierUpperBound (0, \f$\infty\f$]
       The maximum passive torque angle multipler that you expect
       to see when the muscle moves through the values in
       jointAngle. Note that a value of 1 is quite large: this
       corresponds to 1 maximum active isometric torque.
    @param parametersOfBestFit
       A structure that contains the parameters of best fit along
       with a summary of the internal variables of the muscle
       model at the points in the data where the peak passive
       forces are developed and where the maximum activation is
       developed.
    @param verbose
       Setting this to will print information from IPOPT to the screen
       during the solution process.

  */
  static void fitTorqueMuscleParameters(
    Millard2016TorqueMuscle const &tqMcl,    
    RigidBodyDynamics::Math::VectorNd const &jointAngle,
    RigidBodyDynamics::Math::VectorNd const &jointAngularVelocity,
    RigidBodyDynamics::Math::VectorNd const &jointTorque,
    double activationUpperBound,
    double passiveTorqueAngleMultiplierUpperBound,
    TorqueMuscleParameterFittingData &parametersOfBestFit,
    bool verbose=false);


};




//A class that defines the virtual functions needed
//to use Ipopt to solve the problem
class FitTorqueMuscleParameters : public Ipopt::TNLP{
  public:
    FitTorqueMuscleParameters(
        const RigidBodyDynamics::Math::VectorNd &jointAngle,
        const RigidBodyDynamics::Math::VectorNd &jointAngularVelocity,
        const RigidBodyDynamics::Math::VectorNd &jointTorque,
        double maxActivation,
        double maxPassiveTorqueAngleMultiplier,
        double taLambda,
        double tvLambda,
        Millard2016TorqueMuscle &tqMcl);


    //Manditory functions of the TNLP interface
    virtual bool get_nlp_info(Ipopt::Index &n,
                              Ipopt::Index &m,
                              Ipopt::Index &nnz_jac_g,
                              Ipopt::Index &nnz_h_lag,
                              Ipopt::TNLP::IndexStyleEnum &index_style);


    virtual bool get_bounds_info(Ipopt::Index n,
                                 Ipopt::Number *x_l,
                                 Ipopt::Number *x_u,
                                 Ipopt::Index m,
                                 Ipopt::Number *g_l,
                                 Ipopt::Number *g_u);


    virtual bool get_starting_point(Ipopt::Index n,
                                    bool init_x,
                                    Ipopt::Number *x,
                                    bool init_z,
                                    Ipopt::Number *z_L,
                                    Ipopt::Number *z_U,
                                    Ipopt::Index m,
                                    bool init_lambda,
                                    Ipopt::Number *lambda);

    virtual bool eval_f(Ipopt::Index n,
                        const Ipopt::Number *x,
                        bool new_x,
                        Ipopt::Number &obj_value);

    virtual bool eval_grad_f(Ipopt::Index n,
                             const Ipopt::Number *x,
                             bool new_x,
                             Ipopt::Number *grad_f);

    virtual bool eval_g(Ipopt::Index n,
                        const Ipopt::Number *x,
                        bool new_x,
                        Ipopt::Index m,
                        Ipopt::Number *g);

    virtual bool eval_jac_g(Ipopt::Index n,
                            const Ipopt::Number *x,
                            bool new_x,
                            Ipopt::Index m,
                            Ipopt::Index nele_jac,
                            Ipopt::Index *iRow,
                            Ipopt::Index *jCol,
                            Ipopt::Number *values);

    virtual void finalize_solution(
                      Ipopt::SolverReturn status,
                      Ipopt::Index n,
                      const Ipopt::Number *x,
                      const Ipopt::Number *z_L,
                      const Ipopt::Number *z_U,
                      Ipopt::Index m,
                      const Ipopt::Number *g,
                      const Ipopt::Number *lambda,
                      Ipopt:: Number obj_value,
                      const Ipopt::IpoptData *ip_data,
                      Ipopt::IpoptCalculatedQuantities *ip_cq);

    //Optional functions
      virtual bool eval_h(Ipopt::Index n,
                          const Ipopt::Number *x,
                          bool new_x,
                          Ipopt::Number obj_factor,
                          Ipopt::Index m,
                          const Ipopt::Number *lambda,
                          bool new_lambda,
                          Ipopt::Index nele_hess,
                          Ipopt::Index *iRow,
                          Ipopt::Index *jCol,
                          Ipopt::Number *values);

    double getSolutionActiveTorqueAngleAngleScaling();
    double getSolutionPassiveTorqueAngleBlendingParameter();
    double getSolutionTorqueAngularVelocityOmegaMaxScale();
    double getSolutionPassiveTorqueAngleCurveOffset();
    double getSolutionMaximumActiveIsometricTorqueScale();
    double getObjectiveValue();
    RigidBodyDynamics::Math::VectorNd& getConstraintError();

    void updOptimizationVariables(const Ipopt::Number *x);
  private:
    unsigned int mN, mM;

    double mMaxActivation;
    double mMinActivation;
    double mMaxTp;
    double mTauIso;

    double mTaAngleAtOneNormTorque;
    double mOmegaMax;
    double mTaLambda;
    double mTvLambda;


    double mTaAngleScaleStart;
    double mTvOmegaMaxScaleStart;
    double mTpLambdaStart;
    double mTpAngleOffsetStart;
    double mTauScalingStart;


    double mTaAngleScaleLB;
    double mTvOmegaMaxScaleLB;
    double mTpLambdaLB;
    double mTpAngleOffsetLB;
    double mTauScalingLB;


    double mTaAngleScaleUB;
    double mTvOmegaMaxScaleUB;
    double mTpLambdaUB;
    double mTpAngleOffsetUB;
    double mTauScalingUB;


    double mTaAngleScale;
    double mTvOmegaMaxScale;
    double mTpLambda;
    double mTpAngleOffset;
    double mTauScaling;

    unsigned int mIndexTaAngleScale;
    unsigned int mIndexTvOmegaMaxScale;
    unsigned int mIndexTpLambda;
    unsigned int mIndexTpOffset;
    unsigned int mIndexTauScaling;

    unsigned int mConIdxTauActMaxStart;
    unsigned int mConIdxTauActMaxEnd;
    unsigned int mConIdxTauActMinStart;
    unsigned int mConIdxTauActMinEnd;
    unsigned int mConIdxTauPassiveStart;
    unsigned int mConIdxTauPassiveEnd;


    const RigidBodyDynamics::Math::VectorNd &mJointAngle;
    const RigidBodyDynamics::Math::VectorNd &mJointAngularVelocity;
    const RigidBodyDynamics::Math::VectorNd &mJointTorque;

    RigidBodyDynamics::Math::VectorNd mXOffset;
    RigidBodyDynamics::Math::VectorNd mDtp_Dx;
    RigidBodyDynamics::Math::VectorNd mConstraintErrors;
    double mObjValue;
    RigidBodyDynamics::Math::VectorNd mWeights;
    Millard2016TorqueMuscle &mTqMcl;
    TorqueMuscleInfo mTmi;
    TorqueMuscleSummary mTms;
};




    }
  }
}

#endif

