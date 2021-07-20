/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2019 Matthew Millard <millard.matthew@gmail.com>
 * Licensed under the zlib license. See LICENSE for more details.
 */

#ifndef RBDL_CONTACT_CONSTRAINT_H
#define RBDL_CONTACT_CONSTRAINT_H

#include <rbdl/rbdl_math.h>
#include <rbdl/rbdl_mathutils.h>
#include <rbdl/Kinematics.h>
#include <rbdl/Constraint.h>
#include <assert.h>

namespace RigidBodyDynamics {

/**
  @brief Implements a rigid kinematic body-point--to--ground constraint along a 
          normal direction as described in Ch. 11 of Featherstone's Rigid Body 
          Dynamics Algorithms book.

  \image html fig_Constraint_Contact.png "A contact constraint restricts the movement of a point p located on a body such that its velocity and acceleration in direction n_i is zero."

    For details on this constraint please do read this documentation. However
    note that a typical user of RBDL should not need to use any of the functions 
    described in this class but instead should use the functions that are 
    defined in ConstraintSet (AddContactConstraint, calcForces, 
    calcPositionError, calcVelocityError, calcBaumgarteStabilizationForces, etc)
    when working with this constraint. For those interested in all of the 
    details of this constraint and how it works please refer to the source
    code for this class, and its base class.

    This class should be named PointGroundConstraint but retains the name
    ConstactConstraint for historical reasons. A ContactConstraint will prevent
    a body-fixed point \f$p\f$, from moving in a ground-fixed direction 
    \f$n_i\f$. The user specifies \f$p\f$ by passing in the body fixed vector 
    \f$_{1}^{1}r_{P}\f$ (the vector from the origin of the body's frame, to point 
    \f$p\f$, resolved in the coordinates of the body's frame) and the normal
    vector \f$^{0}n_{i}\f$ which is fixed in the base frame.
    
    As with any other constraint, this restriction is only applied
    at the acceleration-level. This constraint will only be satisfied at the
    velocity level if point \f$p\f$ has a velocity of zero
    in the direction \f$n_i\f$ before the constraint is switched on.
    When simulating foot ground contact, for example, you can compute the 
    state that simulates a plastic impact (which will zero the velocity of 
    point \f$p\f$ in direction \f$n_i\f$) using the family of functions 
    named ComputeConstraintImpulses (e.g. ComputeConstraintImpulsesDirect,
    ComputeConstraintImpulsesNullSpace, etc.).

    During the process of integration numerical error may accumulate so that
    point \f$p\f$ has a non-zero velocity in direction \f$n_i\f$. To prevent
    this error from growing Baumgarte stabilization can be enabled using
    the function setEnableBaumgarteStabilization provided in the base class 
    Constraint.h. Numerical drift is usually quite small with this constraint
    because \f$n_i\f$ is time and state invariant. By default Baumgarte 
    stabilization is not enabled.

    By default this constraint does not have any position-level error, only
    errors at the velocity level are defined. This has two consequences: the
    method CalcAssemblyQ will not modify the model at the position-level to
    satisfy this constraint; and Baumgarte stabilization forces are only applied
    in response to errors at the velocity-level. This gives the user a 
    certain amount of flexibility when using this constraint to simulate
    foot-ground contact as the user does not have to remember to update the
    reference position on the ground of this constraint class prior to enabling
    the constraint. 
  

    To extend this class to include position-level 
    constraints one would have to add functions to update the groundPoint and
    then use the methods enableConstraintErrorFromPositionLevel provided
    in the base class to turn on the computation of position-level errors for
    the desired index. Given that there exists a large number of models that
    make use of this constraint assuming that the position errors are
    zero any future developers should retain this default behaviour.

*/
class RBDL_DLLAPI ContactConstraint : public Constraint {

public:


  ContactConstraint();

  /**

    @param bodyId the body which is affected directly by the constraint

    @param bodyPoint the point that is constrained relative to the
            contact body

    @param groundConstraintNormalVectors the normal direction in which to apply 
            the constraint

     @param name a human readable name (optional, default: NULL).
            Set this field to a unique name (within this ConstraintSet) so that
            the function GetConstraintIndex can find it.

     @param userDefinedId a user defined id (optional, defaults to max()).
            Set this field to a unique number (within this ConstraintSet) so that
            the function GetConstraintIndex can find it.

    @param enableBaumgarteStabilization (optional, default false) setting this
            flag to true will modify the right hand side of the acceleration
            equation with a penaltiy term that is proportional to the constraint
            error scaled by a constant.

    @param stabilizationTimeConstant (optional, defaults to 0.1 sec) this
            value scales the strength of Baumgarte stabilization so that the
            settling time of the error is proportional the value given here.

    @param velocityLevelConstraint (advanced, optional, defaults to true) :
                This flag controls whether or not velocity errors are computed
                for this constraint. When velocity errors are computed
                they are used by CalcAssemblyQDot (to assemble this constraint
                at the velocity level) and by Baumgarte stabilization (if it is
                enabled) to modify the right hand side of the acceleration
                equation with a penalty term proportional to error. To be
                consistent with the original RBDL implementation position level
                errors are not computed (all 0's) for this constraint type.
  */

  ContactConstraint(
      const unsigned int bodyId,
      const Math::Vector3d &bodyPoint,
      const Math::Vector3d &groundConstraintNormalVectors,
      const char *contactConstraintName = NULL,
      unsigned int userDefinedId = std::numeric_limits<unsigned int>::max(),
      bool enableBaumgarteStabilization=false,
      double stabilizationTimeConstant=0.1,      
      bool velocityLevelConstraint=true);


  void bind( const Model &model) override;

  void calcConstraintJacobian(  Model &model,
                                const double time,
                                const Math::VectorNd &Q,
                                const Math::VectorNd &QDot,
                                Math::MatrixNd &GSysUpd,
                                ConstraintCache &cache,
                                bool updateKinematics=false) override;

  void calcGamma( Model &model,
                  const double time,
                  const Math::VectorNd &Q,
                  const Math::VectorNd &QDot,
                  const Math::MatrixNd &GSys,
                  Math::VectorNd &gammaSysUpd,
                  ConstraintCache &cache,
                  bool updateKinematics=false) override;

  void calcPositionError( Model &model,
                          const double time,
                          const Math::VectorNd &Q,
                          Math::VectorNd &errSysUpd,
                          ConstraintCache &cache,
                          bool updateKinematics=false) override;

  void calcVelocityError( Model &model,
                          const double time,
                          const Math::VectorNd &Q,
                          const Math::VectorNd &QDot,
                          const Math::MatrixNd &GSys,
                          Math::VectorNd &derrSysUpd,
                          ConstraintCache &cache,
                          bool updateKinematics=false) override;

  void calcConstraintForces(
        Model &model,
        const double time,
        const Math::VectorNd &Q,
        const Math::VectorNd &QDot,
        const Math::MatrixNd &GSys,
        const Math::VectorNd &lagrangeMultipliersSys,
        std::vector< unsigned int > &constraintBodiesUpd,
        std::vector< Math::SpatialTransform > &constraintBodyFramesUpd,
        std::vector< Math::SpatialVector > &constraintForcesUpd,
        ConstraintCache &cache,
        bool resolveAllInRootFrame = false,
        bool updateKinematics=false) override;



  /**
    @return the vector of constraint normals which are resolved in the 
            coordinate system of the ground or base frame.
  */
  const std::vector< Math::Vector3d >& getConstraintNormalVectors(){
    return T;
  }

    /**
    @param normal vector to append. This normal vector must have a length of 1
            and be orthogonal to the normal vectors already in the constraint.
    @param velocityLevelConstraint if true velocity errors will be evalulated
            for this constraint entry.                        
  */
  void appendNormalVector(const Math::Vector3d &normal,
                          bool velocityLevelConstraint=true);

  /*** @brief Added to support ForwardDynamicsKokkevis

    @param model: the multibody model
    @param Q: the generalized positions of the model
    @param QDot: the generalized velocities of the model    
    @param QDDot: the generalized accelerations of the model    
    @param pointAccelerationSysUpd the system's vector of point accelerations
    @param updateKinematics setting this to true will force kinematic functions
            to use the values of Q, QDot, and QDDot passed into this function.
  */
  void calcPointAccelerations(Model &model,
                              const Math::VectorNd &Q,
                              const Math::VectorNd &QDot,
                              const Math::VectorNd &QDDot,
                              std::vector<Math::Vector3d> &pointAccelerationSysUpd,
                              bool updateKinematics=false);

  /*** @brief To support ForwardDynamicsKokkevis
    @param model: the multibody model
    @param Q: the generalized positions of the model
    @param QDot: the generalized velocities of the model    
    @param QDDot: the generalized accelerations of the model    
    @param pointAccelerationUpd the acclerations of the one point in this constraint
    @param updateKinematics setting this to true will force kinematic functions
            to use the values of Q, QDot, and QDDot passed into this function.

  */

  void calcPointAccelerations(Model &model,
                              const Math::VectorNd &Q,
                              const Math::VectorNd &QDot,
                              const Math::VectorNd &QDDot,
                              Math::Vector3d &pointAccelerationUpd,
                              bool updateKinematics=false);


  /*** @brief To support ForwardDynamicsKokkevis
    @param model a reference to the multibody model
    @param pointAccelerationSysUpd the system's vector of point accelerations
    @param ddErrSysUpd: the error in the the acceleration of the point in this
                        constraint along the normal directions.

  */
  void calcPointAccelerationError(
                    const std::vector<Math::Vector3d> &pointAccelerationsSys,
                    Math::VectorNd &ddErrSysUpd);

  /*** @brief To support ForwardDynamicsKokkevis
    @param model a reference to the multibody model
    @param Q: the generalized positions of the model
    @param cache: a ConstraintCache struct used for working memory
    @param fExtSysUpd: the point force constraint Jacobian
    @param updateKinematics setting this to true will force kinematic functions
            to use the value of Q passed in.
  */
  void calcPointForceJacobian(
          Model &model,
          const Math::VectorNd &Q,
          ConstraintCache &cache,
          std::vector<Math::SpatialVector> &fExtSysUpd,
          bool updateKinematics=false);

private:
  ///A vector of the ground normal vectors used in this constraint.
  std::vector< Math::Vector3d > T;
  ///The location of the ground reference point
  Math::Vector3d groundPoint;
  ///A working double 
  double dblA;

};



} 

/* RBDL_CONTACT_CONSTRAINT_H */
#endif
