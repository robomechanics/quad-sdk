/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2019 Matthew Millard <millard.matthew@gmail.com>
 * Licensed under the zlib license. See LICENSE for more details.
 */

#ifndef RBDL_LOOP_CONSTRAINT_H
#define RBDL_LOOP_CONSTRAINT_H

#include <rbdl/rbdl_math.h>
#include <rbdl/rbdl_mathutils.h>
#include <rbdl/Kinematics.h>
#include <rbdl/Constraint.h>
#include <assert.h>

namespace RigidBodyDynamics {




/**
  @brief Implements a rigid kinematic loop (or body-to-body) constraints as 
        described in Ch. 8 of Featherstone's Rigid Body Dynamics Algorithms 
        book.

  \image html fig_Constraint_Loop.png "A loop constraint restricts the movement of a point p located on the predecessor body relative to point s on a successor body in the directions T_i."

    For details on this constraint please do read this documentation. However
    note that a typical user of RBDL should not need to use any of the functions 
    described in this class but instead should use the functions that are 
    defined in ConstraintSet (AddContactConstraint, calcForces, 
    calcPositionError, calcVelocityError, calcBaumgarteStabilizationForces, etc)
    when working with this constraint. For those interested in all of the 
    details of this constraint and how it works please refer to the source
    code for this class, and its base class.

    A LoopConstraint will zero the acceleration between body-fixed points 
    \f$p\f$ and \f$s\f$ in the spatial directions \f$^pT_i\f$. The user specifies
    \f$p\f$ and \f$s\f$ by passing in the spatial transform from the origin of
    the predecessor body to the predecessor frame (\f$_{1}^{1}r_{P}\f$, 
    \f$^1E_p\f$) and likewise for the successor body to the successor frame
    (\f$_{2}^{2}r_{S}\f$, \f$^1E_s\f$). The spatial directions that the 
    constraint is applied are specified in the vector \f$^pT_i\f$ which is
    resolved in the coordinates of the predecessor frame.

    As with any other constraint this position-level constraint is index 
    reduced and applied at the acceleration level. This constraint will only
    be satisfied at the position-level and velocity-level if, prior to enabling
    the constraint, the constraint equations are satisified at the position
    and velocity levels. When assembling the constraint the functions 
    CalcAssemblyQ and CalcAssemblyQDot can be used to satisfy this constraint
    at the position and velocity levels.

    During the process of integration numerical error may accumulate. To prevent
    this error from growing Baumgarte stabilization can be enabled using the
    function setEnableBaumgarteStabilization provided in the base class. 
    Numerical drift is usually small with this constraint because \f$^pT_i\f$ is
    body fixed. When this constraint is used in an optimal control problem 
    Baumgarte stabilization may also be used to guide the solver to physically
    valid model configurations. Without stabilization the solver may introduce
    large constraint errors which will otherwise not be reduced. By default
    Baumgarte stabilization is not enabled. 

    By default this constraint does have its position-level and velocity-level
    errors defined. This has the consequence that CalcAssemblyQ will update
    the position of the model to satisfy these constraints, as will  
    CalcAssemblyQDot at the velocity level. In addition, Baumgarte stabilization
    will apply forces to the model in response to constraint errors at 
    both the position and velocity level.

*/
class RBDL_DLLAPI LoopConstraint : public Constraint {

public:


  LoopConstraint();

  /**
    @param bodyIdPredecessor the identifier of the predecessor body

    @param bodyIdSuccessor the identifier of the successor body

    @param XPredecessor a spatial transform localizing the constrained
            frames on the predecessor body, expressed with respect to the 
            predecessor body frame. Note the position vector should be
            the r_BPB (from the body's origin, to the predessor frame, in the
            coordinates of the body's frame) and E_BP (the rotation matrix that
            will rotate vectors from the coordinates of the P frame to the 
            coordinates of the body's frame).
    
    @param XSuccessor a spatial transform localizing the constrained
          frames on the successor body, expressed with respect to the successor
          body frame. Note the position vector should be
            the r_BSB (from the body's origin, to the successor frame, in the
            coordinates of the body's frame) and E_BS (the rotation matrix that
            will rotate vectors from the coordinates of the S frame to the 
            coordinates of the body's frame).
    
    @param constraintAxisInPredessor a spatial vector, resolved in the frame of
            the predecessor frame, indicating the axis along which the 
            constraint acts.


     @param enableBaumgarteStabilization (optional, default false) setting this
            flag to true will modify the right hand side of the acceleration
            equation with a penaltiy term that is proportional to the constraint
            error scaled by a constant.

     @param stabilizationTimeConstant (optional, defaults to 0.1 sec) this
            value scales the strength of Baumgarte stabilization so that the
            settling time of the error is proportional the value given here.

     @param name a human readable name (optional, default: NULL).
            Set this field to a unique name (within this ConstraintSet) so that
            the function GetConstraintIndex can find it.

     @param userDefinedId a user defined id (optional, defaults to max()).
            Set this field to a unique number (within this ConstraintSet) so that
            the function GetConstraintIndex can find it.

     @param positionLevelConstraint (optional, defaults to true to be consistent
              with the original implementation):
              When set to true, position errors will be computed for this
              constraint. This has the consequence that the function
              CalcAssemblyQ will assemble this constraint at the position level.
              In addition if Baumgarte stabilization is enabled, stabilization
              forces will be applied as a function  of the position error.

     @param velocityLevelConstraint (optional, defaults to true to be consistent
              with the original implementation) :
              This flag controls whether or not velocity errors are computed
              for this constraint. When velocity errors are computed
              they are used by CalcAssemblyQDot (to assemble this constraint
              at the velocity level) and by Baumgarte stabilization (if it is
              enabled) to modify the right hand side of the acceleration
              equation with a penalty term proportional to error.

  */
  LoopConstraint(
      const unsigned int bodyIdPredecessor,
      const unsigned int bodyIdSuccessor,
      const Math::SpatialTransform &XPredecessor,
      const Math::SpatialTransform &XSuccessor,
      const Math::SpatialVector &constraintAxisInPredessor,
      bool enableBaumgarteStabilization = false,
      double stabilizationTimeConstant = 0.1,
      const char *loopConstraintName = NULL,
      unsigned int userDefinedId = std::numeric_limits<unsigned int>::max(),
      bool positionLevelConstraint=true,
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
    @return the vector of constraint axes which are resolved in the 
            coordinate system of the predecessor frame
  */
  const std::vector< Math::SpatialVector >& getConstraintAxes(){
    return T;
  }

  /**
    @param constraintAxis to append. This constraintAxis must have a norm of 1
            and be normal to the existing constraintAxis
    @param positionLevelConstraint if true position errors will be evaluated
            for this constraint entry.
    @param velocityLevelConstraint if true velocity errors will be evalulated
            for this constraint entry.                        
  */
  void appendConstraintAxis(const Math::SpatialVector &constraintAxis,
                          bool positionLevelConstraint = true,
                          bool velocityLevelConstraint = true);


private:
  /// Vector of constraint axis resolved in the predecessor frame
  std::vector< Math::SpatialVector > T;
  /// A local working double 
  double dblA;
};

} 

/* RBDL_LOOP_CONSTRAINT_H */
#endif
