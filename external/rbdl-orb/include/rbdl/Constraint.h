/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2019 Matthew Millard <millard.matthew@gmail.com>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#ifndef RBDL_CONSTRAINT_H
#define RBDL_CONSTRAINT_H

#include <rbdl/rbdl_math.h>
#include <rbdl/rbdl_mathutils.h>
#include <assert.h>

namespace RigidBodyDynamics {

/// Enum to describe the type of a constraint.
enum ConstraintType {
  ConstraintTypeContact=0,
  ConstraintTypeLoop,
  ConstraintTypeCustom,
  ConstraintTypeLast,
};

/**
  This struct contains a working memory that can be used by extensions to
  the Constraint interface. The intent by passing in a ConstraintCache variable
  to each of the virtual methods of the Constraint interface is to reduce, 
  as much as possible, the memory footprint of each concrete Constraint class.
  The cache has been sized so that it is more than enough to accomodate all of
  the memory requirements of the contact constraints and loop constraint 
  classes.
*/
struct ConstraintCache {

  ///Here N is taken to mean the number of elements in QDot.
  Math::VectorNd vecNZeros;

  ///Working vectors that are sized to match the length of qdot
  Math::VectorNd vecNA, vecNB, vecNC, vecND;

  ///Working Vector3d entries
  Math::Vector3d  vec3A, vec3B, vec3C, vec3D, vec3E, vec3F;

  ///Working Matrix3d entries
  Math::Matrix3d  mat3A, mat3B, mat3C, mat3D, mat3E, mat3F;

  ///Working 3 x N entry matricies, where N is the length of QDot.
  ///Quite useful for point Jacobians
  Math::MatrixNd mat3NA, mat3NB, mat3NC, mat3ND;

  ///Working 6 x N entry matricies, where N is the length of QDot.
  ///Quite useful for 6D point Jacobians
  Math::MatrixNd mat6NA, mat6NB, mat6NC, mat6ND;

  ///Working SpatialVectors
  Math::SpatialVector  svecA, svecB, svecC, svecD, svecE, svecF;

  ///Working SpatialTransforms
  Math::SpatialTransform stA, stB, stC, stD;

  ConstraintCache(){}
};


/**
  \brief Interface to define general-purpose constraints 

  The Constraint interface is a general-purpose interface that is rich
  enough to define time-varying constraints at the position-level \f$\phi_p(q,t)=0\f$,
  the velocity-level \f$\phi_v(\dot{q},q,t)=0\f$, or the acceleration-level
  \f$\phi_a(\ddot{q},\dot{q},q,t)=0\f$. These constraints all end up being applied at the
  acceleration-level by taking successive derivatives until we are left with
  \f$\Phi(\ddot{q},\dot{q},q,t)=0\f$. A new concrete Constraint class must
  provide implementations for the following methods:

  - bind     
  - calcConstraintJacobian 
  - calcGamma
  - calcPositionError 
  - calcVelocityError
  - calcConstraintForces

  Please see the doxygen for each of these functions in the Constraint
  interface for details. In addition, please have a look at Constraint_Contact
  (and ContactsTests.cc) and Constraint_Loop (and LoopConstraintsTests.cc) for
  working examples to guide your own implementation. 

  Please note that constraints are challenging to derive and implement even for 
  multibody dynamics experts. If you are unfamiliar with the concept please 
  refer to Featherstone's Robot Dynamics Algorithms text. If you are authoring 
  a new constraint please read the section below on how to test your constraint.

  - <b> Physical correctness (\f$G\f$ and \f$\gamma\f$): Workless? </b>
    Ensure that the constraint is workless: perform a forward simulation of a 
    passive system that includes the new constraint oscillating under gravity. 
    Compute the sum of kinetic and potential energy called system energy. System
    energy should be constant, but will exhibit small amounts of accumulating
    error due to integration error. If the tolerances on the integration are
    tightened these errors should also drop. This kind of analysis is usually
    done when the first implementation of the constraint is being developed.
    This kind of test is usually not included as a unit test because it
    requires an accurate numerical integrator (which introduces an external 
    dependency). A faster version of this test is to evaluate the forward 
    dynamics of a given set of states and then sum up the joint powers which
    should sum to zero in a conservative system.

  - <b> Physical correctness (\f$G\f$ and \f$\gamma\f$): Agrees with equivalent joint-coordinate model?</b>
    Evaluate \f$\ddot{q}\f$ by calling ForwardDynamicsConstraintsDirect 
    on your system. If possible construct a model that uses joint coordinates 
    that is perfectly equivalent to the model that uses constraints and evaluate 
    its forward dynamics using the same initial state. The matching indices of
    \f$\ddot{q}\f$ should be identical. Examples of tests like this can be seen 
    in ConstraintCorrectnessTest in LoopConstraintsTests.cc.

  - <b>Compare against static-equilibrum calculations (\f$f\f$)</b>
    Set-up a simple test model for which you can easily compute the correct
    values for the position error, velocity error, and constraint forces. This
    is most easily done by solving for the \f$\tau\f$ that will put the sytem
    in a state of static equilibrium. Then you can easily solve by hand for 
    the forces the constraint should develop and compare those to the values
    returned by calcForce. Examples of tests like this can be seen in 
    TestExtendedConstraintFunctionsContact in ContactsTests.cc and 
    TestExtendedConstraintFunctionsLoop in LoopConstraintsTests.cc 

  - <b>Compare against known errors (\f$\epsilon_{P}\f$ and \f$\epsilon_{V}\f$) </b>
    The values returned for the position and velocity errors of the constraint
    are most easily evaluated by constructing a system in which the constraint
    is satisfied at the position and velocity level and then calling
    calcPositionError and calcVelocityError to ensure
    that the returned errors are zero. A follow up test is then to introduce
    a known error to the generalized positions and velocities and sure that
    the error returned by cal Tests like this can be found in 
    TestExtendedConstraintFunctionsContact in ContactsTests.cc and 
    TestExtendedConstraintFunctionsLoop in LoopConstraintsTests.cc. 


*/
class RBDL_DLLAPI Constraint {
  public:


//==============================================================================
// Extend the functions below for your own constraint
//==============================================================================
    /**
      \brief Any local memory that has a dimension of N, where N is the length 
      of QDot, is resized in this function.   

      @param model: a reference to the multibody model. Use the model's fields
                    of qdot_size to size local memory (if any) that depends on
                    the model.
    */
    virtual void bind(const Model &model)=0;


    /**
      \brief In this function the matrix \f$G_{i}\f$ of this contraint is 
      inserted into the system constraint Jacobian \f$G\f$ (GSysOutput). The
      submatrix \f$G_{i}\f$ begins at rowInSystem, has sizeOfConstraint 
      number of rows, and fills the full number of columns in G.

      @param model: a reference to the multibody model.
      @param time: the time which is included so that rheonomic
      constraints might be included (in the future).
      @param Q: the vector of generalized positions.
      @param QDot: the vector of generalized velocities.
      @param GSysOutput: a reference to the constraint Jacobian for the entire
          system. Insert the G sub-matrix for this constraint beginning and 
          rowInSystem, with a length sizeOfConstraint rows, and a width of
          the full number of columns in GSysOutput.
      @param cache: a ConstraintCache object which contains ample pre-allocated
          memory that can be used to reduce the memory footprint of each 
          Constraint implementation.
      @param updateKinematics: setting this flag to true will cause all
          calls to kinematic dependent functions to be updated using the 
          generalized coordinates passed into this function.          
    */
    virtual void calcConstraintJacobian(  Model &model,
                                          const double time,
                                          const Math::VectorNd &Q,
                                          const Math::VectorNd &QDot,
                                          Math::MatrixNd &GSysOutput,
                                          ConstraintCache &cache,
                                          bool updateKinematics=false) = 0;

    /**
      @brief In this function the vector \f$\gamma_{i}\f$ of this constraint is 
      inserted into the right-hand-side vector \f$\gamma_{SYS}\f$ of the system. 
      The vector \f$\gamma_{i}\f$ begins at rowInSystem, and has 
      sizeOfConstraint rows.

      @param model: a reference to the multibody model.
      @param time: the time which is included so that rheonomic
      constraints might be included (in the future).
      @param Q: the vector of generalized positions.
      @param QDot: the vector of generalized velocities.
      @param GSys: a reference to the constraint Jacobian for the entire 
          system. If \f$G\f$ is needed in this function do not re-evaluate it 
          but instead extract it from the system \f$G\f$.
      @param gammaSysOutput the system's gamma vector. Insert the
          gamma sub-vector for this constraint beginning and rowInSystem and
          with a length of sizeOfConstraint rows.
      @param cache: a ConstraintCache object which contains ample pre-allocated
          memory that can be used to reduce the memory footprint of each 
          Constraint implementation.
      @param updateKinematics: setting this flag to true will cause all
          calls to kinematic dependent functions to be updated using the 
          generalized coordinates passed into this function.          
    */
    virtual void calcGamma( Model &model,
                            const double time,
                            const Math::VectorNd &Q,
                            const Math::VectorNd &QDot,
                            const Math::MatrixNd &GSys,
                            Math::VectorNd &gammaSysOutput,
                            ConstraintCache &cache,
                            bool updateKinematics=false) = 0;


    /**
    @brief In this function the sub vector \f$\phi_{p,i}(q,t)\f$ of this 
    constraint is inserted into the position error vector of the system. If the 
    constraint is velocity-level constraint or higher (noted in the boolean
    member variable positionConstraint) this should be set to zero. 
    The vector \f$\phi_{p,i}(\dot{q},t)\f$ begins at rowInSystem, and has 
    sizeOfConstraint rows.

      @param model: a reference to the multibody model
      @param time: the time which is included so that rheonomic
      constraints might be included (in the future).
      @param Q: the vector of generalized positions.
      @param errSysOutput the system's constraint position error vector.
          Insert the position error sub-vector for this constraint beginning and
          rowInSystem and with a length of sizeOfConstraint rows.
      @param cache: a ConstraintCache object which contains ample pre-allocated
          memory that can be used to reduce the memory footprint of each 
          Constraint implementation.
      @param updateKinematics: setting this flag to true will cause all
          calls to kinematic dependent functions to be updated using the 
          generalized coordinates passed into this function.          
    */
    virtual void calcPositionError( Model &model,
                                    const double time,
                                    const Math::VectorNd &Q,
                                    Math::VectorNd &errSysOutput,
                                    ConstraintCache &cache,
                                    bool updateKinematics=false) = 0;


    /**
    @brief In this function the sub vector \f$\phi_{v,i}(q, \dot{q},t)\f$ of 
    this constraint is inserted into the position error vector of the system. 
    If the constraint is an acceleration-level constraint (noted in the boolean
    member variable velocityConstraint) this sub vector should
    be zero. The vector \f$\phi_{v,i}(q, \dot{q},t)\f$ begins at rowInSystem, 
    and has sizeOfConstraint rows.

      @param model: a reference to the multibody model.
      @param time: the time which is included so that rheonomic
      constraints might be included (in the future).
      @param Q: the vector of generalized positions.
      @param QDot: the vector of generalized velocities.
      @param GSys: a reference to the constraint Jacobian for the entire 
          system. If \f$G\f$ is needed in this function do not re-evaluate it 
          but instead extract it from the system \f$G\f$: this constraint's 
          sub-matrix begins at rowInSystem, has sizeOfConstraint rows, and
          the full number of columns of GSys.      
      @param derrSysOutput the system's constraint velocity error vector.
          Insert the velocity error sub-vector for this constraint beginning 
          and rowInSystem and with a length of sizeOfConstraint rows.
      @param cache: a ConstraintCache object which contains ample pre-allocated
          memory that can be used to reduce the memory footprint of each 
          Constraint implementation.
      @param updateKinematics: setting this flag to true will cause all
          calls to kinematic dependent functions to be updated using the 
          generalized coordinates passed into this function.    
    */
    virtual void calcVelocityError( Model &model,
                                    const double time,
                                    const Math::VectorNd &Q,
                                    const Math::VectorNd &QDot,
                                    const Math::MatrixNd &GSys,
                                    Math::VectorNd &derrSysOutput,
                                    ConstraintCache &cache,
                                    bool updateKinematics=false) = 0;


    /**
      @brief This function resolves the generalized forces this constraint
      applies to the system into the wrenches that are applied to the bodies
      that are involved in the constraint.


      @param model: a reference to the multibody model.

      @param time: the time which is included so that rheonomic
      constraints might be included (in the future).

      @param Q: the vector of generalized positions.

      @param QDot: the vector of generalized velocities.

      @param GSys: a reference to the constraint Jacobian for the entire 
          system. If \f$G\f$ is needed in this function do not re-evaluate it 
          but instead extract it from the system \f$G\f$: this constraint's 
          sub-matrix begins at rowInSystem, has sizeOfConstraint rows, and
          the full number of columns of GSys.      

      @param LagrangeMultipliersSys: the vector of Lagrange multipliers for the
          entire system. The Lagrange multipliers for this constraint begin 
          at rowInSystem and has sizeOfConstraint elements.

      @param constraintBodiesOutput contains the indices of the bodies
              that contain the local frames listed in constraintBodyFramesOutput.
              If resolveAllInRootFrame is true, all of these indicies should
              be set to 0: in this case all of the wrenches are resolved in
              the root frame. 

      @param constraintBodyFramesOutput contains the local transformation from
          the origin of each body frame listed in constraintBodiesOutput to the
          frame in which the constraint wrench is applied. Note the \f$i^{th}\f$
          frame is located on the \f$i^{th}\f$ body.
          If resolveAllInRootFrame is true, all of these frames have 
          their origins resolved in the global frame and their orientation 
          set to match that tof the global frame.

      @param constraintForcesOutput contains the wrenches that the constraint
          applies to the local frames listed in constraintBodyFramesOutput.
          Note the \f$i^{th}\f$ force is resolved in the \f$i^{th}\f$ frame.If
          resolveAllInRootFrame is true, this wrench should be rotated into
          the coordinates of the root frame.

      @param cache: a ConstraintCache object which contains ample pre-allocated
          memory that can be used to reduce the memory footprint of each 
          Constraint implementation.

      @param resolveAllInRootFrame: When this parameter is 
    
          - false: the wrenches are resolved into their local frames. 
          - true: the wrenches are resolved into coordinates of the global frame

      @param updateKinematics: setting this flag to true will cause all
          calls to kinematic dependent functions to be updated using the 
          generalized coordinates passed into this function.            
    */
    virtual void calcConstraintForces(
                 Model &model,
                 const double time,
                 const Math::VectorNd &Q,
                 const Math::VectorNd &QDot,
                 const Math::MatrixNd &GSys,
                 const Math::VectorNd &LagrangeMultipliersSys,
                 std::vector< unsigned int > &constraintBodiesOutput,
                 std::vector< Math::SpatialTransform > &constraintBodyFramesOutput,
                 std::vector< Math::SpatialVector > &constraintForcesOutput,
                 ConstraintCache &cache,
                 bool resolveAllInRootFrame = false,
                 bool updateKinematics=false) = 0;


//==============================================================================
// DO NOT TOUCH!!!
//==============================================================================

    virtual ~Constraint(){};    

    Constraint();

    /**
      @param name: (optional) name of the constraint. Use only as a means to 
                  find a specific constraint.

      @param typeOfConstraint: corresponds to the enums listed in 
              ConstraintTypes. This parameter is used by a few methods that
              only work with specific types of constraints such as 
              ForwardDynamicsContactsKokkevis.

      @param sizeOfConstraint: the number of equations that define the 
              constraint manifold. Equivalently this is the number of rows in 
              this constraints Jacobian.              

      @param userDefinedIdNumber: an integer that the user can set to rapidly
              retrieve this constraint from the set.              
    */
    Constraint(const char* nameOfConstraint,
               unsigned int typeOfConstraint,
               unsigned int sizeOfConstraint,
               unsigned int userDefinedIdNumber):               
               typeOfConstraint(typeOfConstraint),
               id(userDefinedIdNumber),
               sizeOfConstraint(sizeOfConstraint),
               baumgarteParameters(1./0.1,1./0.1),
               baumgarteEnabled(false)
    {
      name = "";
      if(nameOfConstraint){
        name = nameOfConstraint;
      }
      positionConstraint.resize(sizeOfConstraint);
      velocityConstraint.resize(sizeOfConstraint);
      for(unsigned int i=0; i<sizeOfConstraint;++i){
        positionConstraint[i]=false;
        velocityConstraint[i]=false;
      }
    }


    /**
      @brief This function is called by the functions in ConstraintSet that
              add a Constraint to the system. DO NOT TOUCH THIS.
      @param rowIndex: the first index of this constraint in the systems 
                      constraint Jacobian.              

    */
    void addToConstraintSet(
        const unsigned int rowIndex)
    {
      rowInSystem = rowIndex;
    }

    /**
      @return (optional) user-defined-id
    */
    unsigned int getUserDefinedId(){
      return id;
    }

    /**
      @param userDefinedId: (optional) integer id for this constraint 
    */
    void setUserDefinedId(unsigned int userDefinedId){
      id = userDefinedId;
    }

    /**
      @param GSys : a reference to the system's constraint Jacobian
      @param GConstraint : a reference to this constraint's entry within the 
                          system constraint Jacobian
    */
    void getConstraintJacobian(const Math::MatrixNd &GSys,
                               Math::MatrixNd &GConstraint){
      GConstraint = GSys.block(rowInSystem,0,sizeOfConstraint,GSys.cols());
    }


    /**
      @param gammaSys: a reference to the system's gamma vector
      @param gammaConstraint: a reference to this constraint's entry within the 
                          system gamma vector
    */
    void getGamma(const Math::VectorNd &gammaSys,
                  Math::VectorNd &gammaConstraint){
      gammaConstraint = gammaSys.block(rowInSystem,0,sizeOfConstraint,1);
    }

    /**
      @param errSys: a reference to the system's constraint position error
      @param errConstraint: a reference to this constraint's entry within the 
                          system constraint position error.
    */
    void getPositionError(const Math::VectorNd &errSys,
                          Math::VectorNd &errConstraint){
      errConstraint = errSys.block(rowInSystem,0,sizeOfConstraint,1);
    }


    /**
      @param derrSys: a reference to the system's constraint velocity error
      @param derrConstraint: a reference to this constraint's entry within the 
                          system constraint velocity error.
    */
    void getVelocityError(const Math::VectorNd &derrSys,
                          Math::VectorNd &derrConstraint){
      derrConstraint = derrSys.block(rowInSystem,0,sizeOfConstraint,1);
    }

    /**
      @param bgParamsUpd: (output) the Baumgarte stabilization coefficients for this
                        constraint. Note the velocity error coefficient is in
                        the 0 index and the position error coefficient is in 
                        the 1st index. 
    */
    void getBaumgarteStabilizationParameters(Math::Vector2d& bgParamsUpd){
      bgParamsUpd = baumgarteParameters;
    }

    /**
      @param errPos : the position error vector of this constraint
      @param errVel : the velocity error vector of this constraint
      @param baumgarteForces: the Baumgarte stabilization forces
    */
    void getBaumgarteStabilizationForces(const Math::VectorNd &errPos,
                                         const Math::VectorNd &errVel,
                                         Math::VectorNd &baumgarteForces)
    {

        baumgarteForces=(-2*baumgarteParameters[0]*errVel
                         -baumgarteParameters[1]*baumgarteParameters[1]*errPos);
    }

    /**
      @param errPosSys : the position error vector of the system
      @param errVelSys : the velocity error vector of the system
      @param gammaSysOutput: the gamma vector of the system
    */
    void addInBaumgarteStabilizationForces(const Math::VectorNd &errPosSys,
                                           const Math::VectorNd &errVelSys,
                                           Math::VectorNd &gammaSysOutput)
    {

      //Here a for loop is used rather than a block operation
      //to be compatible with SimpleMath.
      for(unsigned int i=0; i<sizeOfConstraint;++i){
        gammaSysOutput[rowInSystem+i] +=
              -2.*baumgarteParameters[0]*errVelSys[rowInSystem+i]
             -(baumgarteParameters[1]*baumgarteParameters[1]
              )*errPosSys[rowInSystem+i];
      }
    }

    /**
      @return the type of this constraint as defined in the struct 
      ConstraintType
    */
    unsigned int getConstraintType(){
      return typeOfConstraint;
    }

    /**
      @return the number of constraint equations in this constraint
    */
    unsigned int getConstraintSize(){
      return sizeOfConstraint;
    }

    /**
      @return the index of the first row in G in which this constraint's
              entries appear.
    */
    unsigned int getConstraintIndex(){
      return rowInSystem;
    }


    /**
      @brief Calculates and sets the Baumgarte stabilization coefficients as
              a function of tStab, the approximate time-constant of 
              stabilization.
      @param tStab : a time-constant that is used to compute values for the 
                  position and velocity level stabilization terms. Note that
                  a smaller time constant means stronger stabilization forces
                  and a numerically stiffer system.
    */
    void setBaumgarteTimeConstant(double tStab){
      assert(tStab > 0);
      baumgarteParameters[0] = 1./tStab;
      baumgarteParameters[1] = 1./tStab;
    }


    /**
      @param flagEnableBaumgarteStabilization: setting this to true will enable
              Baumgarte stabilization for this constraint.
    */
    void setEnableBaumgarteStabilization(bool flagEnableBaumgarteStabilization){
      baumgarteEnabled = flagEnableBaumgarteStabilization;
    }

    /**
      @return true if Baumgarte stabilization is enabled, false otherwise.
    */
    bool isBaumgarteStabilizationEnabled(){
      return baumgarteEnabled;
    }


    /**
      @brief: Will set the vectors positionConstraint and velocityConstraint
              to be consistent with a constraint that is described from the 
              position-level up.
      @param constraintSubIndex: the sub index into this contraint group.
    */
    void enableConstraintErrorFromPositionLevel(unsigned int constraintSubIndex)
    {
      assert(constraintSubIndex < sizeOfConstraint);
      positionConstraint[constraintSubIndex] = true;
      velocityConstraint[constraintSubIndex] = true;      
    }

    /**
      @brief: Will set the vectors positionConstraint and velocityConstraint
              to be consistent with a constraint that is described from the 
              velocity-level up: position errors will be set to zero while
              velocity errors will be computed.

      @param constraintSubIndex: the sub index into this contraint group.
    */
    void enableConstraintErrorFromVelocityLevel(unsigned int constraintSubIndex)
    {
      assert(constraintSubIndex < sizeOfConstraint);
      positionConstraint[constraintSubIndex] = false;
      velocityConstraint[constraintSubIndex] = true;      
    }


    /**
      @brief: Will set the vectors positionConstraint and velocityConstraint
              to be consistent with a constraint that is described from the 
              acceleration-level: both position and velocity errors will be 
              set to zero.
              
      @param constraintSubIndex: the sub index into this contraint group.
    */
    void enableConstraintErrorFromAccelerationLevel(unsigned int constraintSubIndex)
    {
      assert(constraintSubIndex < sizeOfConstraint);
      positionConstraint[constraintSubIndex] = false;
      velocityConstraint[constraintSubIndex] = false;      
    }

    /**
      @brief: Will set the all elements in positionConstraint and 
              velocityConstraint to be consistent with a constraint that is 
              described from the position-level up.
    */
    void enableConstraintErrorFromPositionLevel()
    {
        for(unsigned int i=0; i<sizeOfConstraint;++i){
          positionConstraint[i] = true;
          velocityConstraint[i] = true;                
        }
    }

    /**
      @brief: Will set the all elements in positionConstraint and 
              velocityConstraint to be consistent with a constraint that is 
              described from the velocity-level up.
    */
    void enableConstraintErrorFromVelocityLevel()
    {
        for(unsigned int i=0; i<sizeOfConstraint;++i){
          positionConstraint[i] = false;
          velocityConstraint[i] = true;                
        }
    }

    /**
      @brief: Will set the all elements in positionConstraint and 
              velocityConstraint to be consistent with a constraint that is 
              described from the acceleration-level up.
    */
    void enableConstraintErrorFromAccelerationLevel()
    {
        for(unsigned int i=0; i<sizeOfConstraint;++i){
          positionConstraint[i] = false;
          velocityConstraint[i] = false;                
        }
    }


    /**
      @brief Returns the boolean value that determines whether or not
              position-level errors are computed for this sub-index into
              this constraint.
      @param constraintSubIndex: the sub index of interest in this constraint
    */
    bool getPositionLevelError(unsigned int constraintSubIndex){
      assert(constraintSubIndex < sizeOfConstraint);
      return positionConstraint[constraintSubIndex];      
    }

    /**
      @brief Returns the boolean value that determines whether or not
              velocity-level errors are computed for this sub-index into
              this constraint.
      @param constraintSubIndex: the sub index of interest in this constraint
    */
    bool getVelocityLevelError(unsigned int constraintSubIndex){
      assert(constraintSubIndex < sizeOfConstraint);
      return velocityConstraint[constraintSubIndex];      
    }

    /**
      @return the user defined name
    */
    const char* getName(){
      return name.c_str();
    }

    /**
      @return a vector of the bodyIds that are involved in this constraint
    */
    const std::vector< unsigned int >& getBodyIds(){
      return bodyIds;
    }

    /**
      @return a vector of local frames (which are located on the corresponding
      body in the vector of body Ids returned by getBodyIds) that are involved 
      in this constraint.
    */
    const std::vector< Math::SpatialTransform >& getBodyFrames(){
      return bodyFrames;
    }

  protected:
    ///A user defined name which is unique to this constraint set
    std::string name;

    ///A user defined id which is unique to this constraint set
    unsigned int id;
    ///The type of this constraint
    unsigned int typeOfConstraint;

    ///The number of rows that this constraint adds to G.
    unsigned int sizeOfConstraint;

    ///The first row in G that corresponds to this constraint.
    unsigned int rowInSystem;

    ///The index of the predecessor body in the vector of bodies in Model
    std::vector< unsigned int > bodyIds;

    ///Transform from the frame of the predecessor body to the constraint frame
    std::vector< Math::SpatialTransform > bodyFrames;

    ///The Baumgarte stabilization coefficients at the position and velocity
    /// level
    Math::Vector2d baumgarteParameters;

    ///A flag which enables or disables Baumgarte stabilization
    bool baumgarteEnabled;

    ///A mask that is used to selectively enable/disable the calculation of
    /// position-level and velocity-level errors. These errors are used to
    /// functions that assemble the system at the position and velocity levels,
    /// and also by functions that stablize the constraint error at the
    /// position and velocity level.
    std::vector< bool > positionConstraint;
    std::vector< bool > velocityConstraint;
};




/** @} */

} 

/* namespace RigidBodyDynamics */

/* RBDL_CONSTRAINT_H */
#endif
