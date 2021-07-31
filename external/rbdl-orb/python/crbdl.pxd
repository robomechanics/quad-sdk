#cython: boundscheck=False

from libcpp.string cimport string
from libcpp cimport bool
from libcpp.vector cimport vector
from libcpp.map cimport map as cpp_map

cimport cpython.ref as cpy_ref



cdef extern from "<rbdl/rbdl_math.h>" namespace "RigidBodyDynamics::Math":
    cdef cppclass VectorNd:
        VectorNd ()
        VectorNd (int dim)
        int rows()
        int cols()
        void resize (int)
        double& operator[](int)
        double* data()

    cdef cppclass Vector3d:
        Vector3d ()
        int rows()
        int cols()
        double& operator[](int)
        double* data()

    cdef cppclass Vector2d:
        Vector2d ()
        int rows()
        int cols()
        double& operator[](int)
        double* data()

    cdef cppclass Quaternion:
        Quaternion ()
        int rows()
        int cols()
        double& operator[](int)
        double* data()
        Matrix3d toMatrix()
#        Quaternion fromMatrix (Matrix3d &mat)

    cdef cppclass SpatialVector:
        SpatialVector ()
        int rows()
        int cols()
        double& operator[](int)
        double* data()

    cdef cppclass Matrix3d:
        Matrix3d ()
        int rows()
        int cols()
        double& coeff "operator()"(int,int)
        double* data()

    cdef cppclass MatrixNd:
        MatrixNd ()
        MatrixNd (int rows, int cols)
        int rows()
        int cols()
        void resize (int,int)
        double& coeff "operator()"(int,int)
        double* data()
        void setZero()

    cdef cppclass SpatialMatrix:
        SpatialMatrix ()
        int rows()
        int cols()
        double& coeff "operator()"(int,int)
        double* data()

    cdef cppclass Matrix63:
        Matrix63 ()
        int rows()
        int cols()
        double& coeff "operator()"(int,int)
        double* data()

cdef extern from "<rbdl/Quaternion.h>" namespace "RigidBodyDynamics::Math::Quaternion":
    Quaternion fromMatrix(const Matrix3d &mat)

cdef extern from "<rbdl/SpatialAlgebraOperators.h>" namespace "RigidBodyDynamics::Math":
    cdef cppclass SpatialTransform:
        SpatialTransform()
        SpatialMatrix toMatrix()
        SpatialTransform inverse()
        SpatialTransform operator*(const SpatialTransform&)
        Matrix3d E
        Vector3d r

    cdef cppclass SpatialRigidBodyInertia:
        SpatialRigidBodyInertia()
        SpatialMatrix toMatrix()

        double m
        Vector3d h
        double Ixx, Iyx, Iyy, Izx, Izy, Izz

cdef extern from "<rbdl/Body.h>" namespace "RigidBodyDynamics":
    cdef cppclass Body:
        Body()
        Body(const double mass, const Vector3d &com, const Matrix3d &inertia)
        double mMass
        Vector3d mCenterOfMass
        Matrix3d mInertia
        bool mIsVirtual

    cdef cppclass FixedBody:
        FixedBody()
        double mMass
        Vector3d mCenterOfMass
        Matrix3d mInertia
        unsigned int mMovableParent
        SpatialTransform mParentTransform
        SpatialTransform mBaseTransform
        bool mIsVirtual


cdef extern from "<rbdl/Joint.h>" namespace "RigidBodyDynamics":
    ctypedef enum JointType:
        JointTypeUndefined = 0
        JointTypeRevolute
        JointTypePrismatic
        JointTypeRevoluteX
        JointTypeRevoluteY
        JointTypeRevoluteZ
        JointTypeSpherical
        JointTypeEulerZYX
        JointTypeEulerXYZ
        JointTypeEulerYXZ
        JointTypeTranslationXYZ
        JointTypeFloatingBase
        JointTypeFixed
        JointTypeHelical
        JointType1DoF
        JointType2DoF
        JointType3DoF
        JointType4DoF
        JointType5DoF
        JointType6DoF
        JointTypeCustom

cdef extern from "<rbdl/Joint.h>" namespace "RigidBodyDynamics":
    cdef cppclass Joint:
        Joint()
        #Joint (JointType joint_type)
        Joint (JointType joint_type)
        Joint (JointType joint_type, int degreesOfFreedom)
        
        Joint (const SpatialVector axis_0)
        Joint (const SpatialVector axis_0,
               const SpatialVector axis_1)
        Joint (const SpatialVector axis_0,
               const SpatialVector axis_1,
               const SpatialVector axis_2)
        Joint (const SpatialVector axis_0,
               const SpatialVector axis_1,
               const SpatialVector axis_2,
               const SpatialVector axis_3)
        Joint (const SpatialVector axis_0,
               const SpatialVector axis_1,
               const SpatialVector axis_2,
               const SpatialVector axis_3,
               const SpatialVector axis_4)
        Joint (const SpatialVector axis_0,
               const SpatialVector axis_1,
               const SpatialVector axis_2,
               const SpatialVector axis_3,
               const SpatialVector axis_4,
               const SpatialVector axis_5)
        SpatialVector* mJointAxes
        JointType mJointType
        unsigned int mDoFCount
        unsigned int q_index
        unsigned int custom_joint_index;

cdef extern from "<ICustomJoint.h>" namespace "RigidBodyDynamics":    
    cdef cppclass ICustomJoint:
      ICustomJoint(cpy_ref.PyObject *obj) 
      void jcalc (Model &model,
      unsigned int joint_id,
      const VectorNd &q,
      const VectorNd &qdot
      )
      void jcalc_X_lambda_S (Model &model,
      unsigned int joint_id,
      const VectorNd &q
      )
      unsigned int mDoFCount
      SpatialTransform XJ
      MatrixNd S
      MatrixNd U
      MatrixNd Dinv
      VectorNd u
      VectorNd d_u

    

cdef extern from "<rbdl/Model.h>" namespace "RigidBodyDynamics":
    cdef cppclass Model:
        Model()
        unsigned int AddBody (const unsigned int parent_id,
                const SpatialTransform &joint_frame,
                const Joint &joint,
                const Body &body,
                string body_name
                )
        unsigned int AppendBody (const SpatialTransform &joint_frame,
                const Joint &joint,
                const Body &body,
                string body_name
                )
        
        unsigned int AddBodyCustomJoint (
                const unsigned int parent_id,
                const SpatialTransform &joint_frame,
                ICustomJoint *custom_joint,
                const Body &body,
                string body_name)
       
        unsigned int GetParentBodyId(
                unsigned int body_id)
        unsigned int GetBodyId(
                const char *body_name)
        string GetBodyName (
                unsigned int body_id)
        bool IsBodyId (
                unsigned int body_id)
        bool IsFixedBodyId (
                unsigned int body_id)
        Quaternion GetQuaternion (
                unsigned int body_id,
                const VectorNd &q)
        void SetQuaternion (
                unsigned int body_id,
                const Quaternion &quat,
                VectorNd &q)

        vector[unsigned int] _lambda "RigidBodyDynamics::Model::lambda"
        vector[unsigned int] lambda_q
#        vector[vector[unsigned int]] mu

        unsigned int dof_count
        unsigned int q_size
        unsigned int qdot_size
        unsigned int previously_added_body_id

        Vector3d gravity
        vector[SpatialVector] v
        vector[SpatialVector] a

        vector[Joint] mJoints
        vector[SpatialVector] S
        vector[SpatialTransform] X_J
        vector[SpatialVector] v_J
        vector[SpatialVector] c_J

        vector[unsigned int] mJointUpdateOrder

        vector[SpatialTransform] X_T

        vector[unsigned int] mFixedJointCount

        vector[Matrix63] multdof3_S
        vector[Matrix63] multdof3_U
        vector[Matrix63] multdof3_Dinv
        vector[Matrix63] multdof3_u
        vector[unsigned int] multdof3_w_index

        vector[SpatialVector] c
        vector[SpatialMatrix] IA
        vector[SpatialVector] pA
        vector[SpatialVector] U
        VectorNd d
        VectorNd u
        vector[SpatialVector] f
        vector[SpatialRigidBodyInertia] I
        vector[SpatialRigidBodyInertia] Ic
        vector[SpatialVector] hc

        vector[SpatialTransform] X_lambda
        vector[SpatialTransform] X_base

        vector[FixedBody] mFixedBodies
        unsigned int fixed_body_discriminator

        vector[Body] mBodies
        
        cpp_map[string, unsigned int] mBodyNameMap

cdef extern from "<rbdl/Kinematics.h>" namespace "RigidBodyDynamics":
    cdef void UpdateKinematics (Model& model,
            const VectorNd &q,
            const VectorNd &qdot,
            const VectorNd &qddot)
    
    cdef void UpdateKinematicsCustomPtr (Model& model,
            const double* q_ptr,
            const double* qdot_ptr,
            const double* qddot_ptr)

    cdef Vector3d CalcBodyToBaseCoordinates (Model& model,
            const VectorNd &q,
            const unsigned int body_id,
            const Vector3d &body_point_coordinates,
            bool update_kinematics)

    cdef Vector3d CalcBaseToBodyCoordinates (Model& model,
            const VectorNd &q,
            const unsigned int body_id,
            const Vector3d &body_point_coordinates,
            bool update_kinematics)
            
    cdef Matrix3d CalcBodyWorldOrientation (Model& model,
            const VectorNd &q,
            const unsigned int body_id,
            bool update_kinematics)

    cdef Vector3d CalcPointVelocity (Model& model,
            const VectorNd &q,
            const VectorNd &qdot,
            const unsigned int body_id,
            const Vector3d &body_point_coordinates,
            bool update_kinematics)

    cdef Vector3d CalcPointAcceleration (Model& model,
            const VectorNd &q,
            const VectorNd &qdot,
            const VectorNd &qddot,
            const unsigned int body_id,
            const Vector3d &body_point_coordinates,
            bool update_kinematics)

    cdef SpatialVector CalcPointVelocity6D (Model& model,
            const VectorNd &q,
            const VectorNd &qdot,
            const unsigned int body_id,
            const Vector3d &body_point_coordinates,
            bool update_kinematics)

    cdef SpatialVector CalcPointAcceleration6D (Model& model,
            const VectorNd &q,
            const VectorNd &qdot,
            const VectorNd &qddot,
            const unsigned int body_id,
            const Vector3d &body_point_coordinates,
            bool update_kinematics)


    
    ctypedef enum InverseKinematicsConstraintSet_ConstraintType "RigidBodyDynamics::InverseKinematicsConstraintSet::ConstraintType":
          InverseKinematicsConstraintSet_ConstraintTypePosition "RigidBodyDynamics::InverseKinematicsConstraintSet::ConstraintTypePosition"
          InverseKinematicsConstraintSet_ConstraintTypeOrientation "RigidBodyDynamics::InverseKinematicsConstraintSet::ConstraintTypeOrientation"
          InverseKinematicsConstraintSet_ConstraintTypeFull "RigidBodyDynamics::InverseKinematicsConstraintSet::ConstraintTypeFull"
          InverseKinematicsConstraintSet_ConstraintTypePositionXY "RigidBodyDynamics::InverseKinematicsConstraintSet::ConstraintTypePositionXY"
          InverseKinematicsConstraintSet_ConstraintTypePositionZ "RigidBodyDynamics::InverseKinematicsConstraintSet::ConstraintTypePositionZ"
          InverseKinematicsConstraintSet_ConstraintTypePositionCoMXY "RigidBodyDynamics::InverseKinematicsConstraintSet::ConstraintTypePositionCoMXY"
    
            
            
    cdef cppclass InverseKinematicsConstraintSet:
      
        InverseKinematicsConstraintSet()
      
        unsigned int AddPointConstraint (unsigned int body_id,
                const Vector3d &body_point,
                const Vector3d &target_pos,
                float weight
                )
        
        unsigned int AddPointConstraintXY (unsigned int body_id,
                const Vector3d &body_point,
                const Vector3d &target_pos,
                float weight
                )
        unsigned int AddPointConstraintZ (unsigned int body_id,
                const Vector3d &body_point,
                const Vector3d &target_pos,
                float weight
                )
        unsigned int AddPointConstraintCoMXY (unsigned int body_id,
                const Vector3d &target_pos,
                float weight
                )
                
        unsigned int AddOrientationConstraint (unsigned int body_id, 
                const Matrix3d &target_orientation,
                float weight
                )
                
        unsigned int AddFullConstraint(unsigned int body_id,
                const Vector3d &body_point,
                const Vector3d &target_pos,
                const Matrix3d &target_orientation,
                float weight
                )
                
        unsigned int ClearConstraints()
       

        MatrixNd J
        MatrixNd G
        VectorNd e
        
        unsigned int num_constraints; 
        double dlambda "RigidBodyDynamics::InverseKinematicsConstraintSet::lambda" 
        unsigned int num_steps;  
        unsigned int max_steps;  
        double step_tol; 
        double constraint_tol;  
        double error_norm;  
        
        vector[InverseKinematicsConstraintSet_ConstraintType] constraint_type;
        vector[unsigned int] body_ids;
        vector[Vector3d] body_points;
        vector[Vector3d] target_positions;
        vector[Matrix3d] target_orientations;
        vector[unsigned int] constraint_row_index;
        vector[float] constraint_weight;
  
        

cdef extern from "rbdl_ptr_functions.h" namespace "RigidBodyDynamics":
    cdef void CalcCenterOfMass (Model& model,
            const double* q_ptr,
            const double* qdot_ptr,
            const double* qddot_ptr,
            double &mass,
            Vector3d &com,
            Vector3d *com_velocity,
            Vector3d *com_acceleration,
            Vector3d *angular_momentum,
            Vector3d *change_of_angular_momentum,
            bool update_kinematics)

cdef extern from "<rbdl/Constraints.h>" namespace "RigidBodyDynamics":
  cdef cppclass ConstraintSet:
        ConstraintSet()

        unsigned int getGroupIndexByName(const char *constraintNameChar)        
        
        unsigned int getGroupIndexById(unsigned int userDefinedId)
        
        unsigned int getGroupIndexByAssignedId(unsigned int assignedId)
        
        unsigned int getGroupIndexMax()

        unsigned int getGroupSize(unsigned int groupIndex)
        
        unsigned int getGroupType(unsigned int groupIndex)

        const char* getGroupName(unsigned int groupIndex)
        
        unsigned int getGroupId(unsigned int groupIndex)
        
        unsigned int getGroupAssignedId(unsigned int groupIndex)

        void enableBaumgarteStabilization(unsigned int groupIndex)
        void disableBaumgarteStabilization(unsigned int groupIndex)

        void calcForces(unsigned int groupIndex,
                        Model &model,
                        const VectorNd &q,
                        const VectorNd &qdot,
                        vector[unsigned int] &constraintBodyIdsUpd,
                        vector[SpatialTransform] &constraintFramesUpd,
                        vector[SpatialVector] &constraintForcesUpd,
                        bool resolveAllInRootFrame,
                        bool updateKinematics)

        #The functions to calculate impacts have not been
        #wrapped so it makes no sense, at this time, to expose calcImpulses
        #void calcImpulses(unsigned int groupIndex,
        #                Model &model,
        #                const VectorNd &q,
        #                const VectorNd &qdot,
        #                vector[unsigned int] &constraintBodyIdsUpd,
        #                vector[SpatialTransform] &constraintFramesUpd,
        #                vector[SpatialVector] &constraintImpulsesUpd,
        #                bool resolveAllInRootFrame,
        #                bool updateKinematics)

        void calcPositionError(unsigned int groupIndex,
                        Model &model,
                        const VectorNd &q,
                        VectorNd &posErrUpd,
                        bool updateKinematics)

        void calcVelocityError(unsigned int groupIndex,
                        Model &model,
                        const VectorNd &q,
                        const VectorNd &qdot,
                        VectorNd &velErrUpd,
                        bool updateKinematics)

        void calcBaumgarteStabilizationForces(unsigned int groupIndex,
                        Model &model,
                        const VectorNd &posErr,
                        const VectorNd &velErr,
                        VectorNd &baumgarteForcesUpd)

        bool isBaumgarteStabilizationEnabled(unsigned int groupIndex)

        void getBaumgarteStabilizationCoefficients(
                unsigned int groupIndex,
                Vector2d &baumgartePositionVelocityCoefficientsUpd)

        unsigned int AddContactConstraint (
                unsigned int body_id,
                const Vector3d &body_point,
                const Vector3d &world_normal,
                const char* constraint_name,
                unsigned int user_defined_id)

        
        unsigned int AddLoopConstraint (
                unsigned int id_predecessor, 
                unsigned int id_successor,
                const SpatialTransform &X_predecessor,
                const SpatialTransform &X_successor,
                const SpatialVector &axis,
                bool baumgarte_enabled,
                double T_stab_inv,
                const char *constraint_name,
                unsigned int user_defined_id)
                

        ConstraintSet Copy()
        # void SetSolver (Math::LinearSolver solver)
        bool Bind (const Model &model)

        void SetActuationMap(const Model& model,
                const vector[bool]& actuatedDof);

        size_t size()
        void clear()
        # Math::LinearSolver
        bool bound



        vector[string] name

        VectorNd force
        VectorNd impulse
        VectorNd v_plus

        MatrixNd H
        VectorNd C
        VectorNd gamma
        VectorNd G

        MatrixNd A
        VectorNd b
        VectorNd x

        MatrixNd GT_qr_Q
        MatrixNd Y
        MatrixNd Z
        VectorNd qddot_y
        VectorNd qddot_z

        MatrixNd K
        VectorNd a
        VectorNd QDDot_t
        VectorNd QDDot_0

        vector[SpatialVector] f_t
        vector[SpatialVector] f_ext_constraints
        vector[Vector3d] point_accel_0

        vector[SpatialVector] d_pA
        vector[SpatialVector] d_a
        VectorNd d_u

        vector[SpatialMatrix] d_IA
        vector[SpatialVector] d_U

        VectorNd d_d
        vector[Vector3d] d_multdof3_u
    

cdef extern from "rbdl_ptr_functions.h" namespace "RigidBodyDynamics":
    cdef void CalcPointJacobianPtr (Model& model,
            const double *q_ptr,
            unsigned int body_id,
            const Vector3d &point_position,
            double *G,
            bool update_kinematics)

    cdef void CalcPointJacobian6DPtr (Model &model,
            const double *q_ptr,
            unsigned int body_id,
            const Vector3d &point_position,
            double *G,
            bool update_kinematics)

    cdef void CalcBodySpatialJacobianPtr (
            Model &model,
            const double *q_ptr,
            unsigned int body_id,
            double *G,
            bool update_kinematics)

    cdef void InverseDynamicsPtr (
            Model &model,
            const double* q_ptr,
            const double* qdot_ptr,
            const double* qddot_ptr,
            double* tau_ptr,
            vector[SpatialVector] *f_ext
            )

    cdef void InverseDynamicsConstraintsPtr(
            Model &model,
            const double* q_ptr,
            const double* qdot_ptr,
            const double* qddot_ptr,
            ConstraintSet &CS,
            double* qddot_out_ptr,
            double* tau_ptr,
            vector[SpatialVector] *f_ext
            )

    cdef void InverseDynamicsConstraintsRelaxedPtr(
            Model &model,
            const double* q_ptr,
            const double* qdot_ptr,
            const double* qddot_ptr,
            ConstraintSet &CS,
            double* qddot_out_ptr,
            double* tau_ptr,
            vector[SpatialVector] *f_ext
            )

    cdef bool isConstrainedSystemFullyActuated(
            Model &model,
            const double* q_ptr,
            const double* qdot_ptr,
            ConstraintSet &CS,
            vector[SpatialVector] *f_ext
            )

    cdef void NonlinearEffectsPtr (
            Model &model,
            const double* q_ptr,
            const double* qdot_ptr,
            double* tau_ptr
            )

    cdef void CompositeRigidBodyAlgorithmPtr (Model& model,
            const double *q,
            double *H,
            bool update_kinematics)

    cdef void ForwardDynamicsPtr (
            Model &model,
            const double* q_ptr,
            const double* qdot_ptr,
            double* tau_ptr,
            const double* qddot_ptr,
            vector[SpatialVector] *f_ext
            )
    
    cdef bool InverseKinematicsPtr (
            Model &model,
            const double *qinit_ptr,
            const vector[unsigned int] body_id,
            const vector[Vector3d] body_point,
            const vector[Vector3d] target_pos,
            const double *qres_ptr,
            double step_tol,
            double lambda_,
            unsigned int max_iter
            )
    
    cdef bool InverseKinematicsCSPtr (
           Model &model,
           const double *qinit_ptr,
           InverseKinematicsConstraintSet &CS,
           const double *qres_ptr)  
    
    cdef void ForwardDynamicsConstraintsDirectPtr (
           Model &model,
           const double* q_ptr,
           const double* qdot_ptr,
           const double* tau_ptr,
           ConstraintSet &CS,
           double* qddot_ptr,
           vector[SpatialVector] *f_ext
           )

cdef extern from "rbdl_loadmodel.cc":
    cdef bool rbdl_loadmodel (
           const char* filename,
           Model* model,
           bool floating_base,
           bool verbose)
     
        
        
  
