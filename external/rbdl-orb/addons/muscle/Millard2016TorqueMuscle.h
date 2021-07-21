#ifndef MILLARD2016TORQUEMUSCLE_H_
#define MILLARD2016TORQUEMUSCLE_H_

/* 
 * RBDL - Rigid Body Dynamics Library: Addon : muscle
 * Copyright (c) 2016 Matthew Millard <millard.matthew@gmail.com>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#include <vector>
#include <rbdl/rbdl_math.h>
#include "../geometry/SmoothSegmentedFunction.h"



namespace RigidBodyDynamics {
    namespace Addons {
      namespace Muscle{
        #ifdef RBDL_BUILD_ADDON_MUSCLE_FITTING
          class TorqueMuscleFittingToolkit;
          class FitTorqueMuscleParameters;
        #endif

        /**
          This struct contains an enumerated list of the data sets which
          constain torque muscles. Thus far this list includes

          -Anderson2007: Table 3 from Anderson et al.       
          -Gymnast: A set of torque muscles for the whole body (in progress)
                    
          For details of these data sets please refer to the main description 
          of this class.

        */
        const static struct DataSet{
          enum item{
              Anderson2007 = 0,
              Gymnast,
              Last
          };
          const static char* names[];
          DataSet(){}
        } DataSet;


        /**
          This struct contains an enumerated list of the genders for which 
          data torque data has been reported.
        */
        const static struct GenderSet{
          enum item {
              Male = 0,
              Female,
              Last
          };
          const static char* names[];
          GenderSet(){}
        } GenderSet;

        /**
          This struct contains an enumerated list of the age groups for which 
          data torque data has been reported.
        */
        const static struct AgeGroupSet{
          enum item {
              Young18To25 = 0,
              Middle55To65,
              SeniorOver65,
              Last
          };
          const static char* names[];
          AgeGroupSet(){}
        } AgeGroupSet;

        /**
          This struct contains an enumerated list of the joint-torque-directions 
          for which data torque data has been reported.
        */
        const static struct JointTorqueSet{
          enum item{
            HipExtension                  = 0,
            HipFlexion                    = 1,
            KneeExtension                 = 2,
            KneeFlexion                   = 3,
            AnkleExtension                = 4,
            AnkleFlexion                  = 5,
            ElbowExtension                = 6,
            ElbowFlexion                  = 7,
            ShoulderExtension             = 8,
            ShoulderFlexion               = 9,
            WristExtension                = 10,
            WristFlexion                  = 11,
            ShoulderHorizontalAdduction   = 12,
            ShoulderHorizontalAbduction   = 13,
            ShoulderInternalRotation      = 14,
            ShoulderExternalRotation      = 15,
            WristUlnarDeviation           = 16,
            WristRadialDeviation          = 17,
            WristPronation                = 18,
            WristSupination               = 19,
            LumbarExtension               = 20,
            LumbarFlexion                 = 21,
            UnitExtensor                  = 22,
            UnitFlexor                    = 23,
            Last                          = 24
          };
          const static char* names[];
          JointTorqueSet(){}
        } JointTorqueSet;


        /**
          This struct contains 3 enumerated lists (Gender, AgeGroup, 
          JointTorque) that lists the genders, age groups, and 
          joint-torque-directions for which the Anderson2007 data set contains
          data. Please refer to the class description for more details about 
          this data set and how to use it.
        */
        const static struct Anderson2007{
          enum Gender {
              Male   = GenderSet::Male,
              Female = GenderSet::Female,
              LastGender
          };
          enum AgeGroup {
              Young18To25   = AgeGroupSet::Young18To25,
              Middle55To65  = AgeGroupSet::Middle55To65,
              SeniorOver65  = AgeGroupSet::SeniorOver65,
              LastAgeGroup
          };
          enum JointTorque{
            HipExtension                  = JointTorqueSet::HipExtension  ,
            HipFlexion                    = JointTorqueSet::HipFlexion    ,
            KneeExtension                 = JointTorqueSet::KneeExtension ,
            KneeFlexion                   = JointTorqueSet::KneeFlexion   ,
            AnkleExtension                = JointTorqueSet::AnkleExtension,
            AnkleFlexion                  = JointTorqueSet::AnkleFlexion,
            LastJointTorque
          };
          const static char* GenderNames[];
          const static char* AgeGroupNames[];
          const static char* JointTorqueNames[];
          Anderson2007(){}
        } Anderson2007;

        /**
          This struct contains 3 enumerated lists (Gender, AgeGroup, 
          JointTorque) that lists the genders, age groups, and 
          joint-torque-directions for which the Gymnast data set contains
          data. At the present time Gymnast data set only contains data 
          appropriate for a young (18-25) elite male gymnast. Please refer
          to the main class description for details on this data set and how 
          to use it.
        */
        const static struct Gymnast{
          enum Gender {
              Male   = GenderSet::Male,
              LastGender
          };
          enum AgeGroup {
              Young18To25   = AgeGroupSet::Young18To25,
              LastAgeGroup
          };

          enum TableIndex {
            TauMax = 0,
            OmegaMax,
            ActiveAngleAtOneNormTorque,
            ActiveAngularStandardDeviation,
            TvAtMaxEccentricVelocity,
            TvAtHalfMaxConcentricVelocity,
            PassiveAngleAtZeroTorque,
            PassiveAngleAtOneNormTorque,
            LastTableIndex
          };

          enum JointTorque {
            HipExtension                = JointTorqueSet::HipExtension       ,
            HipFlexion                  = JointTorqueSet::HipFlexion         ,
            KneeExtension               = JointTorqueSet::KneeExtension      ,
            KneeFlexion                 = JointTorqueSet::KneeFlexion        ,
            AnkleExtension              = JointTorqueSet::AnkleExtension     ,
            AnkleFlexion                = JointTorqueSet::AnkleFlexion       ,
            ElbowExtension              = JointTorqueSet::ElbowExtension     ,
            ElbowFlexion                = JointTorqueSet::ElbowFlexion       ,
            ShoulderExtension           = JointTorqueSet::ShoulderExtension  ,
            ShoulderFlexion             = JointTorqueSet::ShoulderFlexion    ,
            WristExtension              = JointTorqueSet::WristExtension     ,
            WristFlexion                = JointTorqueSet::WristFlexion       ,
            ShoulderHorizontalAdduction = 
              JointTorqueSet::ShoulderHorizontalAdduction,
            ShoulderHorizontalAbduction = 
              JointTorqueSet::ShoulderHorizontalAbduction,
            ShoulderInternalRotation    = 
              JointTorqueSet::ShoulderInternalRotation   ,
            ShoulderExternalRotation    = 
              JointTorqueSet::ShoulderExternalRotation   ,
            WristUlnarDeviation         = JointTorqueSet::WristUlnarDeviation ,
            WristRadialDeviation        = JointTorqueSet::WristRadialDeviation,
            WristPronation              = JointTorqueSet::WristPronation      ,
            WristSupination             = JointTorqueSet::WristSupination     ,
            LumbarExtension             = JointTorqueSet::LumbarExtension,
            LumbarFlexion               = JointTorqueSet::LumbarFlexion,
            UnitExtensor                = JointTorqueSet::UnitExtensor,
            UnitFlexor                  = JointTorqueSet::UnitFlexor,
            LastJointTorque
          };
          const static char* GenderNames[];
          const static char* AgeGroupNames[];
          const static char* JointTorqueNames[];
          Gymnast(){}
        } Gymnast;



        /**
          This is a struct that contains subject-specific information that
          does not change for a given subject.

          @param gender
                 Male/Female. Selecting a gender that
                 is not present in a data set will result in
                 the program aborting and an error message
                 being printed to the terminal.

          @param ageGroup:
                 Presently the age group options include Young (18-25),
                 middle aged (55-65), and senior (>65). Selecting an
                 age group that is not present in a data set will result
                 in the program aborting and an error message
                 being printed to the terminal.

          @param mSubjectHeightInMeters
                  This parameter is used to scale from the normalized
                  curves reported by Anderson et al.
                  See the class description for details.

          @param mSubjectMassInKg
                  This parameter is used to scale from the normalized
                  curves reported by Anderson et al.
                  See the class description for details.
        */
        struct SubjectInformation{
          GenderSet::item gender;
          AgeGroupSet::item ageGroup;
          double heightInMeters;
          double massInKg;          
        };



        struct TorqueMuscleSummary{
          /**The angle of the fiber (rad)*/
          double fiberAngle;

          /**The angular velocity of the fiber (rad/s)*/
          double fiberAngularVelocity;


          /**The activation of the muscle*/
          double activation;

          /** The normalized value of the passive-torque-angle curve.
           Here a value of 1 means 1 maximum-isometric-torque. (Nm/Nm)*/
          double fiberPassiveTorqueAngleMultiplier;

          /**The normalized value of the active-torque-angle curve.
           Here a value of 1 means 1 maximum-isometric-torque. (Nm/Nm)*/
          double fiberActiveTorqueAngleMultiplier;

          /**The normalized value of the torque-angular-velocity curve.
             Here a value of 1 means 1 maximum-isometric-torque. (Nm/Nm)*/
          double fiberTorqueAngularVelocityMultiplier;

          /**The torque generated by the damping element (Nm/Nm)*/
          double fiberNormalizedDampingTorque;

          /**The torque generated by the entire fiber (Nm)*/
          double fiberTorque;

          /**The joint torque developed by the muscle. This is signed so
          that it is consistent with the sign convention of the joint
          chosen by the user. (Nm)*/
          double jointTorque;


          TorqueMuscleSummary():
            fiberAngle(nan("1")),
            fiberAngularVelocity(nan("1")),
            activation(nan("1")),
            fiberPassiveTorqueAngleMultiplier(nan("1")),
            fiberActiveTorqueAngleMultiplier(nan("1")),
            fiberTorqueAngularVelocityMultiplier(nan("1")),
            fiberNormalizedDampingTorque(nan("1")),
            fiberTorque(nan("1")),
            jointTorque(nan("1")){}

        };

        struct TorqueMuscleInfo{

            /**The angle of the joint (radians)*/
            double jointAngle;

            /**The angular velocity of the joint, where the sign convention
               is chosen by the user at the time the torque muscle is created 
               (radians/sec)*/
            double jointAngularVelocity;

            /**The angle that the muscle fiber spans (radians)*/   
            double fiberAngle;

            //The angle that the tendon spans (radians)              
            //double tendonAngle;

            /**The rate-of-angular-lengthening of the fiber.
               A positive sign is for a concentric contraction, 
               that is where the fibers are shortening. (radians/sec)*/   
            double fiberAngularVelocity;

            /*The rate-of-angular-lengthening of the tendon.
               A positive sign is for a concentric contraction, 
               that is where the tendon is shortening. (radians/sec)*/   
            //double tendonAngularVelocity;

            /** The normalized value of the passive-torque-angle curve. 
             Here a value of 1 means 1 maximum-isometric-torque. (Nm/Nm)*/
            double fiberPassiveTorqueAngleMultiplier;
            double DfiberPassiveTorqueAngleMultiplier_DblendingVariable;
            double DfiberPassiveTorqueAngleMultiplier_DangleOffset;
            /**The normalized value of the active-torque-angle curve. 
             Here a value of 1 means 1 maximum-isometric-torque. (Nm/Nm)*/
            double fiberActiveTorqueAngleMultiplier;
            double DfiberActiveTorqueAngleMultiplier_DblendingVariable;
            /**The normalized value of the torque-angular-velocity curve. 
               Here a value of 1 means 1 maximum-isometric-torque. (Nm/Nm)*/
            double fiberTorqueAngularVelocityMultiplier;
            double DfiberTorqueAngularVelocityMultiplier_DblendingVariable;
            /*The normalized value of the tendon-torque-angle curve. 
               Here a value of 1 means 1 maximum-isometric-torque. (Nm/Nm)*/
            //double tendonTorqueAngleMultiplier;
            
            /**The activation of the muscle*/
            double activation;

            /**The torque generated by the active element of the muscle
            fiber (Nm)*/
            double fiberActiveTorque;

            /**The total torque generated by the passive elements of the muscle
            fiber (Nm)*/
            double fiberPassiveTorque;            

            /**The torque generated by the passive elastic element of the
             muscle fiber (Nm)
            */
            double fiberPassiveElasticTorque;

            /**The torque generated by the damping element (Nm)*/
            double fiberDampingTorque;

            /**The torque generated by the damping element (Nm)*/
            double fiberNormDampingTorque;

            /**The torque generated by the entire fiber (Nm)*/            
            double fiberTorque;

            /*The torque transmitted through the tendon across the joint (Nm)*/
            //double tendonTorque;

            /**The joint torque developed by the muscle. This is signed so 
            that it is consistent with the sign convention of the joint
            chosen by the user. (Nm)*/
            double jointTorque;

            /**The stiffness of the fiber (Nm/rad)*/
            double fiberStiffness;

            /*The stiffness of the tendon (Nm/rad)*/
            //double tendonStiffness;

            /** The stiffness of the joint. This is signed so 
            that it is consistent with the sign convention of the joint
            chosen by the user. (Nm/rad)*/
            double jointStiffness;
            
            /**The power output of the active fiber element. A positive
            power means that the fiber is contracting concentrically. 
            (Watts - Nm/s)*/
            double fiberActivePower;

            /**The power output of the passive fiber element. A positive
            power means that the passive element is recoiling concentrically
            (Watts - Nm/s)*/            
            double fiberPassivePower;

            /**The total power output of the fiber element.(Watts - Nm/s)*/  
            double fiberPower;

            /** The power output by this muscle at the joint. This is
            signed so that it is consistent with the sign convention of the
            joint chosen by the user. (Nm/rad)*/
            double jointPower;

            /**The partial derivative of joint torque w.r.t activation*/
            double DjointTorque_Dactivation;
            
            /**The partial derivative of joint torque w.r.t the joint angle*/
            double DjointTorque_DjointAngle;

            /**The partial derivative of joint torque w.r.t the joint 
            angular velocity*/            
            double DjointTorque_DjointAngularVelocity;

            ///First derivatives for the fitting algorithm
            double DjointTorque_DactiveTorqueAngleBlendingVariable;
            double DjointTorque_DpassiveTorqueAngleBlendingVariable;
            double DjointTorque_DtorqueAngularVelocityBlendingVariable;
            double DjointTorque_DmaximumIsometricTorque;
            double DjointTorque_DpassiveTorqueAngleCurveAngleOffset;

            double DjointTorque_DactiveTorqueAngleAngleScaling;
            double DjointTorque_DmaximumAngularVelocity;


            ///This vector contains additional quantities that are needed by
            /// the fitting functions, and is not for external use. Use this
            /// vector at your own risk, as its definition/sizing etc may
            /// change.
            RigidBodyDynamics::Math::VectorNd fittingInfo;

            TorqueMuscleInfo():
              jointAngle(nan("1")),
              jointAngularVelocity(nan("1")),
              fiberAngle(nan("1")),
              fiberAngularVelocity(nan("1")),
              fiberPassiveTorqueAngleMultiplier(nan("1")),
              DfiberPassiveTorqueAngleMultiplier_DblendingVariable(nan("1")),
              DfiberPassiveTorqueAngleMultiplier_DangleOffset(nan("1")),
              fiberActiveTorqueAngleMultiplier(nan("1")),
              DfiberActiveTorqueAngleMultiplier_DblendingVariable(nan("1")),
              fiberTorqueAngularVelocityMultiplier(nan("1")),
              DfiberTorqueAngularVelocityMultiplier_DblendingVariable(nan("1")),
              activation(nan("1")),
              fiberActiveTorque(nan("1")),
              fiberPassiveTorque(nan("1")),
              fiberPassiveElasticTorque(nan("1")),
              fiberDampingTorque(nan("1")),
              fiberNormDampingTorque(nan("1")),
              fiberTorque(nan("1")),
              jointTorque(nan("1")),
              fiberStiffness(nan("1")),
              jointStiffness(nan("1")),
              fiberActivePower(nan("1")),
              fiberPassivePower(nan("1")),
              fiberPower(nan("1")),
              jointPower(nan("1")),
              DjointTorque_Dactivation(nan("1")),
              DjointTorque_DjointAngle(nan("1")),
              DjointTorque_DjointAngularVelocity(nan("1")),
              DjointTorque_DactiveTorqueAngleBlendingVariable(nan("1")),
              DjointTorque_DpassiveTorqueAngleBlendingVariable(nan("1")),
              DjointTorque_DtorqueAngularVelocityBlendingVariable(nan("1")),
              DjointTorque_DmaximumIsometricTorque(nan("1")),
              DjointTorque_DpassiveTorqueAngleCurveAngleOffset(nan("1")),
              DjointTorque_DactiveTorqueAngleAngleScaling(nan("1")),
              DjointTorque_DmaximumAngularVelocity(nan("1"))
            {}

        };


        struct TorqueMuscleParameterFittingData {

           unsigned int indexAtMaximumActivation;
           unsigned int indexAtMinimumActivation;
           unsigned int indexAtMaxPassiveTorqueAngleMultiplier;

           bool isTorqueMuscleActive;

           double activeTorqueAngleBlendingVariable;
           double passiveTorqueAngleBlendingVariable;
           double torqueVelocityBlendingVariable;
           double passiveTorqueAngleCurveOffset;
           double maximumActiveIsometricTorque;

           double activeTorqueAngleAngleScaling;
           double maximumAngularVelocity;

           //double objectiveValue;
           //double constraintError;
           bool fittingConverged;

           TorqueMuscleSummary summaryDataAtMinimumActivation;
           TorqueMuscleSummary summaryDataAtMaximumActivation;
           TorqueMuscleSummary summaryDataAtMaximumPassiveTorqueAngleMultiplier;


           TorqueMuscleParameterFittingData():
             indexAtMaximumActivation(std::numeric_limits<unsigned int>::infinity()),
             indexAtMinimumActivation(std::numeric_limits<unsigned int>::infinity()),
             indexAtMaxPassiveTorqueAngleMultiplier(std::numeric_limits<unsigned int>::infinity()),
             isTorqueMuscleActive(true),
             activeTorqueAngleBlendingVariable(nan("1")),
             passiveTorqueAngleBlendingVariable(nan("1")),
             torqueVelocityBlendingVariable(nan("1")),
             passiveTorqueAngleCurveOffset(nan("1")),
             maximumActiveIsometricTorque(nan("1")),
             activeTorqueAngleAngleScaling(nan("1")),
             maximumAngularVelocity(nan("1")),
             fittingConverged(false),
             summaryDataAtMinimumActivation(),
             summaryDataAtMaximumActivation(),
             summaryDataAtMaximumPassiveTorqueAngleMultiplier(){}
        };

        struct TorqueMuscleDataFeatures{
          unsigned int indexOfMaxActivation;
          unsigned int indexOfMinActivation;
          unsigned int indexOfMaxPassiveTorqueAngleMultiplier;

          bool isInactive;

          TorqueMuscleSummary summaryAtMaxActivation;
          TorqueMuscleSummary summaryAtMinActivation;
          TorqueMuscleSummary summaryAtMaxPassiveTorqueAngleMultiplier;
        };

        /**
            \brief This class implements a rigid-tendon muscle-torque-generator 
            (MTG) for a growing list of joints and torque-directions. For a 
            detailed description of the MTGs available and the automatic 
            fitting routine (implemented in TorqueMuscleFittingToolkit) please
            see the publication: M.Millard, A.L.Kleesattel, M.Harant, & 
            K.Mombaur. A reduced muscle model and planar musculoskeletal model 
            fit for the synthesis of whole body movements. Journal of 
            Biomechanics. (under review as of August 2018)

            This rigid-tendon torque muscle model provides modeling support 
            for 3 phenomena

            - torque-angle curve (\f$\mathbf{t}_A(\theta)\f$): the variation of active isometric torque in one direction as a function of joint angle
            - torque-velocity curve (\f$\mathbf{t}_V(\dot{\theta})\f$): the variation of torque as a function of angular velocity
            - passive-torque-angle curve (\f$\mathbf{t}_P(\theta-\theta_S)\f$): the variation of passive torque as a function of joint angle. Here \f$s_P\f$ and \f$\theta_S\f$ are user-defined scaling and shift parameters. 
            
            each of which are represented as smooth normalized curves that 
            vary between 0 and 1. A series of scaling 
            (\f$s^A, s^V, s^\tau\f$), shifting (\f$\Delta^P\f$), and blending 
            (\f$\lambda^A,\lambda^V,\lambda^P\f$) variables have been introduced
            to these curves to make it possible to easily fit these curves to
            a specific subject. These fitting variables can be set/fitted by
            making use of the functions in this class. Alternatively these 
            fitting variables can be automatically adjusted (using IPOPT) by 
            making use of the TorqueMuscleFittingToolkit.

            \image html fig_MuscleAddon_BlendableTorqueMuscle.png "Examples of the adjustable characteristic curves"

            These three curves are used to compute the 
            torque developed \f$\tau\f$ given the angle of the joint 
            \f$\theta\f$, the angular-velocity of the joint \f$\dot{\theta}\f$, 
            and the activation of the muscle \f$\mathbf{a}\f$ (a 0-1 quantity 
            that defines how much the muscle is turned-on, or activated), and 
            the maximum-isometric torque \f$\tau_{ISO}\f$
            \f[
              \tau (\mathbf{a}, \theta,\dot{\theta}) =
                    \tau_{ISO} ( \mathbf{a} \, \mathbf{t}_A(\theta) \mathbf{t}_V(\dot{\theta}/\dot{\theta}_{MAX})
                                 +\mathbf{t}_P(1- \beta (\dot{\theta}/\dot{\theta}_{MAX})) \, )
            \f]                      
            The damping term \f$\beta\f$ is necessary to supress vibration
            that will occur as the passive element \f$\mathbf{t}_P\f$ is streched,
            its stiffness increases, and the natural frequency of the overall
            system rises. By default \f$\beta\f$ is set to 0.1 which has proven 
            effective for supressing vibration in the trunk segments during a 
            stoop lift in which the stiffness of the lumbar back muscles grows 
            appreciably. This model does not yet provide support for the 
            following phenomena but will in the future.

            - activation dynamics: currently is left to the user 
            - tendon-elasticity
            - muscle short-range-stiffness

            All of these characteristic curves are represented using \f$C_2\f$ 
            continuous \f$5^{th}\f$ order Bezier curves that have been fitted to 
            the data from data in the literature. In many cases these 
            curves have been carefully edited so that they fit the curves of 
            the original papers, but have more desireable numerical properties
            for optimal control work. The characterisic curves provided by this 
            class have been fitted to a growing list of data sets:
            -Anderson Data Set: from Anderson et al. 2007
            -Whole-body Gymnast Data Set: from Jackson, Kentel et al., Anderson et al., Dolan et al. and Raschke et al.

             <b>Data Set: Anderson2007</b>

            This data set uses the mean value of the coefficients published
            in Anderson et al. The standard deviation table has also been
            entered. However, since it is unclear how to use the standard
            deviation in a consistent way across all joints/parameters this
            table is not yet accessible through the constructor. This data
            set includes coefficients for the following

            - Number of subjects: 34
            - Gender: male and female
            - Age: young (18-25, 14 subjects), middle-aged (55-65, 14 subjects), senior (> 65, 6 subjects)
            - Joint: hip/knee/ankle
            - Direction: extension/flexion

            <b>Notes</b>
             -# Angles are plotted using units of degrees for readability. The
              actual curves are described in units of radians
             -# See Anderson et al. for further details.

           \image html fig_MuscleAddon_Anderson2007AllPositiveSigns.png "Characteristic from Anderson et al. 2007 [1]"

            <b>Data Set: Gymnast</b>

            This data set is an attempt at making enough torque muscles for a
            whole body. Since no single source in the literature comes close to
            measuring the characteristics of all of the joints, data from
            Jackson et al., Kentel et al., Anderson et al., Dolan et al, and 
            Raschke et al. 
            have been combined. Since the subjects used in these various studies
            are wildly different (Jackson et al. measured an elite male gymnast;
            Kentel et al. measured an elite tennis player; Anderson et al. measured,
            in the category of young male, a selection of active undergraduate
            students, Dolan et al from 126 women and 23 men, and Raschke from 5
            male subjects) scaling has been used to make the strength of the subject
            consistent. Scaling coefficients for the lower body, shoulders and
            elbow, and forearm/wrist have been calculated using measurements 
            that overlapped between datasets. Presently this data set includes 
            curves for 22 torque muscles.

            - Number of subjects: 1 elite gymnast (69.6 kg, 1.732 m)
            - Gender: male
            - Age: 21 years old
            - Joint and Directions available
              -# Ankle: flexion/extension (scaled from Anderson)
              -# Knee: flexion/extension (from Jackson)
              -# Hip: flexion/extension (from Jackson)
              -# Lumbar: active extension curves (\f$\mathbf{t}_A\f$ and \f$\mathbf{t}_P\f$) from Raschke et al. passive extension from Dolan et al. The torque velocity curve has since been updated using an estimate from the archiecture of the lumbar extensors
              -# Lumbar: active flexion \f$\tau_{ISO}\f$ from Beimborn et al.  
              -# Shoulder: flexion/extension (from Jackson)
              -# Shoulder: horizontal adduction/abduction (from Kentel, scaled to Jackson's subject)
              -# Shoulder: internal rotation/external rotation (from Kentel, scaled to Jackson's subject)
              -# Elbow: flexion/extension (from Kentel, scaled to Jackson's subject)
              -# Wrist: pronation/supination (from Kentel, scaled to Jackson's subject)
              -# Wrist: extension/flextion (from Jackson)
              -# Wrist: ulnar/radial deviation (from Kentel, scaled to Jackson's subject)
            - Missing Joint and directions
                -# Ankle inversion/eversion
                -# Hip adduction/abduction
                -# Hip internal rotation/external rotation
                -# Lumbar extension/flexion
                -# Lumbar bending
                -# Lumbar twisting
                -# Shoulder Adduction
                -# Shoulder Abduction
                -# Scapular elevation/depression
                -# Scapular adduction/abduction
                -# Scapular upward/downward rotation

            In all cases the curves have been fitted to Bezier curves that 
            are constructed using functions in TorqueMuscleFunctionFactory.

            <b>Notes</b>
            -# Angles are plotted using units of degrees for readability. The actual curves are described in units of radians
            -# Hip and Knee characteristics taken from Jackson. Ankle extension is from Anderson et al., scaled using Jackson-to-Anderson hip/knee strength ratios from Jackson ratios
            -# Shoulder horizontal adduction/abduction and internal/external rotation is a scaled version of the Kentel. Strength was scaled using the Jackson-to-Kentel shoulder flex/ext ratios.
            -# Elbow extension/flexion and forearm pronation/supination. Elbow strength scaled from Kentel using the ratio of maximum isometric shoulder ext/flextion between Kentel and Jackson. Forearm pronation/supination scaled using the maximum torque strength ratio of wrist extension/flextion between Kentel and Jackson
            -# Wrist ext/flexion directly from Jackson, while the curves for ulnar and radial deviation have been scaled (using the maximum isometric torque ratios of wrist extension and flexion from both models) from Kentel et al.
            -# Lumbar-extension active-torque-angle-curve in extension comes from Raschke et al.
            -# Lumbar-extension passive-torque-angle-curve comes from Dolan et al.
            -# Lumbar-flexion \f$\tau_{ISO}\f$ comes from Beimborn et al.'s observation that the strength ratio of back extensors to flextors at a flextion angle of zero is most often repored as 1.3:1.
            -# Lumbar-extension and flexion torque-velocity-curves have been estumated using the architecture of these two muscles. This update has been made because the force-velocity curve proposed by Raschke and Chaffin in Fig. 4 has such a low maximum angular velocity (60 dec/sec) that none of our optimal control simulations predicted lumbar flexion during movement. Using muscle archectural information for the erector spinae (ES) and rectus abdominus (RA), assuming both are made of slow twitch fibers the maximum angular velocites are 433 deg/sec and 1102 deg/sec about the constant assumed moment arms for the lumbar extensors and flexors respectively. The following architectural information was used:
              -# 7.1 cm  :ES moment arm from Németh & Ohlsén
              -# 8.08 cm :ES optimal fiber length taken from a weighted PSCA average of the ES muscles from Christophy et al. Table 1
              -# 7.02 \f$\ell/s\f$ :ES maximum angular velocity for slow twitch muscle from Ranatunga
              -# 0.151 :ES tv at half max. angular velocity
              -# 10.9 cm :RA moment arm from Németh & Ohlsén
              -# 29.9 cm :RA optimal fiber length from Christophy et al. Table 1
              -# 7.02 \f$\ell/s\f$ :RA maximum angular velocity for slow twitch muscle from Ranatunga
              -# 0.151 :RA tv at half max. angular velocity for slow twitch muscle from Ranatunga
            -# Any passive curve that is not accompanied by a curve from the literature (see the plots for details) is an educated guess.
            

           \image html fig_MuscleAddon_Gymnast_HipKneeAnkle.png " Hip/Knee/Ankle: from Jackson and Anderson et al. "
           \image html fig_MuscleAddon_Gymnast_Lumbar.png " Lumbar Extension/Flexion: from Dolan et al.,Raschke et al., and Beimborn et al."
           \image html fig_MuscleAddon_Gymnast_Shoulder3Dof.png " Shoulder 3 DoF torques: from Jackson and Kentel et al. "
           \image html fig_MuscleAddon_Gymnast_ElbowForearm.png " Elbow flexion/extension: from Kentel et al."
           \image html fig_MuscleAddon_Gymnast_Wrist3Dof.png " Wrist 3 DoF torques: from Jackson and Kentel et al."

            <b>Parameterized Curves used here vs. Literature</b>

            The curves used in this implementation are 2nd order 2-dimensional 
            Bezier curves. The curves described in Anderson et al., Jackson, 
            Kentel were not directly used because they are not continuous to the 
            second derivative (a requirement for most gradient based optimization 
            routines).  There are some other detailed differences that might be 
            of interest:

            -# Anderson et al.'s torque-velocity curve tends to large
            negative values for fast eccentric contractions. This is 
            in contrast to the literature which says that at large
            eccentric contractions the torque-velocity curve (or the 
            force-velocity-curve) tends to a value between 1.0 and 1.4.
            -# Anderson et al.'s torque-velcity curve for ankle extension
            did not cross the x-axis on the concentric side of the curve.
            This would endow the plantar flexors with super-human abilities.
            This error has been corrected by fitting a Bezier curve to a 
            Hill-type curve that passes through the point where 
            \f$\dot{\theta}= \frac{1}{2} \dot{\theta}_{MAX}\f$
            -# Anderson et al.'s, Jackson, and Kentel et al. had discontinuities 
             in the first derivative of the force velocity curve at 
             \f$\dot{\theta}=0\f$. While this follows Huxley's famous observations
             that the slope does discontinuously change at at \f$\dot{\theta}=0\f$,
             this is not a phenomena that is not compatible with most optimal
             control formulations and thus this discontinuity is not present in
             the force velocity curves used in this model.
            -# Anderson et al. and Kentel et al.'s active-torque-angle curves 
            can achieve negative values - this is obviously undesirable as it
            will allow a muscle to push.
            -# Kentel et al.'s activation inhibition function does not always
            cross 1.0 for \f$\dot{\theta}=0\f$, which means that \f$\tau_{ISO}\f$
            is not reached. This makes for a confusing model to use.


             <b>Coordinate Mapping</b>

            Every author chose a particular convention for measuring
            the angles of the hip, knee, ankle joint, shoulder, elbow, wrist and 
            lumbar --- see the figure for details. These conventions have all
            been mapped to the one used in the illustrations. You will need to 
            use the figure, your model, and the constructors appropriately
            so that

            -# the joint angle of your model is correctly mapped to the 
               fiber angle of the Millard2016TorqueMuscle;
            -# the sign of the muscle's output torque matches the 
               sign associated with your model.

            To map from your model's joint coordinates to the joint coordines
            used in this model (see the figure in the description)
            the followinq equation is used at the torque level

            \f$ jointTorque = signOfJointTorque*fiberTorque \f$

            where fiberTorque is the torque produced by Anderson et al.'s curves, 
            which is always positive. At the position level, the angles from your
            models joint angle to Anderson et al.'s joint angle (called fiberAngle)
            are mapped using

            \f$ fiberAngle = signOfJointAngleRelativeToAnderson2007*(jointAngle-jointAngleOffsetRelativeToAnderson2007). \f$
            
            Internally the sign of the fiber velocity follows signOfJointTorque so
            that the signs of joint power and muscle power are consistent.

            <b>Strength Scaling: Anderson2007 Data Set</b>

            The leg strength (here we mean \f$\tau_{ISO}\f$) 
            predicted by Anderson et al.'s curves should be taken
            as a good first approximation. While Anderson et al.'s data set is 
            the most comprehensive in the literature, they only measured torques 
            from active people: they did not include people at the extremes 
            (both very weak, and very strong), nor did they include children. 
            Finally, the torques produced by each subject were normalized by 
            mSubjectMassInKg*subjectHeightInM*accelerationDueToGravity. Strength
            is a strange phenomena which is not nicely normalized by just these
            quantites, and so the strength predicted by Anderson et al.'s curves
            might not fit your subject even if they are represented in Anderson 
            et al.'s data set.


            <b>Strength Scaling: Gymnast Data Set</b>

            The strength used in the Gymnast data set is fitted to an elite
            male gymnast. It goes without saying that an elite gymnast has
            strength proportions, and an absolute strength that are not typical.
            In the future it would be nice to have a function that could provide
            an educated guess about how to map Gymnast's strengths to that of
            another subject. For the moment I have no idea how to do this, nor
            am I aware of any works in the literature that can provide insight
            of how to do this. For now the whole-body Gymnast model should be 
            viewed as being a representation of what is possible for a human,
            but not a typical human. At the present time the default strength
            settings of the Gymnast are not scaled by subject height, nor
            weight.

            If you happen to know the maximum-isometric-active-torque (note this
            does not include the passive component) that your subject can 
            produce, you can update the strength of the torque-muscle using the 
            functions getMaximumActiveIsometricTorque(), and 
            setMaximumActiveIsometricTorque().

            <b>Fitting to a Specific Subject</b>

            If you have recorded the motions and external forces acting on a
            subject during a movement of interest, you can make use of the 
            TorqueMuscleFittingToolkit to adjust the MTG so that it is strong
            enough and flexible enough to allow a model of the subject to 
            reproduce the experimental data. Please see the 
            TorqueMuscleFittingToolkit class for details.


            <b>Limitations</b>

            This rigid-tendon torque muscle has some limitations that you should
            be aware of:

            -# There are no elastic tendons. That means that the mapping between
            the mechanical work done by this torque actuator will greatly differ
            from the positive mechanical work done by a torque actuator that
            includes an elastic tendon. This difference is greatest for those
            muscles with long tendons - namely the Achilles tendon in humans. 
            If you are
            interested in fiber kinematics, fiber work, or metabolic energy 
            consumption you cannot use this model especially for muscles that
            have long tendons.
            -# This model formulation predicts torque well, but does a poor job
            of predicting joint stiffness. In this model stiffness is given
            by the partial derivative of torque w.r.t. joint angle. Since the
            active-torque-angle curve fits a bell-shaped curve, it is possible
            to construct a torque muscle that has a region of physically
            impossible negative stiffness. Real muscle, in constrast, always
            has a positive stiffness even on the descending limb of the
            active-torque-angle curve (see Rassier et al. for details).
            -# Muscles that cross 2 joints (e.g. the hamstrings) produce coupled
            torques at both of those joints. In this model there is no coupling
            between joints. Furthermore, because of the lack of coupling the 
            curves used here are only valid for the posture that Anderson et al.,
            Jackson, and Kentel et al. 
            used when they made their data collection. If you are interested 
            in simulating postures that are very different from those described
            in by these authors then the results produced by this model should
            be treated as very rough. 
            -# Because this is a joint-torque muscle, none of the joint contact
            forces predicted will come close to matching what is produced by
            line-type muscles. If you are interested in joint-contact forces you
            cannot use this model.

            This simple model is a fast approximate means to constrain the joint
            torque developed in the body to something that is physiologically
            possible. That is it.

            <b>Units</b>
            Although the figure in this description has angles in units
            of degrees, this is only to help intuitition: when using
            the model, use radians. This model uses MKS:

                -Distance: m                
                -Angles: radians
                -Angular velocity: radians/s
                -Mass: kg
                -Torque: Nm
                -Time: second
                -Power: Nm/second

            <b> Developer Notes </b>

              All of the numerical evaluations of this model take place
              in a few private stateless functions:
                      -updateTorqueMuscleSummary
                      -updTorqueMuscleSummaryCurveValues
                      -updTorqueMuscleInfo
                      -updInvertTorqueMuscleSummary
              If you want to change something fundamental about the 
              mathematics of the model, then you have to update these functions.

             <b>References</b>
           
            -# Anderson, D. E., Madigan, M. L., & Nussbaum, M. A. (2007). 
            Maximum voluntary joint torque as a function of joint angle 
            and angular velocity: model development and application to 
            the lower limb. Journal of biomechanics, 40(14), 3105-3113.

            -# Beimborn, D. S., & Morrissey, M. C. (1988). A review of the 
            literature related to trunk muscle performance. Spine, 13(6), 655-660.

            -# Christophy, M., Senan, N. A. F., Lotz, J. C., & O’Reilly, O. M.
            (2012). A musculoskeletal model for the lumbar spine.
            Biomechanics and Modeling in Mechanobiology, 11(1-2), 19-34.

            -# Dolan, P., A. F. Mannion, and M. A. Adams. Passive tissues help 
            the back muscles to generate extensor moments during lifting. 
            Journal of Biomechanics 27, no. 8 (1994): 1077-1085.


            -# Jackson, M.I. (2010). The mechanics of the Table Contact
            Phase of Gymnastics Vaulting. Doctoral Thesis, Loughborough
            University.

            -# Kentel, B.B., King, M.A., & Mitchell, S.R. (2011).
            Evaluation of a subject-specific torque-driven computer simulation
            model of one-handed tennis backhand ground strokes. Journal of
            Applied Biomechanics, 27(4),345-354.

            -# Millard, M., Uchida, T., Seth, A., & Delp, S. L. (2013). 
            Flexing computational muscle: modeling and simulation of 
            musculotendon dynamics. Journal of biomechanical engineering, 
            135(2), 021005.

            -# Németh, G., & Ohlsén, H. (1986). Moment arm lengths of trunk
            muscles to the lumbosacral joint obtained in vivo with computed
            tomography. Spine, 11(2), 158-160.

            -# Ranatunga, K. W. (1984). The force-velocity relation of rat
            fast-and slow-twitch muscles examined at different temperatures.
            The Journal of Physiology, 351, 517.

            -# Raschke, U., & Chaffin, D. B. (1996). Support for a linear 
            length-tension relation of the torso extensor muscles: an 
            investigation of the length and velocity EMG-force relationships. 
            Journal of biomechanics, 29(12), 1597-1604.

            -# Rassier, D. E., Herzog, W., Wakeling, J., & Syme, D. A. (2003).
            Stretch-induced, steady-state force enhancement in single
            skeletal muscle fibers exceeds the isometric force at optimum
            fiber length. Journal of biomechanics, 36(9), 1309-1316.


            
        */
        class Millard2016TorqueMuscle {
#ifdef RBDL_BUILD_ADDON_MUSCLE_FITTING
          friend class TorqueMuscleFittingToolkit;
          friend class FitTorqueMuscleParameters;
#endif

            public:
                /**
                Default constructor, which for the moment does nothing. 
                Calling any of the models functions after the default
                construction will result in a runtime error.
                */
                Millard2016TorqueMuscle();


                /**
                This constructor  allows you to easily access the large 
                table of built-in torque muscle coefficients to create 
                a torque muscle that best represents the joint of interest.

                <b> Note: directions </b> 
                This constructs a single joint-torque muscle: 
                it can only generate torque in one direction. If you want 
                to generate a torque in two directions, you need 2 torque 
                muscles.

                <b> Note: signs and offsets </b> 

                All of the angles in these models are defined anatomically.
                You will need to set a series of variables to correctly
                map from your model's joint coordinates and sign conventions
                to that of the models:
                jointAngleOffsetRelativeToDoxygenFigures,
                signOfJointAngleRelativeToDoxygenFigures,
                signOfJointTorqueToDoxygenFigures. Also note that due to 
                the anatomical angle definitions some left and right handed
                joints will require different signs. This will be true 
                for internal/external rotation at the shoulder,
                horizontal adduction/abduction at the shoulder, ulnar/radial
                deviation at the wrist, pronation/supination of the wrist,
                and others as the list of directions grows.
              
                <b> Note: blending variables </b>

                When this constructor is called the blending variables for the
                active-torque-angle-curve, torque-angular-velocity-curve, and
                passive-torque-angle-curve are set to 0.

                @param dataSet
                       The desired source of joint torque
                       coefficients. Use the DataSet structure to choose
                       the desired data set (e.g. DataSet::Anderson2007,
                       or DataSet::Gymnast)

                @param subjectInfo
                        A struct that contains metadata about the subject
                        which is used to scale the maximum torque of the 
                        torque muscle.

                @param jointTorque
                       Select the joint and torque direction of interest.
                       Use the struct for each data set to choose a 
                       joint-torque-direction that is in the set (e.g.
                       Anderson2007::HipExtension, or 
                       Gymnast::ShoulderHorizontalAdduction)
        
                @param jointAngleOffsetRelativeToDoxygenFigures
                        Offset angle between your model's joints and the
                        reference figures in class description. 

                @param signOfJointAngleRelativeToDoxygenFigures
                        The sign convention that converts your model's joint
                        angles to the angles used in the reference figures.

                @param signOfJointTorqueToDoxygenFigures
                        The sign that maps fiberTorque from Anderson's model
                        (which is always positive) to the correctly signed
                        joint torque for your model.

                @param name
                        The name of the muscle. This is needed to do useful
                        things like provide error messages that are human
                        readable.
                
                @throws abort() when
                    -# The combination of dataSet, gender, joint, and jointDirection does not correspond to a valid entry
                    -# mSubjectHeightInMeters <= 0
                    -# mSubjectMassInKg <= 0
                    -# abs(signOfJointTorque)-1 > epsilon

                */
                Millard2016TorqueMuscle(
                    DataSet::item   dataSet,
                    const SubjectInformation &subjectInfo,
                    int jointTorque,
                    double  jointAngleOffsetRelativeToDoxygenFigures,
                    double  signOfJointAngleRelativeToDoxygenFigures,
                    double  signOfJointTorqueToDoxygenFigures,
                    const   std::string& name
                    );


                /**
                Calculates the signed joint torque developed by the 
                muscle. Note that the signs that are needed to map from the 
                native curves to those of your specific model are set when the 
                muscle is constructed.
                \f[
                \tau_{M} = \pm \tau_{ISO}( \mathbf{a} \, \mathbf{t}_A(\theta) \mathbf{t}_V(\dot{\theta}/\dot{\theta}_{MAX})
                                 +\mathbf{t}_P(1- \beta (\dot{\theta}/\dot{\theta}_{MAX})) \, )
                \f]
                @param jointAngle (radians)
                
                @param jointAngularVelocity (radians/sec)

                @param activation: the percentage of the muscle that is
                        turned on [0-1]. This function allows activations to be 
                        outside [0,1], because this is useful during the 
                        intermediate solutions of an optimization run. However,
                        you must ensure after the fact that your activations
                        fall within a bound of [0,1].

                @returns torque developed by the muscle in (Nm).
                */
                double calcJointTorque(                        
                        double jointAngle,
                        double jointAngularVelocity,
                        double activation) const;


                /**
                This function will calculate the muscle activation needed to
                generate the joint torque. This function is useful in the
                process of fitting the strength of a model to inverse-dynamics
                data. The value for activation is arrived at by evaluating this
                equation
                \f[
               \mathbf{a} = \dfrac{ \dfrac{\tau (\mathbf{a}, \theta,\dot{\theta})}{ \tau_{ISO}}
               - \mathbf{t}_P(1- \beta (\dot{\theta}/\dot{\theta}_{MAX}))}{
                  \mathbf{t}_A(\theta) \mathbf{t}_V(\dot{\theta}/\dot{\theta}_{MAX})}.
                \f]
                If the passive forces at the desired angle actually exceed the
                desired torque, a negative value for activation will be returned.
                If you see this it just means the passive-torque-angle curve of
                this muscle needs to be adjusted.

                <b>Note </b>
                  An activation value of 0 is passed out if the sign of the
                 desired jointTorque differs from the sign of the joint torque
                 that this muscle can produce.



                @param jointAngle (radians)

                @param jointAngularVelocity (radians/sec)

                @param jointTorque (Nm).

                @param updTorqueMuscleSummaryStruct
                       TorqueMuscleSummary struct which contains the
                       calculated activation along with all of the
                       internal parameters of the muscle so that you can
                       understand why the activation value takes the
                       value that it does. See the TorqueMuscleSummary 
                       (in this file) for details.
                */
                void calcActivation(double jointAngle,
                                    double jointAngularVelocity,
                                    double jointTorque,
                                    TorqueMuscleSummary
                                      &updTorqueMuscleSummaryStruct) const;


                /**
                  This function will compute the scaling factor \f$A\f$ that would
                  be needed in order for this muscle to generate a specific
                  joint torque \f$\tau^*\f$ at a specific activation level. 
                  \f[
                  A = \dfrac{\tau^*}{ 
                  \tau_{ISO}( \mathbf{a} \, \mathbf{t}_A(\theta) \mathbf{t}_V(\dot{\theta}/\dot{\theta}_{MAX})
                                 +\mathbf{t}_P(1- \beta (\dot{\theta}/\dot{\theta}_{MAX})) \, )}
                  \f]
                  This function
                  is useful in the process of determining how to much to
                  scale the strength of a default model to match
                  inverse-dynamics data of a subject. In contrast to calcActivation,
                  if the passive forces of the muscle exceed the desired joint
                  torque the resulting scale factor will drop.

                  @param jointAngle (radians)

                  @param jointAngularVelocity (radians/sec)

                  @param jointTorque (Nm)

                  @param activation

                  @returns scaleFactor: The scale that would make the current
                          muscle produce exactly the desired torque at the
                         specfied angle and angular velocity. Note: a value
                         of 0 is passed out if the sign of the desired
                         jointTorque differs from the sign of the
                         joint torque that this muscle can produce.
                */
                double calcMaximumActiveIsometricTorqueScalingFactor(
                          double jointAngle,
                          double jointAngularVelocity,
                          double activation,
                          double jointTorque) const;

                /**
                Calculates a large number of internal quantities of the
                torque muscle ranging from the values of the muscle's 
                components, the stiffness of the muscle, and its power output.
                See the struct TorqueMuscleInfo (in this file) for details.

                @param jointAngle (radians)

                @param jointAngularVelocity (radians/sec)

                @param activation: the percentage of the muscle that is
                        turned on [0-1]. This function allows activations to be 
                        outside [0,1], because this is useful during the 
                        intermediate solutions of an optimization run. However,
                        you must ensure after the fact that your activations
                        fall within a bound of [0,1].

                @param updTorqueMuscleInfoStruct: A torque muscle struct
                */
                void calcTorqueMuscleInfo(
                        double jointAngle,
                        double jointAngularVelocity,
                        double activation,
                        TorqueMuscleInfo& updTorqueMuscleInfoStruct) const;

                /**
                    @return the sign of the joint torque (+/- 1)
                */
                double getJointTorqueSign() const;

                /**
                    @return the sign of the angle sign relative to the
                            figures in the class description (+/- 1)
                */
                double getJointAngleSign() const;

                /**
                    @return the offset angle between the model's joint
                            and the figures in the class description (rad)
                */
                double getJointAngleOffset() const;


                /**
                    @return the maximum-active-isometric torque that this muscle 
                            can produce in Nm.
                */
                double  getMaximumActiveIsometricTorque() const; 

                /**
                    @return the joint angle at which the normalized
                            active-torque-angle curve peaks at its
                            maximum value of 1.0. Angle is in radians
                 */
                double getJointAngleAtMaximumActiveIsometricTorque() const;

                /**
                  @return the width of the active torque angle curve.
                          Note that this will return the angular width of
                          the non-blended curve.
                          Angle is in radians.

                */
                double getActiveTorqueAngleCurveWidth() const;

                /**
                    @return the joint angle at which the normalized
                            passive-torque-angle curve reaches a value
                            of 1.0. If this curve never reaches a value
                            of 1.0 (because it is flat, or the
                            mPassiveTorqueScale has been set to 0) a value
                            of std::numeric_limits<double>::signaling_NaN()
                            is returned. Use the std function isfinite to
                            test if a signaling_NaN has been returned.
                            Angle is in radians
                 */
                double getJointAngleAtOneNormalizedPassiveIsometricTorque() const;

                /**
                    @return the joint angle at which the normalized
                            passive-torque-angle curve reaches its
                            minimum value
                            Angle is in radians
                 */
                double getJointAngleAtSmallestNormalizedPassiveIsometricTorque() const;


                /**
                  @return the maximum concentric angular velocity
                          in radians/sec. This scalar positive quantity
                          corresponds to the angular speed at which
                          the torque-velocity curve crosses zero.
                */
                double getMaximumConcentricJointAngularVelocity() const;

                /**
                    @return the passive-torque-scale \f$s_P\f$ that is applied 
                    to the passive-torque-curve.
                */
                double  getPassiveTorqueScale() const;

                /**
                  @return the angle \f$\theta_S\f$ that the passive curve has 
                  been shifted. Note that the sign convention is in the sense
                  of the fiber angle, not in the sense of the joint angle.
                  (radians).
                */
                double getPassiveCurveAngleOffset() const;

                /**
                  @return The maximum normalized damping term \f$\beta_{MAX}\f$ term
                          in the torque muscle model. See class description
                          for details.
                */
                double getNormalizedDampingCoefficient() const;

                /**
                  @param beta -t normalized damping term \f$\beta\f$ term
                          in the torque muscle model. See class description
                          for details.
                  @throw abort() if beta < 0
                */
                void setNormalizedDampingCoefficient(double beta);
                /**
                    Sets the scaling of the passive-joint-torques. By default 
                    this scale
                    is one.

                    @param passiveTorqueScale
                            The scale \f$s_P\f$ applied to the 
                            passive-joint-torque curve (unitless)
                */
                void  setPassiveTorqueScale(double passiveTorqueScale);

                /**
                  @param passiveCurveAngleOffsetVal the angle \f$\Delta_p\f$ 
                  that the passive curve should be shifted. Note that the
                  sign convention is in the sense of the fiber angle, not
                  in the sense of the joint angle. Angles in radians
                */
                void setPassiveCurveAngleOffset(
                  double passiveCurveAngleOffsetVal);


                double getTorqueVelocityMultiplierAtHalfOmegaMax() const;
                void setTorqueVelocityMultiplierAtHalfOmegaMax(double tvAtHalfOmegaMax);

                /**
                  This function iteratively solves for the scaling
                  so that at the specified jointAngle the passive curve
                  develops the specified passiveTorque

                  @param jointAngle the target joint angle in radians
                  @param passiveFiberTorque the target passive joint torque in Nm.
                  @throws abort if jointAngle is not in the domain of the curve
                  @throws abort if passiveFiberTorque <  sqrt(eps)
                */
                void fitPassiveTorqueScale(double jointAngle,
                                           double passiveFiberTorque);

                /**
                  This function solves for the passive curve angle offset
                  so that at the specified jointAngle the passive curve
                  develops the specified passiveTorque

                  @param jointAngle the target joint angle in radians
                  @param passiveFiberTorque the target passive joint torque in Nm.
                  @throws abort if passiveTorque < sqrt(eps)
                */
                void fitPassiveCurveAngleOffset(double jointAngle,
                                           double passiveFiberTorque);

                /**
                    Sets the strength of the muscle to match a desired value.

                    @param maxIsometricTorque
                            The desired maximum-active-isometric torque of the 
                            muscle (Nm)
                            
                */
                void setMaximumActiveIsometricTorque(
                            double maxIsometricTorque);    

                /**
                  @param maxAngularVelocity the maximum concentric joint
                         angular velocity magnitude in radians/sec. This scalar
                         quantity is not signed and must be greater than 0.
                */
                void setMaximumConcentricJointAngularVelocity(
                          double maxAngularVelocity);

                /**
                    @return the SmoothSegmentedFunction represents the 
                    active-torque-angle curve profile.
                */

                const RigidBodyDynamics::Addons::Geometry::
                SmoothSegmentedFunction& getActiveTorqueAngleCurve() const;

                /**
                    @return the SmoothSegmentedFunction that represents the
                    passive-torque-angle curve.
                */

                const RigidBodyDynamics::Addons::Geometry::
                SmoothSegmentedFunction& getPassiveTorqueAngleCurve() const;

                /**
                    @return the SmoothSegmentedFunction the represents the
                    torque-angular-velocity curve.
                */
                const RigidBodyDynamics::Addons::Geometry::
                SmoothSegmentedFunction& getTorqueAngularVelocityCurve() const;


                /**
                  @returns Value of \f$\lambda^A\f$ that is currently in use.
                */
                double getActiveTorqueAngleCurveBlendingVariable() const;

                /**
                  @returns Value of \f$\lambda^P\f$ that is currently in use.
                */
                double getPassiveTorqueAngleCurveBlendingVariable() const;

                /**
                  @returns Value of \f$\lambda^V\f$ that is currently in use.
                */
                double getTorqueAngularVelocityCurveBlendingVariable() const;

                /**
                  @param blendingVariable [0-1]
                          The desired value of \f$\lambda^A\f$. Note at 
                          \f$\lambda^A=1\f$ the curve is a horizontal line
                          with a value of 1.
                */
                void setActiveTorqueAngleCurveBlendingVariable(
                      double blendingVariable);

                /**
                  @param blendingVariable [0-1]
                          The desired value of \f$\lambda^P\f$. Note at 
                          \f$\lambda^P=1\f$ the curve is a horizontal line
                          with a value of 0.
                */
                void setPassiveTorqueAngleCurveBlendingVariable(
                      double blendingVariable);

                /**
                  @param blendingVariable [0-1]
                          The desired value of \f$\lambda^V\f$. Note at 
                          \f$\lambda^V=1\f$ the curve is a horizontal line
                          with a value of 1.
                */
                void setTorqueAngularVelocityCurveBlendingVariable(
                      double blendingVariable);

                /**
                  @returns the scaling parameter that is applied to the 
                           domain of the active-torque-angle curve. Note that
                           a value greater than 1 means that the curve has
                           been stretched.
                */
                double getActiveTorqueAngleCurveAngleScaling() const;

                /**
                  @param angleScaling
                          Scale the domain of the active torque angle curve. 
                          This scaling is applied so that the angle at which
                          the curve peaks remains unchanged. Note that an
                          angleScaling > 1 means that the typically bell-shaped
                          curve will be wider.
                */
                void setActiveTorqueAngleCurveAngleScaling(double angleScaling);


                /**
                  @param fittedParameters: the structure returned after 
                          one of the optimization-based fitting functions from
                          TorqueMuscleFittingToolkit has been called. If the
                          fitting was successful, calling this function will
                          update all of the adjusted parameters.
                */
                void setFittedParameters (
                    const TorqueMuscleParameterFittingData &fittedParameters);


                /**
                Prints 2 csv files:
                -# 'fileName' + '_variableLengthfixedVelocity': All of the 
                fields in TorqueMuscleInfo are recorded to file as the 
                jointAngle varies but the jointAngularVelocity is zero.                        
                -#'fileName' + '_fixedLengthVariableVelocity': All of the fields
                   in TorqueMuscleInfo are recorded to file as the jointAngle is 
                   fixed but the jointAngularVelocity varies.

                    Each column has a header, so that you can tell what each 
                    piece of data means.

                   @param path: the path to the destination folder. Don't put 
                   an '\' on the end.
                   @param fileNameWithoutExtension: the name of the file, but 
                   without an extension.                                    
                   @param numberOfSamplePoints: the number of sample points to 
                   use in the files.                                
                */
                void printJointTorqueProfileToFile(
                        const std::string& path,
                        const std::string& fileNameWithoutExtension,
                        int numberOfSamplePoints);

                std::string getName();
                void setName(std::string& name);

                DataSet::item getDataSet();
                GenderSet::item getGender();
                AgeGroupSet::item getAgeGroup();
                JointTorqueSet::item getJointTorque();
                double getSubjectMass();
                double getSubjectHeight();

            private:

              bool mMuscleCurvesAreDirty;
              void updateTorqueMuscleCurves();
              TorqueMuscleInfo mTmInfo;
              TorqueMuscleSummary mTmSummary;

              RigidBodyDynamics::Addons::Geometry::
                SmoothSegmentedFunction mTaCurve;
              double mTaLambda;
              static const double mTaLambdaMax;// = 1.0 -> defined in the cc file
              RigidBodyDynamics::Addons::Geometry::
                SmoothSegmentedFunction mTpCurve;
              double mTpLambda;
              static const double mTpLambdaMax;// = 0.0 -> defined in the cc file
              RigidBodyDynamics::Addons::Geometry::
                SmoothSegmentedFunction mTvCurve;
              double mTvLambda;

              ///This is set to the maximum eccentric value of the
              /// torque-velocity curve so that the optimization problem
              /// of solving for mTaLambda, mTvLambda &
              /// mMaxActiveIsometricTorque is convex. This parameter is
              /// updated in updateTorqueMuscleCurves
              double mTvLambdaMax;


              RigidBodyDynamics::Math::VectorNd mAnderson2007c1c2c3c4c5c6;
              RigidBodyDynamics::Math::VectorNd mAnderson2007b1k1b2k2;
              RigidBodyDynamics::Math::VectorNd mGymnastParams;

              DataSet::item mDataSet;
              GenderSet::item mGender;
              AgeGroupSet::item mAgeGroup;
              JointTorqueSet::item mJointTorque;

              bool mUseTabularOmegaMax;
              bool mUseTabularMaxActiveIsometricTorque;
              bool mUseTabularTorqueVelocityMultiplierAtHalfOmegaMax;

              double mTorqueVelocityMultiplierAtHalfOmegaMax;

              double mMaxActiveIsometricTorque;
              double mAngleAtOneNormActiveTorque;
              double mTaAngleScaling;
              double mOmegaMax;
              double mAngleAtOneNormPassiveTorque;
              double mAngleAtSmallestNormPassiveTorque;
              double mPassiveTorqueScale;
              double mPassiveCurveAngleOffset;

              double mBetaMax; //passive damping coefficient

              double mSubjectHeightInMeters;
              double mSubjectMassInKg;
              double mScaleFactorAnderson2007;

              double mSignOfJointAngle;
              double mSignOfConcentricAnglularVelocity;
              double mSignOfJointTorque;
              double mAngleOffset;

              std::string mMuscleName;

              double calcJointAngle(double fiberAngle) const;
              double calcFiberAngle(double jointAngle) const;
              double calcFiberAngularVelocity(
                double jointAngularVelocity) const;
              double calcJointAngularVelocity(
                double fiberAngularVelocity) const;

              //const static RigidBodyDynamics::Math::MatrixNd& 
              //getAnderson2007ParameterMatrix();
              static double const Anderson2007Table3Mean[36][14];
              static double const Anderson2007Table3Std[36][14];
              static double const GymnastWholeBody[24][12];


              void calcTorqueMuscleDataFeatures(
                  RigidBodyDynamics::Math::VectorNd const &jointTorque,
                  RigidBodyDynamics::Math::VectorNd const &jointAngle,
                  RigidBodyDynamics::Math::VectorNd const &jointAangularVelocity,
                  double activeTorqueAngleBlendingVariable,
                  double passiveTorqueAngleBlendingVariable,
                  double torqueVelocityBlendingVariable,
                  double activeTorqueAngleAngleScaling,
                  double activeTorqueAngleAtOneNormTorque,
                  double passiveTorqueAngleCurveOffset,
                  double maxAngularVelocity,
                  double maxActiveIsometricTorque,
                  TorqueMuscleDataFeatures &tmf) const;

              /**
                This function will calculate the angle at which the blendable
                passive torque angle curve develops a desired torque. Note
                that if the blendingVariable is set to 1.0 then the curve is
                y = 0, and a NaN will be returned.

                @param normPassiveFiberTorque: the normalized passive fiber torque
                @param blendingVariable: A variable between [0,1] where 0
                       is equivalent to using the default passive-torque-angle
                       curve and 1 will set this curve to 0. Any value between
                       0-1 will smoothly interpolate between these extremes.
                @param passiveTorqueAngleCurveOffset: the angular shift of the
                       passive-torque-angle curve.
                @returns fiber angle in radians at which the
                         blendable passive curve developes the desired curve.
              */
              double calcFiberAngleGivenNormalizedPassiveTorque(
                  double normPassiveFiberTorque,
                  double blendingVariable,
                  double passiveTorqueAngleCurveOffset) const;

              /**
              This function will calculate the value of a blended smooth
              segmented curve \f$\mathbf{f}(\mathbf{c}, a,\lambda,\lambda^{max})\f$,
              where \f$\mathbf{c}\f$ is a smooth-segmented-curve
              \f[
                \mathbf{f}(a,\lambda,\lambda^{max},\mathbf{c}()) = \mathbf{c}(a)
                (1-\lambda) + \lambda\lambda^{max}
              \f]

              <b>Note </b>
              At the moment this function only returns first derivatives.

              @param curveArgument: [\f$-\infty,infty\f$]
                     the argument \f$a\f$ to the smooth-segmented-curve
                     \f$\mathbf{c}()\f$. Units: depend on the curve.

              @param blendingVariable: [0-1].
                     the value of \f$\lambda\f$ the blending variable.
                     Unitless

              @param maximumBlendingValue: the scalar value the blended function
                 tends to with a blending variable \f$\lambda=1\f$.

              @param derivativeOrderArgument: [0,1].
                      The order of the derivative of the 
                      SmoothSegmentedFunction w.r.t. 
                      its scalar input argument.

              @param derivativeOrderBlendingVariable: [0,1].
                      This parameter is set to choose the derivative order
                      with respect to the blending variable. 

              @param curve: the SmoothSegmentedCurve to evaluate.

              @returns the value of the blended curve or its derivative
              */
              double calcBlendedCurveDerivative(
                  double curveArgument,
                  double blendingVariable,
                  double maximumBlendingValue,
                  unsigned int derivativeOrderArgument,
                  unsigned int derivativeOrderBlendingVariable,
                  RigidBodyDynamics::Addons::Geometry::SmoothSegmentedFunction
                    const &curve) const;


              /**
              This function will calculate the inverse value of a blended smooth
              segmented curve, that is \f$a\f$, in the function  
              \f$\mathbf{f}(\mathbf{c}, a,\lambda,\lambda^{max})\f$,
              where \f$\mathbf{c}\f$ is a smooth-segmented-curve
              \f[
                \mathbf{f}(a,\lambda,\lambda^{max},\mathbf{c}()) = \mathbf{c}(a)
                (1-\lambda) + \lambda\lambda^{max}
              \f]

              <b>Note </b>
              At the moment this function only returns first derivatives.


              @param blendedCurveValue: 
                     the value of \f$\mathbf{f}(a,\lambda,\lambda^{max},\mathbf{c}())\f$ 

              @param argGuess: 
                      an initial guess for the value of \f$a\f$, which is necessary
                      as not all blended functions are monotonic.

              @param blendingVariable: [0-1].
                     the value of \f$\lambda\f$ the blending variable.
                     Unitless

              @param maximumBlendingValue: the scalar value the blended function
                 tends to with a blending variable \f$\lambda=1\f$.

              @param curve: the SmoothSegmentedCurve to evaluate.

              @returns the value of the argument \f$a\f$ which will evaluate
                       to the given value of \f$\mathbf{f}(a,\lambda,\lambda^{max},\mathbf{c}())\f$
              */
              double calcInverseBlendedCurveValue(
                  double blendedCurveValue,
                  double argGuess,
                  double blendingVariable,
                  double maximumBlendingValue,
                  RigidBodyDynamics::Addons::Geometry::SmoothSegmentedFunction
                    const &curve) const;

              /**                                  
                This function evaluates all of the quantities needed to
                compute the torque the muscle is generating but nothing extra.
 

                @param activation: the activation of the muscle which can
                       range from 0-1.
                @param jointAngle: the angle of the joint(radians)
                @param jointAngularVelocity: the angular velocity of the joint
                       ((rad/s))
                @param activeTorqueAngleBlendingVariable: the blending variable
                       associated with the active-torque-angle curve
                @param passiveTorqueAngleBlendingVariable: the blending variable
                       associated with the passive-torque-angle curve
                @param activeTorqueAngularVelocityBlendingVariable: the blending
                       variable associated with the torque-angular velocity
                       curve.
                @param activeTorqueAngleAngleScaling:
                       the scaling of the angle. By default this is set to 1.0,
                       but if desired, the active torque angle curve can be
                       re-scaled to make it wider or narrower. This scaling is
                       done such that the angular location of peak
                       active-torque-angle multiplier does not change.
                @param activeTorqueAngleAtOneNormTorque:
                       the angle at which the torque-angle curve hits a
                       normalized value of 1.0.
                @param passiveTorqueAngleCurveOffset: the angular shift of the
                       passive-torque-angle curve.
                @param maxAngularVelocity:
                       the maximum angular velocity of the muscle - used
                       to normalize the angular velocity prior to evaluating
                       the force-velocity curve.
                @param maxActiveIsometricTorque: the maximum isometric torque
                @param updTms: the torque-muscle-summary structure to be
                       updated.
              */
              void updTorqueMuscleSummary(
                    double activation,
                    double jointAngle,
                    double jointAngularVelocity,
                    double activeTorqueAngleBlendingVariable,
                    double passiveTorqueAngleBlendingVariable,
                    double activeTorqueAngularVelocityBlendingVariable,
                    double activeTorqueAngleAngleScaling,
                    double activeTorqueAngleAtOneNormTorque,
                    double passiveTorqueAngleCurveOffset,
                    double maxAngularVelocity,
                    double maxActiveIsometricTorque,
                    TorqueMuscleSummary &updTms) const;
              /**
                This function evaluates the curves values that are a part of the
                torque-muscle-summary structure.

                @param fiberAngle: the angle of the muscle fiber (radians)
                @param normFiberAngularVelocity: the normalized angular
                       velocity of the fiber ((rad/s)/(rad/s))
                @param activeTorqueAngleBlendingVariable: the blending variable
                       associated with the active-torque-angle curve
                @param passiveTorqueAngleBlendingVariable: the blending variable
                       associated with the passive-torque-angle curve
                @param activeTorqueAngularVelocityBlendingVariable: the blending
                       variable associated with the torque-angular velocity
                       curve.
                @param activeTorqueAngleAngleScaling:
                       the scaling of the angle. By default this is set to 1.0,
                       but if desired, the active torque angle curve can be
                       re-scaled to make it wider or narrower. This scaling is
                       done such that the angular location of peak
                       active-torque-angle multiplier does not change.
                @param activeTorqueAngleAtOneNormTorque:
                       the angle at which the torque-angle curve hits a
                       normalized value of 1.0.
                @param passiveTorqueAngleCurveOffset: the angular shift of the
                       passive-torque-angle curve.
                @param updTms: the torque-muscle-summary structure to be
                       updated.
              */
              void updTorqueMuscleSummaryCurveValues(
                    double fiberAngle,
                    double normFiberAngularVelocity,
                    double activeTorqueAngleBlendingVariable,
                    double passiveTorqueAngleBlendingVariable,
                    double activeTorqueAngularVelocityBlendingVariable,
                    double activeTorqueAngleAngleScaling,
                    double activeTorqueAngleAtOneNormTorque,
                    double passiveTorqueAngleCurveOffset,
                    TorqueMuscleSummary &updTms) const;

              /**
                This function evaluates the curves values and derivatives in
                the TorqueMuscleInfo structue that are functions of fiberAngle
                and the normalized fiber angular velocity.

                @param activation: the activation of the muscle [0-1]
                @param jointAngle: the angle of the joint(radians)
                @param jointAngularVelocity: the angular velocity of the joint
                       ((rad/s))
                @param activeTorqueAngleBlendingVariable: the blending variable
                       associated with the active-torque-angle curve
                @param passiveTorqueAngleBlendingVariable: the blending variable
                       associated with the passive-torque-angle curve
                @param activeTorqueAngularVelocityBlendingVariable: the blending
                       variable associated with the torque-angular velocity
                       curve.
                @param activeTorqueAngleAngleScaling:
                       the scaling of the angle. By default this is set to 1.0,
                       but if desired, the active torque angle curve can be
                       re-scaled to make it wider or narrower. This scaling is
                       done such that the angular location of peak
                       active-torque-angle multiplier does not change.
                @param activeTorqueAngleAtOneNormTorque:
                       the angle at which the torque-angle curve hits a
                       normalized value of 1.0.
                @param passiveTorqueAngleCurveOffset: the angular shift of the
                       passive-torque-angle curve.
                @param maxAngularVelocity:
                       the maximum angular velocity of the muscle - used
                       to normalize the angular velocity prior to evaluating
                       the force-velocity curve.
                @param maxActiveIsometricTorque:
                       the maximum active isometric torque the muscle can
                       generate
                @param updTmi: the torque-muscle-info structure to be
                       updated.
              */
              void updTorqueMuscleInfo(
                    double activation,
                    double jointAngle,
                    double jointAngularVelocity,
                    double activeTorqueAngleBlendingVariable,
                    double passiveTorqueAngleBlendingVariable,
                    double activeTorqueAngularVelocityBlendingVariable,
                    double activeTorqueAngleAngleScaling,
                    double activeTorqueAngleAtOneNormTorque,
                    double passiveTorqueAngleCurveOffset,
                    double maxAngularVelocity,
                    double maxActiveIsometricTorque,
                    TorqueMuscleInfo &updTmi) const;

              /**
                This function evaluates all of the quantities needed to
                compute the activation of the muscle given its torque. In this
                case the activation assigned to the TorqueMuscleSummary
                structure is allowed to go outside of the bounds [0,1].

                @param jointTorque: the torque generated by the muscle at the
                                    joint (Nm)
                @param jointAngle: the angle of the joint(radians)
                @param jointAngularVelocity: the angular velocity of the joint
                       ((rad/s))
                @param activeTorqueAngleBlendingVariable: the blending variable
                       associated with the active-torque-angle curve
                @param passiveTorqueAngleBlendingVariable: the blending variable
                       associated with the passive-torque-angle curve
                @param activeTorqueAngularVelocityBlendingVariable: the blending
                       variable associated with the torque-angular velocity
                       curve.                
                @param activeTorqueAngleAngleScaling:
                       the scaling of the angle. By default this is set to 1.0,
                       but if desired, the active torque angle curve can be
                       re-scaled to make it wider or narrower. This scaling is
                       done such that the angular location of peak
                       active-torque-angle multiplier does not change.
                @param activeTorqueAngleAtOneNormTorque:
                       the angle at which the torque-angle curve hits a
                       normalized value of 1.0.
                @param passiveTorqueAngleCurveOffset: the angular shift of the
                       passive-torque-angle curve.
                @param maxAngularVelocity:
                       the maximum angular velocity of the muscle - used
                       to normalize the angular velocity prior to evaluating
                       the force-velocity curve.
                @param maxActiveIsometricTorque: the maximum isometric torque
                @param updTms: the torque-muscle-summary structure to be
                       updated.

              */
              void updInvertTorqueMuscleSummary(
                    double jointTorque,
                    double jointAngle,
                    double jointAngularVelocity,
                    double activeTorqueAngleBlendingVariable,
                    double passiveTorqueAngleBlendingVariable,
                    double activeTorqueAngularVelocityBlendingVariable,
                    double activeTorqueAngleAngleScaling,                    
                    double activeTorqueAngleAtOneNormTorque,
                    double passiveTorqueAngleCurveOffset,
                    double maxAngularVelocity,
                    double maxActiveIsometricTorque,
                    TorqueMuscleSummary &updTms) const;



};




}
}
}



#endif
