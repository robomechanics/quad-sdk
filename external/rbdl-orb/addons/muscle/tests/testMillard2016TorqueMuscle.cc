/*                                                                             *
 *
 * Copyright (c) 2016 Matthew Millard <millard.matthew@gmail.com>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

//Some particularly aggressive experimental data
unsigned int TorqueMuscleFittingHardTestCaseRows = 100;
unsigned int TorqueMuscleFittingHardTestCaseCols = 4;
double const TorqueMuscleFittingHardTestCase[100][4] = 
                   {{0.,1.9148,-17.618,107.25         },                  
                    {0.0044242,1.8318,-18.277,108.39  },                         
                    {0.0088485,1.7485,-18.949,110.49  },                         
                    {0.013273,1.6636,-19.618,114.57   },                        
                    {0.017697,1.5761,-20.243,121.32   },                        
                    {0.022121,1.486,-20.758,130.97    },                       
                    {0.026545,1.3938,-21.085,143.31   },                        
                    {0.03097,1.3004,-21.144,157.68    },                       
                    {0.035394,1.2074,-20.866,173.11   },                        
                    {0.039818,1.1164,-20.199,188.42   },                        
                    {0.044242,1.0292,-19.118,202.36   },                        
                    {0.048667,0.94777,-17.626,213.77  },                         
                    {0.053091,0.87376,-15.762,221.64  },                         
                    {0.057515,0.80872,-13.594,225.25  },                         
                    {0.061939,0.75378,-11.22,224.18   },                        
                    {0.066364,0.70958,-8.7587,218.42  },                         
                    {0.070788,0.67624,-6.3387,208.34  },                         
                    {0.075212,0.65329,-4.0821,194.73  },                         
                    {0.079636,0.63975,-2.0906,178.7   },                        
                    {0.084061,0.63427,-0.43189,161.6  },                         
                    {0.088485,0.63528,0.87052,144.75  },                         
                    {0.092909,0.6412,1.8387,129.18    },                       
                    {0.097333,0.6507,2.5306,115.24    },                       
                    {0.10176,0.66277,3.015,102.36     },                      
                    {0.10618,0.67685,3.3393,88.889    },                       
                    {0.11061,0.69264,3.517,72.481     },                      
                    {0.11503,0.71007,3.5756,50.979    },                       
                    {0.11945,0.72927,3.6372,23.504    },                       
                    {0.12388,0.75048,3.852,-8.9796    },                       
                    {0.1283,0.77332,4.1461,-43.995    },                       
                    {0.13273,0.79611,4.1835,-78.49    },                       
                    {0.13715,0.81635,3.6883,-109.71   },                        
                    {0.14158,0.83172,2.6498,-135.63   },                        
                    {0.146,0.84066,1.2202,-154.98     },                      
                    {0.15042,0.84231,-0.42406,-167.16 },                          
                    {0.15485,0.83637,-2.1355,-172.03  },                         
                    {0.15927,0.82297,-3.8027,-169.8   },                        
                    {0.1637,0.80254,-5.3408,-160.95   },                        
                    {0.16812,0.7758,-6.6819,-146.16   },                        
                    {0.17255,0.74371,-7.771,-126.31   },                        
                    {0.17697,0.70746,-8.5684,-102.56  },                         
                    {0.18139,0.66837,-9.0566,-76.369  },                         
                    {0.18582,0.62778,-9.2487,-49.623  },                         
                    {0.19024,0.58691,-9.1941,-24.563  },                         
                    {0.19467,0.54667,-8.9737,-3.6038  },                         
                    {0.19909,0.50763,-8.6741,11.165   },                        
                    {0.20352,0.47001,-8.3313,18.713   },                        
                    {0.20794,0.43418,-7.8528,19.766   },                        
                    {0.21236,0.40128,-6.9609,16.835   },                        
                    {0.21679,0.37397,-5.266,12.889    },                       
                    {0.22121,0.35642,-2.5349,9.4338   },                        
                    {0.22564,0.35296,1.0546,6.2364    },                       
                    {0.23006,0.36634,4.9935,2.8315    },                       
                    {0.23448,0.39689,8.7586,-0.69647  },                         
                    {0.23891,0.44304,11.998,-4.1689   },                        
                    {0.24333,0.50198,14.522,-7.5873   },                        
                    {0.24776,0.57037,16.269,-11.01    },                       
                    {0.25218,0.64488,17.306,-14.511   },                        
                    {0.25661,0.72269,17.793,-18.219   },                        
                    {0.26103,0.8018,17.929,-22.301    },                       
                    {0.26545,0.8811,17.902,-26.91     },                      
                    {0.26988,0.96017,17.846,-32.128   },                        
                    {0.2743,1.0391,17.834,-37.93      },                     
                    {0.27873,1.1181,17.879,-44.158    },                       
                    {0.28315,1.1973,17.961,-50.527    },                       
                    {0.28758,1.277,18.039,-56.666     },                      
                    {0.292,1.3569,18.078,-62.174      },                     
                    {0.29642,1.4368,18.054,-66.692    },                       
                    {0.30085,1.5165,17.955,-69.98     },                      
                    {0.30527,1.5956,17.784,-71.983    },                       
                    {0.3097,1.6738,17.552,-72.861     },                      
                    {0.31412,1.7509,17.272,-72.976    },                       
                    {0.31855,1.8266,16.952,-72.834    },                       
                    {0.32297,1.9008,16.594,-72.982    },                       
                    {0.32739,1.9733,16.191,-73.899    },                       
                    {0.33182,2.0439,15.724,-75.903    },                       
                    {0.33624,2.1123,15.162,-79.088    },                       
                    {0.34067,2.1779,14.47,-83.322     },                      
                    {0.34509,2.24,13.609,-88.273      },                     
                    {0.34952,2.2979,12.55,-93.472     },                      
                    {0.35394,2.3507,11.283,-98.369    },                       
                    {0.35836,2.3974,9.8194,-102.39    },                       
                    {0.36279,2.4373,8.1988,-105.01    },                       
                    {0.36721,2.4698,6.4835,-105.77    },                       
                    {0.37164,2.4947,4.7502,-104.37    },                       
                    {0.37606,2.512,3.0759,-100.74     },                      
                    {0.38048,2.5221,1.5245,-95.063    },                       
                    {0.38491,2.5257,0.13371,-87.783   },                        
                    {0.38933,2.5235,-1.0913,-79.537   },                        
                    {0.39376,2.5163,-2.178,-71.042    },                       
                    {0.39818,2.5045,-3.1764,-62.958   },                        
                    {0.40261,2.4884,-4.1428,-55.768   },                        
                    {0.40703,2.468,-5.122,-49.731     },                      
                    {0.41145,2.443,-6.136,-44.898     },                      
                    {0.41588,2.4132,-7.1827,-41.179   },                        
                    {0.4203,2.3784,-8.2437,-38.413    },                       
                    {0.42473,2.3383,-9.2979,-36.422   },                        
                    {0.42915,2.2933,-10.332,-35.021   },                        
                    {0.43358,2.2443,-11.347,-34.007   },                        
                    {0.438,2.193,-12.35,-33.158       }};

//==============================================================================
// INCLUDES
//==============================================================================

#define CATCH_CONFIG_MAIN
#include "../Millard2016TorqueMuscle.h"
#ifdef RBDL_BUILD_ADDON_MUSCLE_FITTING
  #include "../TorqueMuscleFittingToolkit.h"
#endif
#include "../csvtools.h"
#include "../../geometry/tests/numericalTestFunctions.h"
#include <rbdl/rbdl_math.h>
#include <ctime>
#include <string>
#include <ostream>
#include <sstream>
#include <stdio.h>
#include <exception>
#include <cassert>
#include <vector>

#include "rbdl_tests.h"

using namespace RigidBodyDynamics::Addons::Muscle;
using namespace RigidBodyDynamics::Addons::Geometry;
using namespace std;
/*
   Constructor tests:
   1. Coefficients are copied over correctly.
   2. Curves are made correctly

   calcTorqueMuscleInfo test
   stiffness calculation
   power calculation

*/


TEST_CASE(__FILE__"_ConstructorRegularCallCheck", "")
{


    //Check that the 3 constructors when called properly
    //do not abort
    Millard2016TorqueMuscle test0 = Millard2016TorqueMuscle();
    

    SubjectInformation subjectInfo;
      subjectInfo.gender          = GenderSet::Male;
      subjectInfo.ageGroup        = AgeGroupSet::Young18To25;
      subjectInfo.heightInMeters  =  1.732;
      subjectInfo.massInKg        = 69.0;


    Millard2016TorqueMuscle test2 =
    Millard2016TorqueMuscle(
          DataSet::Anderson2007,
          subjectInfo,
          Anderson2007::HipExtension,
          0.0,
          1.0,
          1.0,
          "test_easyConstructor");

    CHECK(fabs( test2.getPassiveTorqueScale()-1.0) < TOL);

}



TEST_CASE(__FILE__"_calcJointTorqueCorrectnessTests", ""){

    double jointAngleOffset     = 0;    
    double signOfJointAngle     = 1;
    double signOfJointTorque    = 1;
    double err = 0.0;

    std::string name("test");

    SubjectInformation subjectInfo;
      subjectInfo.gender          = GenderSet::Male;
      subjectInfo.ageGroup        = AgeGroupSet::Young18To25;
      subjectInfo.heightInMeters  =  1.732;
      subjectInfo.massInKg        = 69.0;

    Millard2016TorqueMuscle tq =
            Millard2016TorqueMuscle(DataSet::Anderson2007,
                                     subjectInfo,
                                     Anderson2007::HipExtension,
                                     jointAngleOffset,
                                     signOfJointAngle,
                                     signOfJointTorque,
                                     name);

    bool flagMakeTestVector = false;
    if(flagMakeTestVector){
      Millard2016TorqueMuscle tqG =
              Millard2016TorqueMuscle(DataSet::Gymnast,
                                       subjectInfo,
                                       Gymnast::HipExtension,
                                       jointAngleOffset,
                                       signOfJointAngle,
                                       signOfJointTorque,
                                       name);
      TorqueMuscleInfo tmiG;
      tqG.calcTorqueMuscleInfo(M_PI/3.0,0.1,0.77,tmiG);

      printf("%f\n",tmiG.fiberAngle);
      printf("%f\n",tmiG.fiberAngularVelocity);
      printf("%f\n",tmiG.activation);
      printf("%f\n",tmiG.fiberTorque);
      printf("%f\n",tmiG.fiberStiffness);
      printf("%f\n",tmiG.fiberPassiveTorqueAngleMultiplier);
      printf("%f\n",tmiG.fiberActiveTorqueAngleMultiplier);
      printf("%f\n",tmiG.fiberTorqueAngularVelocityMultiplier);
      printf("%f\n",tmiG.fiberPassiveTorque);
      printf("%f\n",tmiG.fiberActiveTorque);
      printf("%f\n",tmiG.fiberDampingTorque);
      printf("%f\n",tmiG.fiberNormDampingTorque);
      printf("%f\n",tmiG.fiberActivePower);
      printf("%f\n",tmiG.fiberPassivePower);
      printf("%f\n",tmiG.fiberPower);
      printf("%f\n",tmiG.DjointTorque_DjointAngle);
      printf("%f\n",tmiG.DjointTorque_DjointAngularVelocity);
      printf("%f\n",tmiG.DjointTorque_Dactivation);

    }


    //Zero out the passive forces so that calcMuscleTorque reports
    //just the active force - this allows us to test its correctness.
    tq.setPassiveTorqueScale(0.0);
    double tmp = tq.calcJointTorque(0,0,1.0);

    //Test that the get and set functions work for
    //maximum isometric torque
    double tauMaxOld = tq.getMaximumActiveIsometricTorque();
    double tauMax = tauMaxOld*10.0;
    tq.setMaximumActiveIsometricTorque(tauMax);
    tmp = tq.calcJointTorque(0,0,1.0);
    //ensures updateTorqueMuscleCurves is called
    CHECK(fabs( tq.getMaximumActiveIsometricTorque()-tauMax)
          < TOL );

    double omegaMaxOld = tq.getMaximumConcentricJointAngularVelocity();
    double omegaMax    = 2.0*fabs(omegaMaxOld);
    tq.setMaximumConcentricJointAngularVelocity(omegaMax);
    tmp = tq.calcJointTorque(0,0,1.0);
    //ensures updateTorqueMuscleCurves is called
    CHECK(fabs( fabs(tq.getMaximumConcentricJointAngularVelocity())-omegaMax)
          < TOL );


    double taAngleScalingOld = tq.getActiveTorqueAngleCurveAngleScaling();
    double taAngleScaling = 2.0*taAngleScalingOld;
    tq.setActiveTorqueAngleCurveAngleScaling(taAngleScaling);

    CHECK(fabs(taAngleScaling-tq.getActiveTorqueAngleCurveAngleScaling()) <TOL);
}

TEST_CASE(__FILE__"_dampingTermTests", ""){

  double err = 0.;
  double jointAngleOffset     = 0;
  double signOfJointAngle     = 1;
  double signOfJointTorque    = 1;
  double signOfJointVelocity  = signOfJointTorque;

  std::string name("test");

  SubjectInformation subjectInfo;
    subjectInfo.gender          = GenderSet::Female;
    subjectInfo.ageGroup        = AgeGroupSet::SeniorOver65;
    subjectInfo.heightInMeters  =  1.732;
    subjectInfo.massInKg        = 69.0;

  Millard2016TorqueMuscle tq =
          Millard2016TorqueMuscle(DataSet::Anderson2007,
                                   subjectInfo,
                                   Anderson2007::HipExtension,
                                   jointAngleOffset,
                                   signOfJointAngle,
                                   signOfJointTorque,
                                   name);
  TorqueMuscleInfo tmi0;
  //Test the damping term
  double beta = tq.getNormalizedDampingCoefficient();
  tq.setNormalizedDampingCoefficient(beta+0.1);
  CHECK(fabs(beta+0.1-tq.getNormalizedDampingCoefficient())<SQRTEPSILON);

  double omegaMax = tq.getMaximumConcentricJointAngularVelocity();
  double tau = tq.calcJointTorque(-M_PI/3.0, omegaMax,0);
  CHECK(fabs(tau) < SQRTEPSILON );

  tq.calcTorqueMuscleInfo(-M_PI/3.0,omegaMax,0.1,tmi0);
  err = fabs(tmi0.activation
            *tmi0.fiberActiveTorqueAngleMultiplier
            *tmi0.fiberTorqueAngularVelocityMultiplier
            +tmi0.fiberPassiveTorqueAngleMultiplier
            *tq.getPassiveTorqueScale()
            +tmi0.fiberNormDampingTorque);
  CHECK( err < SQRTEPSILON);

  beta    = tq.getNormalizedDampingCoefficient();
  double tauMax  = tq.getMaximumActiveIsometricTorque();
  tq.calcTorqueMuscleInfo(tq.getJointAngleAtOneNormalizedPassiveIsometricTorque(),
                          -omegaMax,
                          0,
                          tmi0);
  CHECK( fabs(tmi0.fiberDampingTorque
             - 1.0*beta*1.0*tauMax) < SQRTEPSILON );
  CHECK( fabs(tmi0.fiberNormDampingTorque -beta*1) < SQRTEPSILON );

}

TEST_CASE(__FILE__"_simpleFittingFunctionTests", ""){
  double err = 0.;

  double jointAngleOffset     = 0;
  double signOfJointAngle     = 1;
  double signOfJointTorque    = 1;
  double signOfJointVelocity  = signOfJointTorque;

  std::string name("test");

  SubjectInformation subjectInfo;
    subjectInfo.gender          = GenderSet::Female;
    subjectInfo.ageGroup        = AgeGroupSet::SeniorOver65;
    subjectInfo.heightInMeters  =  1.732;
    subjectInfo.massInKg        = 69.0;

  Millard2016TorqueMuscle tq =
          Millard2016TorqueMuscle(DataSet::Anderson2007,
                                   subjectInfo,
                                   Anderson2007::HipExtension,
                                   jointAngleOffset,
                                   signOfJointAngle,
                                   signOfJointTorque,
                                   name);

  TorqueMuscleInfo tmi0, tmi1;

  tq.setPassiveTorqueScale(1.0);
  tq.setPassiveCurveAngleOffset(0.0);
  double jointAngleAtPassiveTauMax =
      tq.getJointAngleAtOneNormalizedPassiveIsometricTorque();

  double activation = 0.1;
  tq.calcTorqueMuscleInfo(jointAngleAtPassiveTauMax,
                          0.-SQRTEPSILON,
                          activation,
                          tmi0);

  tq.setPassiveCurveAngleOffset(M_PI/3.0);
  double updJointAngleAtPassiveTauMax =
    tq.getJointAngleAtOneNormalizedPassiveIsometricTorque() ;

  CHECK( fabs(updJointAngleAtPassiveTauMax-jointAngleAtPassiveTauMax-M_PI/3.0)
        < SQRTEPSILON);

  tq.calcTorqueMuscleInfo(jointAngleAtPassiveTauMax+M_PI/3.0,
                          0.-SQRTEPSILON,
                          activation,
                          tmi1);

  CHECK( fabs(tmi0.fiberPassiveTorqueAngleMultiplier
            -tmi1.fiberPassiveTorqueAngleMultiplier) < SQRTEPSILON);

  //fitPassiveCurveAngleOffset: Extension test
  double tauMax = tq.getMaximumActiveIsometricTorque();
  jointAngleAtPassiveTauMax
      = tq.getJointAngleAtOneNormalizedPassiveIsometricTorque();
  tq.fitPassiveCurveAngleOffset(1.0,
                                tauMax);

  tq.calcTorqueMuscleInfo(1.0,
                          0.,
                          0.,
                          tmi0);

  CHECK(fabs(tmi0.fiberPassiveTorque - tauMax) < SQRTEPSILON);

  //fitPassiveCurveAngleOffset: flexion test
  Millard2016TorqueMuscle tqF =
          Millard2016TorqueMuscle(DataSet::Anderson2007,
                                   subjectInfo,
                                   Anderson2007::HipFlexion,
                                   jointAngleOffset,
                                   signOfJointAngle,
                                   -1*signOfJointTorque,
                                   "flexion");
  tauMax = tqF.getMaximumActiveIsometricTorque();
  jointAngleAtPassiveTauMax =
      tqF.getJointAngleAtOneNormalizedPassiveIsometricTorque();
  tqF.fitPassiveCurveAngleOffset(-1.0, tauMax);

  tqF.calcTorqueMuscleInfo(-1.0,
                          0.,
                          0.,
                          tmi0);

  CHECK(fabs(tmi0.fiberPassiveTorque - tauMax) < SQRTEPSILON);

  tauMax = tq.getMaximumActiveIsometricTorque();
  jointAngleAtPassiveTauMax
      = tq.getJointAngleAtOneNormalizedPassiveIsometricTorque();

  tq.fitPassiveTorqueScale(jointAngleAtPassiveTauMax, tauMax*0.5);

  tq.calcTorqueMuscleInfo(jointAngleAtPassiveTauMax,
                          0.,
                          0.,
                          tmi0);

  CHECK(fabs(tmi0.fiberPassiveTorque - tauMax*0.5) < SQRTEPSILON);

  //Now for the flexor ...
  tauMax = tqF.getMaximumActiveIsometricTorque();
  jointAngleAtPassiveTauMax
      = tqF.getJointAngleAtOneNormalizedPassiveIsometricTorque();

  tqF.fitPassiveTorqueScale(jointAngleAtPassiveTauMax, tauMax*0.5);

  tqF.calcTorqueMuscleInfo(jointAngleAtPassiveTauMax,
                          0.,
                          0.,
                          tmi0);

  CHECK(fabs(tmi0.fiberPassiveTorque - tauMax*0.5) < SQRTEPSILON);

  //Now test the calcMaximumActiveIsometricTorqueScalingFactor and
  //calcActivation functions

  double jointTorque =
      tq.calcJointTorque(M_PI/3.0, M_PI/5.0, 0.5);
  TorqueMuscleSummary tms;
  tq.calcActivation(M_PI/3.0, M_PI/5.0, jointTorque,tms);
  activation = tms.activation;
  CHECK(fabs(activation-0.5) < SQRTEPSILON );

  double scaling = tq.calcMaximumActiveIsometricTorqueScalingFactor(
        M_PI/3.0,M_PI/5.0,0.5, jointTorque*1.1);
  CHECK(fabs(1.1-scaling) < SQRTEPSILON);


}

TEST_CASE(__FILE__"_calcTorqueMuscleInfoCorrectnessTests", ""){

  double jointAngleOffset     = 0;
  double signOfJointAngle     = 1;
  double signOfJointTorque    = 1;
  double signOfJointVelocity  = signOfJointTorque;

  std::string name("test");

  SubjectInformation subjectInfo;
    subjectInfo.gender          = GenderSet::Female;
    subjectInfo.ageGroup        = AgeGroupSet::SeniorOver65;
    subjectInfo.heightInMeters  =  1.732;
    subjectInfo.massInKg        = 69.0;

  Millard2016TorqueMuscle tq =
          Millard2016TorqueMuscle(DataSet::Anderson2007,
                                   subjectInfo,
                                   Anderson2007::HipExtension,
                                   jointAngleOffset,
                                   signOfJointAngle,
                                   signOfJointTorque,
                                   name);
  tq.setActiveTorqueAngleCurveBlendingVariable(0.);
  tq.setPassiveTorqueAngleCurveBlendingVariable(0.);
  tq.setTorqueAngularVelocityCurveBlendingVariable(0.);

  double jointAngle       = 0.;
  double jointVelocity    = 0.;
  double activation       = 1.0;

  tq.setPassiveTorqueScale(0.5);
  double tmp = tq.calcJointTorque(0,0,1.0);
  tq.setPassiveTorqueScale(1.0);
  tmp = tq.calcJointTorque(0,0,1.0);

  double tauMax = tq.getMaximumActiveIsometricTorque();

  double jointAngleAtTauMax = tq.getJointAngleAtMaximumActiveIsometricTorque();
  TorqueMuscleInfo tmi;
  tq.calcTorqueMuscleInfo(jointAngleAtTauMax,
                              0.,
                              activation,
                              tmi);

  //Keypoint check: active force components + fiber kinematics

  CHECK(fabs(tmi.activation-1)                 < EPSILON);
  CHECK(fabs(tmi.jointAngle-jointAngleAtTauMax)< TOL);
  CHECK(fabs(tmi.jointAngularVelocity-0.)      < TOL);

  CHECK(fabs(tmi.activation-1)                 < EPSILON);
  //CHECK(fabs(tmi.fiberAngle-thetaAtTauMax)     < TOL);
  CHECK(fabs(tmi.fiberAngularVelocity-0.)      < TOL);

  CHECK(fabs(tmi.fiberActiveTorque - tauMax)   < TOL);
  CHECK(fabs(tmi.fiberActiveTorqueAngleMultiplier-1.0) < TOL);
  CHECK(fabs(tmi.fiberTorqueAngularVelocityMultiplier-1.0)<TOL);

  //Total force check
  double torque = tq.calcJointTorque(jointAngleAtTauMax,0,activation);
  double err    = fabs(torque
                      - signOfJointTorque*(
                          tmi.fiberActiveTorque+tmi.fiberPassiveTorque));
  CHECK(fabs(torque
            - signOfJointTorque*(
                tmi.fiberActiveTorque+tmi.fiberPassiveTorque)) < TOL);

  //Total active force scales with activation
  tq.calcTorqueMuscleInfo( jointAngleAtTauMax,
                           0.,
                           activation*0.5,
                           tmi);

  CHECK(fabs(tmi.fiberActiveTorque - tauMax*0.5)   < TOL);

  //Keypoint check - power
  CHECK(fabs(tmi.jointPower-tmi.fiberPower) < TOL);

  //Numerically check the active and passive fiber stiffnesses
  double h = sqrt(EPSILON);
  tq.calcTorqueMuscleInfo(jointAngleAtTauMax*0.75,
                          0.,
                          activation,
                          tmi);


  TorqueMuscleInfo tmiL;
  TorqueMuscleInfo tmiR;

  tq.calcTorqueMuscleInfo(jointAngleAtTauMax*0.75-h,
                          0.,
                          activation,
                          tmiL);

  tq.calcTorqueMuscleInfo(jointAngleAtTauMax*0.75+h,
                          0.,
                          activation,
                          tmiR);

  double jointK = (tmiR.jointTorque-tmiL.jointTorque)/(2*h);
  err = tmi.jointStiffness - jointK;

  CHECK(fabs(tmi.jointStiffness-jointK) < 1e-5);

  double fiberK = signOfJointAngle*(tmiR.fiberTorque-tmiL.fiberTorque)/(2*h);
  err = tmi.fiberStiffness - fiberK;
  CHECK(fabs(tmi.fiberStiffness - fiberK) < 1e-5);

  tq.setPassiveTorqueScale(1.5);
  tmp = tq.calcJointTorque(0,0,1.0);

  CHECK(fabs(tq.getPassiveTorqueScale()-1.5)<TOL);

  tq.setPassiveTorqueScale(1.0);
  tmp = tq.calcJointTorque(0,0,1.0);

  TorqueMuscleInfo tmi0;
  TorqueMuscleInfo tmi1;
  TorqueMuscleInfo tmi2;

  double jointAngleAtPassiveTauMax =
      tq.getJointAngleAtOneNormalizedPassiveIsometricTorque();
  tq.calcTorqueMuscleInfo(jointAngleAtPassiveTauMax,
                         0.,
                         activation,
                         tmi0);

  tq.setPassiveTorqueScale(2.0);
  tq.calcTorqueMuscleInfo(jointAngleAtPassiveTauMax,
                          0.,
                          activation,
                          tmi1);

  CHECK(fabs(tmi0.fiberPassiveTorqueAngleMultiplier -
            0.5*tmi1.fiberPassiveTorqueAngleMultiplier) < TOL);

  CHECK(fabs(tmi0.fiberPassiveTorque -
            0.5*tmi1.fiberPassiveTorque) < TOL);

  double jtq = tq.calcJointTorque(jointAngleAtPassiveTauMax,
                                  0.,
                                  activation);
  err = jtq-tmi1.jointTorque;
  CHECK(fabs(jtq-tmi1.jointTorque) < TOL );


  tq.calcTorqueMuscleInfo(jointAngleAtPassiveTauMax,
                          0.,
                          activation-SQRTEPSILON,
                          tmi0);

  tq.calcTorqueMuscleInfo(jointAngleAtPassiveTauMax,
                          0.,
                          activation,
                          tmi1);

  tq.calcTorqueMuscleInfo(jointAngleAtPassiveTauMax,
                          0.,
                          activation+SQRTEPSILON,
                          tmi2);

  double DtqDa = tmi1.DjointTorque_Dactivation;
  double DtqDa_NUM = (tmi2.jointTorque-tmi0.jointTorque)/(2*SQRTEPSILON);
  err = fabs(DtqDa-DtqDa_NUM);
  CHECK(fabs(DtqDa-DtqDa_NUM) < fabs(DtqDa)*1e-5 );


  tq.calcTorqueMuscleInfo(jointAngleAtPassiveTauMax-SQRTEPSILON,
                          0.,
                          activation,
                          tmi0);

  tq.calcTorqueMuscleInfo(jointAngleAtPassiveTauMax+SQRTEPSILON,
                          0.,
                          activation,
                          tmi2);

  double DtqDq = tmi1.DjointTorque_DjointAngle;
  double DtqDq_NUM = (tmi2.jointTorque-tmi0.jointTorque)/(2*SQRTEPSILON);
  err = fabs(DtqDq-DtqDq_NUM);
  CHECK(fabs(DtqDq-DtqDq_NUM) < fabs(DtqDq)*1e-5 );

  tq.calcTorqueMuscleInfo(jointAngleAtPassiveTauMax,
                          0.-SQRTEPSILON,
                          activation,
                          tmi0);

  tq.calcTorqueMuscleInfo(jointAngleAtPassiveTauMax,
                          0.+SQRTEPSILON,
                          activation,
                          tmi2);

  double DtqDqdot = tmi1.DjointTorque_DjointAngularVelocity;
  double DtqDqdot_NUM = (tmi2.jointTorque-tmi0.jointTorque)/(2*SQRTEPSILON);
  err = fabs(DtqDqdot-DtqDqdot_NUM);
  CHECK(fabs(DtqDqdot-DtqDqdot_NUM) < fabs(DtqDqdot)*1e-5 );


}

TEST_CASE(__FILE__"_calcTorqueMuscleInfoFittingVariableCorrectnessTests", ""){

  double jointAngleOffset     = 0;
  double signOfJointAngle     = 1;
  double signOfJointTorque    = 1;
  double signOfJointVelocity  = signOfJointTorque;

  std::string name("test");

  SubjectInformation subjectInfo;
    subjectInfo.gender          = GenderSet::Male;
    subjectInfo.ageGroup        = AgeGroupSet::Young18To25;
    subjectInfo.heightInMeters  =  1.732;
    subjectInfo.massInKg        = 69.0;

  Millard2016TorqueMuscle tq =
          Millard2016TorqueMuscle(DataSet::Gymnast,
                                   subjectInfo,
                                   Gymnast::HipExtension,
                                   jointAngleOffset,
                                   signOfJointAngle,
                                   signOfJointTorque,
                                   name);

  TorqueMuscleInfo tmi;
  double tvAtHalfOmegaMax =
      tq.getTorqueVelocityMultiplierAtHalfOmegaMax();
  double omegaMax =
      tq.getMaximumConcentricJointAngularVelocity();
  double angleAtTaMax =
      tq.getJointAngleAtMaximumActiveIsometricTorque();

  tq.calcTorqueMuscleInfo(angleAtTaMax,omegaMax*0.5,1.0,tmi);

  //We can only get within 0.001 of the tvAtHalfOmega target because
  //this itself is a fitting problem to match Hill's hyperbola as closely
  //as possible while passing through this target point.
  CHECK(fabs(tmi.fiberTorqueAngularVelocityMultiplier
             -tvAtHalfOmegaMax)<0.005);

  tvAtHalfOmegaMax = 0.4;
  tq.setTorqueVelocityMultiplierAtHalfOmegaMax(tvAtHalfOmegaMax);

  tq.calcTorqueMuscleInfo(angleAtTaMax,omegaMax*0.5,1.0,tmi);
  CHECK(fabs(tmi.fiberTorqueAngularVelocityMultiplier
             -tvAtHalfOmegaMax)<0.005);

  tq.setActiveTorqueAngleCurveBlendingVariable(0.10);
  tq.setPassiveTorqueAngleCurveBlendingVariable(0.20);
  tq.setTorqueAngularVelocityCurveBlendingVariable(0.30);

  CHECK(fabs(tq.getActiveTorqueAngleCurveBlendingVariable()-0.1)
            < SQRTEPSILON);
  CHECK(fabs(tq.getPassiveTorqueAngleCurveBlendingVariable()-0.2)
            < SQRTEPSILON);
  CHECK(fabs(tq.getTorqueAngularVelocityCurveBlendingVariable()-0.3)
            < SQRTEPSILON);


  //Get reference kinematic and torque quantities for later tests.
  double jointAngleTaTiso =
      tq.getJointAngleAtMaximumActiveIsometricTorque();

  //Using the plots of the hip extension curves to get this minimum value.
  double jointAngleTaMin = jointAngleTaTiso - M_PI;

  double jointAngleTpTiso =
      tq.getJointAngleAtOneNormalizedPassiveIsometricTorque();

  double jointAngleTpZero =
      tq.getJointAngleAtSmallestNormalizedPassiveIsometricTorque();

  omegaMax = tq.getMaximumConcentricJointAngularVelocity();

  double tiso = tq.getMaximumActiveIsometricTorque();


  //Test that the blending coefficients are modifying the correct variables
  //and in the correct ways.
  tq.setActiveTorqueAngleCurveBlendingVariable(0.0);
  tq.setPassiveTorqueAngleCurveBlendingVariable(0.0);
  tq.setTorqueAngularVelocityCurveBlendingVariable(0.0);

  double taLambda = 0.1;
  double tvLambda = 0.2;
  double tpLambda = 0.3;

  //Check the values of the blendable torque-angle-curve
  tq.setActiveTorqueAngleCurveBlendingVariable(0.0);
  tq.calcTorqueMuscleInfo(jointAngleTaMin,0,1.0,tmi);
  double multiplier0 = tmi.fiberActiveTorqueAngleMultiplier;

  tq.setActiveTorqueAngleCurveBlendingVariable(taLambda);
  tq.calcTorqueMuscleInfo(jointAngleTaMin,0,1.0,tmi);
  double multiplier1 = tmi.fiberActiveTorqueAngleMultiplier;
  double err = fabs( (taLambda+(1-taLambda)*multiplier0)-multiplier1);
  CHECK( err < SQRTEPSILON);

  tq.setActiveTorqueAngleCurveBlendingVariable(0.0);  
  tq.calcTorqueMuscleInfo(jointAngleTaTiso,0,1.0,tmi);
  multiplier0 = tmi.fiberActiveTorqueAngleMultiplier;

  tq.setActiveTorqueAngleCurveBlendingVariable(taLambda);
  tq.calcTorqueMuscleInfo(jointAngleTaTiso,0,1.0,tmi);
  multiplier1 = tmi.fiberActiveTorqueAngleMultiplier;
  err = fabs(multiplier0-multiplier1);
  CHECK( err < SQRTEPSILON);

  //Check the values of the blendable torque-angular-velocity-curve
  tq.setTorqueAngularVelocityCurveBlendingVariable(0.0);

  tq.calcTorqueMuscleInfo(jointAngleTaTiso,-omegaMax,1.0,tmi);
  double tvLambdaMax = tmi.fiberTorqueAngularVelocityMultiplier;

  tq.calcTorqueMuscleInfo(jointAngleTaTiso,omegaMax,1.0,tmi);
  multiplier0 = tmi.fiberTorqueAngularVelocityMultiplier;  
  tq.setTorqueAngularVelocityCurveBlendingVariable(tvLambda);
  tq.calcTorqueMuscleInfo(jointAngleTaTiso,omegaMax, 1.0,tmi);
  multiplier1 = tmi.fiberTorqueAngularVelocityMultiplier;

  err = fabs( (tvLambdaMax*tvLambda+(1-tvLambda)*multiplier0)-multiplier1);
  CHECK( err < SQRTEPSILON);

  tq.setTorqueAngularVelocityCurveBlendingVariable(0.0);
  tq.calcTorqueMuscleInfo(jointAngleTaTiso, -omegaMax, 1.0, tmi);
  multiplier0 = tmi.fiberTorqueAngularVelocityMultiplier;

  tq.setTorqueAngularVelocityCurveBlendingVariable(tvLambda);
  tq.calcTorqueMuscleInfo(jointAngleTaTiso, -omegaMax, 1.0, tmi);
  multiplier1 = tmi.fiberTorqueAngularVelocityMultiplier;
  err = fabs(multiplier0-multiplier1);
  CHECK( err < SQRTEPSILON);

  //Check the values of the blendable passive-torque-angle curve
  tq.setPassiveTorqueAngleCurveBlendingVariable(0.0);
  tq.calcTorqueMuscleInfo(jointAngleTpTiso,0,0.0,tmi);
  multiplier0 = tmi.fiberPassiveTorqueAngleMultiplier;

  tq.setPassiveTorqueAngleCurveBlendingVariable(tpLambda);
  tq.calcTorqueMuscleInfo(jointAngleTpTiso,0,0.0,tmi);
  multiplier1 = tmi.fiberPassiveTorqueAngleMultiplier;
  err = fabs(((1-tpLambda)*multiplier0)-multiplier1 );
  CHECK( err < SQRTEPSILON);

  tq.setPassiveTorqueAngleCurveBlendingVariable(0.0);
  tq.calcTorqueMuscleInfo(jointAngleTpZero,0,0.0,tmi);
  multiplier0 = tmi.fiberPassiveTorqueAngleMultiplier;

  tq.setPassiveTorqueAngleCurveBlendingVariable(tpLambda);
  tq.calcTorqueMuscleInfo(jointAngleTpZero,0,0.0,tmi);
  multiplier1 = tmi.fiberPassiveTorqueAngleMultiplier;
  err = fabs(multiplier0-multiplier1 );
  CHECK( err < SQRTEPSILON);

  //Check the values of the active-torque-angle curve when the scaling is
  //changed

  tq.setActiveTorqueAngleCurveBlendingVariable(0.0);
  tq.setPassiveTorqueAngleCurveBlendingVariable(0.0);
  tq.setTorqueAngularVelocityCurveBlendingVariable(0.0);
  tq.setActiveTorqueAngleCurveAngleScaling(1.0);


  //Values at the peak should not change.
  tq.setActiveTorqueAngleCurveAngleScaling(1.0);
  double jointAngleAtOneNormTorque =
        tq.getJointAngleAtMaximumActiveIsometricTorque();
  tq.calcTorqueMuscleInfo(jointAngleAtOneNormTorque,0.,1.0,tmi);
  double ta1 = tmi.fiberActiveTorqueAngleMultiplier;

  tq.setActiveTorqueAngleCurveAngleScaling(2.0);
  tq.calcTorqueMuscleInfo(jointAngleAtOneNormTorque,0.,1.0,tmi);
  double ta2 = tmi.fiberActiveTorqueAngleMultiplier;
  err = fabs(ta1-ta2);
  CHECK(err < SQRTEPSILON);

  //Values should change in proportion to their distance from the peak
  //divided by the scaling factor.
  tq.setActiveTorqueAngleCurveAngleScaling(1.0);
  tq.calcTorqueMuscleInfo(jointAngleAtOneNormTorque+0.5,0.,1.0,tmi);
  ta1 = tmi.fiberActiveTorqueAngleMultiplier;
  tq.setActiveTorqueAngleCurveAngleScaling(2.0);
  tq.calcTorqueMuscleInfo(jointAngleAtOneNormTorque+1.0,0.,1.0,tmi);
  ta2 = tmi.fiberActiveTorqueAngleMultiplier;
  err = fabs(ta1-ta2);
  CHECK(err < SQRTEPSILON);

  //Check the values of the torque-velocity curve when omega max is
  //changed.
  tq.setActiveTorqueAngleCurveAngleScaling(1.0);
  omegaMax = tq.getMaximumConcentricJointAngularVelocity();
  tq.calcTorqueMuscleInfo(jointAngleAtOneNormTorque, omegaMax*0.5,1.0,tmi);
  double tv1 = tmi.fiberTorqueAngularVelocityMultiplier;
  tq.setMaximumConcentricJointAngularVelocity(omegaMax*2.0);
  tq.calcTorqueMuscleInfo(jointAngleAtOneNormTorque, omegaMax,1.0,tmi);
  double tv2 = tmi.fiberTorqueAngularVelocityMultiplier;
  err = fabs(tv1-tv2);
  CHECK(err < SQRTEPSILON);


  TorqueMuscleInfo tmiL,tmiR; //Here we have 3 to do numerical derivatives.
  double h = SQRTEPSILON;

  tq.setActiveTorqueAngleCurveBlendingVariable(taLambda);
  tq.setPassiveTorqueAngleCurveBlendingVariable(tvLambda);
  tq.setTorqueAngularVelocityCurveBlendingVariable(tpLambda);

  //Get reference points where the curves have a non-zero value and where
  //the lambda parameter makes a difference
  double jointAngleTaMid = jointAngleTaTiso-M_PI*0.25;
  double jointAngleTpMid = (jointAngleTpTiso + jointAngleTpZero)*0.5;
  double omegaMid        = omegaMax*0.5;

  //Calc. numerical derivative of DjointTorque_DtaLambda
  tq.calcTorqueMuscleInfo(jointAngleTaMid,0,1.0,tmi);
  tq.setActiveTorqueAngleCurveBlendingVariable(taLambda-h);
  tq.calcTorqueMuscleInfo(jointAngleTaMid,0,1.0,tmiL);
  tq.setActiveTorqueAngleCurveBlendingVariable(taLambda+h);
  tq.calcTorqueMuscleInfo(jointAngleTaMid,0,1.0,tmiR);
  tq.setActiveTorqueAngleCurveBlendingVariable(taLambda);

  double Dtau_DtaLambda = (tmiR.jointTorque-tmiL.jointTorque)/(2*h);
  err = fabs(Dtau_DtaLambda
                    - tmi.DjointTorque_DactiveTorqueAngleBlendingVariable);
  CHECK( err < SQRTEPSILON*100.0 );

  //Calc. numerical derivative of DjointTorque_DtvLambda
  tq.setTorqueAngularVelocityCurveBlendingVariable(tvLambda);
  tq.calcTorqueMuscleInfo(jointAngleTaMid,omegaMid,1.0,tmi);
  tq.setTorqueAngularVelocityCurveBlendingVariable(tvLambda-h);
  tq.calcTorqueMuscleInfo(jointAngleTaMid,omegaMid,1.0,tmiL);
  tq.setTorqueAngularVelocityCurveBlendingVariable(tvLambda+h);
  tq.calcTorqueMuscleInfo(jointAngleTaMid,omegaMid,1.0,tmiR);
  tq.setTorqueAngularVelocityCurveBlendingVariable(tvLambda);

  double Dtau_DtvLambda = (tmiR.jointTorque - tmiL.jointTorque)/(2*h);
  err = fabs(Dtau_DtvLambda
                    - tmi.DjointTorque_DtorqueAngularVelocityBlendingVariable);
  CHECK( err < SQRTEPSILON*100.0 );

  //Calc. numerical derivative of DjointTorque_DtpLambda
  tq.setPassiveTorqueAngleCurveBlendingVariable(tpLambda);
  tq.calcTorqueMuscleInfo(jointAngleTpMid,omegaMid,0.0,tmi);
  tq.setPassiveTorqueAngleCurveBlendingVariable(tpLambda-h);
  tq.calcTorqueMuscleInfo(jointAngleTpMid,omegaMid,0.0,tmiL);
  tq.setPassiveTorqueAngleCurveBlendingVariable(tpLambda+h);
  tq.calcTorqueMuscleInfo(jointAngleTpMid,omegaMid,0.0,tmiR);

  double Dtau_DtpLambda = (tmiR.jointTorque - tmiL.jointTorque)/(2*h);
  err = fabs(Dtau_DtpLambda
                    - tmi.DjointTorque_DpassiveTorqueAngleBlendingVariable);
  CHECK( err < SQRTEPSILON*100.0 );

  //Calc. numerical derivatives of DjointTorque_DtpOffset
  double angleOffset = 0.;
  tq.setPassiveCurveAngleOffset(angleOffset);
  tq.calcTorqueMuscleInfo(jointAngleTpMid,omegaMid,0.0,tmi);
  tq.setPassiveCurveAngleOffset(angleOffset-h);
  tq.calcTorqueMuscleInfo(jointAngleTpMid,omegaMid,0.0,tmiL);
  tq.setPassiveCurveAngleOffset(angleOffset+h);
  tq.calcTorqueMuscleInfo(jointAngleTpMid,omegaMid,0.0,tmiR);

  double Dtp_DtpOffset = (tmiR.fiberPassiveTorqueAngleMultiplier
                         -tmiL.fiberPassiveTorqueAngleMultiplier)/(2*h);
  err = fabs(Dtp_DtpOffset
             - tmi.DfiberPassiveTorqueAngleMultiplier_DangleOffset);
  CHECK(err < SQRTEPSILON*100.0);

  double Dtau_DtpOffset = (tmiR.jointTorque - tmiL.jointTorque)/(2*h);
  err = fabs(Dtau_DtpOffset
             - tmi.DjointTorque_DpassiveTorqueAngleCurveAngleOffset);
  CHECK(err < SQRTEPSILON*100.0);

  //Calc. numerical derivative of DjointTorque_DtaAngleScaling
  tq.setActiveTorqueAngleCurveBlendingVariable(0.0);
  tq.setPassiveTorqueAngleCurveBlendingVariable(0.0);
  tq.setTorqueAngularVelocityCurveBlendingVariable(0.0);

  tq.setActiveTorqueAngleCurveAngleScaling(1.0);
  tq.calcTorqueMuscleInfo(jointAngleAtOneNormTorque+1.0,0.0,1.0,tmi);
  tq.setActiveTorqueAngleCurveAngleScaling(1.0-h);
  tq.calcTorqueMuscleInfo(jointAngleAtOneNormTorque+1.0,0.0,1.0,tmiL);
  tq.setActiveTorqueAngleCurveAngleScaling(1.0+h);
  tq.calcTorqueMuscleInfo(jointAngleAtOneNormTorque+1.0,0.0,1.0,tmiR);

  double Dtau_DtaScaling = (tmiR.jointTorque-tmiL.jointTorque)/(2*h);
  err = fabs(Dtau_DtaScaling
             -tmi.DjointTorque_DactiveTorqueAngleAngleScaling);
  CHECK(err < SQRTEPSILON*100.0);

  //Calc. numerical derivative of DjointTorque_DomegaMax
  tq.setActiveTorqueAngleCurveAngleScaling(1.0);
  omegaMax = tq.getMaximumConcentricJointAngularVelocity();

  tq.setMaximumConcentricJointAngularVelocity(omegaMax);
  tq.calcTorqueMuscleInfo(jointAngleAtOneNormTorque,omegaMax*0.5,1.0,tmi);
  tq.setMaximumConcentricJointAngularVelocity(omegaMax-h);
  tq.calcTorqueMuscleInfo(jointAngleAtOneNormTorque,omegaMax*0.5,1.0,tmiL);
  tq.setMaximumConcentricJointAngularVelocity(omegaMax+h);
  tq.calcTorqueMuscleInfo(jointAngleAtOneNormTorque,omegaMax*0.5,1.0,tmiR);

  double Dtau_DomegaMax = (tmiR.jointTorque-tmiL.jointTorque)/(2*h);

  err = fabs( Dtau_DomegaMax
              -tmi.DjointTorque_DmaximumAngularVelocity);
  CHECK(err < SQRTEPSILON*100.0);

#ifdef RBDL_BUILD_ADDON_MUSCLE_FITTING
  //Check the second derivatives.
  /*
   The lower right triangle of the Hessian of
   jointTorque(angle,angularVelocity,activation,x) where x
     x = (taLambda,tpLambda,tvLambda,tpOffsetAngle)
   is the vector of fitting variables  is stored in the vector fittingInfo.

   For brevity, the short forms will be used just in the comments below
    x = [a, v, p, o]
   For reference:
      t(a,v,p,o) = mSignOfJointTorque*(
                    (maxActIsoTorque*(
                        activation*ta(a)*tv(v)) + (tp(p,o)+tb(p,o))))
   Where 't' is the shortform used to represent joint-torque.

   The lower right triangle for the Hessian of tau(a,v,p,o) w.r.t. x is
   stored, row-wise, in the vector fittingInfo. Thus we have

                           a           v           p           o
   H(tau(a,v,p,o),x) = a [ 0: d2t/da2
                       v [ 1: d2t/dadv 2: d2t/dv2
                       p [ 3: d2t/dadp 4: d2t/dvdp 5: d2t/dp2
                       o [ 6: d2t/dado 7: d2t/dvdo 8: d2t/dpdo 9: d2t/do2
    The numbers indicate the index of the quantity in fittingInfo. Proceeding
    to numerically check that these quantities are correct.
  */
  //Make sure the size of fittingInfo is at least large enough for .
  // 10 :Hessian with x = [a,v,p,o] for joint torque related constraints
  //  3 :Hessian for constraints on the passive-torque-angle multiplier
  // 10 :Hessian with x = [s,m,p,o] for joint torque related constraints
  CHECK(tmi.fittingInfo.rows() >= 23);

  //For joint-torque related constrains that use x = [a,v,p,o]
  //(see the code for the parts of calcTorqueMuscleInfo that evaluate
  //the second derivatives for further detail.
  //Test the first column of the Hessian
  //0: D^2 tau / D lambdaTa^2
  tq.setPassiveTorqueAngleCurveBlendingVariable(    tpLambda);
  tq.setTorqueAngularVelocityCurveBlendingVariable( tvLambda);
  tq.setActiveTorqueAngleCurveBlendingVariable(     taLambda);

  tq.calcTorqueMuscleInfo(jointAngleTpMid,omegaMid,1.0,tmi);
  tq.setActiveTorqueAngleCurveBlendingVariable(     taLambda-h);
  tq.calcTorqueMuscleInfo(jointAngleTpMid,omegaMid,1.0,tmiL);
  tq.setActiveTorqueAngleCurveBlendingVariable(     taLambda+h);
  tq.calcTorqueMuscleInfo(jointAngleTpMid,omegaMid,1.0,tmiR);

  double D2tau_DtaLambda2 =
      (tmiR.DjointTorque_DactiveTorqueAngleBlendingVariable
     - tmiL.DjointTorque_DactiveTorqueAngleBlendingVariable)/(2.0*h);

  err = fabs(D2tau_DtaLambda2
             - tmi.fittingInfo[0]);
  CHECK(err < SQRTEPSILON*100);


  //1: Test D^2 tau / D TaLambda D TvLambda
  double D2tau_DtaLambdaDtvLambda =
      (tmiR.DjointTorque_DtorqueAngularVelocityBlendingVariable
     - tmiL.DjointTorque_DtorqueAngularVelocityBlendingVariable)/(2.0*h);

  err = fabs(D2tau_DtaLambdaDtvLambda
            -tmi.fittingInfo[1]);
  CHECK(err < SQRTEPSILON*100);

  //3: Test D^2 tau/ D TaLambda D TpLambda
  double D2tau_DtaLambdaDtpLambda =
      (tmiR.DjointTorque_DpassiveTorqueAngleBlendingVariable
      -tmiL.DjointTorque_DpassiveTorqueAngleBlendingVariable)/(2.0*h);

  err = fabs(D2tau_DtaLambdaDtpLambda
             -tmi.fittingInfo[3]);
  CHECK(err < SQRTEPSILON*100);

  //6: Test D^2 tau/ D TaLambda DTpOffset
  double D2tau_DtaLambdaDtpOffset =
      (tmiR.DjointTorque_DpassiveTorqueAngleCurveAngleOffset
      -tmiL.DjointTorque_DpassiveTorqueAngleCurveAngleOffset)/(2.0*h);

  err = fabs(D2tau_DtaLambdaDtpOffset -
              tmi.fittingInfo[6]);

  //Check the 2nd column of the Hessian
  //2: Test D^2 tau/ D Tvlambda^2
  tq.setActiveTorqueAngleCurveBlendingVariable(     taLambda);

  tq.calcTorqueMuscleInfo(jointAngleTpMid,omegaMid,1.0,tmi);
  tq.setTorqueAngularVelocityCurveBlendingVariable( tvLambda - h);
  tq.calcTorqueMuscleInfo(jointAngleTpMid,omegaMid,1.0,tmiL);
  tq.setTorqueAngularVelocityCurveBlendingVariable( tvLambda + h);
  tq.calcTorqueMuscleInfo(jointAngleTpMid,omegaMid,1.0,tmiR);

  double D2tau_DtvLambda2 =
      (tmiR.DjointTorque_DtorqueAngularVelocityBlendingVariable
     - tmiL.DjointTorque_DtorqueAngularVelocityBlendingVariable)/(2.0*h);

  err = fabs(D2tau_DtvLambda2
             - tmi.fittingInfo[2]);
  CHECK(err < SQRTEPSILON*100);

  //4: Test D^2 tau/ D TvLambda D Tplambda
  double D2tau_DtvLambdaDtpLambda =
      (tmiR.DjointTorque_DpassiveTorqueAngleBlendingVariable
     - tmiL.DjointTorque_DpassiveTorqueAngleBlendingVariable)/(2.0*h);

  err = fabs(D2tau_DtvLambdaDtpLambda
             - tmi.fittingInfo[4]);
  CHECK(err < SQRTEPSILON*100);

  //7: Test D^2 tau/ D TvLambda D TpOffset
  double D2tau_DtvLambdaDtpOffset =
      (tmiR.DjointTorque_DpassiveTorqueAngleCurveAngleOffset
     - tmiL.DjointTorque_DpassiveTorqueAngleCurveAngleOffset)/(2.0*h);

  err = fabs(D2tau_DtvLambdaDtpOffset
             - tmi.fittingInfo[7]);
  CHECK(err < SQRTEPSILON*100);

  //Check the 3rd column of the Hessian.
  //5: D^2 tau / D TpLambda^2
  tq.setTorqueAngularVelocityCurveBlendingVariable(tvLambda);

  tq.calcTorqueMuscleInfo(jointAngleTpMid,omegaMid,1.0,tmi);
  tq.setPassiveTorqueAngleCurveBlendingVariable( tpLambda - h);
  tq.calcTorqueMuscleInfo(jointAngleTpMid,omegaMid,1.0,tmiL);
  tq.setPassiveTorqueAngleCurveBlendingVariable( tpLambda + h);
  tq.calcTorqueMuscleInfo(jointAngleTpMid,omegaMid,1.0,tmiR);

  double D2tau_DtpLambda2 =
      (tmiR.DjointTorque_DpassiveTorqueAngleBlendingVariable
     - tmiL.DjointTorque_DpassiveTorqueAngleBlendingVariable)/(2.0*h);

  err = fabs(D2tau_DtpLambda2
             - tmi.fittingInfo[5]);
  CHECK(err < SQRTEPSILON*100);

  //8: D^2 tau/ D TpLambda DTpOffset
  double D2tau_DtpLambdaDtpOffset =
      (tmiR.DjointTorque_DpassiveTorqueAngleCurveAngleOffset
     - tmiL.DjointTorque_DpassiveTorqueAngleCurveAngleOffset)/(2.0*h);

  err = fabs(D2tau_DtpLambdaDtpOffset
             - tmi.fittingInfo[8]);
  CHECK(err < SQRTEPSILON*100);

  //Check the 4th column of the Hessian.
  //9: D^2 tau/ D TpOffset^2
  tq.setTorqueAngularVelocityCurveBlendingVariable(tvLambda);

  tq.setPassiveCurveAngleOffset(0.);
  tq.calcTorqueMuscleInfo(jointAngleTpMid,omegaMid,1.0,tmi);
  tq.setPassiveCurveAngleOffset(-h);
  tq.calcTorqueMuscleInfo(jointAngleTpMid,omegaMid,1.0,tmiL);
  tq.setPassiveCurveAngleOffset( h);
  tq.calcTorqueMuscleInfo(jointAngleTpMid,omegaMid,1.0,tmiR);

  double D2tau_DtpOffset2 =
      (tmiR.DjointTorque_DpassiveTorqueAngleCurveAngleOffset
     - tmiL.DjointTorque_DpassiveTorqueAngleCurveAngleOffset)/(2.0*h);

  err = fabs(D2tau_DtpOffset2
             - tmi.fittingInfo[9]);
  CHECK(err < SQRTEPSILON*1000);


//  For constraints on the value of the passive element
//   10: d2p/dp2
  tq.setPassiveTorqueAngleCurveBlendingVariable(tpLambda);
  tq.calcTorqueMuscleInfo(jointAngleTpMid,omegaMid,1.0,tmi);
  tq.setPassiveTorqueAngleCurveBlendingVariable(tpLambda-h);
  tq.calcTorqueMuscleInfo(jointAngleTpMid,omegaMid,1.0,tmiL);
  tq.setPassiveTorqueAngleCurveBlendingVariable(tpLambda+h);
  tq.calcTorqueMuscleInfo(jointAngleTpMid,omegaMid,1.0,tmiR);

  double D2tp_DtpLambda2 =
      (tmiR.DfiberPassiveTorqueAngleMultiplier_DblendingVariable
     - tmiL.DfiberPassiveTorqueAngleMultiplier_DblendingVariable)/(2.0*h);

  err = fabs(D2tp_DtpLambda2
             - tmi.fittingInfo[10]);
  CHECK(err < SQRTEPSILON*1000);

  //   11: d2p/dpdo
  double D2tp_DtpLambda_DangleOffset =
      (tmiR.DfiberPassiveTorqueAngleMultiplier_DangleOffset
      -tmiL.DfiberPassiveTorqueAngleMultiplier_DangleOffset)/(2.0*h);

  err = fabs(D2tp_DtpLambda_DangleOffset
             -tmi.fittingInfo[11]);
  CHECK(err < SQRTEPSILON*1000);

//   12: d2p/do2
  tq.setPassiveTorqueAngleCurveBlendingVariable(tpLambda);
  tq.setPassiveCurveAngleOffset(0.);
  tq.calcTorqueMuscleInfo(jointAngleTpMid,omegaMid,1.0,tmi);
  tq.setPassiveCurveAngleOffset(-h);
  tq.calcTorqueMuscleInfo(jointAngleTpMid,omegaMid,1.0,tmiL);
  tq.setPassiveCurveAngleOffset( h);
  tq.calcTorqueMuscleInfo(jointAngleTpMid,omegaMid,1.0,tmiR);

  double D2tp_DtpOffset2 =
      (tmiR.DfiberPassiveTorqueAngleMultiplier_DangleOffset
     - tmiL.DfiberPassiveTorqueAngleMultiplier_DangleOffset)/(2.0*h);

  err = fabs(D2tp_DtpOffset2
             - tmi.fittingInfo[12]);
  CHECK(err < SQRTEPSILON*1000);


//  For joint-torque related constraints that use x = [s,m,p,o]
//   13: d2t/ds2
  tq.setPassiveTorqueAngleCurveBlendingVariable(    0.);
  tq.setTorqueAngularVelocityCurveBlendingVariable( 0.);
  tq.setActiveTorqueAngleCurveBlendingVariable(     0.);

  jointAngleAtOneNormTorque = tq.getJointAngleAtMaximumActiveIsometricTorque();

  tq.setActiveTorqueAngleCurveAngleScaling(1.0);
  tq.calcTorqueMuscleInfo(jointAngleAtOneNormTorque+0.5,omegaMid,1.0,tmi);
  tq.setActiveTorqueAngleCurveAngleScaling(1.0-h);
  tq.calcTorqueMuscleInfo(jointAngleAtOneNormTorque+0.5,omegaMid,1.0,tmiL);
  tq.setActiveTorqueAngleCurveAngleScaling(1.0+h);
  tq.calcTorqueMuscleInfo(jointAngleAtOneNormTorque+0.5,omegaMid,1.0,tmiR);

  double D2tau_DtaAngleScaling2 =
      (tmiR.DjointTorque_DactiveTorqueAngleAngleScaling
     - tmiL.DjointTorque_DactiveTorqueAngleAngleScaling)/(2.0*h);

  err = fabs(D2tau_DtaAngleScaling2
             - tmi.fittingInfo[13]);
  CHECK(err < SQRTEPSILON*1000);

//   14: d2t/dsdm
  double D2tau_DtaAngleScaling_DomegaMax =
      (tmiR.DjointTorque_DmaximumAngularVelocity
     - tmiL.DjointTorque_DmaximumAngularVelocity)/(2.0*h);

  err = fabs(D2tau_DtaAngleScaling_DomegaMax
             - tmi.fittingInfo[14]);
  CHECK(err < SQRTEPSILON*100);

  //   16: d2t/dsdp
  double D2tau_DtaAngleScaling_DtpLambda =
      (tmiR.DjointTorque_DpassiveTorqueAngleBlendingVariable
      -tmiL.DjointTorque_DpassiveTorqueAngleBlendingVariable)/(2.0*h);

  err = fabs(D2tau_DtaAngleScaling_DtpLambda
            -tmi.fittingInfo[16]);
  CHECK(err < SQRTEPSILON*100);
  //   19: d2t/dsdo
  double D2tau_DtaAngleScaling_DtpOffset =
      (tmiR.DjointTorque_DpassiveTorqueAngleCurveAngleOffset
      -tmiL.DjointTorque_DpassiveTorqueAngleCurveAngleOffset)/(2.0*h);

  err = fabs(D2tau_DtaAngleScaling_DtpOffset
            -tmi.fittingInfo[19]);
  CHECK(err < SQRTEPSILON*100);

//   15: d2t/dm2
  tq.setActiveTorqueAngleCurveAngleScaling(1.0);
  omegaMax = tq.getMaximumConcentricJointAngularVelocity();
  omegaMid = 0.5*omegaMax;

  tq.setMaximumConcentricJointAngularVelocity(omegaMax);
  tq.calcTorqueMuscleInfo(jointAngleAtOneNormTorque,omegaMid,1.0,tmi);
  tq.setMaximumConcentricJointAngularVelocity(omegaMax-h);
  tq.calcTorqueMuscleInfo(jointAngleAtOneNormTorque,omegaMid,1.0,tmiL);
  tq.setMaximumConcentricJointAngularVelocity(omegaMax+h);
  tq.calcTorqueMuscleInfo(jointAngleAtOneNormTorque,omegaMid,1.0,tmiR);

  double D2tau_DomegaMax2 =
      (tmiR.DjointTorque_DmaximumAngularVelocity
     - tmiL.DjointTorque_DmaximumAngularVelocity)/(2.0*h);

  err = fabs(D2tau_DomegaMax2
             - tmi.fittingInfo[15]);
  CHECK(err < SQRTEPSILON*100);

  //   17: d2t/dmdp
  double D2tau_DomegaMax_DtpLambda =
      (tmiR.DjointTorque_DpassiveTorqueAngleBlendingVariable
      -tmiL.DjointTorque_DpassiveTorqueAngleBlendingVariable)/(2.0*h);
  err = fabs(D2tau_DomegaMax_DtpLambda
             - tmi.fittingInfo[17]);
  CHECK(err < SQRTEPSILON*100);

  //   20: d2t/dmdo
  double D2tau_DomegaMax_DtpOffset =
      (tmiR.DjointTorque_DpassiveTorqueAngleCurveAngleOffset
      -tmiL.DjointTorque_DpassiveTorqueAngleCurveAngleOffset)/(2.0*h);
  err = fabs(D2tau_DomegaMax_DtpOffset
             - tmi.fittingInfo[20]);
  CHECK(err < SQRTEPSILON*100);

//   18: d2t/dp2
  tq.setActiveTorqueAngleCurveBlendingVariable(taLambda);
  tq.setPassiveTorqueAngleCurveBlendingVariable(tpLambda);
  tq.setTorqueAngularVelocityCurveBlendingVariable(tvLambda);
  tq.setPassiveCurveAngleOffset(0.0);

  tq.setActiveTorqueAngleCurveAngleScaling(1.0);
  tq.setMaximumConcentricJointAngularVelocity(omegaMax);


  tq.calcTorqueMuscleInfo(jointAngleTpMid,omegaMid,1.0,tmi);
  tq.setPassiveTorqueAngleCurveBlendingVariable( tpLambda - h);
  tq.calcTorqueMuscleInfo(jointAngleTpMid,omegaMid,1.0,tmiL);
  tq.setPassiveTorqueAngleCurveBlendingVariable( tpLambda + h);
  tq.calcTorqueMuscleInfo(jointAngleTpMid,omegaMid,1.0,tmiR);

  D2tau_DtpLambda2 =
      (tmiR.DjointTorque_DpassiveTorqueAngleBlendingVariable
     - tmiL.DjointTorque_DpassiveTorqueAngleBlendingVariable)/(2.0*h);

  err = fabs(D2tau_DtpLambda2
             - tmi.fittingInfo[18]);
  CHECK(err < SQRTEPSILON*100);

  //   21: d2t/dpdo

  D2tau_DtpLambdaDtpOffset =
      (tmiR.DjointTorque_DpassiveTorqueAngleCurveAngleOffset
     - tmiL.DjointTorque_DpassiveTorqueAngleCurveAngleOffset)/(2.0*h);

  err = fabs(D2tau_DtpLambdaDtpOffset
             - tmi.fittingInfo[21]);
  CHECK(err < SQRTEPSILON*100);


  //   22: d2t/do2
  tq.setPassiveTorqueAngleCurveBlendingVariable(tpLambda);
  tq.setPassiveCurveAngleOffset(0.);

  tq.calcTorqueMuscleInfo(jointAngleTpMid,omegaMid,1.0,tmi);
  tq.setPassiveCurveAngleOffset(-h);
  tq.calcTorqueMuscleInfo(jointAngleTpMid,omegaMid,1.0,tmiL);
  tq.setPassiveCurveAngleOffset( h);
  tq.calcTorqueMuscleInfo(jointAngleTpMid,omegaMid,1.0,tmiR);

  D2tau_DtpOffset2 =
      (tmiR.DjointTorque_DpassiveTorqueAngleCurveAngleOffset
     - tmiL.DjointTorque_DpassiveTorqueAngleCurveAngleOffset)/(2.0*h);

  err = fabs(D2tau_DtpOffset2
             - tmi.fittingInfo[22]);
  CHECK(err < SQRTEPSILON*1000);



#endif

}


#ifdef RBDL_BUILD_ADDON_MUSCLE_FITTING

TEST_CASE(__FILE__"_fittingEasyTest", ""){

  double jointAngleOffset     = 0;
  double signOfJointAngle     = 1;
  double signOfJointTorque    = 1;
  double signOfJointVelocity  = signOfJointTorque;

  std::string name("test");

  SubjectInformation subjectInfo;
    subjectInfo.gender          = GenderSet::Male;
    subjectInfo.ageGroup        = AgeGroupSet::Young18To25;
    subjectInfo.heightInMeters  =  1.732;
    subjectInfo.massInKg        = 69.0;

  Millard2016TorqueMuscle tq =
          Millard2016TorqueMuscle(DataSet::Gymnast,
                                   subjectInfo,
                                   Gymnast::HipExtension,
                                   jointAngleOffset,
                                   signOfJointAngle,
                                   signOfJointTorque,
                                   name);

//   Testing approach
//    1. Generate a data set consistent with a muscle that is more flexible,
//       is stronger, and has modified curves from the default.
//    2. Return the muscle back to its default values of flexibility, maximum
//       isometric torque and blending variables.
//    3. Run the fitting algorithm to see if the parameters from step 1 can be
//       identified.


  double tisoOriginal = tq.getMaximumActiveIsometricTorque();
  double tisoUpd      = 1.2*tisoOriginal;
  double omegaMaxOrig = tq.getMaximumConcentricJointAngularVelocity();
  double omegaMaxUpd  = 2.5*omegaMaxOrig;

  double taAngleScalingOrig = tq.getActiveTorqueAngleCurveAngleScaling();
  double taAngleScalingUpd  = taAngleScalingOrig*1.35;

  double taLambda = 0.0;
  double tpLambda = 0.0;
  double tvLambda = 0.0;


  double angleToOneOrig =
        tq.getJointAngleAtOneNormalizedPassiveIsometricTorque();

  tq.setActiveTorqueAngleCurveBlendingVariable(taLambda);
  tq.setPassiveTorqueAngleCurveBlendingVariable(tpLambda);
  tq.setTorqueAngularVelocityCurveBlendingVariable(tvLambda);
  tq.setMaximumActiveIsometricTorque(tisoUpd);
  tq.setMaximumConcentricJointAngularVelocity(omegaMaxUpd);
  tq.setActiveTorqueAngleCurveAngleScaling(taAngleScalingUpd);

  unsigned int npts = 100;
  RigidBodyDynamics::Math::VectorNd angle, angularVelocity, torque;
  angle.resize(npts);
  angularVelocity.resize(npts);
  torque.resize(npts);

  //Generate the data set.
  double activationUpperBound = 0.95;
  double passiveTorqueAngleMultiplierUpperBound = 0.;
  TorqueMuscleInfo tmi;
  TorqueMuscleSummary tms;

  double angularRange  = M_PI/3.0;
  double angleTpOne =
      tq.getJointAngleAtOneNormalizedPassiveIsometricTorque();
  double angularOffset = angleTpOne-angularRange;
  double time = 0.;
  double normTime = 0.;
  double omega = omegaMaxUpd/angularRange;
  for(unsigned int i = 0; i<npts;++i){
    normTime  = (double)(i)/ (double)(npts-1);
    time      = normTime*2.0*M_PI/omega;

    angle[i]           = angularOffset + angularRange*sin(omega*time);
    angularVelocity[i] = angularRange*omega*cos(omega*time);
    tq.calcTorqueMuscleInfo(angle[i],angularVelocity[i],
                            activationUpperBound, tmi);

    torque[i]          = tmi.jointTorque;
    if(tmi.fiberPassiveTorqueAngleMultiplier
        > passiveTorqueAngleMultiplierUpperBound){
      passiveTorqueAngleMultiplierUpperBound =
           tmi.fiberPassiveTorqueAngleMultiplier;
    }
  }
  //Return the muscle back to its defaults.
  tq.setActiveTorqueAngleCurveBlendingVariable(0.0);
  tq.setPassiveTorqueAngleCurveBlendingVariable(0.0);
  tq.setTorqueAngularVelocityCurveBlendingVariable(0.0);
  tq.setMaximumActiveIsometricTorque(tisoOriginal);
  tq.setMaximumConcentricJointAngularVelocity(omegaMaxOrig);
  tq.setActiveTorqueAngleCurveAngleScaling(taAngleScalingOrig);

  //Run the fitting routine.
  passiveTorqueAngleMultiplierUpperBound *= 0.75;
  TorqueMuscleParameterFittingData tmFittingData;


  TorqueMuscleFittingToolkit::fitTorqueMuscleParameters(  
                                tq, angle,angularVelocity,torque,
                                activationUpperBound,
                                passiveTorqueAngleMultiplierUpperBound,
                                tmFittingData, false);
  //Now to test the result, update the parameters of the
  //model and run through all of the test vectors. The muscle
  //should produce a torque that is >= the desired torque
  //of the same sign. There should be one value where it
  //,to numerical precision produces just the required torque
  //and no more.

  CHECK(tmFittingData.fittingConverged == true);


  tq.setFittedParameters(tmFittingData);

  double minActivation = 1.e20;
  double maxActivation = -1.e20;
  double maxPassiveTorqueAngleMultiplier = -1.0e20;
  for(unsigned int i=0; i<angle.rows();++i){

    if(torque[i]*tq.getJointTorqueSign() > 0){
      tq.calcActivation(angle[i],angularVelocity[i],torque[i],tms);

      if(tms.activation < minActivation){
        minActivation = tms.activation;
      }
      if(tms.activation > maxActivation){
        maxActivation = tms.activation;
      }

      if(tms.fiberPassiveTorqueAngleMultiplier
          > maxPassiveTorqueAngleMultiplier){
        maxPassiveTorqueAngleMultiplier = tms.fiberPassiveTorqueAngleMultiplier;
      }
    }
  }
  double err = maxActivation-activationUpperBound;
  CHECK(err <= 10.0*SQRTEPSILON);

  err = minActivation;
  CHECK(err >= -10.0*SQRTEPSILON);

  err = (maxPassiveTorqueAngleMultiplier
           - passiveTorqueAngleMultiplierUpperBound);
  CHECK(err < SQRTEPSILON);

}

TEST_CASE(__FILE__"_fittingHardTest", "")
{

  std::string name("hardTest");

  SubjectInformation subjectInfo;
    subjectInfo.gender          = GenderSet::Male;
    subjectInfo.ageGroup        = AgeGroupSet::Young18To25;
    subjectInfo.heightInMeters  =  1.732;
    subjectInfo.massInKg        = 69.0;

  double angleOffset = 0.;
  double angleSign =  1;
  double torqueSign =-1;

  Millard2016TorqueMuscle kneeExt(DataSet::Gymnast,
                                 subjectInfo,
                                 Gymnast::KneeExtension,
                                 angleOffset,
                                 angleSign,
                                 torqueSign,
                                 name);

  RigidBodyDynamics::Math::VectorNd q, qDot, tau;
  q.resize(   TorqueMuscleFittingHardTestCaseRows);
  qDot.resize(TorqueMuscleFittingHardTestCaseRows);
  tau.resize( TorqueMuscleFittingHardTestCaseRows);

  double tauMax = 0;
  double omegaMax = 0;

  for(unsigned int i=0; i<TorqueMuscleFittingHardTestCaseRows;++i){
    q[i]    = TorqueMuscleFittingHardTestCase[i][1];
    qDot[i] = TorqueMuscleFittingHardTestCase[i][2];
    tau[i]  = TorqueMuscleFittingHardTestCase[i][3];
    
    if(fabs(qDot[i])>omegaMax){
      omegaMax = fabs(qDot[i]);
    }
    if(fabs(tau[i])>tauMax){
      tauMax = fabs(tau[i]);
    }
  }


  double activationUB = 0.9;
  double tpUB         = 0.75;
  bool verbose = false;


  TorqueMuscleParameterFittingData fittingData;
  TorqueMuscleFittingToolkit::fitTorqueMuscleParameters(kneeExt,q,qDot,tau,
                                                        activationUB,tpUB,
                                                        fittingData,verbose);
  CHECK(fittingData.fittingConverged == true);
}

#endif


TEST_CASE(__FILE__"_exampleUsage", ""){


  bool printCurves = false;
  bool printAllCurves = false;

  //int dataSet = DataSetAnderson2007;

  //int gender  = 0; //male
  //int age     = 0; //young

  double jointAngleOffset     = 0;
  double signOfJointAngle     = 1;
  double signOfJointTorque    = 1;

  std::string name("test");

  SubjectInformation subjectInfo;
    subjectInfo.gender          = GenderSet::Male;
    subjectInfo.ageGroup        = AgeGroupSet::Young18To25;
    subjectInfo.heightInMeters  =  1.732;
    subjectInfo.massInKg        = 69.0;

  std::vector< Millard2016TorqueMuscle > muscleVector;

  bool exception = false;

  double angleTorqueSigns[][2] = {{-1, 1},
                                  {-1,-1},
                                  { 1,-1},
                                  { 1, 1},
                                  {-1, 1},
                                  {-1,-1}};

  Millard2016TorqueMuscle tqMuscle;
  std::stringstream tqName;
  int tqIdx;

  for(int i=0; i < Anderson2007::LastJointTorque; ++i){

      tqName.str("");
      tqName << DataSet.names[0]
             <<Anderson2007::JointTorqueNames[i];

      tqMuscle = Millard2016TorqueMuscle(
                    DataSet::Anderson2007,
                    subjectInfo,
                    Anderson2007::JointTorque(i),
                    0.0,
                    1.0,
                    1.0,
                    tqName.str() );


      if(printCurves)
          tqMuscle.printJointTorqueProfileToFile("",tqMuscle.getName() ,100);
  }

  for(int i=0; i < Gymnast::LastJointTorque; ++i){

      tqName.str("");
      tqName << DataSet.names[1]
             << Gymnast::JointTorqueNames[i];

      tqMuscle = Millard2016TorqueMuscle(
                    DataSet::Gymnast,
                    subjectInfo,
                    Gymnast::JointTorque(i),
                    0.0,
                    1.0,
                    1.0,
                    tqName.str() );


      if(printCurves)
          tqMuscle.printJointTorqueProfileToFile("",tqMuscle.getName() ,100);
  }

  tqIdx = -1;


  if(printAllCurves){
      std::stringstream muscleName;

      Millard2016TorqueMuscle muscle;
      int genderIdx,ageIdx,tqIdx;

      for(int age =0; age < Anderson2007::LastAgeGroup; ++age){
          for(int gender=0; gender < Anderson2007::LastGender; ++gender){
            for( int tqDir = 0; tqDir < Anderson2007::LastJointTorque; ++tqDir){
              //for(int joint=0; joint < 3; ++joint){
              //   for(int dir = 0; dir < 2; ++dir){
                      muscleName.str(std::string());

                      genderIdx = Anderson2007::Gender(gender);
                      ageIdx    = Anderson2007::AgeGroup(age);
                      tqIdx     = Anderson2007::JointTorque(tqDir);


                      muscleName  << "Anderson2007_"
                                  << AgeGroupSet::names[ageIdx]
                                  << "_"
                                  << GenderSet::names[genderIdx]
                                  << "_"
                                  << JointTorqueSet::names[tqIdx];

                      subjectInfo.ageGroup = AgeGroupSet::item(age);
                      subjectInfo.gender   = GenderSet::item(gender);

                       muscle = Millard2016TorqueMuscle(
                                 DataSet::Anderson2007,
                                 subjectInfo,
                                 Anderson2007::JointTorque(tqDir),
                                 0,
                                 1.0,
                                 1.0,
                                 muscleName.str());

                      const SmoothSegmentedFunction &tp
                          = muscle.getPassiveTorqueAngleCurve();
                      const SmoothSegmentedFunction &ta
                          = muscle.getActiveTorqueAngleCurve();
                      const SmoothSegmentedFunction &tv
                          = muscle.getTorqueAngularVelocityCurve();

                      RigidBodyDynamics::Math::VectorNd tpDomain
                          = tp.getCurveDomain();
                      RigidBodyDynamics::Math::VectorNd tvDomain
                          = tv.getCurveDomain();
                      RigidBodyDynamics::Math::VectorNd taDomain
                          = ta.getCurveDomain();



                      tp.printCurveToCSVFile(
                        "",
                        tp.getName(),
                        tpDomain[0]-0.1*(tpDomain[1]-tpDomain[0]),
                        tpDomain[1]+0.1*(tpDomain[1]-tpDomain[0]));
                      tv.printCurveToCSVFile(
                        "",
                        tv.getName(),
                        tvDomain[0]-0.1*(tvDomain[1]-tvDomain[0]),
                        tvDomain[1]+0.1*(tvDomain[1]-tvDomain[0]));
                      ta.printCurveToCSVFile(
                        "",
                        ta.getName(),
                        taDomain[0]-0.1*(taDomain[1]-taDomain[0]),
                        taDomain[1]+0.1*(taDomain[1]-taDomain[0]));

                  //}
              //}
            }
          }
      }


  }

   //catch(...){
   //     exceptionThrown = true;
   // }
   CHECK(true);




}
