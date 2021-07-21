//==============================================================================
/*
 * RBDL - Rigid Body Dynamics Library: Addon : muscle
 * Copyright (c) 2016 Matthew Millard <millard.matthew@gmail.com>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#include "Millard2016TorqueMuscle.h"
#include "TorqueMuscleFunctionFactory.h"
#include "csvtools.h"

#include <rbdl/rbdl_errors.h>


#include <limits>

#include <cmath>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <vector>

#include <ostream>


static double EPSILON = std::numeric_limits<double>::epsilon();
static double SQRTEPSILON = sqrt(EPSILON);


using namespace RigidBodyDynamics::Math;
using namespace RigidBodyDynamics::Addons::Muscle;
using namespace RigidBodyDynamics::Addons::Geometry;
using namespace std;

const double Millard2016TorqueMuscle::mTaLambdaMax = 1.0;
const double Millard2016TorqueMuscle::mTpLambdaMax = 0.0;
//static double mTvLambdaMax = 1.0;


/*************************************************************
 Table Access Structure Names
*************************************************************/
const double Gravity = 9.81; 
//Needed for the strength scaling used
//by Anderson et al. And this value has to
//be equal to the gravity parameter used by
//Anderson et al.: it should not be changed if
//the force of gravity differs in the model!

const char* DataSet::names[] = {  "Anderson2007",
                                  "Gymnast"
                               };

const char* GenderSet::names[] = {"Male",
                                  "Female"
                                 };

const char* AgeGroupSet::names[] = {"Young18To25",
                                    "Middle55To65",
                                    "SeniorOver65"
                                   };

const char* JointTorqueSet::names[] = { "HipExtension",
                                        "HipFlexion",
                                        "KneeExtension",
                                        "KneeFlexion",
                                        "AnkleExtension",
                                        "AnkleFlexion",
                                        "ElbowExtension",
                                        "ElbowFlexion",
                                        "ShoulderExtension",
                                        "ShoulderFlexion",
                                        "WristExtension",
                                        "WristFlexion",
                                        "ShoulderHorizontalAdduction",
                                        "ShoulderHorizontalAbduction",
                                        "ShoulderInternalRotation",
                                        "ShoulderExternalRotation",
                                        "WristUlnarDeviation",
                                        "WristRadialDeviation",
                                        "WristPronation",
                                        "WristSupination",
                                        "LumbarExtension",
                                        "LumbarFlexion",
                                        "UnitExtensor",
                                        "UnitFlexor"
                                      };

const char* Anderson2007::GenderNames[] = {"Male","Female"};

const char* Anderson2007::AgeGroupNames[] = { "Young18To25",
                                              "Middle55To65",
                                              "SeniorOver65"
                                            };

const char* Anderson2007::JointTorqueNames[] = {"HipExtension",
                                                "HipFlexion",
                                                "KneeExtension",
                                                "KneeFlexion",
                                                "AnkleExtension",
                                                "AnkleFlexion"
                                               };

const char* Gymnast::GenderNames[] = {"Male"};
const char* Gymnast::AgeGroupNames[] = {"Young18To25"};
const char* Gymnast::JointTorqueNames[] = {
  "HipExtension",
  "HipFlexion",
  "KneeExtension",
  "KneeFlexion",
  "AnkleExtension",
  "AnkleFlexion",
  "ElbowExtension",
  "ElbowFlexion",
  "ShoulderExtension",
  "ShoulderFlexion",
  "WristExtension",
  "WristFlexion",
  "ShoulderHorizontalAdduction",
  "ShoulderHorizontalAbduction",
  "ShoulderInternalRotation",
  "ShoulderExternalRotation",
  "WristUlnarDeviation",
  "WristRadialDeviation",
  "WristPronation",
  "WristSupination",
  "LumbarExtension",
  "LumbarFlexion",
  "UnitExtensor",
  "UnitFlexor"
};

/*************************************************************
 Coefficient Tables
*************************************************************/

/*
This data is taken from Table 3 of

Anderson, D. E., Madigan, M. L., & Nussbaum, M. A. (2007).
Maximum voluntary joint torque as a function of joint angle
and angular velocity: model development and application to
the lower limb. Journal of biomechanics, 40(14), 3105-3113.

Each row contains the coefficients for the active and
passive torque characteristics for a specific joint,
direction, gender and age group. Each row corresponds
to a single block taken from Table 3, as read from
left to right top to bottom. The first 4 columns have
been added to describe the joint, direction, gender
and age group.

Column labels:
Parameter Set Meta Data
  0: joint:     hip0_knee1_ankle2,
  1: direction: ext0_flex1,
  2: gender:    male0_female1,
  3: age:       age18to25_0_55to65_1_g65_2,

Active Torque-Angle and Torque-Velocity Curves
  4:  c1,
  5:  c2,
  6:  c3,
  7:  c4,
  8:  c5,
  9:  c6,
Passive Torque-Angle Curves
  10: b1,
  11: k1,
  12: b2,
  13: k2,
*/
double const Millard2016TorqueMuscle::Anderson2007Table3Mean[36][14] = {
  {0,0,0,0,0.161,0.958,0.932,1.578,3.19,0.242,-1.21,-6.351,0.476,5.91         },
  {0,0,1,0,0.181,0.697,1.242,1.567,3.164,0.164,-1.753,-6.358,0.239,3.872      },
  {0,0,0,1,0.171,0.922,1.176,1.601,3.236,0.32,-2.16,-8.073,0.108,4.593        },
  {0,0,1,1,0.14,0.83,1.241,1.444,2.919,0.317,-1.361,-7.128,0.013,6.479        },
  {0,0,0,2,0.144,0.896,1.125,1.561,3.152,0.477,-2.671,-7.85,0.092,5.192       },
  {0,0,1,2,0.138,0.707,1.542,1.613,3.256,0.36,-0.758,-7.545,0.018,6.061       },
  {0,1,0,0,0.113,0.738,-0.214,2.095,4.267,0.218,1.21,-6.351,-0.476,5.91       },
  {0,1,1,0,0.127,0.65,-0.35,2.136,4.349,0.156,1.753,-6.358,-0.239,3.872       },
  {0,1,0,1,0.107,0.712,-0.192,2.038,4.145,0.206,2.16,-8.073,-0.108,4.593      },
  {0,1,1,1,0.091,0.812,-0.196,2.145,4.366,0.186,1.361,-7.128,-0.013,6.479     },
  {0,1,0,2,0.101,0.762,-0.269,1.875,3.819,0.296,2.671,-7.85,-0.092,5.192      },
  {0,1,1,2,0.081,0.625,-0.422,2.084,4.245,0.196,0.758,-7.545,-0.018,6.061     },
  {1,0,0,0,0.163,1.258,1.133,1.517,3.952,0.095,0,0,-6.25,-4.521               },
  {1,0,1,0,0.159,1.187,1.274,1.393,3.623,0.173,0,0,-8.033,-5.25               },
  {1,0,0,1,0.156,1.225,1.173,1.518,3.954,0.266,0,0,-12.83,-5.127              },
  {1,0,1,1,0.128,1.286,1.141,1.332,3.469,0.233,0,0,-6.576,-4.466              },
  {1,0,0,2,0.137,1.31,1.067,1.141,3.152,0.386,0,0,-10.519,-5.662              },
  {1,0,1,2,0.124,1.347,1.14,1.066,2.855,0.464,0,0,-8.8,-6.763                 },
  {1,1,0,0,0.087,0.869,0.522,2.008,5.233,0.304,0,0,6.25,-4.521                },
  {1,1,1,0,0.08,0.873,0.635,1.698,4.412,0.175,0,0,8.033,-5.25                 },
  {1,1,0,1,0.081,0.986,0.523,1.83,4.777,0.23,0,0,12.83,-5.127                 },
  {1,1,1,1,0.06,0.967,0.402,1.693,4.41,0.349,0,0,6.576,-4.466                 },
  {1,1,0,2,0.069,0.838,0.437,1.718,4.476,0.414,0,0,10.519,-5.662              },
  {1,1,1,2,0.06,0.897,0.445,1.121,2.922,0.389,0,0,8.8,-6.763                  },
  {2,0,0,0,0.095,1.391,0.408,0.987,3.558,0.295,-0.0005781,-5.819,0.967,6.09   },
  {2,0,1,0,0.104,1.399,0.424,0.862,3.109,0.189,-0.005218,-4.875,0.47,6.425    },
  {2,0,0,1,0.114,1.444,0.551,0.593,2.128,0.35,-0.001311,-10.943,0.377,8.916   },
  {2,0,1,1,0.093,1.504,0.381,0.86,3.126,0.349,-2.888e-05,-17.189,0.523,7.888  },
  {2,0,0,2,0.106,1.465,0.498,0.49,1.767,0.571,-5.693e-05,-21.088,0.488,7.309  },
  {2,0,1,2,0.125,1.299,0.58,0.587,1.819,0.348,-2.35e-05,-12.567,0.331,6.629   },
  {2,1,0,0,0.033,1.51,-0.187,0.699,1.94,0.828,0.0005781,-5.819,-0.967,6.09    },
  {2,1,1,0,0.027,1.079,-0.302,0.864,2.399,0.771,0.005218,-4.875,-0.47,6.425   },
  {2,1,0,1,0.028,1.293,-0.284,0.634,1.759,0.999,0.001311,-10.943,-0.377,8.916 },
  {2,1,1,1,0.024,1.308,-0.254,0.596,1.654,1.006,2.888e-05,-17.189,-0.523,7.888},
  {2,1,0,2,0.029,1.419,-0.174,0.561,1.558,1.198,5.693e-05,-21.088,-0.488,7.309},
  {2,1,1,2,0.022,1.096,-0.369,0.458,1.242,1.401,2.35e-05,-12.567,-0.331,6.629 }
};


//See the description for the mean data. This table constains the
//parameter standard deviations
double const Millard2016TorqueMuscle::Anderson2007Table3Std[36][14] = {
  {0,0,0,0,0.049,0.201,0.358,0.286,0.586,0.272,0.66,0.97,0.547,4.955       },
  {0,0,1,0,0.047,0.13,0.418,0.268,0.542,0.175,1.93,2.828,0.292,1.895       },
  {0,0,0,1,0.043,0.155,0.195,0.306,0.622,0.189,1.297,2.701,0.091,0.854     },
  {0,0,1,1,0.032,0.246,0.365,0.223,0.45,0.14,1.294,2.541,0.02,2.924        },
  {0,0,0,2,0.039,0.124,0.077,0.184,0.372,0.368,0.271,3.402,0.111,1.691     },
  {0,0,1,2,0.003,0.173,0.279,0.135,0.273,0.237,0.613,0.741,0.031,2.265     },
  {0,1,0,0,0.025,0.217,0.245,0.489,0.995,0.225,0.66,0.97,0.547,4.955       },
  {0,1,1,0,0.033,0.178,0.232,0.345,0.702,0.179,1.93,2.828,0.292,1.895      },
  {0,1,0,1,0.02,0.248,0.274,0.318,0.652,0.088,1.297,2.701,0.091,0.854      },
  {0,1,1,1,0.016,0.244,0.209,0.375,0.765,0.262,1.294,2.541,0.02,2.924      },
  {0,1,0,2,0.025,0.151,0.234,0.164,0.335,0.102,0.271,3.402,0.111,1.691     },
  {0,1,1,2,0.008,0.062,0.214,0.321,0.654,0.28,0.613,0.741,0.031,2.265      },
  {1,0,0,0,0.04,0.073,0.073,0.593,1.546,0.171,0,0,2.617,0.553              },
  {1,0,1,0,0.028,0.084,0.181,0.38,0.989,0.27,0,0,3.696,1.512               },
  {1,0,0,1,0.031,0.063,0.048,0.363,0.947,0.06,0,0,2.541,2.148              },
  {1,0,1,1,0.016,0.094,0.077,0.319,0.832,0.133,0,0,1.958,1.63              },
  {1,0,0,2,0.017,0.127,0.024,0.046,0.04,0.124,0,0,1.896,1.517              },
  {1,0,1,2,0.018,0.044,0.124,0.128,0.221,0.129,0,0,6.141,0.742             },
  {1,1,0,0,0.015,0.163,0.317,1.364,3.554,0.598,0,0,2.617,0.553             },
  {1,1,1,0,0.015,0.191,0.287,0.825,2.139,0.319,0,0,3.696,1.512             },
  {1,1,0,1,0.017,0.138,0.212,0.795,2.067,0.094,0,0,2.541,2.148             },
  {1,1,1,1,0.015,0.21,0.273,0.718,1.871,0.143,0,0,1.958,1.63               },
  {1,1,0,2,0.022,0.084,0.357,0.716,1.866,0.201,0,0,1.896,1.517             },
  {1,1,1,2,0.005,0.145,0.21,0.052,0.135,0.078,0,0,6.141,0.742              },
  {2,0,0,0,0.022,0.089,0.083,0.595,2.144,0.214,0.001193,7.384,0.323,1.196  },
  {2,0,1,0,0.034,0.19,0.186,0.487,1.76,0.213,0.01135,6.77,0.328,1.177      },
  {2,0,0,1,0.029,0.136,0.103,0.165,0.578,0.133,0.003331,10.291,0.403,3.119 },
  {2,0,1,1,0.026,0.235,0.143,0.448,1.613,0.27,3.562e-05,7.848,0.394,1.141  },
  {2,0,0,2,0.035,0.136,0.132,0.262,0.944,0.313,3.164e-05,1.786,0.258,0.902 },
  {2,0,1,2,0.006,0.095,0.115,0.258,0.423,0.158,2.535e-05,10.885,0.247,2.186},
  {2,1,0,0,0.005,0.19,0.067,0.108,0.301,0.134,0.001193,7.384,0.323,1.196   },
  {2,1,1,0,0.006,0.271,0.171,0.446,1.236,0.206,0.01135,6.77,0.328,1.177    },
  {2,1,0,1,0.005,0.479,0.178,0.216,0.601,0.214,0.003331,10.291,0.403,3.119 },
  {2,1,1,1,0.002,0.339,0.133,0.148,0.41,0.284,3.562e-05,7.848,0.394,1.141  },
  {2,1,0,2,0.002,0.195,0.056,0.188,0.521,0.29,3.164e-05,1.786,0.258,0.902  },
  {2,1,1,2,0.003,0.297,0.109,0.089,0.213,0.427,2.535e-05,10.885,0.247,2.186}
};

/*
  This table contains parameters for the newly made torque muscle curves:

 1. maxIsometricTorque              Nm
 2. maxAngularVelocity              rad/s
 3. angleAtOneActiveNormTorque      rad
 4. angularWidthActiveTorque        rad
 5. tvAtMaxEccentricVelocity        Nm/Nm
 6. tvAtHalfMaxConcentricVelocity   Nm/Nm
 7. angleAtZeroPassiveTorque        rad
 8. mAngleAtOneNormPassiveTorque     rad

*/



double const Millard2016TorqueMuscle::GymnastWholeBody[24][12] = {
  {0,0,0,0,175.746, 9.02335,  1.06465,  1.05941,  1.1,     0.163849,  0.79158,   1.5708   },
  {0,1,0,0,157.293, 9.18043,  0.733038, 1.21999,  1.11905, 0.25,     -0.0888019,-0.515207 },
  {1,0,0,0,285.619, 19.2161,  0.942478, 0.509636, 1.13292, 0.115304,  2.00713,   2.70526  },
  {1,1,0,0,98.7579, 16.633,   1.02974,  1.11003,  1.12,    0.19746,   0,        -0.174533 },
  {2,0,0,0,127.561, 11.7646,  0.408,    0.660752, 1.159,   0.410591,  0.0292126, 0.785398 },
  {2,1,0,0,44.3106, 17.2746, -0.187,    0.60868,  1.2656,  0.112303, -0.523599, -1.0472   },
  {3,0,0,0,127.401, 16.249,   2.14675,  1.83085,  1.1,     0.250134,  2.8291,    3.55835  },
  {3,1,0,0,91.1388, 19.0764,  0.890118, 1.2898,   1.23011, 0.249656, -0.523599, -1.0472   },
  {5,0,0,0,15.5653, 36.5472,  1.55334,  1.38928,  1.16875, 0.115356,  1.0472,    1.5708   },
  {5,1,0,0,39.2252, 36.3901,  0.663225, 1.71042,  1.14,    0.115456, -0.793739, -1.49714  },
  {3,2,0,0,128.496, 18.05,    0.839204, 1.28041,  1.25856, 0.214179,  1.5708,    2.26893  },
  {3,3,0,0,94.6839, 18,  -0.277611, 2.37086,  1.23042, 0.224227, -0.523599, -1.0472   },
  {3,5,0,0,50.522, 19.47,   -1.18761,  2.80524,  1.27634, 0.285399,  1.39626,   1.91986  },
  {3,4,0,0,43.5837, 18,      -0.670796, 1.98361,  1.35664, 0.229104, -1.0472,   -1.5708   },
  {4,1,0,0,101.384, 18.1,     0.33,     3.62155,  1.37223, 0.189909,  0,        -0.174533 },
  {4,0,0,0,69.8728, 18.45,    1.64319,  1.30795,  1.31709, 0.189676,  2.0944,    2.61799  },
  {5,6,0,0,13.5361, 35.45,   -0.209204, 1.33735,  1.23945, 0.250544, -0.785398, -1.5708   },
  {5,7,0,0,12.976, 27.88,   -0.212389, 0.991803, 1.3877,  0.207506,  0.785398,  1.5708   },
  {5,9,0,0,31.4217, 18.02,    0.43,     1.47849,  1.34817, 0.196913,  0,         -0.523599},
  {5,8,0,0,23.8345, 21.77,   -1.14319,  2.56082,  1.31466, 0.2092,    0.349066,  0.872665 },
  {6,0,0,0,687.864, 7.98695,  1.5506,   1.14543,  1.1,     0.150907,  0.306223,  1.35342  },
  {6,1,0,0,211.65, 19.2310,       0,    6.28319,  1.1,     0.150907,  0,        -0.785398 },
  {7,0,0,0,  1.,    1.,          0.,    1.,       1.1,     0.25,      0.,        1.       },
  {7,1,0,0,  1.,    1.,          0.,    1.,       1.1,     0.25,      0.,       -1.       },
};


/*
 Original lumbar parameters
  {6,0,0,0,687.864, 1.0472,   1.5506,   1.14543,  1.1,     0.45,      0.306223,  1.35342  },
  {6,1,0,0,211.65 , 0.523599, 0,        6.28319,  1.1,     0.45,      0,        -0.785398 }};
*/

/*************************************************************
 Map that goes from a single joint-torque-direction index to
 the pair of joint and direction indicies used in the tables
*************************************************************/

const static struct JointSet {
  enum item { Hip = 0,
              Knee,
              Ankle,
              Shoulder,
              Elbow,
              Wrist,
              Lumbar,
              Generic,
              Last
            };
  JointSet() {}
} JointSet;


struct DirectionSet {
  enum item {
    Extension = 0,
    Flexion,
    HorizontalAdduction,
    HorizontalAbduction,
    ExternalRotation,
    InternalRotation,
    Supination,
    Pronation,
    RadialDeviation,
    UlnarDeviation,
    Last
  };
  DirectionSet() {}
} DirectionSet;


const static int JointTorqueMap[24][3] = {
  {(int)JointTorqueSet::HipExtension, (int)JointSet::Hip, (int)DirectionSet::Extension          },
  {(int)JointTorqueSet::HipFlexion, (int)JointSet::Hip, (int)DirectionSet::Flexion            },
  {(int)JointTorqueSet::KneeExtension, (int)JointSet::Knee, (int)DirectionSet::Extension          },
  {(int)JointTorqueSet::KneeFlexion, (int)JointSet::Knee, (int)DirectionSet::Flexion            },
  {(int)JointTorqueSet::AnkleExtension, (int)JointSet::Ankle, (int)DirectionSet::Extension          },
  {(int)JointTorqueSet::AnkleFlexion, (int)JointSet::Ankle, (int)DirectionSet::Flexion            },
  {(int)JointTorqueSet::ElbowExtension, (int)JointSet::Elbow, (int)DirectionSet::Extension          },
  {(int)JointTorqueSet::ElbowFlexion, (int)JointSet::Elbow, (int)DirectionSet::Flexion            },
  {(int)JointTorqueSet::ShoulderExtension, (int)JointSet::Shoulder, (int)DirectionSet::Extension          },
  {(int)JointTorqueSet::ShoulderFlexion, (int)JointSet::Shoulder, (int)DirectionSet::Flexion            },
  {(int)JointTorqueSet::WristExtension, (int)JointSet::Wrist, (int)DirectionSet::Extension          },
  {(int)JointTorqueSet::WristFlexion, (int)JointSet::Wrist, (int)DirectionSet::Flexion            },
  {(int)JointTorqueSet::ShoulderHorizontalAdduction, (int)JointSet::Shoulder, (int)DirectionSet::HorizontalAdduction},
  {(int)JointTorqueSet::ShoulderHorizontalAbduction, (int)JointSet::Shoulder, (int)DirectionSet::HorizontalAbduction},
  {(int)JointTorqueSet::ShoulderInternalRotation, (int)JointSet::Shoulder, (int)DirectionSet::InternalRotation   },
  {(int)JointTorqueSet::ShoulderExternalRotation, (int)JointSet::Shoulder, (int)DirectionSet::ExternalRotation   },
  {(int)JointTorqueSet::WristUlnarDeviation, (int)JointSet::Wrist, (int)DirectionSet::UlnarDeviation     },
  {(int)JointTorqueSet::WristRadialDeviation, (int)JointSet::Wrist, (int)DirectionSet::RadialDeviation    },
  {(int)JointTorqueSet::WristPronation, (int)JointSet::Wrist, (int)DirectionSet::Pronation          },
  {(int)JointTorqueSet::WristSupination, (int)JointSet::Wrist, (int)DirectionSet::Supination         },
  {(int)JointTorqueSet::LumbarExtension, (int)JointSet::Lumbar, (int)DirectionSet::Extension          },
  {(int)JointTorqueSet::LumbarFlexion, (int)JointSet::Lumbar, (int)DirectionSet::Flexion            },
  {(int)JointTorqueSet::UnitExtensor, (int)JointSet::Generic, (int)DirectionSet::Extension            },
  {(int)JointTorqueSet::UnitFlexor, (int)JointSet::Generic, (int)DirectionSet::Flexion            }
};


/*************************************************************
 Constructors
*************************************************************/

Millard2016TorqueMuscle::
Millard2016TorqueMuscle( )
  :mAngleOffset(1.0),
   mSignOfJointAngle(1.0),
   mSignOfJointTorque(1.0),
   mSignOfConcentricAnglularVelocity(1.0),
   mMuscleName("empty")
{
  mMuscleCurvesAreDirty = true;
  mUseTabularMaxActiveIsometricTorque = true;
  mUseTabularOmegaMax = true;
  mPassiveTorqueScale = 1.0;

  mTaLambda = 0.0;
  mTpLambda = 0.0;
  mTvLambda = 0.0;
  mTvLambdaMax = 1.0;
  mTaAngleScaling = 1.0;
}

Millard2016TorqueMuscle::Millard2016TorqueMuscle(
  DataSet::item dataSet,
  const SubjectInformation &subjectInfo,
  int jointTorque,
  double  jointAngleOffsetRelativeToDoxygenFigures,
  double  signOfJointAngleRelativeToDoxygenFigures,
  double  mSignOfJointTorque,
  const   std::string& name
):mAngleOffset(jointAngleOffsetRelativeToDoxygenFigures),
  mSignOfJointAngle(signOfJointAngleRelativeToDoxygenFigures),
  mSignOfJointTorque(mSignOfJointTorque),
  mSignOfConcentricAnglularVelocity(mSignOfJointTorque),
  mMuscleName(name),
  mDataSet(dataSet)
{

  mSubjectHeightInMeters   = subjectInfo.heightInMeters;
  mSubjectMassInKg         = subjectInfo.massInKg;
  mPassiveCurveAngleOffset = 0.;
  mPassiveTorqueScale      = 1.0;
  mBetaMax = 0.1;

  int gender                    = (int) subjectInfo.gender;
  int ageGroup                  = (int) subjectInfo.ageGroup;

  mGender     = subjectInfo.gender;
  mAgeGroup   = subjectInfo.ageGroup;

  int joint           = -1;
  int jointDirection  = -1;

  for(int i=0; i < JointTorqueSet::Last; ++i) {
    if(JointTorqueMap[i][0] == jointTorque) {
      mJointTorque    = JointTorqueSet::item(i);
      joint           = JointTorqueMap[i][1];
      jointDirection  = JointTorqueMap[i][2];
    }
  }

  if(joint == -1 || jointDirection == -1) {
    ostringstream errormsg;
    errormsg << "Millard2016TorqueMuscle::"
             << "Millard2016TorqueMuscle:"
             << mMuscleName
             << ": A jointTorqueDirection of " << jointTorque
             << " does not exist.";
    throw RigidBodyDynamics::Errors::RBDLError(errormsg.str());
  }

  if( abs(abs(mSignOfJointAngle)-1) >  EPSILON) {
    ostringstream errormsg;
    errormsg << "Millard2016TorqueMuscle::"
             << "Millard2016TorqueMuscle:"
             << mMuscleName
             << ": signOfJointAngleRelativeToAnderson2007 must be [-1,1] not "
             << mSignOfJointAngle;
    throw RigidBodyDynamics::Errors::RBDLError(errormsg.str());
  }

  if( abs(abs(mSignOfConcentricAnglularVelocity)-1) >  EPSILON) {
    ostringstream errormsg;
    cerr << "Millard2016TorqueMuscle::"
         << "Millard2016TorqueMuscle:"
         << mMuscleName
         << ": signOfJointAngularVelocityDuringConcentricContraction "
         << "must be [-1,1] not "
         << mSignOfConcentricAnglularVelocity;
    throw RigidBodyDynamics::Errors::RBDLError(errormsg.str());
  }

  if( abs(abs(mSignOfJointTorque)-1) >  EPSILON) {
    ostringstream errormsg;
    errormsg << "Millard2016TorqueMuscle::"
             << "Millard2016TorqueMuscle:"
             << mMuscleName
             << ": mSignOfJointTorque must be [-1,1] not "
             << mSignOfJointTorque;
    throw RigidBodyDynamics::Errors::RBDLError(errormsg.str());
  }


  if(mSubjectHeightInMeters <= 0) {
    ostringstream errormsg;
    errormsg << "Millard2016TorqueMuscle::"
             << "Millard2016TorqueMuscle:"
             << mMuscleName
             << ": mSubjectHeightInMeters > 0, but it's "
             << mSubjectHeightInMeters;
    throw RigidBodyDynamics::Errors::RBDLError(errormsg.str());
  }

  if(mSubjectMassInKg <= 0) {
    ostringstream errormsg;
    errormsg << "Millard2016TorqueMuscle::"
             << "Millard2016TorqueMuscle:"
             << mMuscleName
             << ": mSubjectMassInKg > 0, but it's "
             << mSubjectMassInKg;
    throw RigidBodyDynamics::Errors::RBDLError(errormsg.str());
  }



  int idx         = -1;
  int jointIdx    = 0;
  int dirIdx      = 1;
  int genderIdx   = 2;
  int ageIdx      = 3;

  switch(mDataSet) {
  case DataSet::Anderson2007: {
    mAnderson2007c1c2c3c4c5c6.resize(6);
    mAnderson2007b1k1b2k2.resize(4);

    for(int i=0; i<36; ++i) {

      if( abs(Anderson2007Table3Mean[i][jointIdx]
              -(double)joint) < EPSILON
          && abs(Anderson2007Table3Mean[i][dirIdx]
                 -(double)jointDirection) < EPSILON
          && abs(Anderson2007Table3Mean[i][genderIdx]
                 -(double)gender) < EPSILON
          && abs(Anderson2007Table3Mean[i][ageIdx]
                 -(double)ageGroup) < EPSILON) {
        idx = i;
      }
    }

    if(idx != -1) {
      for(int i=0; i<6; ++i) {
        mAnderson2007c1c2c3c4c5c6[i] = Anderson2007Table3Mean[idx][i+4];
      }
      for(int i=0; i<4; ++i) {
        mAnderson2007b1k1b2k2[i] = Anderson2007Table3Mean[idx][i+10];
      }
    }

  }
  break;

  case DataSet::Gymnast: {
    mGymnastParams.resize(8);
    for(int i=0; i<JointTorqueSet::Last; ++i) {

      if( abs(GymnastWholeBody[i][jointIdx]
              -(double)joint) < EPSILON
          && abs(GymnastWholeBody[i][dirIdx]
                 -(double)jointDirection) < EPSILON
          && abs(GymnastWholeBody[i][genderIdx]
                 -(double)gender) < EPSILON
          && abs(GymnastWholeBody[i][ageIdx]
                 -(double)ageGroup) < EPSILON) {
        idx = i;
      }
    }

    if(idx != -1) {
      for(int i=0; i<8; ++i) {
        mGymnastParams[i] = GymnastWholeBody[idx][i+4];
      }
    }
  }
  break;

  default: {
    ostringstream errormsg;
    errormsg << "Millard2016TorqueMuscle::"
             << "Millard2016TorqueMuscle:"
             << mMuscleName
             << "The requested DataSet does not exist.";
    throw RigidBodyDynamics::Errors::RBDLError(errormsg.str());
  }
  };



  if(idx == -1) {
    ostringstream errormsg;
    errormsg << "Millard2016TorqueMuscle::"
             << "Millard2016TorqueMuscle:"
             << mMuscleName
             << "The combination of data set (" << mDataSet << ")"
             << " joint (" << joint << ")"
             << " joint direction (" << jointDirection << ")"
             << " gender, (" << gender << ")"
             << "and age " << ageGroup << ")"
             << "could not be found";
    throw RigidBodyDynamics::Errors::RBDLError(errormsg.str());
  }

  mGender = subjectInfo.gender;
  mAgeGroup = subjectInfo.ageGroup;


  mMuscleCurvesAreDirty = true;
  mUseTabularMaxActiveIsometricTorque = true;
  mUseTabularOmegaMax = true;
  mUseTabularTorqueVelocityMultiplierAtHalfOmegaMax = true;
  mTaLambda = 0.0;
  mTpLambda = 0.0;
  mTvLambda = 0.0;
  mTaAngleScaling = 1.0;
  updateTorqueMuscleCurves();


  #ifdef RBDL_BUILD_ADDON_MUSCLE_FITTING
  mTmInfo.fittingInfo.resize(3);
  /*
    [0]: D^2 jointTorque / D lambdaA ^2
    [1]: D^2 jointTorque / D lambdaV ^2
    [2]: D^2 jointTorque / D lambdaA D lambdaV
  */
  #endif

}


/*************************************************************
 Muscle Model Code
*************************************************************/
/*
 To explain the const casting:
 1. calcJointTorque does not modify any member variables which affect
    the output fo the model, and thus it should be const for the user.

 2. To make the code maintainable the model is evaluated only in 2
    places updTorqueMuscleSummary & updTorqueMuscleInfo.

 3. To save on memory, these upd functions modify a struct, which is a member
    variable, but is only being used as cache.

 To achieve a const calcJointTorque function, maintainable code (e.g. model
 evaluated in only 2 spots), and with minimal use of memory/copying I am
 using const_casting.

 There is perhaps a more elegant way to do this. I dislike
 const_casting, but for now I do not see a better solution.
*/
double Millard2016TorqueMuscle::
calcJointTorque(    double jointAngle,
                    double jointAngularVelocity,
                    double activation) const
{
  if(mMuscleCurvesAreDirty) {
    Millard2016TorqueMuscle* mutableThis =
      const_cast<Millard2016TorqueMuscle* >(this);
    mutableThis->updateTorqueMuscleCurves();
  }

  Millard2016TorqueMuscle* mutableThis =
    const_cast<Millard2016TorqueMuscle* >(this);
  updTorqueMuscleSummary(activation,
                         jointAngle,jointAngularVelocity,
                         mTaLambda,mTpLambda,mTvLambda,
                         mTaAngleScaling, mAngleAtOneNormActiveTorque,
                         mPassiveCurveAngleOffset,
                         mOmegaMax,
                         mMaxActiveIsometricTorque,
                         mutableThis->mTmSummary);

  return mTmSummary.jointTorque;
}



void Millard2016TorqueMuscle::
calcActivation(double jointAngle,
               double jointAngularVelocity,
               double jointTorque,
               TorqueMuscleSummary &tms) const
{
  if(mMuscleCurvesAreDirty) {
    Millard2016TorqueMuscle* mutableThis =
      const_cast<Millard2016TorqueMuscle* >(this);
    mutableThis->updateTorqueMuscleCurves();
  }

  updInvertTorqueMuscleSummary(jointTorque,jointAngle,jointAngularVelocity,
                               mTaLambda,mTpLambda,mTvLambda,
                               mTaAngleScaling,
                               mAngleAtOneNormActiveTorque,
                               mPassiveCurveAngleOffset,
                               mOmegaMax,
                               mMaxActiveIsometricTorque,
                               tms);
}

double Millard2016TorqueMuscle::
calcMaximumActiveIsometricTorqueScalingFactor(
  double jointAngle,
  double jointAngularVelocity,
  double activation,
  double jointTorque) const
{

  if(mMuscleCurvesAreDirty) {
    Millard2016TorqueMuscle* mutableThis =
      const_cast<Millard2016TorqueMuscle* >(this);
    mutableThis->updateTorqueMuscleCurves();
  }

  double scaleFactor = 0.;

  Millard2016TorqueMuscle* mutableThis =
    const_cast<Millard2016TorqueMuscle* >(this);

  updTorqueMuscleSummary(activation,
                         jointAngle,jointAngularVelocity,
                         mTaLambda,mTpLambda,mTvLambda,
                         mTaAngleScaling,
                         mAngleAtOneNormActiveTorque,
                         mPassiveCurveAngleOffset,
                         mOmegaMax,
                         mMaxActiveIsometricTorque,
                         mutableThis->mTmSummary);

  scaleFactor = jointTorque/mTmSummary.jointTorque;

  return scaleFactor;
}


void Millard2016TorqueMuscle::
calcTorqueMuscleInfo(double jointAngle,
                     double jointAngularVelocity,
                     double activation,
                     TorqueMuscleInfo& tmi) const
{

  if(mMuscleCurvesAreDirty) {
    Millard2016TorqueMuscle* mutableThis =
      const_cast<Millard2016TorqueMuscle* >(this);
    mutableThis->updateTorqueMuscleCurves();
  }


  updTorqueMuscleInfo(activation, jointAngle, jointAngularVelocity,
                      mTaLambda,mTpLambda,mTvLambda,
                      mTaAngleScaling, mAngleAtOneNormActiveTorque,
                      mPassiveCurveAngleOffset,
                      mOmegaMax,
                      mMaxActiveIsometricTorque,
                      tmi);
}


/*************************************************************
 Get / Set Functions
*************************************************************/

double Millard2016TorqueMuscle::
getJointTorqueSign() const
{
  if(mMuscleCurvesAreDirty) {
    Millard2016TorqueMuscle* mutableThis =
      const_cast<Millard2016TorqueMuscle* >(this);
    mutableThis->updateTorqueMuscleCurves();
  }
  return mSignOfJointTorque;
}

double Millard2016TorqueMuscle::
getJointAngleSign() const
{
  if(mMuscleCurvesAreDirty) {
    Millard2016TorqueMuscle* mutableThis =
      const_cast<Millard2016TorqueMuscle* >(this);
    mutableThis->updateTorqueMuscleCurves();
  }
  return mSignOfJointAngle;
}

double Millard2016TorqueMuscle::
getJointAngleOffset() const
{
  if(mMuscleCurvesAreDirty) {
    Millard2016TorqueMuscle* mutableThis =
      const_cast<Millard2016TorqueMuscle* >(this);
    mutableThis->updateTorqueMuscleCurves();
  }
  return mAngleOffset;
}


double Millard2016TorqueMuscle::
getNormalizedDampingCoefficient() const
{
  if(mMuscleCurvesAreDirty) {
    Millard2016TorqueMuscle* mutableThis =
      const_cast<Millard2016TorqueMuscle* >(this);
    mutableThis->updateTorqueMuscleCurves();
  }
  return mBetaMax;
}

void Millard2016TorqueMuscle::
setNormalizedDampingCoefficient(double betaUpd)
{
  if(betaUpd < 0) {
    ostringstream errormsg;
    errormsg << "Millard2016TorqueMuscle::"
             << "setNormalizedDampingCoefficient:"
             << mMuscleName
             << "mBetaMax is " << betaUpd
             << " but mBetaMax must be > 0 "
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  mBetaMax = betaUpd;
}




double Millard2016TorqueMuscle::
getMaximumActiveIsometricTorque() const
{
  if(mMuscleCurvesAreDirty) {
    Millard2016TorqueMuscle* mutableThis =
      const_cast<Millard2016TorqueMuscle* >(this);
    mutableThis->updateTorqueMuscleCurves();
  }

  /*
  if(mUseTabularMaxActiveIsometricTorque){
    return mMaxActiveIsometricTorque;
  }else{
    return mMaxActiveIsometricTorque*mMaxActiveIsometricMultiplerProduct;
  }
  */
  return mMaxActiveIsometricTorque;

}

double Millard2016TorqueMuscle::
getMaximumConcentricJointAngularVelocity() const
{
  if(mMuscleCurvesAreDirty) {
    Millard2016TorqueMuscle* mutableThis =
      const_cast<Millard2016TorqueMuscle* >(this);
    mutableThis->updateTorqueMuscleCurves();
  }
  return calcJointAngularVelocity( mOmegaMax );
}

double Millard2016TorqueMuscle::
getTorqueVelocityMultiplierAtHalfOmegaMax() const
{
  if(mMuscleCurvesAreDirty) {
    Millard2016TorqueMuscle* mutableThis =
      const_cast<Millard2016TorqueMuscle* >(this);
    mutableThis->updateTorqueMuscleCurves();
  }
  return mTorqueVelocityMultiplierAtHalfOmegaMax;
}


void Millard2016TorqueMuscle::
setTorqueVelocityMultiplierAtHalfOmegaMax(double tvAtHalfOmegaMax)
{
  if(mDataSet == DataSet::Anderson2007) {
    ostringstream errormsg;
    errormsg << "Millard2016TorqueMuscle::"
             << "setTorqueVelocityMultiplierAtHalfOmegaMax:"
             << mMuscleName
             << " This function is only compatible with the Gymnast dataset"
             << " but this muscle is from the Anderson2007 dataset. Switch"
             << " data sets or stop using this function.";
    throw RigidBodyDynamics::Errors::RBDLError(errormsg.str());
  }
  mMuscleCurvesAreDirty                = true;
  mUseTabularTorqueVelocityMultiplierAtHalfOmegaMax = false;
  mTorqueVelocityMultiplierAtHalfOmegaMax = tvAtHalfOmegaMax;
}


void Millard2016TorqueMuscle::
setMaximumActiveIsometricTorque(double maxIsoTorque)
{
  mMuscleCurvesAreDirty                = true;
  mUseTabularMaxActiveIsometricTorque  = false;
  //mMaxActiveIsometricTorqueUserDefined = maxIsoTorque;
  mMaxActiveIsometricTorque = maxIsoTorque;
}

void Millard2016TorqueMuscle::
setMaximumConcentricJointAngularVelocity(double maxAngularVelocity)
{
  if(fabs(maxAngularVelocity) < SQRTEPSILON) {
    ostringstream errormsg;
    errormsg << "Millard2016TorqueMuscle::"
             << "setMaximumJointAngularVelocity:"
             << mMuscleName
             << " The value of maxJointAngularVelocity needs to be greater "
             << " than sqrt(epsilon), but it is "
             << maxAngularVelocity;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  mMuscleCurvesAreDirty = true;
  mUseTabularOmegaMax   = false;
  mOmegaMax             = fabs(maxAngularVelocity);
}

double Millard2016TorqueMuscle::
getJointAngleAtMaximumActiveIsometricTorque() const
{
  if(mMuscleCurvesAreDirty) {
    Millard2016TorqueMuscle* mutableThis =
      const_cast<Millard2016TorqueMuscle* >(this);
    mutableThis->updateTorqueMuscleCurves();
  }
  return calcJointAngle(mAngleAtOneNormActiveTorque);
}

double Millard2016TorqueMuscle::
getActiveTorqueAngleCurveWidth() const
{
  if(mMuscleCurvesAreDirty) {
    Millard2016TorqueMuscle* mutableThis =
      const_cast<Millard2016TorqueMuscle* >(this);
    mutableThis->updateTorqueMuscleCurves();
  }
  VectorNd domain = mTaCurve.getCurveDomain();
  double activeTorqueAngleAngleScaling
    = getActiveTorqueAngleCurveAngleScaling();
  double width = fabs(domain[1]-domain[0])/activeTorqueAngleAngleScaling;

  return width;

}


double Millard2016TorqueMuscle::
getJointAngleAtOneNormalizedPassiveIsometricTorque() const
{
  if(mMuscleCurvesAreDirty) {
    Millard2016TorqueMuscle* mutableThis =
      const_cast<Millard2016TorqueMuscle* >(this);
    mutableThis->updateTorqueMuscleCurves();
  }
  return calcJointAngle(mAngleAtOneNormPassiveTorque);
}

double Millard2016TorqueMuscle::
getJointAngleAtSmallestNormalizedPassiveIsometricTorque() const
{
  if(mMuscleCurvesAreDirty) {
    Millard2016TorqueMuscle* mutableThis =
      const_cast<Millard2016TorqueMuscle* >(this);
    mutableThis->updateTorqueMuscleCurves();
  }
  return calcJointAngle(mAngleAtSmallestNormPassiveTorque);
}



double Millard2016TorqueMuscle::
getPassiveTorqueScale() const
{
  if(mMuscleCurvesAreDirty) {
    Millard2016TorqueMuscle* mutableThis =
      const_cast<Millard2016TorqueMuscle* >(this);
    mutableThis->updateTorqueMuscleCurves();
  }
  return mPassiveTorqueScale;
}

void Millard2016TorqueMuscle::
setPassiveTorqueScale(double passiveTorqueScaling)
{
  mMuscleCurvesAreDirty = true;
  mPassiveTorqueScale = passiveTorqueScaling;
}


double Millard2016TorqueMuscle::
getPassiveCurveAngleOffset() const
{
  if(mMuscleCurvesAreDirty) {
    Millard2016TorqueMuscle* mutableThis =
      const_cast<Millard2016TorqueMuscle* >(this);
    mutableThis->updateTorqueMuscleCurves();
  }
  return mPassiveCurveAngleOffset;
}

void Millard2016TorqueMuscle::
setPassiveCurveAngleOffset(double passiveCurveAngleOffsetVal)
{
  mMuscleCurvesAreDirty = true;
  mPassiveCurveAngleOffset = passiveCurveAngleOffsetVal;
}


void Millard2016TorqueMuscle::
fitPassiveCurveAngleOffset(double jointAngleTarget,
                           double passiveFiberTorqueTarget)
{
  mMuscleCurvesAreDirty = true;
  setPassiveCurveAngleOffset(0.0);
  updateTorqueMuscleCurves();

  if(passiveFiberTorqueTarget < SQRTEPSILON) {
    ostringstream errormsg;
    errormsg << "Millard2016TorqueMuscle::"
             << "fitPassiveTorqueScale:"
             << mMuscleName
             << ": passiveTorque " << passiveFiberTorqueTarget
             << " but it should be greater than sqrt(eps)"
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  //Solve for the fiber angle at which the curve develops
  //the desired passiveTorque
  double normPassiveFiberTorque = passiveFiberTorqueTarget
                                  /mMaxActiveIsometricTorque;
  double currentFiberAngle = calcFiberAngleGivenNormalizedPassiveTorque(
                               normPassiveFiberTorque,
                               mTpLambda,
                               mPassiveCurveAngleOffset); //0.);

  //double fiberAngleOffset = mPassiveCurveAngleOffset
  //                          +calcFiberAngle(jointAngleTarget)
  //                          -currentFiberAngle;
  double fiberAngleOffset =  calcFiberAngle(jointAngleTarget)
                             -currentFiberAngle;

  setPassiveCurveAngleOffset(fiberAngleOffset);
  updateTorqueMuscleCurves();

}

void Millard2016TorqueMuscle::
fitPassiveTorqueScale(double jointAngleTarget,
                      double passiveFiberTorqueTarget)
{
  mMuscleCurvesAreDirty = true;
  setPassiveTorqueScale(1.0);
  updateTorqueMuscleCurves();

  double normPassiveFiberTorqueTarget = passiveFiberTorqueTarget
                                        /mMaxActiveIsometricTorque;
  double fiberAngle = calcFiberAngle(jointAngleTarget);
  double normPassiveFiberTorqueCurrent=
    calcBlendedCurveDerivative(fiberAngle-mPassiveCurveAngleOffset,
                               mTpLambda, mTpLambdaMax,
                               0,0,
                               mTpCurve);

  double passiveTorqueScale = normPassiveFiberTorqueTarget
                              /normPassiveFiberTorqueCurrent;


  setPassiveTorqueScale(passiveTorqueScale);
  updateTorqueMuscleCurves();
}

void Millard2016TorqueMuscle::calcTorqueMuscleDataFeatures(
  RigidBodyDynamics::Math::VectorNd const &jointTorque,
  RigidBodyDynamics::Math::VectorNd const &jointAngle,
  RigidBodyDynamics::Math::VectorNd const &jointAngularVelocity,
  double activeTorqueAngleBlendingVariable,
  double passiveTorqueAngleBlendingVariable,
  double torqueVelocityBlendingVariable,
  double activeTorqueAngleAngleScaling,
  double activeTorqueAngleAtOneNormTorque,
  double passiveTorqueAngleCurveOffset,
  double maxAngularVelocity,
  double maxActiveIsometricTorque,
  TorqueMuscleDataFeatures &tmf) const
{
  TorqueMuscleSummary tmsCache;
  double activation;
  double tp;
  double minActivation  = 1.0;
  double maxActivation  = 0.;
  double maxTp          = 0.;

  tmf.isInactive = true;

  for(unsigned int i=0; i<jointAngle.rows(); ++i) {

    updInvertTorqueMuscleSummary(jointTorque[i],
                                 jointAngle[i],
                                 jointAngularVelocity[i],
                                 activeTorqueAngleBlendingVariable,
                                 passiveTorqueAngleBlendingVariable,
                                 torqueVelocityBlendingVariable,
                                 activeTorqueAngleAngleScaling,
                                 activeTorqueAngleAtOneNormTorque,
                                 passiveTorqueAngleCurveOffset,
                                 maxAngularVelocity,
                                 maxActiveIsometricTorque,
                                 tmsCache);
    activation = tmsCache.activation;
    tp         = tmsCache.fiberPassiveTorqueAngleMultiplier;
    if(mSignOfJointTorque*jointTorque[i] > 0) {
      tmf.isInactive=false;
      if(activation <= minActivation) {
        minActivation               = activation;
        tmf.indexOfMinActivation    = i;
        tmf.summaryAtMinActivation  = tmsCache;
      }
      if(activation >= maxActivation) {
        tmf.indexOfMaxActivation    = i;
        tmf.summaryAtMaxActivation  = tmsCache;
        maxActivation               = activation;
      }
    }
    if(tmsCache.fiberPassiveTorqueAngleMultiplier >= maxTp) {
      maxTp        = tmsCache.fiberPassiveTorqueAngleMultiplier;
      tmf.summaryAtMaxPassiveTorqueAngleMultiplier = tmsCache;
      tmf.indexOfMaxPassiveTorqueAngleMultiplier   = i;
    }

  }

}



const SmoothSegmentedFunction& Millard2016TorqueMuscle::
getActiveTorqueAngleCurve() const
{
  //This must be updated if the parameters have changed
  if(mMuscleCurvesAreDirty) {
    Millard2016TorqueMuscle* mutableThis =
      const_cast<Millard2016TorqueMuscle* >(this);
    mutableThis->updateTorqueMuscleCurves();
  }
  return mTaCurve;
}

const SmoothSegmentedFunction& Millard2016TorqueMuscle::
getPassiveTorqueAngleCurve() const
{
  //This must be updated if the parameters have changed
  if(mMuscleCurvesAreDirty) {
    Millard2016TorqueMuscle* mutableThis =
      const_cast<Millard2016TorqueMuscle* >(this);
    mutableThis->updateTorqueMuscleCurves();
  }
  return mTpCurve;
}

const SmoothSegmentedFunction& Millard2016TorqueMuscle::
getTorqueAngularVelocityCurve() const
{
  //This must be updated if the parameters have changed
  if(mMuscleCurvesAreDirty) {
    Millard2016TorqueMuscle* mutableThis =
      const_cast<Millard2016TorqueMuscle* >(this);
    mutableThis->updateTorqueMuscleCurves();
  }
  return mTvCurve;
}



string Millard2016TorqueMuscle::getName()
{
  return mMuscleName;
}

void Millard2016TorqueMuscle::setName(string &name)
{
  mMuscleCurvesAreDirty = true;
  mMuscleName = name;
}


double Millard2016TorqueMuscle::
getActiveTorqueAngleCurveBlendingVariable() const
{
  return mTaLambda;
}

double Millard2016TorqueMuscle::
getPassiveTorqueAngleCurveBlendingVariable() const
{
  return mTpLambda;
}

double Millard2016TorqueMuscle::
getTorqueAngularVelocityCurveBlendingVariable() const
{
  return mTvLambda;
}

double Millard2016TorqueMuscle::getActiveTorqueAngleCurveAngleScaling() const
{
  return mTaAngleScaling;
}

void Millard2016TorqueMuscle::
setActiveTorqueAngleCurveAngleScaling(double angleScaling)
{
  if(angleScaling < SQRTEPSILON) {
    ostringstream errormsg;
    cerr << "Millard2016TorqueMuscle::"
         << "setActiveTorqueAngleCurveAngleScaling:"
         << mMuscleName
         << ": angleScaling must be > sqrt(eps), this "
         << angleScaling
         <<" is outside the acceptable range."
         << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  mMuscleCurvesAreDirty = true;
  mTaAngleScaling = angleScaling;
}

void Millard2016TorqueMuscle::
setActiveTorqueAngleCurveBlendingVariable(double blendingVariable)
{
  if(blendingVariable < 0. || blendingVariable > 1.0) {
    ostringstream errormsg;
    errormsg << "Millard2016TorqueMuscle::"
             << "setActiveTorqueAngleCurveBlendingVariable:"
             << mMuscleName
             << ": blending variable must be [0,1] and this "
             << blendingVariable
             << " is outside the acceptable range."
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }
  mMuscleCurvesAreDirty = true;
  mTaLambda = blendingVariable;
}

void Millard2016TorqueMuscle::
setPassiveTorqueAngleCurveBlendingVariable(double blendingVariable)
{
  if(blendingVariable < 0. || blendingVariable > 1.0) {
    ostringstream errormsg;
    errormsg << "Millard2016TorqueMuscle::"
             << "setPassiveTorqueAngleCurveBlendingVariable:"
             << mMuscleName
             << ": blending variable must be [0,1] and this "
             << blendingVariable
             << " is outside the acceptable range."
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }
  mMuscleCurvesAreDirty = true;
  mTpLambda = blendingVariable;
}

void Millard2016TorqueMuscle::
setTorqueAngularVelocityCurveBlendingVariable(double blendingVariable)
{

  if(blendingVariable < 0. || blendingVariable > 1.0) {
    ostringstream errormsg;
    errormsg << "Millard2016TorqueMuscle::"
             << "setTorqueAngularVelocityCurveBlendingVariable:"
             << mMuscleName
             << ": blending variable must be [0,1] and this "
             << blendingVariable
             << " is outside the acceptable range."
             << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }
  mMuscleCurvesAreDirty = true;
  mTvLambda = blendingVariable;
}

void Millard2016TorqueMuscle::setFittedParameters(
  const TorqueMuscleParameterFittingData &fittedParameters)
{

  if(!fittedParameters.fittingConverged) {
    ostringstream errormsg;
    errormsg  << "Millard2016TorqueMuscle::"
              << "setTorqueMuscleParameters:"
              << mMuscleName
              << ": The fittingConverged field of fittedParameters is false! "
              << endl;
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }

  setPassiveTorqueAngleCurveBlendingVariable(
    fittedParameters.passiveTorqueAngleBlendingVariable);
  setActiveTorqueAngleCurveBlendingVariable(
    fittedParameters.activeTorqueAngleBlendingVariable);
  setTorqueAngularVelocityCurveBlendingVariable(
    fittedParameters.torqueVelocityBlendingVariable);
  setPassiveCurveAngleOffset(
    fittedParameters.passiveTorqueAngleCurveOffset);
  setMaximumActiveIsometricTorque(
    fittedParameters.maximumActiveIsometricTorque);
  setMaximumConcentricJointAngularVelocity(
    fittedParameters.maximumAngularVelocity);
  setActiveTorqueAngleCurveAngleScaling(
    fittedParameters.activeTorqueAngleAngleScaling);

}

/*************************************************************
 Utilities
*************************************************************/
//fa = signJointAngle*(ja - jaO)
// ja = signJointAngle*fa + ja0
//dja = signJointAngle*dfa

double Millard2016TorqueMuscle::
calcFiberAngle(double jointAngle) const
{
  return mSignOfJointAngle*(jointAngle-mAngleOffset);
}

double Millard2016TorqueMuscle::
calcJointAngle(double fiberAngle) const
{
  return fiberAngle*mSignOfJointAngle + mAngleOffset;
}


double Millard2016TorqueMuscle::
calcFiberAngularVelocity(double jointAngularVelocity) const
{
  return mSignOfConcentricAnglularVelocity*jointAngularVelocity;
}

double Millard2016TorqueMuscle::
calcJointAngularVelocity(double fiberAngularVelocity) const
{
  return mSignOfConcentricAnglularVelocity*fiberAngularVelocity;
}

void Millard2016TorqueMuscle::updateTorqueMuscleCurves()
{
  std::string tempName = mMuscleName;

  switch(mDataSet) {
  case DataSet::Anderson2007: {
    double c4 = mAnderson2007c1c2c3c4c5c6[3];
    double c5 = mAnderson2007c1c2c3c4c5c6[4];

    if(mUseTabularOmegaMax) {
      mOmegaMax = abs( 2.0*c4*c5/(c5-3.0*c4) );
    }

    mScaleFactorAnderson2007  = mSubjectHeightInMeters
                                *mSubjectMassInKg
                                *Gravity;

    if(mUseTabularMaxActiveIsometricTorque) {
      mMaxActiveIsometricTorque = mScaleFactorAnderson2007
                                  *mAnderson2007c1c2c3c4c5c6[0];
    }

    mAngleAtOneNormActiveTorque = mAnderson2007c1c2c3c4c5c6[2];

    TorqueMuscleFunctionFactory::
    createAnderson2007ActiveTorqueAngleCurve(
      mAnderson2007c1c2c3c4c5c6[1],
      mAnderson2007c1c2c3c4c5c6[2],
      tempName.append("_taCurve"),
      mTaCurve);

    tempName = mMuscleName;

    TorqueMuscleFunctionFactory::
    createAnderson2007ActiveTorqueVelocityCurve(
      mAnderson2007c1c2c3c4c5c6[3],
      mAnderson2007c1c2c3c4c5c6[4],
      mAnderson2007c1c2c3c4c5c6[5],
      1.1,
      1.4,
      tempName.append("_tvCurve"),
      mTvCurve);

    mTorqueVelocityMultiplierAtHalfOmegaMax = mTvCurve.calcValue(0.5);

    tempName = mMuscleName;

    double normMaxActiveIsometricTorque = mMaxActiveIsometricTorque
                                          /mScaleFactorAnderson2007;

    TorqueMuscleFunctionFactory::
    createAnderson2007PassiveTorqueAngleCurve(
      mScaleFactorAnderson2007,
      normMaxActiveIsometricTorque,
      mAnderson2007b1k1b2k2[0],
      mAnderson2007b1k1b2k2[1],
      mAnderson2007b1k1b2k2[2],
      mAnderson2007b1k1b2k2[3],
      tempName.append("_tpCurve"),
      mTpCurve);

    //mTpCurve.shift(mPassiveCurveAngleOffset,0);
    mTpCurve.scale(1.0,mPassiveTorqueScale);

    double k = 0;
    double b = 0;

    if(mAnderson2007b1k1b2k2[0] > 0) {
      b = mAnderson2007b1k1b2k2[0];
      k = mAnderson2007b1k1b2k2[1];
    } else if(mAnderson2007b1k1b2k2[2] > 0) {
      b = mAnderson2007b1k1b2k2[2];
      k = mAnderson2007b1k1b2k2[3];
    }


    VectorNd xDomain = mTpCurve.getCurveDomain();
    double tpAtX0 = mTpCurve.calcValue(xDomain[0]);
    double tpAtX1 = mTpCurve.calcValue(xDomain[1]);

    if(fabs(tpAtX0) < SQRTEPSILON) {
      mAngleAtSmallestNormPassiveTorque = xDomain[0];
    } else if(fabs(tpAtX1) < SQRTEPSILON) {
      mAngleAtSmallestNormPassiveTorque = xDomain[1];
    } else {
      mAngleAtSmallestNormPassiveTorque =
        std::numeric_limits<double>::signaling_NaN();
    }

    if(   fabs(tpAtX0) > SQRTEPSILON
          || fabs(tpAtX1) > SQRTEPSILON) {
      double argGuess = (1/k)*log(abs(mMaxActiveIsometricTorque/b));
      //                              + mPassiveCurveAngleOffset;
      mAngleAtOneNormPassiveTorque = calcInverseBlendedCurveValue(1.0,
                                     argGuess,
                                     mTpLambda,
                                     mTpLambdaMax,
                                     mTpCurve);
      mAngleAtOneNormPassiveTorque += mPassiveCurveAngleOffset;
    } else {
      mAngleAtOneNormPassiveTorque =
        std::numeric_limits<double>::signaling_NaN();
    }


    //mAngleAtOneNormPassiveTorque
    //mGymnastParams[Gymnast::PassiveAngleAtOneNormTorque]


  }
  break;
  case DataSet::Gymnast: {
    if(mUseTabularOmegaMax) {
      mOmegaMax                  = mGymnastParams[
                              Gymnast::OmegaMax];
    }
    if(mUseTabularMaxActiveIsometricTorque) {
      mMaxActiveIsometricTorque  = mGymnastParams[
                              Gymnast::TauMax];
    }
    mAngleAtOneNormActiveTorque = mGymnastParams[
                               Gymnast::ActiveAngleAtOneNormTorque];

    TorqueMuscleFunctionFactory::
    createGaussianShapedActiveTorqueAngleCurve(
      mGymnastParams[Gymnast::ActiveAngleAtOneNormTorque],
      mGymnastParams[Gymnast::ActiveAngularStandardDeviation],
      tempName.append("_taCurve"),
      mTaCurve);

    TorqueMuscleFunctionFactory::createPassiveTorqueAngleCurve(
      mGymnastParams[Gymnast::PassiveAngleAtZeroTorque],
      mGymnastParams[Gymnast::PassiveAngleAtOneNormTorque],
      tempName.append("_tpCurve"),
      mTpCurve);

    //mTpCurve.shift(mPassiveCurveAngleOffset,0);
    mTpCurve.scale(1.0,mPassiveTorqueScale);



    if(mUseTabularTorqueVelocityMultiplierAtHalfOmegaMax) {
      mTorqueVelocityMultiplierAtHalfOmegaMax
        = mGymnastParams[Gymnast::TvAtHalfMaxConcentricVelocity];
    }

    TorqueMuscleFunctionFactory::createTorqueVelocityCurve(
      mGymnastParams[Gymnast::TvAtMaxEccentricVelocity],
      mTorqueVelocityMultiplierAtHalfOmegaMax,
      tempName.append("_tvCurve"),
      mTvCurve);


    //If the passive curve is non-zero, recalculate the angle at which
    //it hits one normalized force.
    mAngleAtSmallestNormPassiveTorque =
      mGymnastParams[Gymnast::PassiveAngleAtZeroTorque]
      +mPassiveCurveAngleOffset;

    //Verify that the curve hits zero at this angle
    if(fabs(mTpCurve.calcValue(mAngleAtSmallestNormPassiveTorque))
        >SQRTEPSILON) {
      mAngleAtSmallestNormPassiveTorque =
        numeric_limits<double>::signaling_NaN();
    }

    VectorNd xDomain = mTpCurve.getCurveDomain();
    double tpAtX0 = mTpCurve.calcValue(xDomain[0]);
    double tpAtX1 = mTpCurve.calcValue(xDomain[1]);

    if(   fabs(tpAtX0) > SQRTEPSILON
          || fabs(tpAtX1) > SQRTEPSILON) {
      double argGuess =mGymnastParams[Gymnast::PassiveAngleAtOneNormTorque];
      //+ mPassiveCurveAngleOffset;
      mAngleAtOneNormPassiveTorque =
        calcInverseBlendedCurveValue(1.0,argGuess,mTpLambda,mTpLambdaMax,
                                     mTpCurve);
      mAngleAtOneNormPassiveTorque += mPassiveCurveAngleOffset;
    } else {
      mAngleAtOneNormPassiveTorque =
        std::numeric_limits<double>::signaling_NaN();
    }
  }
  break;
  default: {
    ostringstream errormsg;
    errormsg << "Millard2016TorqueMuscle::"
             << "Millard2016TorqueMuscle:"
             << mMuscleName
             << "mDataSet has a value of " << mDataSet
             << " which is not a valid choice";
    throw RigidBodyDynamics::Errors::RBDLInvalidParameterError(errormsg.str());
  }
  };

  VectorNd tvDomain = mTvCurve.getCurveDomain();
  mTvLambdaMax = mTvCurve.calcValue(tvDomain[0]);
  assert(mTvLambdaMax >= 1.0); //If this fails you've gotten the incorrect
  //domain end.

  /*
  double taIsoMax = calcBlendedCurveDerivative(mAngleAtOneNormActiveTorque,
                                               mTaLambda, mTaLambdaMax,
                                               0,0, mTaCurve);
  double tvIsoMax = calcBlendedCurveDerivative(0,
                                               mTvLambda, mTvLambdaMax,
                                               0,0, mTvCurve);
  mMaxActiveIsometricMultiplerProduct = taIsoMax*tvIsoMax;

  if(mUseTabularMaxActiveIsometricTorque == false){
    mMaxActiveIsometricTorque = mMaxActiveIsometricTorqueUserDefined
                               /mMaxActiveIsometricMultiplerProduct;
  }
  */
  mMuscleCurvesAreDirty = false;
}



void Millard2016TorqueMuscle::printJointTorqueProfileToFile(
  const std::string& path,
  const std::string& fileNameWithoutExtension,
  int numberOfSamplePoints)
{
  if(mMuscleCurvesAreDirty) {
    updateTorqueMuscleCurves();
  }

  VectorNd activeDomain  = mTaCurve.getCurveDomain();
  VectorNd passiveDomain = mTpCurve.getCurveDomain();
  VectorNd velocityDomain= mTvCurve.getCurveDomain();

  double jointMin = calcJointAngle( activeDomain[0] );
  double jointMax = calcJointAngle( activeDomain[1] );

  if(mTpCurve.calcValue( activeDomain[0] ) >= 0.99) {
    jointMin = calcJointAngle( passiveDomain[0] );
  }

  if(mTpCurve.calcValue(activeDomain[1]) >= 0.99) {
    jointMax = calcJointAngle( passiveDomain[1] );
  }



  if(jointMin > jointMax) {
    double tmp = jointMin;
    jointMin=jointMax;
    jointMax=tmp;
  }
  double range = jointMax-jointMin;
  jointMin = jointMin -range*0.5;
  jointMax = jointMax +range*0.5;
  double jointDelta = (jointMax-jointMin)
                      /((double)numberOfSamplePoints-1.);

  double velMin = calcJointAngularVelocity( -mOmegaMax );
  double velMax = calcJointAngularVelocity(  mOmegaMax );

  if(velMin > velMax) {
    double tmp = velMin;
    velMin = velMax;
    velMax = tmp;
  }
  double velRange = velMax-velMin;
  velMin = velMin-0.5*velRange;
  velMax = velMax+0.5*velRange;
  double velDelta = (velMax-velMin)/((double)numberOfSamplePoints-1.0);

  double angleAtMaxIsoTorque = mAngleAtOneNormActiveTorque;

  std::vector< std::vector < double > > matrix;
  std::vector < double > row(21);
  std::string header("jointAngle,"
                     "jointVelocity,"
                     "activation,"
                     "fiberAngle,"
                     "fiberAngularVelocity,"
                     "passiveTorqueAngleMultiplier,"
                     "activeTorqueAngleMultiplier,"
                     "torqueVelocityMultiplier,"
                     "activeTorque,"
                     "passiveTorque,"
                     "fiberTorque,"
                     "jointTorque,"
                     "fiberStiffness,"
                     "jointStiffness,"
                     "fiberActivePower,"
                     "fiberPassivePower,"
                     "fiberPower,"
                     "jointPower,"
                     "DjointTorqueDactivation,"
                     "DjointTorqueDjointAngularVelocity,"
                     "DjointTorqueDjointAngle");

  double activation  =1.0;
  double jointAngle  = 0.;
  double jointVelocity = 0.;


  for(int i=0; i<numberOfSamplePoints; ++i) {
    jointAngle = jointMin + i*jointDelta;
    jointVelocity = 0.;

    calcTorqueMuscleInfo(jointAngle,
                         jointVelocity,
                         activation,
                         mTmInfo);

    row.at(0) = jointAngle;
    row.at(1) = jointVelocity;
    row.at(2) = activation;

    row.at(3) = mTmInfo.fiberAngle;
    row.at(4) = mTmInfo.fiberAngularVelocity;
    row.at(5) = mTmInfo.fiberPassiveTorqueAngleMultiplier;
    row.at(6) = mTmInfo.fiberActiveTorqueAngleMultiplier;
    row.at(7) = mTmInfo.fiberTorqueAngularVelocityMultiplier;
    row.at(8) = mTmInfo.fiberActiveTorque;
    row.at(9) = mTmInfo.fiberPassiveTorque;
    row.at(10)= mTmInfo.fiberTorque;
    row.at(11)= mTmInfo.jointTorque;
    row.at(12)= mTmInfo.fiberStiffness;
    row.at(13)= mTmInfo.jointStiffness;
    row.at(14)= mTmInfo.fiberActivePower;
    row.at(15)= mTmInfo.fiberPassivePower;
    row.at(16)= mTmInfo.fiberPower;
    row.at(17)= mTmInfo.jointPower;
    row.at(18)= mTmInfo.DjointTorque_Dactivation;
    row.at(19)= mTmInfo.DjointTorque_DjointAngularVelocity;
    row.at(20)= mTmInfo.DjointTorque_DjointAngle;

    matrix.push_back(row);
  }

  std::string fullFilePath = path;
  if(!path.empty()) {
    fullFilePath.append("/");
  }


  fullFilePath.append(fileNameWithoutExtension);
  fullFilePath.append("_variableLengthFixedVelocity");
  fullFilePath.append(".csv");
  printMatrixToFile(matrix,header,fullFilePath);

  matrix.clear();


  for(int i=0; i<numberOfSamplePoints; ++i) {

    jointAngle = calcJointAngle( angleAtMaxIsoTorque );
    jointVelocity = calcJointAngularVelocity( velMin + i*velDelta );

    calcTorqueMuscleInfo(jointAngle,
                         jointVelocity,
                         activation,
                         mTmInfo);

    row.at(0) = jointAngle;
    row.at(1) = jointVelocity;
    row.at(2) = activation;

    row.at(3) = mTmInfo.fiberAngle;
    row.at(4) = mTmInfo.fiberAngularVelocity;
    row.at(5) = mTmInfo.fiberPassiveTorqueAngleMultiplier;
    row.at(6) = mTmInfo.fiberActiveTorqueAngleMultiplier;
    row.at(7) = mTmInfo.fiberTorqueAngularVelocityMultiplier;
    row.at(8) = mTmInfo.fiberActiveTorque;
    row.at(9) = mTmInfo.fiberPassiveTorque;
    row.at(10)= mTmInfo.fiberTorque;
    row.at(11)= mTmInfo.jointTorque;
    row.at(12)= mTmInfo.fiberStiffness;
    row.at(13)= mTmInfo.jointStiffness;
    row.at(14)= mTmInfo.fiberActivePower;
    row.at(15)= mTmInfo.fiberPassivePower;
    row.at(16)= mTmInfo.fiberPower;
    row.at(17)= mTmInfo.jointPower;
    row.at(18)= mTmInfo.DjointTorque_Dactivation;
    row.at(19)= mTmInfo.DjointTorque_DjointAngularVelocity;
    row.at(20)= mTmInfo.DjointTorque_DjointAngle;

    matrix.push_back(row);
  }

  fullFilePath = path;
  if(!path.empty()) {
    fullFilePath.append("/");
  }
  fullFilePath.append(fileNameWithoutExtension);
  fullFilePath.append("_fixedLengthVariableVelocity");
  fullFilePath.append(".csv");
  printMatrixToFile(matrix,header,fullFilePath);

  matrix.clear();

}

//=============================================================================
//=============================================================================
double Millard2016TorqueMuscle::
calcInverseBlendedCurveValue(
  double blendedCurveValue,
  double argGuess,
  double blendingVariable,
  double maximumBlendingValue,
  RigidBodyDynamics::Addons::Geometry::SmoothSegmentedFunction
  const &curve) const
{
  double arg = 0;
  double curveValue =
    (blendedCurveValue-(blendingVariable*maximumBlendingValue))
    /(1-blendingVariable);
  arg = curve.calcInverseValue(curveValue,argGuess);
  return arg;
}
double Millard2016TorqueMuscle::
calcBlendedCurveDerivative(
  double curveArgument,
  double blendingVariable,
  double maximumBlendingValue,
  unsigned int derivativeOrderArgument,
  unsigned int derivativeOrderBlendingVariable,
  RigidBodyDynamics::Addons::Geometry::SmoothSegmentedFunction
  const &curve) const
{
  double output = 0;

  switch(derivativeOrderArgument) {
  case 0: {
    switch(derivativeOrderBlendingVariable) {
    case 0: {
      output = (1.-blendingVariable)*curve.calcValue(curveArgument)
               + blendingVariable*maximumBlendingValue;
    }
    break;
    case 1: {
      output = maximumBlendingValue - curve.calcValue(curveArgument);
    }
    break;
    case 2: {
      output = 0.;
    }
    break;
    default: {
      output =0.; //Higher order derivatives are zero.
    }
    }
  }
  break;
  case 1: {
    switch(derivativeOrderBlendingVariable) {
    case 0: {
      output = (1.-blendingVariable)*curve.calcDerivative(curveArgument,1);
    }
    break;
    case 1: {
      output = -1.0*curve.calcDerivative(curveArgument,1);
    }
    break;
    case 2: {
      output = 0.;
    }
    break;
    default: {
      output = 0.; //Higher order derivatives are zero.
    }
    }
  }
  break;
  case 2: {
    switch(derivativeOrderBlendingVariable) {
    case 0: {
      output = (1.-blendingVariable)*curve.calcDerivative(curveArgument,2);
    }
    break;
    case 1: {
      output = -1.0*curve.calcDerivative(curveArgument,2);
    }
    break;
    case 2: {
      output = 0.;
    }
    break;
    default: {
      output = 0.; //Higher order derivatives are zero.
    }
    }
  }
  break;

  //! [Missing Implementation]
  default: {
    throw RigidBodyDynamics::Errors::RBDLMissingImplementationError("3rd order derivatives not yet implemented\n");
  }
  //! [Missing Implementation]

  };
  return output;
}



//=============================================================================
//=============================================================================

double Millard2016TorqueMuscle::
calcFiberAngleGivenNormalizedPassiveTorque(
  double normPassiveFiberTorque,
  double blendingVariable,
  double passiveTorqueAngleCurveOffset) const
{

  double angle = numeric_limits<double>::signaling_NaN();
  VectorNd domain = mTpCurve.getCurveDomain();
  double y0 = mTpCurve.calcValue(domain[0]);
  double y1 = mTpCurve.calcValue(domain[1]);

  if( (blendingVariable < 1) && (y0 > SQRTEPSILON || y1 > SQRTEPSILON) ) {
    double tp = normPassiveFiberTorque/(1-blendingVariable);
    angle     = mTpCurve.calcInverseValue(tp,0.)-passiveTorqueAngleCurveOffset;
  }

  return angle;
}


//=============================================================================
//=============================================================================

void Millard2016TorqueMuscle::
updTorqueMuscleSummaryCurveValues(double fiberAngle,
                                  double normFiberAngularVelocity,
                                  double activeTorqueAngleBlendingVariable,
                                  double passiveTorqueAngleBlendingVariable,
                                  double torqueAngularVelocityBlendingVariable,
                                  double activeTorqueAngleAngleScaling,
                                  double activeTorqueAngleAtOneNormTorque,
                                  double passiveTorqueAngleCurveOffset,
                                  TorqueMuscleSummary &updTms) const
{

  double taAngle = (fiberAngle-activeTorqueAngleAtOneNormTorque)
                   /activeTorqueAngleAngleScaling
                   + activeTorqueAngleAtOneNormTorque;

  updTms.fiberActiveTorqueAngleMultiplier =
    calcBlendedCurveDerivative(taAngle,
                               activeTorqueAngleBlendingVariable,
                               mTaLambdaMax,
                               0,0,
                               mTaCurve);

  updTms.fiberPassiveTorqueAngleMultiplier =
    calcBlendedCurveDerivative(fiberAngle-passiveTorqueAngleCurveOffset,
                               passiveTorqueAngleBlendingVariable,
                               mTpLambdaMax,
                               0,0,
                               mTpCurve);

  updTms.fiberTorqueAngularVelocityMultiplier =
    calcBlendedCurveDerivative(
      normFiberAngularVelocity,
      torqueAngularVelocityBlendingVariable,
      mTvLambdaMax,
      0,0,
      mTvCurve);

  updTms.fiberNormalizedDampingTorque = updTms.fiberPassiveTorqueAngleMultiplier
                                        * (-normFiberAngularVelocity*mBetaMax);

}

void Millard2016TorqueMuscle::
updTorqueMuscleInfo(
  double activation,
  double jointAngle,
  double jointAngularVelocity,
  double activeTorqueAngleBlendingVariable,
  double passiveTorqueAngleBlendingVariable,
  double torqueAngularVelocityBlendingVariable,
  double activeTorqueAngleAngleScaling,
  double activeTorqueAngleAtOneNormTorque,
  double passiveTorqueAngleCurveOffset,
  double maxAngularVelocity,
  double maxActIsoTorque,
  TorqueMuscleInfo &updTmi) const
{

  //Update state quantities
  updTmi.activation = activation;
  updTmi.jointAngle = jointAngle;
  updTmi.jointAngularVelocity = jointAngularVelocity;

  double fiberAngle           = calcFiberAngle(jointAngle);
  double fiberAngularVelocity = calcFiberAngularVelocity(jointAngularVelocity);
  double omegaNorm            = fiberAngularVelocity/maxAngularVelocity;
  double D_wn_D_w = 1.0/maxAngularVelocity;
  double D_wn_D_wmax = -fiberAngularVelocity
                       /(maxAngularVelocity*maxAngularVelocity);

  double D2_wn_D_wmax2 = -2.0*D_wn_D_wmax/maxAngularVelocity;

  double taAngle = (fiberAngle-activeTorqueAngleAtOneNormTorque)
                   /activeTorqueAngleAngleScaling
                   + activeTorqueAngleAtOneNormTorque;

  double D_taAngle_D_fiberAngle = 1.0/activeTorqueAngleAngleScaling;

  double D_taAngle_D_angleScaling =
    -(fiberAngle-activeTorqueAngleAtOneNormTorque)
    /(activeTorqueAngleAngleScaling*activeTorqueAngleAngleScaling);

  double D2_taAngle_D_angleScaling2 = -2.0*D_taAngle_D_angleScaling
                                      /activeTorqueAngleAngleScaling;

  //Update force component values
  double ta =
    calcBlendedCurveDerivative(taAngle,
                               activeTorqueAngleBlendingVariable,
                               mTaLambdaMax,
                               0,0,
                               mTaCurve);

  double tp =
    calcBlendedCurveDerivative(fiberAngle-passiveTorqueAngleCurveOffset,
                               passiveTorqueAngleBlendingVariable,
                               mTpLambdaMax,
                               0,0,
                               mTpCurve);

  double tv = calcBlendedCurveDerivative(
                omegaNorm,
                torqueAngularVelocityBlendingVariable,
                mTvLambdaMax,
                0,0,
                mTvCurve);

  double tb = tp * (-omegaNorm*mBetaMax);

  double D_tb_D_omegaMax   = tp*(-D_wn_D_wmax*mBetaMax);


  //Update force component derivative values;
  //1st derivatives w.r.t fiber angle/velocity
  double D_ta_D_taAngle = calcBlendedCurveDerivative(
                            taAngle,
                            activeTorqueAngleBlendingVariable,
                            mTaLambdaMax,
                            1,0,
                            mTaCurve);

  double D_ta_D_angleScaling = D_ta_D_taAngle*D_taAngle_D_angleScaling;

  double D_ta_D_fiberAngle = D_ta_D_taAngle*D_taAngle_D_fiberAngle;

  double D_tp_D_fiberAngle = calcBlendedCurveDerivative(
                               fiberAngle-passiveTorqueAngleCurveOffset,
                               passiveTorqueAngleBlendingVariable,
                               mTpLambdaMax,
                               1,0,
                               mTpCurve);

  double D_tp_D_fiberAngleOffset=  -D_tp_D_fiberAngle;

  double D_tv_D_wn        = calcBlendedCurveDerivative(
                              omegaNorm,
                              torqueAngularVelocityBlendingVariable,
                              mTvLambdaMax,
                              1,0,
                              mTvCurve);



  double D_tv_D_fiberAngularVelocity = D_tv_D_wn*D_wn_D_w;
  double D_tv_D_omegaMax             = D_tv_D_wn*D_wn_D_wmax;


  //1st derivatives w.r.t fitting-related variables
  double D_ta_D_taLambda = calcBlendedCurveDerivative(
                             taAngle,
                             activeTorqueAngleBlendingVariable,
                             mTaLambdaMax,
                             0,1,
                             mTaCurve);

  double D_tp_D_tpLambda = calcBlendedCurveDerivative(
                             fiberAngle-passiveTorqueAngleCurveOffset,
                             passiveTorqueAngleBlendingVariable,
                             mTpLambdaMax,
                             0,1,
                             mTpCurve);
  double D_tv_D_tvLambda = calcBlendedCurveDerivative(
                             omegaNorm,
                             torqueAngularVelocityBlendingVariable,
                             mTvLambdaMax,
                             0,1,
                             mTvCurve);

  //2nd derivatives w.r.t fitting-related variables.
  double D2_ta_D_taLambda2 = calcBlendedCurveDerivative(
                               taAngle,
                               activeTorqueAngleBlendingVariable,
                               mTaLambdaMax,
                               0,2,
                               mTaCurve);

  double D2_tv_D_tvLambda2 = calcBlendedCurveDerivative(
                               omegaNorm,
                               torqueAngularVelocityBlendingVariable,
                               mTvLambdaMax,
                               0,2,
                               mTvCurve);
  double D2_tp_D_tpLambda2 = calcBlendedCurveDerivative(
                               fiberAngle-passiveTorqueAngleCurveOffset,
                               passiveTorqueAngleBlendingVariable,
                               mTpLambdaMax,
                               0,2,
                               mTpCurve);


  double D2_tp_D_fiberAngle2 =calcBlendedCurveDerivative(
                                fiberAngle-passiveTorqueAngleCurveOffset,
                                passiveTorqueAngleBlendingVariable,
                                mTpLambdaMax,
                                2,0,
                                mTpCurve);

  double D2_ta_D_taAngle2 = calcBlendedCurveDerivative(
                              taAngle,
                              activeTorqueAngleBlendingVariable,
                              mTaLambdaMax,
                              2,0,
                              mTaCurve);

  double D2_ta_D_angleScaling2 =
    D2_ta_D_taAngle2*D_taAngle_D_angleScaling*D_taAngle_D_angleScaling
    +  D_ta_D_taAngle*D2_taAngle_D_angleScaling2;

  double D2_tv_D_wn2      = calcBlendedCurveDerivative(
                              omegaNorm,
                              torqueAngularVelocityBlendingVariable,
                              mTvLambdaMax,
                              2,0,
                              mTvCurve);

  double D2_tb_D_omegaMax2 = tp*(-D2_wn_D_wmax2*mBetaMax);
  double D2_tb_D_omegaMax_D_tpLambda = D_tp_D_tpLambda*(-D_wn_D_wmax*mBetaMax);
  double D2_tb_D_omegaMax_D_tpOffset = D_tp_D_fiberAngleOffset
                                       *(-D_wn_D_wmax*mBetaMax);
  double D2_tv_D_omegaMax2           = D2_tv_D_wn2*D_wn_D_wmax*D_wn_D_wmax
                                       + D_tv_D_wn*D2_wn_D_wmax2;



  //Note that d/d_tpOffsetAngle kicks out another -1, so this 2nd derivative has
  //a positive sign.
  double D2_tp_D_fiberAngleOffset2= D2_tp_D_fiberAngle2;

  double D2_tp_D_tpLambda_D_fiberAngle =
    calcBlendedCurveDerivative( fiberAngle-passiveTorqueAngleCurveOffset,
                                passiveTorqueAngleBlendingVariable,
                                mTpLambdaMax,
                                1,1,
                                mTpCurve);
  double D2_tp_D_tpLambda_D_fiberAngleOffset = -D2_tp_D_tpLambda_D_fiberAngle;

  //Damping derivatives
  double D_tb_D_wn = tp * (-1.0*mBetaMax);

  double D_tb_D_tpLambda    = -D_tp_D_tpLambda*mBetaMax*omegaNorm;


  double D_tb_D_fiberAngularVelocity = D_tb_D_wn*D_wn_D_w;

  double D_tb_D_fiberAngle = -D_tp_D_fiberAngle*mBetaMax*omegaNorm;

  double D_tb_D_fiberAngleOffset= -D_tp_D_fiberAngleOffset*mBetaMax*omegaNorm;

  //Damping second derivatives
  double D2_tb_D_tpLambda_D_fiberAngleOffset =
    -D2_tp_D_tpLambda_D_fiberAngleOffset*mBetaMax*omegaNorm;
  double D2_tb_D_tpLambda2 =
    -D2_tp_D_tpLambda2*mBetaMax*omegaNorm;
  double D2_tb_D_fiberAngleOffset2 =
    -D2_tp_D_fiberAngleOffset2*mBetaMax*omegaNorm;

  //Sign conventions
  double D_fiberAngle_D_jointAngle = mSignOfJointAngle;
  double D_fiberAngularVelocity_D_jointAngularVelocity =
    mSignOfConcentricAnglularVelocity;


  updTmi.jointAngle           = jointAngle;
  updTmi.jointAngularVelocity = jointAngularVelocity;
  updTmi.fiberAngle           = fiberAngle;
  updTmi.fiberAngularVelocity = fiberAngularVelocity;

  updTmi.fiberPassiveTorqueAngleMultiplier    = tp;
  updTmi.fiberActiveTorqueAngleMultiplier     = ta;
  updTmi.fiberTorqueAngularVelocityMultiplier = tv;

  updTmi.activation         = activation;
  updTmi.fiberActiveTorque  = maxActIsoTorque*(activation*ta*tv);
  updTmi.fiberPassiveTorque = maxActIsoTorque*(tp+tb);
  updTmi.fiberPassiveElasticTorque = maxActIsoTorque*tp;
  updTmi.fiberDampingTorque = maxActIsoTorque*tb;
  updTmi.fiberNormDampingTorque = tb;

  updTmi.fiberTorque = updTmi.fiberActiveTorque + updTmi.fiberPassiveTorque;
  updTmi.jointTorque = mSignOfJointTorque*updTmi.fiberTorque;

  updTmi.fiberStiffness = maxActIsoTorque*(
                            activation*D_ta_D_fiberAngle*tv
                            + D_tp_D_fiberAngle
                            + D_tb_D_fiberAngle);

  updTmi.jointStiffness = mSignOfJointTorque
                          *updTmi.fiberStiffness
                          *D_fiberAngle_D_jointAngle;

  updTmi.fiberActivePower  = updTmi.fiberActiveTorque
                             * updTmi.fiberAngularVelocity;

  updTmi.fiberPassivePower = updTmi.fiberPassiveTorque
                             * updTmi.fiberAngularVelocity;

  updTmi.fiberPower        = updTmi.fiberActivePower
                             + updTmi.fiberPassivePower;

  updTmi.jointPower        = updTmi.jointTorque * jointAngularVelocity;


  updTmi.DfiberPassiveTorqueAngleMultiplier_DblendingVariable
    = D_tp_D_tpLambda;
  updTmi.DfiberActiveTorqueAngleMultiplier_DblendingVariable
    = D_ta_D_taLambda;
  updTmi.DfiberTorqueAngularVelocityMultiplier_DblendingVariable
    = D_tv_D_tvLambda;
  updTmi.DfiberPassiveTorqueAngleMultiplier_DangleOffset
    = D_tp_D_fiberAngleOffset;


  //tau = signTq*tauIso*(a*ta(theta)*tv(thetaDot) + tp(theta)*(1-beta*omegaNorm)
  // Dtau_Da = tauMaxIso*(ta(theta) * tv(thetaDot) )
  updTmi.DjointTorque_Dactivation =
    mSignOfJointTorque
    *maxActIsoTorque
    *(ta * tv);

  //tau = signTq*tauIso*(a*ta(theta)*tv(thetaDot) + tp(theta)*(1-beta*omegaNorm)
  //Dtau_Dtheta =   signTq*tauIso*(a*Dta_Dtheta(theta)*tv(thetaDot)
  //                          + Dtp_Dtheta(theta)*(1-beta*omegaNorm)
  updTmi.DjointTorque_DjointAngle           =
    mSignOfJointTorque
    * maxActIsoTorque
    * ( activation
        *D_ta_D_fiberAngle
        * tv
        + ( D_tp_D_fiberAngle
            + D_tb_D_fiberAngle)
      )* D_fiberAngle_D_jointAngle;


  //tau = signTq*tauIso*(a*ta(theta)*tv(thetaDot) + tp(theta)*(1-beta*omegaNorm)
  //Dtau_Domega = signTq*tauIso*(a * ta(theta) * Dtv_DthetaDot(thetaDot)
  //                             - tp(theta)*beta*DomegaNorm_thetaDot
  updTmi.DjointTorque_DjointAngularVelocity =
    mSignOfJointTorque
    * maxActIsoTorque
    * ( activation
        * ta
        * ( D_tv_D_fiberAngularVelocity
            *D_fiberAngularVelocity_D_jointAngularVelocity)
        + ( D_tb_D_fiberAngularVelocity
            *D_fiberAngularVelocity_D_jointAngularVelocity)
      );

  //tau = signTq*tauIso*(a*ta(theta)*tv(thetaDot) + tp(theta)*(1-beta*omegaNorm)
  //Dtau_DtaLambda = signTq*tauIso*(a * Dta(theta)_Dlambda * tv(thetaDot)
  updTmi.DjointTorque_DactiveTorqueAngleBlendingVariable =
    mSignOfJointTorque
    * maxActIsoTorque
    *(activation*D_ta_D_taLambda*tv);

  //tau = signTq*tauIso*(a*ta(theta)*tv(thetaDot) + tp(theta)*(1-beta*omegaNorm)
  //Dtau_DtpLambda = signTq*tauIso*( D_tp(theta)_D_lambda(1-beta*omegaNorm)
  updTmi.DjointTorque_DpassiveTorqueAngleBlendingVariable =
    mSignOfJointTorque
    * maxActIsoTorque
    *(D_tp_D_tpLambda + D_tb_D_tpLambda);

  //tau = signTq*tauIso*(a*ta(theta)*tv(thetaDot) + tp(theta)*(1-beta*omegaNorm)
  //Dtau_tvLambda = signTq*tauIso*(a*ta(theta)*Dtv_Dlambda(thetaDot)
  updTmi.DjointTorque_DtorqueAngularVelocityBlendingVariable =
    mSignOfJointTorque
    * maxActIsoTorque
    *(activation*ta*D_tv_D_tvLambda);

  // mSignOfJointTorque*( (maxActIsoTorque*(activation*ta*tv)
  //                    + maxActIsoTorque*(tp+tb)))
  updTmi.DjointTorque_DmaximumIsometricTorque =
    mSignOfJointTorque
    *((activation*ta*tv) + (tp+tb));

  updTmi.DjointTorque_DpassiveTorqueAngleCurveAngleOffset =
    mSignOfJointTorque
    *maxActIsoTorque
    *(D_tp_D_fiberAngleOffset + D_tb_D_fiberAngleOffset);
  //* D_fiberAngle_D_jointAngle;

  //tau = signTq*tauIso*(a*ta(theta)*tv(thetaDot) + tp(theta)+tb(theta,omega)
  //Dtau_DomegaMax = signTq*tauIso*(a*ta(theta)*Dtv_DomegaMax(thetaDot)
  //                              + tp(theta)*(-beta*D_omegaNorm_DomegaMax)
  updTmi.DjointTorque_DmaximumAngularVelocity =
    mSignOfJointTorque
    *maxActIsoTorque
    *(activation*ta*D_tv_D_omegaMax + D_tb_D_omegaMax);

  //tau = signTq*tauIso*(a*ta(theta)*tv(thetaDot) + tp(theta)+tb(theta,omega)
  //Dtau_DtaAngleScaling = signTq*tauIso*(a*Dta(theta)_DangleScaling*tv(thetaDot)

  updTmi.DjointTorque_DactiveTorqueAngleAngleScaling =
    mSignOfJointTorque
    *maxActIsoTorque
    *(activation*D_ta_D_angleScaling*tv);


  /*
   Second derivatives of the exposed variables, x, for fitting:

   Option 1:
      x = [taLambda,tvLambda,tpLambda,tpOffset]
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
      The numbers indicate the index of the quantity in fittingInfo.

    Option 2:
      x = [taAngleScaling, tvOmegaMax, tpLambda,tpOffset]
    for brevity
      x = [s, m, p, o]

     For reference:
        t(s,m,p,o) = mSignOfJointTorque*(
                      (maxActIsoTorque*(
                          activation*ta(s)*tv(m)) + (tp(p,o)+tb(p,o))))
     Where 't' is the shortform used to represent joint-torque.

     The lower right triangle for the Hessian of tau(s,m,p,o) w.r.t. x is
     stored, row-wise, in the vector fittingInfo. Thus we have

                             s           m           p           o
     H(tau(s,m,p,o),x) = s [ 0: d2t/ds2
                         m [ 1: d2t/dsdm 2: d2t/dm2
                         p [ 3: d2t/dsdp 4: d2t/dmdp 5: d2t/dp2
                         o [ 6: d2t/dsdo 7: d2t/dmdo 8: d2t/dpdo 9: d2t/do2
     The numbers indicate the index of the quantity in fittingInfo.

   So that both fitting options are possible, the extra entries for the
   above Hessian will just be concatentated to the existing vector. Three
   of the entries are duplicates, but for now they are being included
   just to make this slightly easier to use for now. Thus the vector of
   2nd derivatives will look like:

   For joint-torque related constraints that use x = [a,v,p,o]
    0: d2t/da2
    1: d2t/dadv
    2: d2t/dv2
    3: d2t/dadp
    4: d2t/dvdp
    5: d2t/dp2
    6: d2t/dado
    7: d2t/dvdo
    8: d2t/dpdo
    9: d2t/do2
   For constraints on the value of the passive element
    10: d2p/dp2
    11: d2p/dpdo
    12: d2p/do2
   For joint-torque related constraints that use x = [s,m,p,o]
    13: d2t/ds2
    14: d2t/dsdm
    15: d2t/dm2
    16: d2t/dsdp
    17: d2t/dmdp
    18: d2t/dp2
    19: d2t/dsdo
    20: d2t/dmdo
    21: d2t/dpdo
    22: d2t/do2

  */

  #ifdef RBDL_BUILD_ADDON_MUSCLE_FITTING
  updTmi.fittingInfo.conservativeResize(23);

  //0: d2t/da2
  updTmi.fittingInfo[0] = mSignOfJointTorque*(maxActIsoTorque*(
                            activation*D2_ta_D_taLambda2*tv));

  //1: d2t/dadv
  updTmi.fittingInfo[1] = mSignOfJointTorque*(maxActIsoTorque*
                          (activation*D_ta_D_taLambda*D_tv_D_tvLambda));

  //2: d2t/dv2
  updTmi.fittingInfo[2] = mSignOfJointTorque*(maxActIsoTorque*
                          (activation*ta*D2_tv_D_tvLambda2));

  //3: d2t/dadp
  updTmi.fittingInfo[3] = 0.;

  //4: d2t/dvdp
  updTmi.fittingInfo[4] = 0.;

  //5: d2t/dp2
  updTmi.fittingInfo[5] = mSignOfJointTorque*(maxActIsoTorque*(
                            D2_tp_D_tpLambda2+D2_tb_D_tpLambda2));
  //6: d2t/dado
  updTmi.fittingInfo[6] = 0.;
  //7: d2t/dvdo
  updTmi.fittingInfo[7] = 0.;
  //8: d2t/dpdo
  updTmi.fittingInfo[8] = mSignOfJointTorque*(maxActIsoTorque*(
                            D2_tp_D_tpLambda_D_fiberAngleOffset
                            +D2_tb_D_tpLambda_D_fiberAngleOffset
                          ));//*D_fiberAngle_D_jointAngle);
  //9: d2t/do2
  updTmi.fittingInfo[9] = mSignOfJointTorque*(maxActIsoTorque*(
                            D2_tp_D_fiberAngleOffset2
                            +D2_tb_D_fiberAngleOffset2
                          ));//*D_fiberAngle_D_jointAngle);
  //10: d2tp/dp2
  updTmi.fittingInfo[10] = D2_tp_D_tpLambda2;
  //11: d2tp/dpdo
  updTmi.fittingInfo[11] = D2_tp_D_tpLambda_D_fiberAngleOffset;
  //12: d2tp/do2
  updTmi.fittingInfo[12] = D2_tp_D_fiberAngleOffset2;

  //13: d2t/ds2
  updTmi.fittingInfo[13] = mSignOfJointTorque
                           *maxActIsoTorque
                           *(activation*D2_ta_D_angleScaling2*tv);
  //14: d2t/dsdm
  updTmi.fittingInfo[14] = mSignOfJointTorque
                           *maxActIsoTorque
                           *(activation*D_ta_D_angleScaling*D_tv_D_omegaMax);

  //15: d2t/dm2
  updTmi.fittingInfo[15] = mSignOfJointTorque
                           *maxActIsoTorque
                           *(activation*ta*D2_tv_D_omegaMax2 + D2_tb_D_omegaMax2);
  //16: d2t/dsdp
  updTmi.fittingInfo[16] = 0.;

  //17: d2t/dmdp
  updTmi.fittingInfo[17] = mSignOfJointTorque
                           *maxActIsoTorque
                           *(D2_tb_D_omegaMax_D_tpLambda);
  //18: d2t/dp2
  updTmi.fittingInfo[18] =  mSignOfJointTorque*(
                              maxActIsoTorque*(
                                D2_tp_D_tpLambda2+D2_tb_D_tpLambda2));
  //19: d2t/dsdo
  updTmi.fittingInfo[19] = 0.;

  //20: d2t/dmdo
  updTmi.fittingInfo[20] = mSignOfJointTorque
                           *maxActIsoTorque
                           *(D2_tb_D_omegaMax_D_tpOffset);

  //21: d2t/dpdo
  updTmi.fittingInfo[21] = mSignOfJointTorque
                           *(maxActIsoTorque
                             *(  D2_tp_D_tpLambda_D_fiberAngleOffset
                                 +D2_tb_D_tpLambda_D_fiberAngleOffset));

  //22: d2t/do2
  updTmi.fittingInfo[22] = mSignOfJointTorque
                           *(maxActIsoTorque
                             *( D2_tp_D_fiberAngleOffset2
                                +D2_tb_D_fiberAngleOffset2));

  #endif

}

//=============================================================================
void Millard2016TorqueMuscle::
updTorqueMuscleSummary( double activation,
                        double jointAngle,
                        double jointAngularVelocity,
                        double activeTorqueAngleBlendingVariable,
                        double passiveTorqueAngleBlendingVariable,
                        double torqueAngularVelocityBlendingVariable,
                        double activeTorqueAngleAngleScaling,
                        double activeTorqueAngleAtOneNormTorque,
                        double passiveTorqueAngleCurveOffset,
                        double maxAngularVelocity,
                        double maxActIsoTorque,
                        TorqueMuscleSummary &updTms) const
{
  double fiberAngle         = calcFiberAngle(jointAngle);
  double fiberVelocity      = calcFiberAngularVelocity(jointAngularVelocity);
  double fiberVelocityNorm  = fiberVelocity/maxAngularVelocity;


  updTorqueMuscleSummaryCurveValues(fiberAngle,
                                    fiberVelocityNorm,
                                    activeTorqueAngleBlendingVariable,
                                    passiveTorqueAngleBlendingVariable,
                                    torqueAngularVelocityBlendingVariable,
                                    activeTorqueAngleAngleScaling,
                                    activeTorqueAngleAtOneNormTorque,
                                    passiveTorqueAngleCurveOffset,
                                    updTms);
  updTms.fiberAngle           = fiberAngle;
  updTms.fiberAngularVelocity = fiberVelocity;
  updTms.activation           = activation;
  updTms.fiberTorque =
    maxActIsoTorque
    *(  activation*( updTms.fiberActiveTorqueAngleMultiplier
                     *updTms.fiberTorqueAngularVelocityMultiplier
                   )
        + (   updTms.fiberPassiveTorqueAngleMultiplier
              + updTms.fiberNormalizedDampingTorque
          )
     );

  updTms.jointTorque = updTms.fiberTorque*mSignOfJointTorque;
}

void Millard2016TorqueMuscle::
updInvertTorqueMuscleSummary(double jointTorque,
                             double jointAngle,
                             double jointAngularVelocity,
                             double activeTorqueAngleBlendingVariable,
                             double passiveTorqueAngleBlendingVariable,
                             double torqueAngularVelocityBlendingVariable,
                             double activeTorqueAngleAngleScaling,
                             double activeTorqueAngleAtOneNormTorque,
                             double passiveTorqueAngleCurveOffset,
                             double maxAngularVelocity,
                             double maxActIsoTorque,
                             TorqueMuscleSummary &updTms) const
{

  double fiberAngle         = calcFiberAngle(jointAngle);
  double fiberVelocity      = calcFiberAngularVelocity(jointAngularVelocity);
  double fiberVelocityNorm  = fiberVelocity/maxAngularVelocity;

  updTorqueMuscleSummaryCurveValues(fiberAngle,
                                    fiberVelocityNorm,
                                    activeTorqueAngleBlendingVariable,
                                    passiveTorqueAngleBlendingVariable,
                                    torqueAngularVelocityBlendingVariable,
                                    activeTorqueAngleAngleScaling,
                                    activeTorqueAngleAtOneNormTorque,
                                    passiveTorqueAngleCurveOffset,
                                    updTms);

  updTms.fiberAngle           = fiberAngle;
  updTms.fiberAngularVelocity = fiberVelocity;

  updTms.jointTorque = jointTorque;
  updTms.fiberTorque = jointTorque*mSignOfJointTorque;

  updTms.activation =
    ( (updTms.fiberTorque/maxActIsoTorque)
      - (updTms.fiberPassiveTorqueAngleMultiplier
         + updTms.fiberNormalizedDampingTorque)
    )/( updTms.fiberActiveTorqueAngleMultiplier
        *updTms.fiberTorqueAngularVelocityMultiplier);
}

//==============================================================================
DataSet::item Millard2016TorqueMuscle::getDataSet(){
  return mDataSet;
}
GenderSet::item Millard2016TorqueMuscle::getGender(){
  return mGender;
}
AgeGroupSet::item Millard2016TorqueMuscle::getAgeGroup(){
  return mAgeGroup;
}
JointTorqueSet::item Millard2016TorqueMuscle::getJointTorque(){
  return mJointTorque;
}
double Millard2016TorqueMuscle::getSubjectMass(){
  return mSubjectMassInKg;
}
double Millard2016TorqueMuscle::getSubjectHeight(){
  return mSubjectHeightInMeters;
}
