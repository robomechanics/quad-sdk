#ifndef TORQUEMUSCLEFUNCTIONFACTORY_H_
#define TORQUEMUSCLEFUNCTIONFACTORY_H_
/*-------------------------------------------------------------------------
                OpenSim:  SmoothSegmentedFunctionFactory.cpp
 --------------------------------------------------------------------------
 The OpenSim API is a toolkit for musculoskeletal modeling and simulation.
 See http:%opensim.stanford.edu and the NOTICE file for more information.
 OpenSim is developed at Stanford University and supported by the US
 National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA
 through the Warrior Web program.

 Copyright (c) 2005-2012 Stanford University and the Authors
 Author(s): Matthew Millard

 Licensed under the Apache License, Version 2.0 (the 'License'); you may
 not use this file except in compliance with the License. You may obtain a
 copy of the License at http:%www.apache.org/licenses/LICENSE-2.0.

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an 'AS IS' BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
 --------------------------------------------------------------------------

 Derivative work
 Date      : September 2016
 Authors(s): Millard
 Updates   : Made active torque-angle, passive-torque-angle, torque-velocity
             and tendon-torque-angle curves based on the equivalent line-type
             curves in OpenSim.
*/

#include "../geometry/SmoothSegmentedFunction.h"
#include "../geometry/SegmentedQuinticBezierToolkit.h"

#include <cstdio>
#include <iostream>
#include <fstream>
#include <cmath>

namespace RigidBodyDynamics {
  namespace Addons {
  namespace Muscle{

class TorqueMuscleFunctionFactory
{
  public:


  /**
  This is a function that will produce a C2 (continuous to the second
  derivative) active torque angle curve. This Bezier curve has been
  fitted to match the active-torque-angle curve described in

  Anderson, Dennis E., Michael L. Madigan, and Maury A. Nussbaum. 
  "Maximum voluntary joint torque as a function of joint angle and 
  angular velocity: model development and application to the lower 
  limb." Journal of biomechanics 40, no. 14 (2007): 3105-3113.

  but note that its range is normalized to [0,1].

  @param c2 (radians)
   The active-torque-angle width parameter. The parameter c2
   is defined by Anderson et al. as

  c2 = pi/(theta_max - theta_min). 

  @param c3 : (radians)
  Then angle which has the largest active-torque.

  @param curveName The name of the joint torque this curve applies to. This
    curve name should have the name of the joint and the
    direction (e.g. hipExtensionTorqueMuscle) so that if
    this curve ever causes an exception, a user friendly
    error message can be displayed to the end user to help
    them debug their model.

  @param smoothSegmentedFunctionToUpdate
      A SmoothSegmentedFunction object that will be erased and filled with 
      the coefficients that are defined by this curve.


*/
static void createAnderson2007ActiveTorqueAngleCurve(
    double c2, 
    double c3,
    const std::string& curveName,
    RigidBodyDynamics::Addons::Geometry::SmoothSegmentedFunction&
      smoothSegmentedFunctionToUpdate);

  /**
  This is a function that will produce a C2 (continuous to the second
  derivative) active torque (angular) velocity curve. This Bezier curve 
  has been fitted to match the active-torque-angle curve described in

  Anderson, Dennis E., Michael L. Madigan, and Maury A. Nussbaum. 
  "Maximum voluntary joint torque as a function of joint angle and 
  angular velocity: model development and application to the lower 
  limb." Journal of biomechanics 40, no. 14 (2007): 3105-3113.

  While the concentric side of the Bezier curve and the original 
  formulation match, the eccentric side does not: the equations 
  Anderson et al. chose decrease down to 0 rapidly. Since Anderson
  et al. did not collect data at the higher eccentric velocities the
  oddities in their chosen curves are likely due to the parameterization
  they chose. The eccentric side of the Bezier curve will be fitted
  so that, if possible, it passes close to the value of the original
  curves for theta = -60 deg/s within the limits imposed by 
  minEccentricMultiplier and maxEccentricMultiplier.

  @param c4 (rads/s)
  Angular velocity when the torque is 75% of the maximum 
  isometric torque.

  @param c5 (rads/s)
  Angular velocity when the torque is 50% of the maximum 
  isometric torque.

  @param c6 
  Multiplier that Anderson et al. uses to describe the 
  change in slope of the curve as the contraction velocity
  changes sign from + to -.

  @param minEccentricMultiplier
  The minimum value of the torque-(angular)-velocity curve 
  tends to at large eccentric contraction velocities. Note
  minEccentricMultiplier > 1.0

  @param maxEccentricMultiplier
  The value of the torque-(angular)-velocity curve tends
  to at large eccentric contraction velocities. Note
  maxEccentricMultiplier > minEccentricMultiplier.

  @param curveName The name of the joint torque this curve applies to. This
    curve name should have the name of the joint and the
    direction (e.g. hipExtensionTorqueMuscle) so that if
    this curve ever causes an exception, a user friendly
    error message can be displayed to the end user to help
    them debug their model.

  @param smoothSegmentedFunctionToUpdate
      A SmoothSegmentedFunction object that will be erased and filled with 
      the coefficients that are defined by this curve.

  */
  static void createAnderson2007ActiveTorqueVelocityCurve(
      double c4, 
      double c5,
      double c6,
      double minEccentricMultiplier,
      double maxEccentricMultiplier,
      const std::string& curveName,
      RigidBodyDynamics::Addons::Geometry::SmoothSegmentedFunction&
        smoothSegmentedFunctionToUpdate);

  /**
  This is a function that will produce a C2 (continuous to the second
  derivative) passive torque angle curve described in

  Anderson, Dennis E., Michael L. Madigan, and Maury A. Nussbaum. 
  "Maximum voluntary joint torque as a function of joint angle and 
  angular velocity: model development and application to the lower 
  limb." Journal of biomechanics 40, no. 14 (2007): 3105-3113.

  Note the following differences between this implementation and
  the original equations presented in Anderson et al.:
  
  1. This function will return a curve that is fitted to the
  positive side of the curve defined by the coefficients 
  b1, k1, b2, and k2. Because of the sign convention employed by
  Anderson et al. the positive side of the curve corresponds to
  the passive curve generated by the torque actuator associated
  with the rest of the coefficients.

  2. This function has been normalized so that a value of 1.0 
   corresponds to one-maximum-isometric-active-contraction
   torque, or c1*subjectWeightInNewtons*subjectHeightInMeters.

  @param scale
   The scaling factor used on the c1 column in Table 3 of 
   Anderson et al.:

   scale = subjectWeightInNewtons * subjectHeightInMeters

  @param c1
   The normalized c1 parameter listed in Tabel 3 of 
   Anderson et al.

  @param b1
   The passive torque angle curve parameter used in 
   Anderson et al.'s Eqn. 1:

   torquePassive = b1*exp(k1*theta) + b2*exp(k2*theta)

  @param k1
   The term k1 in Anderson et al.'s Eqn. 1.

  @param b2
   The term b2 in Anderson et al.'s Eqn. 1.

  @param k2
   The term k2 in Anderson et al.'s Eqn. 1.

  @param curveName The name of the joint torque this curve applies to. This
     curve name should have the name of the joint and the
     direction (e.g. hipExtensionTorqueMuscle) so that if
     this curve ever causes an exception, a user friendly
     error message can be displayed to the end user to help
     them debug their model.

  @param smoothSegmentedFunctionToUpdate
      A SmoothSegmentedFunction object that will be erased and filled with 
      the coefficients that are defined by this curve. 
  */
  static void createAnderson2007PassiveTorqueAngleCurve(
      double scale,
      double c1,
      double b1,
      double k1,
      double b2,
      double k2,      
      const std::string& curveName,
      RigidBodyDynamics::Addons::Geometry::SmoothSegmentedFunction&
        smoothSegmentedFunctionToUpdate);


  /**
    This function creates a normalized torque-velocity curve. The concentric
    side of the curve is fitted to Hill's hyperbola that passes through
    the value of tv-at-half-of-the-maximum-concentric-velocity (a parameter
    supplied by the user). The eccentric side of the curve rapidly, but smoothly
    approaches a terminal value of 
    tv-at-the-maximum-eccentric-contraction-velocity. Outside of the normalized
    velocities of -1 to 1 the curve takes the values of 0, and 
    tvAtEccentricOmegaMax respectively with a slope of 0.

    \image html fig_MuscleAddon_TorqueMuscleFunctionFactory_TorqueVelocityCurveSimple.png

    @param tvAtEccentricOmegaMax
       The value of the torque-velocity-multiplier at the maximum 
       eccentric contraction velocity. This value must be 

    @param tvAtHalfConcentricOmegaMax
       The value of the torque-velocity-

    @param curveName The name of the joint torque this curve applies to. This
       curve name should have the name of the joint and the
       direction (e.g. hipExtensionTorqueMuscle) so that if
       this curve ever causes an exception, a user friendly
       error message can be displayed to the end user to help
       them debug their model.

    @param smoothSegmentedFunctionToUpdate
        A SmoothSegmentedFunction object that will be erased and filled with 
        the coefficients that are defined by this curve. 

    <b>aborts</b>

    -tvAtEccentricOmegaMax < 1.05
    -tvAtHalfOmegaMax >= 0.45    
    -tvAtHalfOmegaMax <= 0.05

    <b>References</b>
    Hill, A. V. (1938). The heat of shortening and the dynamic constants of 
    muscle. Proceedings of the Royal Society of London B: Biological Sciences, 
    126(843), 136-195.

  */
  
  static void createTorqueVelocityCurve(
      double tvAtEccentricOmegaMax,
      double tvAtHalfConcentricOmegaMax,
      const std::string& curveName,
      RigidBodyDynamics::Addons::Geometry::SmoothSegmentedFunction&
        smoothSegmentedFunctionToUpdate );

  /**
    This function creates a normalized torque-velocity curve. The concentric
    side of the curve is fitted to Hill's hyperbola that passes through
    the value of tv-at-half-of-the-maximum-concentric-velocity (a parameter
    supplied by the user). The eccentric side of the curve rapidly, but smoothly
    approaches a terminal value of 
    tv-at-the-maximum-eccentric-contraction-velocity. Shape of the eccentric
    side of the curve can be changed using the slopeNearEccentricOmegaMax 
    and curviness variables. Outside of the normalized
    velocities of -1 to 1 the curve takes the values of 
    slopeAtConcentricOmegaMax and slopeAtEccentricOmegaMax respectively.

    \image html fig_MuscleAddon_TorqueMuscleFunctionFactory_TorqueVelocityCurve.png

    @param tvAtEccentricOmegaMax
       The value of the torque-velocity-multiplier at the maximum 
       eccentric contraction velocity. This value must be 

    @param tvAtHalfConcentricOmegaMax
       The value of the torque-velocity-

    @param slopeAtConcentricOmegaMax
      The slope of the curve at a normalized angular velocity of -1. This slope
      is used to extrapolate \f$\mathbf{t}_V\f$ for normalized angular velocities
      of less than -1.

    @param slopeNearEccentricOmegaMax
      The slope of the eccentric side of the curve as the normalized angular
      velocity approaches 1.

    @param slopeAtEccentricOmegaMax
      The slope of the curve at a normalized angular velocity of 1. This slope
      is used to extrapolate \f$\mathbf{t}_V\f$ for normalized angular velocities
      of greater than 1.

    @param eccentricCurviness
      This parameter controls the shape of the curve between the normalized
      angular velocities of 0 and 1. An eccentricCurviness of 0 will flatten
      the elbow so that the curve closely follows a line that begins at (0,1)
      and ends at (1,tvAtEccentricOmegaMax). An eccentricCurviness of 1 will
      give the curve a strong elbow so that it quickly approaches the line that
      passes through the point (1,tvAtEccentricOmegaMax) and has a slope of 
      slopeNearEccentricOmegaMax.

    @param curveName The name of the joint torque this curve applies to. This
       curve name should have the name of the joint and the
       direction (e.g. hipExtensionTorqueMuscle) so that if
       this curve ever causes an exception, a user friendly
       error message can be displayed to the end user to help
       them debug their model.

    @param smoothSegmentedFunctionToUpdate
        A SmoothSegmentedFunction object that will be erased and filled with 
        the coefficients that are defined by this curve. 

    <b>aborts</b>

    -tvAtEccentricOmegaMax < 1.05
    -tvAtHalfOmegaMax > 0.45  or tvAtHalfOmegaMax < 0.05
    -slopeAtConcentricOmegaMax < 0
    -slopeNearEccentricOmegaMax < 0
    -slopeAtEccentricOmegaMax < 0
    -eccentricCurviness < 0 or eccentricCurviness > 1

    <b>References</b>
    Hill, A. V. (1938). The heat of shortening and the dynamic constants of 
    muscle. Proceedings of the Royal Society of London B: Biological Sciences, 
    126(843), 136-195.

  */
  static void createTorqueVelocityCurve(
      double tvAtEccentricOmegaMax,
      double tvAtHalfConcentricOmegaMax,
      double slopeAtConcentricOmegaMax,
      double slopeNearEccentricOmegaMax,
      double slopeAtEccentricOmegaMax,
      double eccentricCurviness,
      const std::string& curveName,
      RigidBodyDynamics::Addons::Geometry::SmoothSegmentedFunction&
        smoothSegmentedFunctionToUpdate );

  /**
    This function creates a Bezier spline that closely follows the 
    exponential curves that are typically used to model the passive-torque-angle
    characteristic of muscles. This curve has a value and a slope of zero
    for angles that are less than abs(angleAtZeroTorque). For angles that
    have an absolute magnitude larger than abs(angleAtOneNormTorque) the curve
    is simply linearly extrapolated.

    Note that curves can be represented that increase left-to-right, or 
    decrease left-to-right by setting the variables angleAtOneNormTorque and
    angleAtZeroTorque correctly. For example using (0,1) for 
    angleAtOneNormTorque and angleAtZeroTorque produces a curve that increases
    left-to-right while using (-1,0) produces a curve that decreases left to 
    right.

    \image html fig_MuscleAddon_TorqueMuscleFunctionFactory_PassiveTorqueAngleCurveSimple.png

    @param angleAtZeroTorque is the angle at which the curve transitions from
    a flat line and begins curving upwards. (radians)

    @param angleAtOneNormTorque is the angle at which this curve achieves a 
    value of 1.0. (radians)

    @param curveName The name of the joint torque this curve applies to. This
       curve name should have the name of the joint and the
       direction (e.g. hipExtensionTorqueMuscle) so that if
       this curve ever causes an exception, a user friendly
       error message can be displayed to the end user to help
       them debug their model.

    @param smoothSegmentedFunctionToUpdate
        A SmoothSegmentedFunction object that will be erased and filled with 
        the coefficients that are defined by this curve. 

    <b>aborts</b>

    - abs(angleAtOneNormTorque-angleAtZeroTorque) < sqrt(eps)        
  */
  static void createPassiveTorqueAngleCurve(
      double angleAtZeroTorque,    
      double angleAtOneNormTorque,
      const std::string& curveName,
      RigidBodyDynamics::Addons::Geometry::SmoothSegmentedFunction&
        smoothSegmentedFunctionToUpdate );


/**
    This function creates a Bezier spline that closely follows the 
    exponential curves that are typically used to model the passive-torque-angle
    characteristic of muscles. This curve has a value and a slope of zero
    for angles that are less than abs(angleAtZeroTorque). For angles that
    have an absolute magnitude larger than abs(angleAtOneNormTorque) the curve
    is simply linearly extrapolated.

    Note that curves can be represented that increase left-to-right, or 
    decrease left-to-right by setting the variables angleAtOneNormTorque and
    angleAtZeroTorque correctly. For example using (0,1) for 
    angleAtOneNormTorque and angleAtZeroTorque produces a curve that increases
    left-to-right while using (-1,0) produces a curve that decreases left to 
    right.

    \image html fig_MuscleAddon_TorqueMuscleFunctionFactory_PassiveTorqueAngleCurve.png

    @param angleAtZeroTorque is the angle at which the curve transitions from
    a flat line and begins curving upwards. (radians)

    @param angleAtOneNormTorque is the angle at which this curve achieves a 
    value of 1.0. (radians)

    @param stiffnessAtLowTorque
      The normalized stiffness (or slope) of the curve achieves as it begins
      to increase. This is usually chosen to be a small, but non-zero fraction 
      of stiffnessAtOneNormTorque 
      (stiffnessAtLowTorque = 0.025 stiffnessAtOneNormTorque is typical).
      The sign of stiffnessAtLowTorque must be positive if 
      angleAtOneNormTorque > angleAtZeroPassiveTorque. The sign 
      of stiffnessAtLowTorque must be negative if 
      angleAtOneNormTorque < angleAtZeroPassiveTorque.
      (Norm.Torque/radians)  

    @param stiffnessAtOneNormTorque
      The normalized stiffness (or slope) of the fiber curve 
      when the fiber is stretched by  
      angleAtOneNormTorque - angleAtZeroPassiveTorque. The sign 
      of stiffnessAtOneNormTorque must agree with stiffnessAtLowTorque.
      (Norm.Torque/radians)

    @param curviness
      The dimensionless 'curviness' parameter that 
       can vary between 0 (a line) to 1 (a smooth, but 
      sharply bent elbow). A value of 0.5 is typical as it produces a 
       graceful curve.

    @param curveName The name of the joint torque this curve applies to. This
       curve name should have the name of the joint and the
       direction (e.g. hipExtensionTorqueMuscle) so that if
       this curve ever causes an exception, a user friendly
       error message can be displayed to the end user to help
       them debug their model.

    @param smoothSegmentedFunctionToUpdate
        A SmoothSegmentedFunction object that will be erased and filled with 
        the coefficients that are defined by this curve. 

    <b>aborts</b>

    - abs(angleAtOneNormTorque-angleAtZeroTorque) < sqrt(eps)   
    - sign(stiffnessAtLowTorque) != sign(angleAtOneNormTorque-angleAtLowTorque)     
    - sign(stiffnessAtOneNormTorque) != sign(stiffnessAtLowTorque)
    - abs(stiffnessAtLowTorque) > 0.9/abs(angleAtOneNormTorque-angleAtZeroTorque)
    - abs(stiffnessAtOneTorque) <= 1.1/abs(angleAtOneNormTorque-angleAtZeroTorque)
    - curviness < 0 or curviness > 1

  */
  static void createPassiveTorqueAngleCurve(
      double angleAtZeroTorque,
      double angleAtOneNormTorque,
      double stiffnessAtLowTorque,      
      double stiffnessAtOneNormTorque,
      double curviness,
      const std::string& curveName,
      RigidBodyDynamics::Addons::Geometry::SmoothSegmentedFunction&
        smoothSegmentedFunctionToUpdate );


  /**
    This function produces a Bezier curve fitted to a Gaussian function. As the
    tails of the Gaussian curve become small this curve is simply extrapolated
    as a line with a y-value of zero and a slope of zero. 

    \image html fig_MuscleAddon_TorqueMuscleFunctionFactory_GaussianActiveTorqueAngleCurveSimple.png

    @param angleAtOneNormTorque The angle at which the Gaussian curve develops
      a value of 1. 

    @param angularStandardDeviation The angular deviation from 
      the mean at which the Gaussian curve reaches a value of \f$e^{-1/2}\f$.

    @param curveName The name of the joint torque this curve applies to. This
       curve name should have the name of the joint and the
       direction (e.g. hipExtensionTorqueMuscle) so that if
       this curve ever causes an exception, a user friendly
       error message can be displayed to the end user to help
       them debug their model.

    @param smoothSegmentedFunctionToUpdate
        A SmoothSegmentedFunction object that will be erased and filled with 
        the coefficients that are defined by this curve. 

    <b>aborts</b>

    - angularWidthOfOneStandardDeviation < sqrt(eps)  
  */
  static void createGaussianShapedActiveTorqueAngleCurve(
      double angleAtOneNormTorque,
      double angularStandardDeviation,
      const std::string& curveName,
      RigidBodyDynamics::Addons::Geometry::SmoothSegmentedFunction&
        smoothSegmentedFunctionToUpdate 
    );


/**
    This function produces a C2 continuous Bezier curve fitted to a Gaussian 
    function. As the tails of the Gaussian curve become less than 
    minValueAtShoulders, the curve is linearly extrapolated at a sloe of 
    minSlopeOfShoulders.

    \image html fig_MuscleAddon_TorqueMuscleFunctionFactory_GaussianActiveTorqueAngleCurve.png

    @param angleAtOneNormTorque The angle at which the Gaussian curve develops
      a value of 1. 

    @param angularStandardDeviation The angular deviation from 
      the mean at which the Gaussian curve reaches a value of \f$e^{-1/2}\f$.

    @param minSlopeAtShoulders The y-value at which the Bezier curve transitions
    from having a shape like a Gaussian curve to being linearly extrapolated.

    @param minValueAtShoulders The slope of the linear extrapolation of the 
      Bezier curve for y-values that are less than minSlopeAtShoulders. The 
      sign of minValueAtShoulders is automatically set so that it matches the 
      curve near it (see the figure).

    @param curviness
      The dimensionless 'curviness' parameter that 
       can vary between 0 (a line) to 1 (a smooth, but 
      sharply bent elbow). A value of 0.5 is typical as it produces a 
       graceful curve.

    @param curveName The name of the joint torque this curve applies to. This
       curve name should have the name of the joint and the
       direction (e.g. hipExtensionTorqueMuscle) so that if
       this curve ever causes an exception, a user friendly
       error message can be displayed to the end user to help
       them debug their model.

    @param smoothSegmentedFunctionToUpdate
        A SmoothSegmentedFunction object that will be erased and filled with 
        the coefficients that are defined by this curve. 

    <b>aborts</b>

    - angularWidthOfOneStandardDeviation < sqrt(eps)  
    - minSlopeAtShoulders < 0
    - minValueAtShoulders < 0
    - curviness > 1 or curviness < 0

  */
  static void createGaussianShapedActiveTorqueAngleCurve(
      double angleAtOneNormTorque,
      double angularStandardDeviation,
      double minSlopeAtShoulders,
      double minValueAtShoulders,
      double curviness,
      const std::string& curveName,
      RigidBodyDynamics::Addons::Geometry::SmoothSegmentedFunction&
        smoothSegmentedFunctionToUpdate 
    );


  /**
    This function produces a normalized tendon-torque-angle curve with a 
    toe region that is in the range of \f$\mathbf{t}_V\f$ [0,1./3,] after which
    the curve is linearly extrapolated.

    \image html fig_MuscleAddon_TorqueMuscleFunctionFactory_TendonTorqueAngleCurveSimple.png

    @param angularStretchAtOneNormTorque The amount of angular stretch of the 
      joint as the tendon goes from developing zero torque at its slack length
      to developing one maximum isometric torque. (radians)

    @param curveName The name of the joint torque this curve applies to. This
       curve name should have the name of the joint and the
       direction (e.g. hipExtensionTorqueMuscle) so that if
       this curve ever causes an exception, a user friendly
       error message can be displayed to the end user to help
       them debug their model.

    @param smoothSegmentedFunctionToUpdate
        A SmoothSegmentedFunction object that will be erased and filled with 
        the coefficients that are defined by this curve. 

    <b>aborts</b>

    - angularWidthOfOneStandardDeviation < sqrt(eps) 

  */
  static void createTendonTorqueAngleCurve(
      double angularStretchAtOneNormTorque,
      const std::string& curveName,
      RigidBodyDynamics::Addons::Geometry::SmoothSegmentedFunction&
        smoothSegmentedFunctionToUpdate 
    );  

  /**
    This function produces a normalized tendon-torque-angle curve with a 
    toe region, final stiffness, and shape that can be controlled.

    \image html fig_MuscleAddon_TorqueMuscleFunctionFactory_TendonTorqueAngleCurve.png

    @param angularStretchAtOneNormTorque The amount of angular stretch of the 
      joint as the tendon goes from developing zero torque at its slack length
      to developing one maximum isometric torque. (radians)

    @param stiffnessAtOneNormTorque The linear stiffness value of the tendon
      that is used for all y-values greater than the toe region. 
      (Norm. Torque/rad)

    @param normTorqueAtToeEnd The normalized torque value which defines the
      end of the nonlinear-stiffness region of the tendon and the beginning of
      the linear stiffness region of the tendon.

    @param curviness
      The dimensionless 'curviness' parameter that 
       can vary between 0 (a line) to 1 (a smooth, but 
      sharply bent elbow). A value of 0.5 is typical as it produces a 
       graceful curve.

    @param curveName The name of the joint torque this curve applies to. This
       curve name should have the name of the joint and the
       direction (e.g. hipExtensionTorqueMuscle) so that if
       this curve ever causes an exception, a user friendly
       error message can be displayed to the end user to help
       them debug their model.

    @param smoothSegmentedFunctionToUpdate
        A SmoothSegmentedFunction object that will be erased and filled with 
        the coefficients that are defined by this curve. 

    <b>aborts</b>
    
    - angularStretchAtOneNormTorque < sqrt(eps) 
    - stiffnessAtOneNormTorque < 1.1/angularStretchAtOneNormTorque
    - normTorqueAtToeEnd < sqrt(eps) or normTorqueAtToeEnd > 0.99
    - curviness < 0 or curviness > 1

  */
  static void createTendonTorqueAngleCurve(
      double angularStretchAtOneNormTorque,
      double stiffnessAtOneNormTorque,
      double normTorqueAtToeEnd,
      double curviness,
      const std::string& curveName,
      RigidBodyDynamics::Addons::Geometry::SmoothSegmentedFunction&
        smoothSegmentedFunctionToUpdate 
    );  

  /**
    This function creates a C2 sigmoid function that varies
    from 0 to 1 and is used to ramp up the muscle's passive
    damping. The
    blending variable starts to go from 0 at an angular
    velocity of 0. It reaches a value of 1 at the
    normAngularVelocityAtMaximumDamping

    \image html fig_MuscleAddon_TorqueMuscleFunctionFactory_DampingBlendingCurve.png

    @param  normAngularVelocityAtMaximumDamping the normalized
    angular velocity at which the blending function reaches
    a value of 1. This parameter must have an absolute magnitude
    greater than 0.

    @param curveName The name of the joint torque this curve applies to. This
       curve name should have the name of the joint and the
       direction (e.g. hipExtensionTorqueMuscle) so that if
       this curve ever causes an exception, a user friendly
       error message can be displayed to the end user to help
       them debug their model.

    @param smoothSegmentedFunctionToUpdate
        A SmoothSegmentedFunction object that will be erased and filled with 
        the coefficients that are defined by this curve.    
  */
  static void createDampingBlendingCurve(
        double normAngularVelocityAtMaximumDamping,
        const std::string& curveName,
        RigidBodyDynamics::Addons::Geometry::SmoothSegmentedFunction&
        smoothSegmentedFunctionToUpdate
      );

};
}
}
}
#endif //TORQUEMUSCLEFUNCTIONFACTORY_H_
