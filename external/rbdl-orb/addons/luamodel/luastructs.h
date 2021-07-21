//==============================================================================
/* 
 * RBDL - Rigid Body Dynamics Library: Addon : luamodel structs
 * Copyright (c) 2020 Matthew Millard <millard.matthew@gmail.com>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#ifndef LUASTRUCTS_H
#define LUASTRUCTS_H

#include <iostream>
#include <sstream>
#include <assert.h>
#include <cstdlib>
#include <string>
#include <vector>

#include <limits>
#include <rbdl/rbdl_math.h>

#include <rbdl/rbdl_config.h>
#include <rbdl/rbdl_errors.h>

/**
  A struct for a named body-fixed-point. The names used in this
  struct should be updated but remain as seen below for historical
  reasons. In the future perhaps something clearer such as
  BodyFixedPoint should be used with fields of name, body_id,
  body_name, and r.

  @param name the name of the point
  @param body_id the integer id of the body that this point is fixed to
  @param body_name the name of the body that this point is fixed to
  @param point_local the coordinates of this point relative to the body's
          origin in the coordinates of the body.
*/
struct Point {
  Point() :
    name ("unknown"),
    body_id (std::numeric_limits<unsigned int>::signaling_NaN()),
    body_name (""),
    point_local (
      std::numeric_limits<double>::signaling_NaN(),
      std::numeric_limits<double>::signaling_NaN(),
      std::numeric_limits<double>::signaling_NaN()
      )
  { }

  std::string name;
  unsigned int body_id;
  std::string body_name;
  RigidBodyDynamics::Math::Vector3d point_local;
};

/**
  A struct for a named motion capture marker.

  @param name the name of the marker
  @param body_id the integer id of the body that this point is fixed to
  @param body_name the name of the body that this point is fixed to
  @param point_local the coordinates of this point relative to the body's
          origin in the coordinates of the body.
*/
struct MotionCaptureMarker {
  MotionCaptureMarker() :
    name ("unknown"),
    body_id (std::numeric_limits<unsigned int>::signaling_NaN()),
    body_name (""),
    point_local (
      std::numeric_limits<double>::signaling_NaN(),
      std::numeric_limits<double>::signaling_NaN(),
      std::numeric_limits<double>::signaling_NaN()
      )
  { }

  std::string name;
  unsigned int body_id;
  std::string body_name;
  RigidBodyDynamics::Math::Vector3d point_local;
};

/**
  A struct for a named body fixed frame.

  @param name the name of the point
  @param body_id the integer id of the body that this local frame is fixed to
  @param body_name the name of the body that this local frame is fixed to
  @param r the translation from the body's origin to the origin of the
         local frame, in the coordinates of the body.
  @param E the rotation matrix that transforms vectors from the
         coordinates of the local frame to the coordinates of the body.
*/
struct LocalFrame {
  LocalFrame() :
    name ("unknown"),
    body_id (std::numeric_limits<unsigned int>::signaling_NaN()),
    body_name (""),
    r ( std::numeric_limits<double>::signaling_NaN(),
        std::numeric_limits<double>::signaling_NaN(),
        std::numeric_limits<double>::signaling_NaN()),
    E ( std::numeric_limits<double>::signaling_NaN(),
        std::numeric_limits<double>::signaling_NaN(),
        std::numeric_limits<double>::signaling_NaN(),
        std::numeric_limits<double>::signaling_NaN(),
        std::numeric_limits<double>::signaling_NaN(),
        std::numeric_limits<double>::signaling_NaN(),
        std::numeric_limits<double>::signaling_NaN(),
        std::numeric_limits<double>::signaling_NaN(),
        std::numeric_limits<double>::signaling_NaN())
  { }
  std::string name;
  unsigned int body_id;
  std::string body_name;
  RigidBodyDynamics::Math::Vector3d r;
  RigidBodyDynamics::Math::Matrix3d E;
};

/**
  Generic data about a human subject.
  @param gender: "male" or "female"
  @param age: in years
  @param age_group : "Young18To25" / "Middle55to65" / "SeniorOver65"
  @param height: in meters
  @param mass: in kilograms
*/
struct HumanMetaData {
  HumanMetaData() :
    gender("nan"),
    age(std::numeric_limits<double>::signaling_NaN()),
    age_group("nan"),    
    height(std::numeric_limits<double>::signaling_NaN()),
    mass(std::numeric_limits<double>::signaling_NaN())    
  { }
  std::string   gender;
  std::string   age_group;
  double        age;
  double        height;
  double        mass;
};

#ifdef RBDL_BUILD_ADDON_MUSCLE
/**
  A structure that contains all of the information needed to make a joint
  specific Millard2016TorqueMuscle (MTG) using the Millard2016TorqueMuscle class
  and to correctly evaluate its torque and update the model's generalized force
  vector.

  Many of these parameters are used to map the default sign conventions of the
  Millard2016TorqueMuscle (MTG) to the specific model that it is being applied
  to. These mappings currently have to be done manually by making use of the
  model's topology, the default sign conventions of the Millard2016TorqueMuscles
  (see the diagrams that accompany the doxygen) and the right hand rule. Please
  note that while the diagrams that appear in the Millard2016TorqueMuscle are
  in units of degrees this is for intuition and convenience: the curves are
  used in units of radians.

  @param name : [Manditory field]
    The name of the Millard2016TorqueMuscle to choose from the data_set.
    This name must contain an entry that appears in the JointTorque enum that
    appears in Millard2016TOrqueMuscle. Extra text can be added to these names
    to give each MTG a distinct name. For example "HipExtension" is an MTG
    that appears in the JointTorque enum and so "HipExtension_R" and
    "HipExtension_L" are names that will pick out the HipExtention MTG from
    the data_set.

  @param angle_sign : [Manditory field]
    see signOfJointAngleRelativeToDoxygenFigures in Millard2016TorqueMuscle

  @param torque_sign: [Manditory field]
    see signOfJointTorqueToDoxygenFigures in Millard2016TorqueMuscle

  @param body : [Manditory field]
    the MTG applies a torque to the parent joint of this body. For example,
    if the "thigh" segment has a parent of the "pelvis", then a hip torque
    would be applied to the "thigh".

  @param joint_index :
    The index of the joint (to the parent) that this MTG applies torque to.
    For example if the hip flexion MTG will apply torque to the 0th joint_index
    of the "thigh" segment if it is attached to the "pelvis" segment through
    a ball and socket joint with the 0th axis in the flexion/extension
    direction.

  @param q_scale:
    Linear scaling applied to the generalized coordinate prior to evaluating
    the MTG torque. Please note that while the plots of the MTGs for

  @param activation_index:
    Index of this MTG's activation signal in the state vector

  @param q_index: [Internal parameter]
    Index of this MTG's generalized position coordinate in the state vector.

  @param qdot_index [Internal parameter]
    Index of this MTG's generalized velocity coordinate in the state vector

  @param force_index [Internal parameter]
    Index of this MTG's generalized force in the state vector

  @param data_set
    The data set that is used to make the curves : see the DataSet struct in
    Millard2016TorqueMuscle.h for details. By default the Gymnast data set
    (the most complete data set) is used.

  @param joint_angle_offset :
    see jointAngleOffsetRelativeToDoxygenFigures in Millard2016TorqueMuscle

  @param activation_time_constant
    the activatation time constaint of this MTG

  @param deactivation_time_constant
    the deactivatation time constaint of this MTG

  @param passive_element_damping_coeff
    Manually sets the damping coefficient used for the passive element

  @param passive_element_torque_scale
    Manually scales the passive curve

  @param passive_element_angle_offset
    Manually shifts the passive torque angle curve

  @param max_isometric_torque
    An output parameter from the
    TorqueMuscleFittingToolkit::fitTorqueMuscleParameters

  @param max_angular_velocity
    An output parameter from the
    TorqueMuscleFittingToolkit::fitTorqueMuscleParameters

  @param active_torque_angle_blending
    An output parameter from the
    TorqueMuscleFittingToolkit::fitTorqueMuscleParameters

  @param passive_torque_angle_blending
    An output parameter from the
    TorqueMuscleFittingToolkit::fitTorqueMuscleParameters

  @param torque_velocity_blending
    An output parameter from the
    TorqueMuscleFittingToolkit::fitTorqueMuscleParameters

  @param active_torque_angle_scale
    An output parameter from the
    TorqueMuscleFittingToolkit::fitTorqueMuscleParameters

  @param fit_passive_torque_scale
    Pass in tuple (an q value, a passive torque, 0) and the passive curve
    will be scaled so that it passes through this point exactly by calling
    Millard2016TorqueMuscle::fitPassiveTorqueScale. 

    Note: use fit_passive_torque_scale or fit_passive_torque_offset but not
          both

  @param fit_passive_torque_offset
    Pass in tuple (an q value, a passive torque, 0) and the passive curve
    will be shifted so that it passes through this point exactly
    Millard2016TorqueMuscle::fitPassiveCurveAngleOffset

    Note: use fit_passive_torque_scale or fit_passive_torque_offset but not
          both


*/
struct Millard2016TorqueMuscleConfig {
  Millard2016TorqueMuscleConfig() :
    name(""),
    angle_sign(std::numeric_limits<double>::signaling_NaN()),
    torque_sign(std::numeric_limits<double>::signaling_NaN()),
    body(""),
    joint_index(0),
    q_scale(1.0),
    activation_index(std::numeric_limits<unsigned int>::signaling_NaN()),
    q_index(0),
    qdot_index(0),
    force_index(0),
    data_set("Gymnast"),
    age_group(""),
    gender(""),
    joint_angle_offset(0.0),
    activation_time_constant(0.015), //Thelen 2003, Adjustment of muscle ...
    deactivation_time_constant(0.05),//Thelen 2003, Adjustment of muscle ...
    max_isometric_torque(std::numeric_limits<double>::signaling_NaN()),
    max_isometric_torque_scale(std::numeric_limits<double>::signaling_NaN()),
    max_angular_velocity(std::numeric_limits<double>::signaling_NaN()),
    max_angular_velocity_scale(std::numeric_limits<double>::signaling_NaN()),
    passive_element_damping_coeff(std::numeric_limits<double>::signaling_NaN()),
    passive_element_torque_scale(std::numeric_limits<double>::signaling_NaN()),
    passive_element_angle_offset(std::numeric_limits<double>::signaling_NaN()),
    active_torque_angle_blending(std::numeric_limits<double>::signaling_NaN()),
    passive_torque_angle_blending(std::numeric_limits<double>::signaling_NaN()),
    torque_velocity_blending(std::numeric_limits<double>::signaling_NaN()),
    active_torque_angle_scale(std::numeric_limits<double>::signaling_NaN()),
    fit_passive_torque_scale(RigidBodyDynamics::Math::Vector3d(
                             std::numeric_limits<double>::signaling_NaN(),
                             std::numeric_limits<double>::signaling_NaN(),
                             std::numeric_limits<double>::signaling_NaN())),
    fit_passive_torque_offset(RigidBodyDynamics::Math::Vector3d(
                              std::numeric_limits<double>::signaling_NaN(),
                              std::numeric_limits<double>::signaling_NaN(),
                              std::numeric_limits<double>::signaling_NaN())){
    }
  std::string   name;
  double        angle_sign;
  double        torque_sign;
  std::string   body;
  unsigned int  joint_index;
  double        q_scale;
  unsigned int  activation_index;
  unsigned int  q_index;      //internal parameter
  unsigned int  qdot_index;   //internal parameter
  unsigned int  force_index;  //internal parameter
  std::string   data_set;
  std::string   age_group;
  std::string   gender;
  //double        angle_scale;
  double        joint_angle_offset;
  double        activation_time_constant;
  double        deactivation_time_constant;
  double        max_isometric_torque;
  double        max_angular_velocity;
  double        max_isometric_torque_scale;
  double        max_angular_velocity_scale;
  double        passive_element_damping_coeff;
  double        passive_element_torque_scale;
  double        passive_element_angle_offset;
  double        active_torque_angle_blending;
  double        passive_torque_angle_blending;
  double        torque_velocity_blending;
  double        active_torque_angle_scale;
  //double        torque_velocity_scaling;
  //double        passive_torque_angle_scaling;
  RigidBodyDynamics::Math::Vector3d        fit_passive_torque_scale;
  RigidBodyDynamics::Math::Vector3d        fit_passive_torque_offset;
};
#endif

/* LUASTRUCTS_H */
#endif
