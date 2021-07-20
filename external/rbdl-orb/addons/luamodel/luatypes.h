//==============================================================================
/* 
 * RBDL - Rigid Body Dynamics Library: Addon : luamodel types
 * Copyright (c) 2013 Martin Felis <martin@fyxs.org>.
 *               2020  Matthew Millard <millard.matthew@gmail.com> (extension)
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#ifndef LUATYPES_H
#define LUATYPES_H

#include <iostream>
#include <sstream>
#include <string>
#include <assert.h>
#include <cstdlib>
#include <string>
#include <vector>

#include <rbdl/rbdl.h>
#include <rbdl/rbdl_math.h>

#include <rbdl/rbdl_config.h>
#include <rbdl/rbdl_errors.h>

#include "luastructs.h"


//==============================================================================
template<>
RigidBodyDynamics::Math::Vector3d
RBDL_ADDON_DLLAPI
LuaTableNode::getDefault<RigidBodyDynamics::Math::Vector3d>(
    const RigidBodyDynamics::Math::Vector3d &default_value)
{
  RigidBodyDynamics::Math::Vector3d result = default_value;

  if (stackQueryValue()) {
    LuaTable vector_table = LuaTable::fromLuaState (luaTable->L);

    if (vector_table.length() != 3) {
      throw RigidBodyDynamics::Errors::RBDLFileParseError(
            "LuaModel Error: invalid 3d vector!");
    }

    result[0] = vector_table[1];
    result[1] = vector_table[2];
    result[2] = vector_table[3];
  }

  stackRestore();

  return result;
}

//==============================================================================
template<>
RigidBodyDynamics::Math::SpatialVector
RBDL_ADDON_DLLAPI
LuaTableNode::getDefault<RigidBodyDynamics::Math::SpatialVector>(
  const RigidBodyDynamics::Math::SpatialVector &default_value
)
{
  RigidBodyDynamics::Math::SpatialVector result = default_value;

  if (stackQueryValue()) {
    LuaTable vector_table = LuaTable::fromLuaState (luaTable->L);

  //! [Parse Failed]
    if (vector_table.length() != 6) {
      throw RigidBodyDynamics::Errors::RBDLFileParseError(
            "LuaModel Error: invalid 6d vector!");
    }
  //! [Parse Failed]

    result[0] = vector_table[1];
    result[1] = vector_table[2];
    result[2] = vector_table[3];
    result[3] = vector_table[4];
    result[4] = vector_table[5];
    result[5] = vector_table[6];
  }

  stackRestore();

  return result;
}

//==============================================================================
template<>
RigidBodyDynamics::Math::MatrixNd
RBDL_ADDON_DLLAPI
LuaTableNode::getDefault<RigidBodyDynamics::Math::MatrixNd>(
    const RigidBodyDynamics::Math::MatrixNd &default_value)
{
  RigidBodyDynamics::Math::MatrixNd result = default_value;

  if (stackQueryValue()) {
    LuaTable vector_table = LuaTable::fromLuaState (luaTable->L);

    result.resize( int(vector_table.length()),
                   int(vector_table[1].length()));

    for(int r=0; r<int(vector_table.length()); ++r) {
      for(int c=0; c<int(vector_table[1].length()); ++c) {
        result(r,c) = vector_table[r+1][c+1];
      }
    }
  }
  stackRestore();

  return result;
}

//==============================================================================
template<>
RigidBodyDynamics::Math::Matrix3d
RBDL_ADDON_DLLAPI
LuaTableNode::getDefault<RigidBodyDynamics::Math::Matrix3d>(
    const RigidBodyDynamics::Math::Matrix3d &default_value)
{
  RigidBodyDynamics::Math::Matrix3d result = default_value;

  if (stackQueryValue()) {
    LuaTable vector_table = LuaTable::fromLuaState (luaTable->L);

    if (vector_table.length() != 3) {
      throw RigidBodyDynamics::Errors::RBDLFileParseError(
            "LuaModel Error: invalid 3d matrix!");
    }

    if (vector_table[1].length() != 3
        || vector_table[2].length() != 3
        || vector_table[3].length() != 3) {
      throw RigidBodyDynamics::Errors::RBDLFileParseError(
            "LuaModel Error: invalid 3d matrix!");
    }

    result(0,0) = vector_table[1][1];
    result(0,1) = vector_table[1][2];
    result(0,2) = vector_table[1][3];

    result(1,0) = vector_table[2][1];
    result(1,1) = vector_table[2][2];
    result(1,2) = vector_table[2][3];

    result(2,0) = vector_table[3][1];
    result(2,1) = vector_table[3][2];
    result(2,2) = vector_table[3][3];
  }

  stackRestore();

  return result;
}
//==============================================================================
template<>
RigidBodyDynamics::Math::SpatialTransform
RBDL_ADDON_DLLAPI
LuaTableNode::getDefault<RigidBodyDynamics::Math::SpatialTransform>(
  const RigidBodyDynamics::Math::SpatialTransform &default_value
)
{
  RigidBodyDynamics::Math::SpatialTransform result = default_value;

  if (stackQueryValue()) {
    LuaTable vector_table = LuaTable::fromLuaState (luaTable->L);

    result.r = vector_table["r"].getDefault<RigidBodyDynamics::Math::Vector3d>(
          RigidBodyDynamics::Math::Vector3d::Zero(3));
    result.E = vector_table["E"].getDefault<RigidBodyDynamics::Math::Matrix3d>(
          RigidBodyDynamics::Math::Matrix3d::Identity (3,3));
  }

  stackRestore();

  return result;
}

//==============================================================================
template<>
RigidBodyDynamics::Joint
RBDL_ADDON_DLLAPI
LuaTableNode::getDefault<RigidBodyDynamics::Joint>(
    const RigidBodyDynamics::Joint &default_value)
{
  RigidBodyDynamics::Joint result = default_value;

  if (stackQueryValue()) {
    LuaTable vector_table = LuaTable::fromLuaState (luaTable->L);

    int joint_dofs = vector_table.length();

    if (joint_dofs == 1) {
      std::string dof_string = vector_table[1].getDefault<std::string>("");
      if (dof_string == "JointTypeSpherical") {
        stackRestore();
        return RigidBodyDynamics::Joint(
              RigidBodyDynamics::JointTypeSpherical);
      } else if (dof_string == "JointTypeEulerZYX") {
        stackRestore();
        return RigidBodyDynamics::Joint(
              RigidBodyDynamics::JointTypeEulerZYX);
      }
      if (dof_string == "JointTypeEulerXYZ") {
        stackRestore();
        return RigidBodyDynamics::Joint(
              RigidBodyDynamics::JointTypeEulerXYZ);
      }
      if (dof_string == "JointTypeEulerYXZ") {
        stackRestore();
        return RigidBodyDynamics::Joint(
              RigidBodyDynamics::JointTypeEulerYXZ);
      }
      if (dof_string == "JointTypeTranslationXYZ") {
        stackRestore();
        return RigidBodyDynamics::Joint(
              RigidBodyDynamics::JointTypeTranslationXYZ);
      }
      if (dof_string == "JointTypeFloatingBase") {
        stackRestore();
        return RigidBodyDynamics::Joint(
              RigidBodyDynamics::JointTypeFloatingBase);
      }
    }

    if (joint_dofs > 0) {
      if (vector_table[1].length() != 6) {
        std::ostringstream errormsg;
        errormsg << "LuaModel Error: invalid joint motion "
                 << "subspace description at "
                 << this->keyStackToString() << std::endl;
        throw RigidBodyDynamics::Errors::RBDLFileParseError(errormsg.str());
      }
    }
    switch (joint_dofs) {
    case 0:
      result = RigidBodyDynamics::Joint(
            RigidBodyDynamics::JointTypeFixed);
      break;
    case 1:
      result = RigidBodyDynamics::Joint (
            vector_table[1].get<RigidBodyDynamics::Math::SpatialVector>());
      break;
    case 2:
      result = RigidBodyDynamics::Joint(
                 vector_table[1].get<RigidBodyDynamics::Math::SpatialVector>(),
                 vector_table[2].get<RigidBodyDynamics::Math::SpatialVector>()
               );
      break;
    case 3:
      result = RigidBodyDynamics::Joint(
                 vector_table[1].get<RigidBodyDynamics::Math::SpatialVector>(),
                 vector_table[2].get<RigidBodyDynamics::Math::SpatialVector>(),
                 vector_table[3].get<RigidBodyDynamics::Math::SpatialVector>()
               );
      break;
    case 4:
      result = RigidBodyDynamics::Joint(
                 vector_table[1].get<RigidBodyDynamics::Math::SpatialVector>(),
                 vector_table[2].get<RigidBodyDynamics::Math::SpatialVector>(),
                 vector_table[3].get<RigidBodyDynamics::Math::SpatialVector>(),
                 vector_table[4].get<RigidBodyDynamics::Math::SpatialVector>()
               );
      break;
    case 5:
      result = RigidBodyDynamics::Joint(
                 vector_table[1].get<RigidBodyDynamics::Math::SpatialVector>(),
                 vector_table[2].get<RigidBodyDynamics::Math::SpatialVector>(),
                 vector_table[3].get<RigidBodyDynamics::Math::SpatialVector>(),
                 vector_table[4].get<RigidBodyDynamics::Math::SpatialVector>(),
                 vector_table[5].get<RigidBodyDynamics::Math::SpatialVector>()
               );
      break;
    case 6:
      result = RigidBodyDynamics::Joint(
                 vector_table[1].get<RigidBodyDynamics::Math::SpatialVector>(),
                 vector_table[2].get<RigidBodyDynamics::Math::SpatialVector>(),
                 vector_table[3].get<RigidBodyDynamics::Math::SpatialVector>(),
                 vector_table[4].get<RigidBodyDynamics::Math::SpatialVector>(),
                 vector_table[5].get<RigidBodyDynamics::Math::SpatialVector>(),
                 vector_table[6].get<RigidBodyDynamics::Math::SpatialVector>()
               );
      break;
    default:
      throw RigidBodyDynamics::Errors::RBDLFileParseError(
            "Invalid number of DOFs for joint.");
      break;
    }
  }

  stackRestore();

  return result;
}


//==============================================================================
template<>
RigidBodyDynamics::Body
RBDL_ADDON_DLLAPI
LuaTableNode::getDefault<RigidBodyDynamics::Body>(
    const RigidBodyDynamics::Body &default_value)
{
  RigidBodyDynamics::Body result = default_value;

  if (stackQueryValue()) {
    LuaTable vector_table = LuaTable::fromLuaState (luaTable->L);

    double mass = 0.;
    RigidBodyDynamics::Math::Vector3d com(
          RigidBodyDynamics::Math::Vector3d::Zero(3));
    RigidBodyDynamics::Math::Matrix3d inertia(
          RigidBodyDynamics::Math::Matrix3d::Identity(3,3));

    mass = vector_table["mass"];
    com = vector_table["com"]
        .getDefault<RigidBodyDynamics::Math::Vector3d>(com);
    inertia = vector_table["inertia"]
        .getDefault<RigidBodyDynamics::Math::Matrix3d>(inertia);

    result = RigidBodyDynamics::Body (mass, com, inertia);
  }

  stackRestore();

  return result;
}

//==============================================================================

template<> Point
RBDL_ADDON_DLLAPI
LuaTableNode::getDefault<Point>(
  const Point &default_value)
{ 
  Point result = default_value;

  if (stackQueryValue()) {
    LuaTable point_table = LuaTable::fromLuaState (luaTable->L);

    result.name         = point_table["name"].get<std::string>();
    result.point_local  = point_table["point"]
                            .getDefault<RigidBodyDynamics::Math::Vector3d>(
                              RigidBodyDynamics::Math::Vector3d::Zero());
    result.body_name    = point_table["body"].get<std::string>();
  }

  stackRestore();

  return result;
}

//==============================================================================

template<> MotionCaptureMarker
RBDL_ADDON_DLLAPI
LuaTableNode::getDefault<MotionCaptureMarker>(
  const MotionCaptureMarker &default_value)
{
  MotionCaptureMarker result = default_value;

  if (stackQueryValue()) {
    LuaTable marker_table = LuaTable::fromLuaState (luaTable->L);

    result.name        = marker_table["name"].get<std::string>();
    result.point_local = marker_table["point"]
                          .getDefault<RigidBodyDynamics::Math::Vector3d>(
                            RigidBodyDynamics::Math::Vector3d::Zero());
    result.body_name   = marker_table["body"].get<std::string>();
  }

  stackRestore();

  return result;
}

//==============================================================================

template<> LocalFrame
RBDL_ADDON_DLLAPI
LuaTableNode::getDefault<LocalFrame>(
  const LocalFrame &default_value)
{
  LocalFrame result = default_value;

  if (stackQueryValue()) {
    LuaTable local_frame_table = LuaTable::fromLuaState (luaTable->L);

    result.name     = local_frame_table["name"].get<std::string>();
    result.body_name= local_frame_table["body"].get<std::string>();

    result.r = local_frame_table["r"]
                .getDefault<RigidBodyDynamics::Math::Vector3d>(
                  RigidBodyDynamics::Math::Vector3d::Zero());
    result.E = local_frame_table["E"]
                .getDefault<RigidBodyDynamics::Math::Matrix3d>(
                  RigidBodyDynamics::Math::Matrix3d::Identity());
  }

  stackRestore();

  return result;
}
//==============================================================================

template<> HumanMetaData
RBDL_ADDON_DLLAPI
LuaTableNode::getDefault<HumanMetaData>(
  const HumanMetaData &default_value)
{
  HumanMetaData result = default_value;

  if (stackQueryValue()) {
    LuaTable metadata_table = LuaTable::fromLuaState (luaTable->L);

    result.gender =
      metadata_table["gender"].get<std::string>();

    result.age  	= metadata_table["age"];
    result.height = metadata_table["height"];
    result.mass   = metadata_table["weight"];
    result.age_group  = metadata_table["age_group"].get<std::string>();    
  }

  stackRestore();
  return result;
}
//==============================================================================

#ifdef RBDL_BUILD_ADDON_MUSCLE
template<> Millard2016TorqueMuscleConfig
RBDL_ADDON_DLLAPI
LuaTableNode::getDefault<Millard2016TorqueMuscleConfig>(
  const Millard2016TorqueMuscleConfig &default_value)
{
  Millard2016TorqueMuscleConfig result = default_value;

  if (stackQueryValue()) {
    LuaTable mtg_table = LuaTable::fromLuaState (luaTable->L);

    // First read mandatory fields
    result.name     		= mtg_table["name"].get<std::string>();
    result.angle_sign 	= mtg_table["angle_sign"].get<double>();
    result.torque_sign 	= mtg_table["torque_sign"].get<double>();
    result.body         = mtg_table["body"].get<std::string>();

    if(mtg_table["joint_index"].exists()){
      result.joint_index  =
          unsigned(int(mtg_table["joint_index"].get<double>()));
    }

    //Optional parameters
    if(mtg_table["data_set"].exists()){
      result.data_set = mtg_table["data_set"].get<std::string>();
    }
    if(mtg_table["age_group"].exists()){
      result.age_group = mtg_table["age_group"].get<std::string>();
    }
    if(mtg_table["gender"].exists()){
      result.gender = mtg_table["gender"].get<std::string>();
    }

    if (mtg_table["q_scale"].exists()) {
        result.q_scale = mtg_table["q_scale"].get<double>();
    }
    if (mtg_table["activation_index"].exists()) {
        result.activation_index =
            unsigned(int(mtg_table["activation_index"].get<double>()));
    }
    if (mtg_table["data_set"].exists()) {
      result.data_set = mtg_table["data_set"].get<std::string>();
    }
    //if (mtg_table["angle_scale"].exists()) {
    //    result.angle_scale = mtg_table["angle_scale"].get<double>();
    //}
    if (mtg_table["joint_angle_offset"].exists()) {
        result.joint_angle_offset =
          mtg_table["joint_angle_offset"].get<double>();
    }
    if (mtg_table["act_time"].exists()) {
        result.activation_time_constant =
          mtg_table["act_time"].get<double>();
    }
    if (mtg_table["deact_time"].exists()) {
        result.deactivation_time_constant =
          mtg_table["deact_time"].get<double>();
    }

    //Parameters for manual tuning
    if (mtg_table["max_isometric_torque_scale"].exists()) {
        result.max_isometric_torque_scale =
          mtg_table["max_isometric_torque_scale"].get<double>();
    }
    if (mtg_table["max_angular_velocity_scale"].exists()) {
        result.max_angular_velocity_scale =
          mtg_table["max_angular_velocity_scale"].get<double>();
    }

    if (mtg_table["passive_element_torque_scale"].exists()) {
        result.passive_element_torque_scale =
          mtg_table["passive_element_torque_scale"].get<double>();
    }
    if (mtg_table["passive_element_angle_offset"].exists()) {
        result.passive_element_angle_offset =
          mtg_table["passive_element_angle_offset"].get<double>();
    }
    if (mtg_table["passive_element_damping_coeff"].exists()) {
        result.passive_element_damping_coeff =
          mtg_table["passive_element_damping_coeff"].get<double>();
    }

    //Optional passive-curve fitting coordinates
    if (mtg_table["fit_passive_torque_scale"].exists()) {
        result.fit_passive_torque_scale =
          mtg_table["fit_passive_torque_scale"]
            .get<RigidBodyDynamics::Math::Vector3d>();
    }
    if (mtg_table["fit_passive_torque_offset"].exists()) {
        result.fit_passive_torque_offset =
          mtg_table["fit_passive_torque_offset"]
            .get<RigidBodyDynamics::Math::Vector3d>();
    }

    //Optional fitting parameters from the fitting routine
    if (mtg_table["max_isometric_torque"].exists()) {
        result.max_isometric_torque =
          mtg_table["max_isometric_torque"].get<double>();
    }
    if (mtg_table["max_angular_velocity"].exists()) {
        result.max_angular_velocity =
          mtg_table["max_angular_velocity"].get<double>();
    }
    if (mtg_table["active_torque_angle_blending"].exists()) {
        result.active_torque_angle_blending =
          mtg_table["active_torque_angle_blending"].get<double>();
    }
    if (mtg_table["passive_torque_angle_blending"].exists()) {
        result.passive_torque_angle_blending =
          mtg_table["passive_torque_angle_blending"].get<double>();
    }
    if (mtg_table["torque_velocity_blending"].exists()) {
        result.torque_velocity_blending =
          mtg_table["torque_velocity_blending"].get<double>();
    }
    if (mtg_table["active_torque_angle_scale"].exists()) {
        result.active_torque_angle_scale =
          mtg_table["active_torque_angle_scale"].get<double>();
    }
    //if (mtg_table["passive_torque_angle_scale"].exists()) {
    //    result.passive_torque_angle_scale =
    //      mtg_table["passive_torque_angle_scale"].get<double>();
    //}
    //if (mtg_table["torque_velocity_scaling"].exists()) {
    //    result.torque_velocity_scaling =
    //      mtg_table["torque_velocity_scaling"].get<double>();
    //}

  }
  stackRestore();
  return result;
}
#endif

/* LUASTRUCTS_H */
#endif
