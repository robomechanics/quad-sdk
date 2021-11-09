// Copyright (c) 2019, NVIDIA CORPORATION.  All rights reserved.
//
// NVIDIA CORPORATION and its licensors retain all intellectual property
// and proprietary rights in and to this software, related documentation
// and any modifications thereto.  Any use, reproduction, disclosure or
// distribution of this software and related documentation without an express
// license agreement from NVIDIA CORPORATION is strictly prohibited.
//
#pragma once

#define GYM_TENSOR_MAX_DIMENSIONS 8


namespace carb
{
namespace gym
{

/// Data types the tensors can be
enum GymTensorDataType
{
    eGymDataTypeNone,
    eGymDataTypeFp32, //!< Floating precision type of 32 bits
    eGymDataTypeUint32, //!< Unsigned integer of 32 bits
    eGymDataTypeUint64, //!< Unsigned integer of 64 bits
    eGymDataTypeUint8, //!< Unsigned integer of 8 bits
    eGymDataTypeInt16, //!< Signed integer of 16 bits
};

/// Generic tensor descriptor for CPU or GPU tensors used for interop with other frameworks
/**
 */
struct GymTensor
{
    int device = -1; //!< -1 for CPU, 0 or more for GPU device ordinal
    GymTensorDataType dtype = eGymDataTypeNone; //!< Type of data in the tensor.
    int numDims = 0; //!< Rank of the tensor. the maximum rank of a tensor is GYM_TENSOR_MAX_DIMENSIONS.
    int dims[GYM_TENSOR_MAX_DIMENSIONS] = { 0 }; //!< Size of each dimension of the tensor.
    void* data = nullptr; //!< Pointer to the data.
    bool ownData = false; //!< Whether this tensor owns the data allocation
};
}
}
