// Copyright (c) 2020, NVIDIA CORPORATION.  All rights reserved.
//
// NVIDIA CORPORATION and its licensors retain all intellectual property
// and proprietary rights in and to this software, related documentation
// and any modifications thereto.  Any use, reproduction, disclosure or
// distribution of this software and related documentation without an express
// license agreement from NVIDIA CORPORATION is strictly prohibited.
//

//
// Known to work with PyTorch 1.4.0 through 1.7.0
//

#include "GymTensor.h"
using namespace carb::gym;

#include <torch/extension.h>

#include <cstdint>
#include <cstdio>
#include <vector>

#ifndef TORCH_EXTENSION_NAME
#    define TORCH_EXTENSION_NAME "gymtorch"
#endif

#if (TORCH_MAJOR == 1 && TORCH_MINOR >= 5) || TORCH_MAJOR > 1
using TheTensorTypeId_ = torch::DispatchKey;
#else
using TheTensorTypeId_ = torch::TensorTypeId;
#endif

torch::Tensor gtWrapTensor(void* data,
                           int dev,
                           int dtype,
                           const std::vector<int64_t>& dimensions,
                           const std::vector<int64_t>& offsets,
                           const std::vector<int64_t>& counts)
{
    if (!data || dimensions.empty())
    {
        // hmmm... return empty tensor?
        printf("*** Can't create empty tensor\n");
        return torch::autograd::make_variable(torch::Tensor(), false);
    }

    if (offsets.size() != dimensions.size() || counts.size() != dimensions.size())
    {
        printf("*** Sizes of dimensions/offsets/counts do not match\n");
        return torch::autograd::make_variable(torch::Tensor(), false);
    }

    int ndims = int(dimensions.size());

    bool isContiguous = true;
    for (int i = 0; i < ndims; i++)
    {
        if (offsets[i] != 0 || counts[i] != dimensions[i])
        {
            isContiguous = false;
            break;
        }
    }

    int64_t numElem = dimensions[0];
    for (int i = 1; i < ndims; i++)
    {
        numElem *= dimensions[i];
    }

    torch::DeviceType deviceType;
    torch::DeviceIndex deviceIndex;
    TheTensorTypeId_ tensorType;
    if (dev < 0)
    {
        // CPU tensor
        deviceType = torch::DeviceType::CPU;
        deviceIndex = -1;
        tensorType = TheTensorTypeId_::CPUTensorId;
    }
    else
    {
        // GPU tensor
        deviceType = torch::DeviceType::CUDA;
        deviceIndex = dev;
        tensorType = TheTensorTypeId_::CUDATensorId;
    }

    torch::Device device(deviceType, deviceIndex);
    torch::DataPtr dataPtr(data, device);
    caffe2::TypeMeta typeMeta;
    switch (dtype)
    {
    case eGymDataTypeFp32:
        typeMeta = caffe2::TypeMeta::Make<float>();
        break;
    case eGymDataTypeUint8:
        typeMeta = caffe2::TypeMeta::Make<uint8_t>();
        break;
    case eGymDataTypeInt16:
        typeMeta = caffe2::TypeMeta::Make<int16_t>();
        break;
    case eGymDataTypeUint32:
        // linker error with uint32_t (not supported as torch dtype)
        // typeMeta = caffe2::TypeMeta::Make<uint32_t>();
        typeMeta = caffe2::TypeMeta::Make<int32_t>();
        break;
    case eGymDataTypeUint64:
        // linker error with uint64_t (not supported as torch dtype)
        // typeMeta = caffe2::TypeMeta::Make<uint64_t>();
        typeMeta = caffe2::TypeMeta::Make<int64_t>();
        break;

    default:
        printf("*** Unrecognized or unsupported dtype %d\n", dtype);
        // hmmm... return empty tensor?
        return torch::autograd::make_variable(torch::Tensor(), false);
    }

#if (TORCH_MAJOR == 1 && TORCH_MINOR >= 6) || TORCH_MAJOR > 1
    torch::Storage storage(
        torch::Storage::use_byte_size_t(), numElem * typeMeta.itemsize(), std::move(dataPtr), nullptr, false);
    auto tensorImpl = torch::make_intrusive<torch::TensorImpl>(std::move(storage), tensorType, typeMeta);
#else
    torch::Storage storage(typeMeta, numElem, std::move(dataPtr), nullptr, false);
    auto tensorImpl = torch::make_intrusive<torch::TensorImpl>(std::move(storage), tensorType);
#endif

    if (isContiguous)
    {
        tensorImpl->set_storage_offset(0);
        tensorImpl->set_sizes_contiguous(torch::IntList(dimensions));
    }
    else
    {
        // compute strides
        std::vector<int64_t> strides(ndims);
        strides[ndims - 1] = 1;
        for (int i = ndims - 2; i >= 0; i--)
        {
            strides[i] = strides[i + 1] * dimensions[i + 1];
        }

        // compute storage offset
        int64_t storageOffset = 0;
        for (int i = 0; i < ndims; i++)
        {
            storageOffset += offsets[i] * strides[i];
        }

        tensorImpl->set_storage_offset(storageOffset);
        tensorImpl->set_sizes_and_strides(torch::IntList(counts), torch::IntList(strides));
    }

    torch::Tensor tensor(tensorImpl);

    return torch::autograd::make_variable(tensor, false);
}


PYBIND11_MODULE(TORCH_EXTENSION_NAME, m)
{
    // auto gymapi = py::module::import("isaacgym.gymapi");
    // auto tensorClass = gymapi.attr("GpuTensor");

    m.def("wrap_tensor_impl", &gtWrapTensor, "Gymtorch wrap tensor");
}
