//
// Copyright (c) 2022 INRIA
//
/**
 * @file wrapper.hpp
 */

#ifndef PROXSUITE_SERIALIZATION_WRAPPER_HPP
#define PROXSUITE_SERIALIZATION_WRAPPER_HPP

#include <cereal/cereal.hpp>
#include <proxsuite/proxqp/dense/wrapper.hpp>

namespace cereal {

template<class Archive, typename T>
void
serialize(Archive& archive, proxsuite::proxqp::dense::QP<T>& qp)
{
  archive(
    CEREAL_NVP(qp.model), CEREAL_NVP(qp.results), CEREAL_NVP(qp.settings));
}
} // namespace cereal
#endif /* end of include guard PROXSUITE_SERIALIZATION_WRAPPER_HPP */
