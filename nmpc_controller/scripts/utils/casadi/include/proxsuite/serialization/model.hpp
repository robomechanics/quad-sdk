//
// Copyright (c) 2022 INRIA
//
/**
 * @file model.hpp
 */

#ifndef PROXSUITE_SERIALIZATION_MODEL_HPP
#define PROXSUITE_SERIALIZATION_MODEL_HPP

#include <cereal/cereal.hpp>
#include <proxsuite/proxqp/dense/model.hpp>

namespace cereal {

template<class Archive, typename T>
void
serialize(Archive& archive, proxsuite::proxqp::dense::Model<T>& model)
{
  archive(CEREAL_NVP(model.dim),
          CEREAL_NVP(model.n_eq),
          CEREAL_NVP(model.n_in),
          CEREAL_NVP(model.n_total),
          CEREAL_NVP(model.H),
          CEREAL_NVP(model.g),
          CEREAL_NVP(model.A),
          CEREAL_NVP(model.b),
          CEREAL_NVP(model.C),
          CEREAL_NVP(model.l),
          CEREAL_NVP(model.u));
}
} // namespace cereal

#endif /* end of include guard PROXSUITE_SERIALIZATION_MODEL_HPP */
