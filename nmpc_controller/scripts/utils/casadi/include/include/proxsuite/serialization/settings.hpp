//
// Copyright (c) 2022 INRIA
//
/**
 * @file settings.hpp
 */

#ifndef PROXSUITE_SERIALIZATION_SETTINGS_HPP
#define PROXSUITE_SERIALIZATION_SETTINGS_HPP

#include <cereal/cereal.hpp>
#include <proxsuite/proxqp/settings.hpp>

namespace cereal {

template<class Archive, typename T>
void
serialize(Archive& archive, proxsuite::proxqp::Settings<T>& settings)
{
  archive(CEREAL_NVP(settings.default_rho),
          CEREAL_NVP(settings.default_mu_eq),
          CEREAL_NVP(settings.default_mu_in),
          CEREAL_NVP(settings.alpha_bcl),
          CEREAL_NVP(settings.beta_bcl),
          CEREAL_NVP(settings.refactor_dual_feasibility_threshold),
          CEREAL_NVP(settings.refactor_rho_threshold),
          CEREAL_NVP(settings.mu_min_eq),
          CEREAL_NVP(settings.mu_min_in),
          CEREAL_NVP(settings.mu_max_eq_inv),
          CEREAL_NVP(settings.mu_update_factor),
          CEREAL_NVP(settings.mu_update_inv_factor),
          CEREAL_NVP(settings.cold_reset_mu_eq),
          CEREAL_NVP(settings.cold_reset_mu_in),
          CEREAL_NVP(settings.cold_reset_mu_eq_inv),
          CEREAL_NVP(settings.cold_reset_mu_in_inv),
          CEREAL_NVP(settings.eps_abs),
          CEREAL_NVP(settings.eps_rel),
          CEREAL_NVP(settings.max_iter),
          CEREAL_NVP(settings.max_iter_in),
          CEREAL_NVP(settings.safe_guard),
          CEREAL_NVP(settings.nb_iterative_refinement),
          CEREAL_NVP(settings.eps_refact),
          CEREAL_NVP(settings.verbose),
          CEREAL_NVP(settings.initial_guess),
          CEREAL_NVP(settings.update_preconditioner),
          CEREAL_NVP(settings.compute_preconditioner),
          CEREAL_NVP(settings.compute_timings),
          CEREAL_NVP(settings.check_duality_gap),
          CEREAL_NVP(settings.eps_duality_gap_abs),
          CEREAL_NVP(settings.eps_duality_gap_rel),
          CEREAL_NVP(settings.preconditioner_max_iter),
          CEREAL_NVP(settings.preconditioner_accuracy),
          CEREAL_NVP(settings.eps_primal_inf),
          CEREAL_NVP(settings.eps_dual_inf),
          CEREAL_NVP(settings.bcl_update),
          CEREAL_NVP(settings.sparse_backend));
}
} // namespace cereal

#endif /* end of include guard PROXSUITE_SERIALIZATION_SETTINGS_HPP */
