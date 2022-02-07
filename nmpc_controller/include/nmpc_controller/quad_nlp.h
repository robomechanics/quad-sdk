// Copyright (C) 2005, 2007 International Business Machines and others.
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// Authors:  Carl Laird, Andreas Waechter     IBM    2005-08-09

#ifndef __quadNLP_HPP__
#define __quadNLP_HPP__

#include "IpTNLP.hpp"

#include "nmpc_controller/eval_g_leg.h"
#include "nmpc_controller/eval_jac_g_leg.h"
#include "nmpc_controller/eval_hess_g_leg.h"
#include "nmpc_controller/eval_g_leg_simple.h"
#include "nmpc_controller/eval_jac_g_leg_simple.h"
#include "nmpc_controller/eval_hess_g_leg_simple.h"
#include "nmpc_controller/eval_g_leg_complex.h"
#include "nmpc_controller/eval_jac_g_leg_complex.h"
#include "nmpc_controller/eval_hess_g_leg_complex.h"
#include "nmpc_controller/eval_g_leg_simple_to_complex.h"
#include "nmpc_controller/eval_jac_g_leg_simple_to_complex.h"
#include "nmpc_controller/eval_hess_g_leg_simple_to_complex.h"
#include "nmpc_controller/eval_g_leg_complex_to_simple.h"
#include "nmpc_controller/eval_jac_g_leg_complex_to_simple.h"
#include "nmpc_controller/eval_hess_g_leg_complex_to_simple.h"
#include "nmpc_controller/eval_g_tail.h"
#include "nmpc_controller/eval_jac_g_tail.h"
#include "nmpc_controller/eval_hess_g_tail.h"
#include "quad_utils/tail_type.h"
#include "quad_utils/function_timer.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <vector>

using namespace Ipopt;

class quadNLP : public TNLP
{
public:
    // Horizon length, state dimension, input dimension, and constraints dimension
    int N_, n_, m_, g_;

    /// State dimension for simple and complex models
    int n_simple_, n_complex_;

    /// Vector of state variables for each finite element
    Eigen::VectorXi n_vec_;

    /// Input dimension for simple and complex models
    int m_simple_, m_complex_;

    /// Constraint dimension for simple and complex models
    int g_simple_, g_complex_;

    /// Number of variables and constraints
    int n_vars_, n_constraints_;

    /// Number of state variables added in complex model
    int n_null_;

    int leg_input_start_idx_;

    int type_;

    bool known_leg_input_;

    Eigen::MatrixXd leg_input_;

    // State cost weighting, input cost weighting
    Eigen::MatrixXd Q_, R_;

    // Scale factor for Q and R
    Eigen::MatrixXd Q_factor_, R_factor_;

    // Feet location from feet to body COM in world frame
    Eigen::MatrixXd feet_location_;

    // Foot locations in world frame
    Eigen::MatrixXd foot_pos_world_;

    // Foot velocities in world frame
    Eigen::MatrixXd foot_vel_world_;

    // Step length
    double dt_;

    // Friction coefficient
    double mu_;

    /// Mass of the platform (set to zero to ignore nominal ff)
    const double mass_ = 13.3;

    /// Gravity constant
    const double grav_ = 9.81;

    // State bounds, input bounds, constraint bounds
    Eigen::VectorXd x_min_, x_max_, u_min_, u_max_, g_min_, g_max_;

    // State bounds, input bounds, constraint bounds
    Eigen::VectorXd x_min_simple_, x_max_simple_, x_min_complex_, x_max_complex_,
        g_min_simple_, g_max_simple_, g_min_complex_, g_max_complex_;

    // Ground height structure for the height bounds
    Eigen::MatrixXd ground_height_;

    // Initial guess
    Eigen::MatrixXd w0_, z_L0_, z_U0_, lambda0_;

    // State reference for computing cost
    Eigen::MatrixXd x_reference_;

    // Current state
    Eigen::MatrixXd x_current_;

    // Feet contact sequence
    Eigen::MatrixXi contact_sequence_;

    // Nonzero entrance number in the constraint jacobian matrix
    int nnz_jac_g_, nnz_step_jac_g_;

    std::vector<int> first_step_idx_jac_g_;

    // Nonzero entrance row and column in the constraint jacobian matrix
    Eigen::MatrixXi iRow_jac_g_, jCol_jac_g_;

    // Nonzero entrance number in the pure hessian matrix (exclude the side jacobian part)
    int nnz_h_, nnz_compact_h_, nnz_step_h_;

    std::vector<int> first_step_idx_hess_g_;

    // Nonzero entrance row and column in the pure hessian matrix (exclude the side jacobian part)
    Eigen::MatrixXi iRow_h_, jCol_h_, iRow_compact_h_, jCol_compact_h_;

    // Penalty on panic variables
    double panic_weights_;

    // Time duration to the next plan index
    double first_element_duration_;

    /// Vector of ids for model complexity schedule
    Eigen::VectorXi complexity_schedule_;

    /// Number of complex finite elements in the horizon
    int num_complex_fe_;

    decltype(eval_g_leg_work) *eval_g_work_;
    decltype(eval_g_leg_incref) *eval_g_incref_;
    decltype(eval_g_leg_checkout) *eval_g_checkout_;
    decltype(eval_g_leg) *eval_g_;
    decltype(eval_g_leg_release) *eval_g_release_;
    decltype(eval_g_leg_decref) *eval_g_decref_;

    decltype(eval_hess_g_leg_work) *eval_hess_g_work_;
    decltype(eval_hess_g_leg_incref) *eval_hess_g_incref_;
    decltype(eval_hess_g_leg_checkout) *eval_hess_g_checkout_;
    decltype(eval_hess_g_leg) *eval_hess_g_;
    decltype(eval_hess_g_leg_release) *eval_hess_g_release_;
    decltype(eval_hess_g_leg_decref) *eval_hess_g_decref_;
    decltype(eval_hess_g_leg_sparsity_out) *eval_hess_g_sparsity_out_;

    decltype(eval_jac_g_leg_work) *eval_jac_g_work_;
    decltype(eval_jac_g_leg_incref) *eval_jac_g_incref_;
    decltype(eval_jac_g_leg_checkout) *eval_jac_g_checkout_;
    decltype(eval_jac_g_leg) *eval_jac_g_;
    decltype(eval_jac_g_leg_release) *eval_jac_g_release_;
    decltype(eval_jac_g_leg_decref) *eval_jac_g_decref_;
    decltype(eval_jac_g_leg_sparsity_out) *eval_jac_g_sparsity_out_;

    /** Default constructor */
    quadNLP(
        int type,
        int N,
        int n,
        int n_null,
        int m,
        double dt,
        double mu,
        double panic_weights,
        Eigen::MatrixXd Q,
        Eigen::MatrixXd R,
        Eigen::MatrixXd Q_factor,
        Eigen::MatrixXd R_factor,
        Eigen::MatrixXd x_min,
        Eigen::MatrixXd x_max,
        Eigen::MatrixXd x_min_complex,
        Eigen::MatrixXd x_max_complex,
        Eigen::MatrixXd u_min,
        Eigen::MatrixXd u_max);

    /** Default destructor */
    virtual ~quadNLP();

    /**@name Overloaded from TNLP */
    //@{
    /** Method to return some info about the NLP */
    virtual bool get_nlp_info(
        Index &n,
        Index &m,
        Index &nnz_jac_g,
        Index &nnz_h_lag,
        IndexStyleEnum &index_style);

    /** Method to return the bounds for my problem */
    virtual bool get_bounds_info(
        Index n,
        Number *x_l,
        Number *x_u,
        Index m,
        Number *g_l,
        Number *g_u);

    /** Method to return the starting point for the algorithm */
    virtual bool get_starting_point(
        Index n,
        bool init_x,
        Number *x,
        bool init_z,
        Number *z_L,
        Number *z_U,
        Index m,
        bool init_lambda,
        Number *lambda);

    /** Method to return the objective value */
    virtual bool eval_f(
        Index n,
        const Number *x,
        bool new_x,
        Number &obj_value);

    /** Method to return the gradient of the objective */
    virtual bool eval_grad_f(
        Index n,
        const Number *x,
        bool new_x,
        Number *grad_f);

    /** Method to return the constraint residuals */
    virtual bool eval_g(
        Index n,
        const Number *x,
        bool new_x,
        Index m,
        Number *g);

    /** Method to return:
    *   1) The structure of the jacobian (if "values" is NULL)
    *   2) The values of the jacobian (if "values" is not NULL)
    */
    virtual bool eval_jac_g(
        Index n,
        const Number *x,
        bool new_x,
        Index m,
        Index nele_jac,
        Index *iRow,
        Index *jCol,
        Number *values);

    virtual void compute_nnz_jac_g();

    /** Method to return:
    *   1) The structure of the hessian of the lagrangian (if "values" is NULL)
    *   2) The values of the hessian of the lagrangian (if "values" is not NULL)
    */
    virtual bool eval_h(
        Index n,
        const Number *x,
        bool new_x,
        Number obj_factor,
        Index m,
        const Number *lambda,
        bool new_lambda,
        Index nele_hess,
        Index *iRow,
        Index *jCol,
        Number *values);

    virtual void compute_nnz_h();

    /** This method is called when the algorithm is complete so the TNLP can store/write the solution */
    virtual void finalize_solution(
        SolverReturn status,
        Index n,
        const Number *x,
        const Number *z_L,
        const Number *z_U,
        Index m,
        const Number *g,
        const Number *lambda,
        Number obj_value,
        const IpoptData *ip_data,
        IpoptCalculatedQuantities *ip_cq);

    virtual void shift_initial_guess();

    virtual void update_solver(
        const Eigen::VectorXd &initial_state,
        const Eigen::MatrixXd &ref_traj,
        const Eigen::MatrixXd &foot_positions,
        const std::vector<std::vector<bool>> &contact_schedule);

    virtual void update_solver(
        const Eigen::VectorXd &initial_state,
        const Eigen::MatrixXd &ref_traj,
        const Eigen::MatrixXd &foot_positions,
        const std::vector<std::vector<bool>> &contact_schedule,
        const Eigen::VectorXd &ground_height);

    virtual void update_solver(
        const Eigen::VectorXd &initial_state,
        const Eigen::MatrixXd &ref_traj,
        const Eigen::MatrixXd &foot_positions,
        const std::vector<std::vector<bool>> &contact_schedule,
        const Eigen::VectorXd &ground_height,
        const double &first_element_duration);

    virtual void update_solver(
        const Eigen::VectorXd &initial_state,
        const Eigen::MatrixXd &ref_traj,
        const Eigen::MatrixXd &foot_positions,
        const std::vector<std::vector<bool>> &contact_schedule,
        const Eigen::MatrixXd &state_traj,
        const Eigen::MatrixXd &control_traj);

    void update_complexity_schedule(const Eigen::VectorXi &complexity_schedule);

    /**
     * @brief Return the number of variables for this NLP
     * @return Number of variables
     */
    inline int getNumVariables() const {
        return (m_*N_ + 3*(n_simple_ * (N_ - num_complex_fe_) + n_complex_ * num_complex_fe_));
    };
    
    /**
     * @brief Return the number of constraints in this NLP
     * @return Number of constraints
     */
    inline int getNumConstraints() const {
        return (g_simple_ * (N_ - num_complex_fe_) + g_complex_ * num_complex_fe_) + 
             2*(n_simple_ * (N_ - num_complex_fe_) + n_complex_ * num_complex_fe_);
    };

    /**
     * @brief Return the first index of the decision variable vector corresponding to the given 
     * finite element
     * @param[in] i Index of requested finite element
     * @return Index in decision variable vector corresponding to the beginning of the requested FE
     */
    inline int getPrimalFEIndex(int i) const {
        int num_complex_before = complexity_schedule_.segment(0,i).sum();
        return (i * m_ + (num_complex_before * n_complex_ + (i - num_complex_before) * n_simple_));
    };

    //@}

private:
    /**@name Methods to block default compiler methods.
    *
    * The compiler automatically generates the following three methods.
    *  Since the default compiler implementation is generally not what
    *  you want (for all but the most simple classes), we usually
    *  put the declarations of these methods in the private section
    *  and never implement them. This prevents the compiler from
    *  implementing an incorrect "default" behavior without us
    *  knowing. (See Scott Meyers book, "Effective C++")
    */
    //@{
    quadNLP(
        const quadNLP &);

    quadNLP &operator=(
        const quadNLP &);
    //@}
};

#endif
