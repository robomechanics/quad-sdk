// Copyright (C) 2005, 2007 International Business Machines and others.
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// Authors:  Carl Laird, Andreas Waechter     IBM    2005-08-09

#ifndef __spiritNLP_HPP__
#define __spiritNLP_HPP__

#include "IpTNLP.hpp"

#include "nmpc_controller/eval_g_leg.h"
#include "nmpc_controller/eval_jac_g_leg.h"
#include "nmpc_controller/eval_hess_g_leg.h"
#include "nmpc_controller/eval_g_tail.h"
#include "nmpc_controller/eval_jac_g_tail.h"
#include "nmpc_controller/eval_hess_g_tail.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <vector>

using namespace Ipopt;

class spiritNLP : public TNLP
{
public:
    // Horizon length, state dimension, input dimension, and constraints dimension
    int N_, n_, m_, g_;

    int leg_input_start_idx_;

    bool with_tail_;

    // State cost weighting, input cost weighting
    Eigen::MatrixXd Q_, R_;

    // Feet location from feet to body COM in world frame
    Eigen::MatrixXd feet_location_;

    // Step length
    double dt_;

    // State bounds, input bounds, constraint bounds
    Eigen::MatrixXd x_min_, x_max_, u_min_, u_max_, g_min_, g_max_;

    // Initial guess
    Eigen::MatrixXd w0_, z_L0_, z_U0_, lambda0_;

    bool refresh_;

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
    spiritNLP(
        int N,
        int n,
        int m,
        double dt,
        bool with_tail,
        Eigen::MatrixXd Q,
        Eigen::MatrixXd R,
        Eigen::MatrixXd x_min,
        Eigen::MatrixXd x_max,
        Eigen::MatrixXd u_min,
        Eigen::MatrixXd u_max);

    /** Default destructor */
    virtual ~spiritNLP();

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
    spiritNLP(
        const spiritNLP &);

    spiritNLP &operator=(
        const spiritNLP &);
    //@}
};

#endif
