#include "nmpc_controller/spirit_nlp.h"

#include <cassert>
#include <iostream>

using namespace Ipopt;

#ifdef __GNUC__
#pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

// Class constructor
spiritNLP::spiritNLP(
    int type,
    int N,
    int n,
    int m,
    double dt,
    Eigen::MatrixXd Q,
    Eigen::MatrixXd R,
    Eigen::MatrixXd x_min,
    Eigen::MatrixXd x_max,
    Eigen::MatrixXd u_min,
    Eigen::MatrixXd u_max)
// N: prediction steps
// n: states dimension
// m: input dimension
// with_tail: system includes tail or not
// Q: states cost weights
// R: input cost weights
// dt: time step length
{
   type_ = type;

   N_ = N;
   n_ = n;
   m_ = m;
   g_ = n_ + 16; // states dynamics plus linear friciton cone

   if (type_ == NONE)
   {
      // Leg controller
      leg_input_start_idx_ = 0;

      eval_g_work_ = eval_g_leg_work;
      eval_g_incref_ = eval_g_leg_incref;
      eval_g_checkout_ = eval_g_leg_checkout;
      eval_g_ = eval_g_leg;
      eval_g_release_ = eval_g_leg_release;
      eval_g_decref_ = eval_g_leg_decref;

      eval_hess_g_work_ = eval_hess_g_leg_work;
      eval_hess_g_incref_ = eval_hess_g_leg_incref;
      eval_hess_g_checkout_ = eval_hess_g_leg_checkout;
      eval_hess_g_ = eval_hess_g_leg;
      eval_hess_g_release_ = eval_hess_g_leg_release;
      eval_hess_g_decref_ = eval_hess_g_leg_decref;
      eval_hess_g_sparsity_out_ = eval_hess_g_leg_sparsity_out;

      eval_jac_g_work_ = eval_jac_g_leg_work;
      eval_jac_g_incref_ = eval_jac_g_leg_incref;
      eval_jac_g_checkout_ = eval_jac_g_leg_checkout;
      eval_jac_g_ = eval_jac_g_leg;
      eval_jac_g_release_ = eval_jac_g_leg_release;
      eval_jac_g_decref_ = eval_jac_g_leg_decref;
      eval_jac_g_sparsity_out_ = eval_jac_g_leg_sparsity_out;
   }
   else
   {
      // Tail controller
      leg_input_start_idx_ = 2;

      eval_g_work_ = eval_g_tail_work;
      eval_g_incref_ = eval_g_tail_incref;
      eval_g_checkout_ = eval_g_tail_checkout;
      eval_g_ = eval_g_tail;
      eval_g_release_ = eval_g_tail_release;
      eval_g_decref_ = eval_g_tail_decref;

      eval_hess_g_work_ = eval_hess_g_tail_work;
      eval_hess_g_incref_ = eval_hess_g_tail_incref;
      eval_hess_g_checkout_ = eval_hess_g_tail_checkout;
      eval_hess_g_ = eval_hess_g_tail;
      eval_hess_g_release_ = eval_hess_g_tail_release;
      eval_hess_g_decref_ = eval_hess_g_tail_decref;
      eval_hess_g_sparsity_out_ = eval_hess_g_tail_sparsity_out;

      eval_jac_g_work_ = eval_jac_g_tail_work;
      eval_jac_g_incref_ = eval_jac_g_tail_incref;
      eval_jac_g_checkout_ = eval_jac_g_tail_checkout;
      eval_jac_g_ = eval_jac_g_tail;
      eval_jac_g_release_ = eval_jac_g_tail_release;
      eval_jac_g_decref_ = eval_jac_g_tail_decref;
      eval_jac_g_sparsity_out_ = eval_jac_g_tail_sparsity_out;
   }

   Q_ = Q;
   R_ = R;

   // feet location initialized by nominal position
   feet_location_ = Eigen::MatrixXd(12, N_);
   for (int i = 0; i < N_; ++i)
   {
      feet_location_.block(0, i, 12, 1) << -0.2263, -0.098, 0.3, -0.2263, 0.098, 0.3, 0.2263, -0.098, 0.3, 0.2263, 0.098, 0.3;
   }

   dt_ = dt;

   x_min_ = x_min;
   x_max_ = x_max;

   u_min_ = u_min;
   u_max_ = u_max;

   // We have dynamics constraints first and friction cone at the end
   g_min_ = Eigen::MatrixXd(g_, 1);
   g_max_ = Eigen::MatrixXd(g_, 1);
   g_min_.block(0, 0, n_, 1).fill(0);
   g_min_.block(n_, 0, 16, 1).fill(-2e19);
   g_max_.block(0, 0, n_, 1).fill(0);
   g_max_.block(n_, 0, 16, 1).fill(0);

   w0_ = Eigen::MatrixXd((n_ + m_) * N_, 1);
   w0_.fill(0);
   z_L0_ = Eigen::MatrixXd((n_ + m_) * N_, 1);
   z_L0_.fill(0);
   z_U0_ = Eigen::MatrixXd((n_ + m_) * N_, 1);
   z_U0_.fill(0);
   lambda0_ = Eigen::MatrixXd(g_ * N_, 1);
   lambda0_.fill(0);

   for (int i = 0; i < N_; ++i)
   {
      w0_(leg_input_start_idx_ + 2 + i * (n_ + m_), 0) = 29;
      w0_(leg_input_start_idx_ + 5 + i * (n_ + m_), 0) = 29;
      w0_(leg_input_start_idx_ + 8 + i * (n_ + m_), 0) = 29;
      w0_(leg_input_start_idx_ + 11 + i * (n_ + m_), 0) = 29;
   }

   x_reference_ = Eigen::MatrixXd(n_, N_);
   x_reference_.fill(0);

   x_current_ = Eigen::MatrixXd(n_, 1);
   x_current_.fill(0);

   contact_sequence_ = Eigen::MatrixXi(4, N_);
   contact_sequence_.fill(1);

   if (type_ == DISTRIBUTED)
   {
      known_leg_input_ = true;

      leg_input_ = Eigen::MatrixXd(m_ - leg_input_start_idx_, N_);
      leg_input_.setZero();
      for (int i = 0; i < N_; ++i)
      {
         leg_input_(2, i) = 29;
         leg_input_(5, i) = 29;
         leg_input_(8, i) = 29;
         leg_input_(11, i) = 29;
      }
   }
   else
   {
      known_leg_input_ = false;
   }

   compute_nnz_jac_g();
   compute_nnz_h();
}

// Destructor
spiritNLP::~spiritNLP()
{
}

// Returns the size of the problem
bool spiritNLP::get_nlp_info(
    Index &n,
    Index &m,
    Index &nnz_jac_g,
    Index &nnz_h_lag,
    IndexStyleEnum &index_style)
{
   // Decision variables
   n = N_ * (n_ + m_);

   // Constraints
   m = g_ * N_;

   // Nonzero entrance in the constraint jacobian matrix
   nnz_jac_g = nnz_jac_g_;

   // Nonzero entrance in the full hessian matrix
   nnz_h_lag = nnz_compact_h_;

   // use the C style indexing (0-based)
   index_style = TNLP::C_STYLE;

   return true;
}

// Returns the variable bounds
bool spiritNLP::get_bounds_info(
    Index n,
    Number *x_l,
    Number *x_u,
    Index m,
    Number *g_l,
    Number *g_u)
{
   Eigen::Map<Eigen::MatrixXd> x_l_matrix(x_l, n, 1);
   Eigen::Map<Eigen::MatrixXd> x_u_matrix(x_u, n, 1);
   Eigen::Map<Eigen::MatrixXd> g_l_matrix(g_l, m, 1);
   Eigen::Map<Eigen::MatrixXd> g_u_matrix(g_u, m, 1);

   // We have the decision variables ordered as u_0, x_1, ..., u_N-1, x_N
   for (int i = 0; i < N_; ++i)
   {
      // Inputs bound
      x_l_matrix.block(i * (n_ + m_), 0, m_, 1) = u_min_;
      x_u_matrix.block(i * (n_ + m_), 0, m_, 1) = u_max_;
      if (known_leg_input_)
      {
         x_l_matrix.block(i * (n_ + m_) + leg_input_start_idx_, 0, m_ - leg_input_start_idx_, 1) = leg_input_.block(0, i, m_ - leg_input_start_idx_, 1);
         x_u_matrix.block(i * (n_ + m_) + leg_input_start_idx_, 0, m_ - leg_input_start_idx_, 1) = leg_input_.block(0, i, m_ - leg_input_start_idx_, 1);
      }
      
      // Contact sequence
      for (int j = 0; j < 4; ++j)
      {
         x_l_matrix.block(i * (n_ + m_) + leg_input_start_idx_ + 3 * j, 0, 3, 1) =
             (x_l_matrix.block(i * (n_ + m_) + leg_input_start_idx_ + 3 * j, 0, 3, 1).array() * contact_sequence_(j, i)).matrix();
         x_u_matrix.block(i * (n_ + m_) + leg_input_start_idx_ + 3 * j, 0, 3, 1) =
             (x_u_matrix.block(i * (n_ + m_) + leg_input_start_idx_ + 3 * j, 0, 3, 1).array() * contact_sequence_(j, i)).matrix();
      }

      // States bound
      x_l_matrix.block(i * (n_ + m_) + m_, 0, n_, 1) = x_min_;
      x_u_matrix.block(i * (n_ + m_) + m_, 0, n_, 1) = x_max_;

      // Constraints bound
      g_l_matrix.block(i * g_, 0, g_, 1) = g_min_;
      g_u_matrix.block(i * g_, 0, g_, 1) = g_max_;
   }

   return true;
}

// Returns the initial point for the problem
bool spiritNLP::get_starting_point(
    Index n,
    bool init_x,
    Number *x,
    bool init_z,
    Number *z_L,
    Number *z_U,
    Index m,
    bool init_lambda,
    Number *lambda)
{
   if (init_x)
   {
      Eigen::Map<Eigen::MatrixXd> w(x, n, 1);
      w = w0_;
   }

   if (init_z)
   {
      Eigen::Map<Eigen::MatrixXd> z_L_matrix(z_L, n, 1);
      Eigen::Map<Eigen::MatrixXd> z_U_matrix(z_L, n, 1);
      z_L_matrix = z_L0_;
      z_U_matrix = z_U0_;
   }

   if (init_lambda)
   {
      Eigen::Map<Eigen::MatrixXd> lambda_matrix(lambda, m, 1);
      lambda_matrix = lambda0_;
   }

   return true;
}

// Returns the value of the objective function
bool spiritNLP::eval_f(
    Index n,
    const Number *x,
    bool new_x,
    Number &obj_value)
{
   Eigen::Map<const Eigen::MatrixXd> w(x, n, 1);

   obj_value = 0;

   for (int i = 0; i < N_; ++i)
   {
      Eigen::MatrixXd uk = w.block(i * (n_ + m_), 0, m_, 1);
      Eigen::MatrixXd xk = w.block(i * (n_ + m_) + m_, 0, n_, 1);

      xk = (xk.array() - x_reference_.block(0, i, n_, 1).array()).matrix();

      obj_value += (xk.transpose() * Q_.asDiagonal() * xk / 2 + uk.transpose() * R_.asDiagonal() * uk / 2)(0, 0);
   }

   return true;
}

// Return the gradient of the objective function
bool spiritNLP::eval_grad_f(
    Index n,
    const Number *x,
    bool new_x,
    Number *grad_f)
{
   Eigen::Map<const Eigen::MatrixXd> w(x, n, 1);

   Eigen::Map<Eigen::MatrixXd> grad_f_matrix(grad_f, n, 1);

   for (int i = 0; i < N_; ++i)
   {
      Eigen::MatrixXd uk = w.block(i * (n_ + m_), 0, m_, 1);
      Eigen::MatrixXd xk = w.block(i * (n_ + m_) + m_, 0, n_, 1);

      xk = (xk.array() - x_reference_.block(0, i, n_, 1).array()).matrix();

      grad_f_matrix.block(i * (n_ + m_), 0, m_, 1) = R_.asDiagonal() * uk;
      grad_f_matrix.block(i * (n_ + m_) + m_, 0, n_, 1) = Q_.asDiagonal() * xk;
   }

   return true;
}

// Return the value of the constraints
bool spiritNLP::eval_g(
    Index n,
    const Number *x,
    bool new_x,
    Index m,
    Number *g)
{
   Eigen::Map<const Eigen::MatrixXd> w(x, n, 1);

   Eigen::Map<Eigen::MatrixXd> g_matrix(g, m, 1);

   for (int i = 0; i < N_; ++i)
   {
      Eigen::MatrixXd pk(13, 1);
      pk(0, 0) = dt_;
      pk.block(1, 0, 12, 1) = feet_location_.block(0, i, 12, 1);

      casadi_int sz_arg;
      casadi_int sz_res;
      casadi_int sz_iw;
      casadi_int sz_w;
      eval_g_work_(&sz_arg, &sz_res, &sz_iw, &sz_w);

      const double *arg[sz_arg];
      double *res[sz_res];
      casadi_int iw[sz_iw];
      double _w[sz_w];

      eval_g_incref_();

      Eigen::MatrixXd tmp_arg(2 * n_ + m_, 1);
      if (i == 0)
      {
         tmp_arg.topRows(n_) = x_current_;
         tmp_arg.bottomRows(n_ + m_) = w.block(0, 0, n_ + m_, 1);
         arg[0] = tmp_arg.data();
      }
      else
      {
         arg[0] = w.block(i * (n_ + m_) - n_, 0, 2 * n_ + m_, 1).data();
      }

      arg[1] = pk.data();
      res[0] = g_matrix.block(i * g_, 0, g_, 1).data();

      int mem = eval_g_checkout_();

      eval_g_(arg, res, iw, _w, mem);

      eval_g_release_(mem);
      eval_g_decref_();
   }

   return true;
}

// Return the structure or values of the Jacobian
bool spiritNLP::eval_jac_g(
    Index n,
    const Number *x,
    bool new_x,
    Index m,
    Index nele_jac,
    Index *iRow,
    Index *jCol,
    Number *values)
{
   if (values == NULL)
   {
      Eigen::Map<Eigen::MatrixXi> iRow_matrix(iRow, nele_jac, 1);
      Eigen::Map<Eigen::MatrixXi> jCol_matrix(jCol, nele_jac, 1);

      iRow_matrix = iRow_jac_g_;
      jCol_matrix = jCol_jac_g_;
   }
   else
   {
      Eigen::Map<const Eigen::MatrixXd> w(x, n, 1);

      Eigen::Map<Eigen::MatrixXd> values_matrix(values, nele_jac, 1);

      for (int i = 0; i < N_; ++i)
      {
         Eigen::MatrixXd pk(13, 1);
         pk(0, 0) = dt_;
         pk.block(1, 0, 12, 1) = feet_location_.block(0, i, 12, 1);

         casadi_int sz_arg;
         casadi_int sz_res;
         casadi_int sz_iw;
         casadi_int sz_w;
         eval_jac_g_work_(&sz_arg, &sz_res, &sz_iw, &sz_w);

         const double *arg[sz_arg];
         double *res[sz_res];
         casadi_int iw[sz_iw];
         double _w[sz_w];

         eval_jac_g_incref_();
         int mem;

         Eigen::MatrixXd tmp_arg(2 * n_ + m_, 1), tmp_res(nnz_step_jac_g_, 1);
         if (i == 0)
         {
            tmp_arg.topRows(n_) = x_current_;
            tmp_arg.bottomRows(n_ + m_) = w.block(0, 0, n_ + m_, 1);
            arg[0] = tmp_arg.data();
            arg[1] = pk.data();
            res[0] = tmp_res.data();

            mem = eval_jac_g_checkout_();

            eval_jac_g_(arg, res, iw, _w, mem);

            for (size_t j = 0; j < first_step_idx_jac_g_.size(); j++)
            {
               values_matrix(j, 0) = tmp_res(first_step_idx_jac_g_.at(j), 0);
            }
         }
         else
         {
            arg[0] = w.block(i * (n_ + m_) - n_, 0, 2 * n_ + m_, 1).data();
            arg[1] = pk.data();
            res[0] = values_matrix.block((i - 1) * nnz_step_jac_g_ + first_step_idx_jac_g_.size(), 0, nnz_step_jac_g_, 1).data();

            mem = eval_jac_g_checkout_();

            eval_jac_g_(arg, res, iw, _w, mem);
         }

         eval_jac_g_release_(mem);
         eval_jac_g_decref_();
      }
   }

   return true;
}

// Return the structure of the Jacobian
void spiritNLP::compute_nnz_jac_g()
{
   const casadi_int *sp_i;
   sp_i = eval_jac_g_sparsity_out_(0);
   casadi_int nrow = *sp_i++;
   casadi_int ncol = *sp_i++;
   const casadi_int *colind = sp_i;
   const casadi_int *row = sp_i + ncol + 1;
   casadi_int nnz = sp_i[ncol];

   nnz_step_jac_g_ = nnz;
   Eigen::MatrixXi iRow(nnz, 1);
   Eigen::MatrixXi jCol(nnz, 1);
   first_step_idx_jac_g_.clear();
   first_step_idx_jac_g_.reserve(nnz);
   int idx = 0;

   for (int i = 0; i < ncol; ++i)
   {
      for (int j = colind[i]; j < colind[i + 1]; ++j)
      {
         iRow(idx, 0) = row[j];
         jCol(idx, 0) = i;

         // We have the decision variable start from u_0, so we should drop the jacobian corresponding to x_0
         if (jCol(idx, 0) >= n_)
         {
            first_step_idx_jac_g_.push_back(idx);
         }

         idx += 1;
      }
   }

   nnz_jac_g_ = first_step_idx_jac_g_.size() + (N_ - 1) * nnz;
   iRow_jac_g_ = Eigen::MatrixXi(nnz_jac_g_, 1);
   jCol_jac_g_ = Eigen::MatrixXi(nnz_jac_g_, 1);

   for (int i = 0; i < N_; ++i)
   {
      if (i == 0)
      {
         // The first step only has u_0 and x_1 as decision variables
         for (size_t j = 0; j < first_step_idx_jac_g_.size(); j++)
         {
            iRow_jac_g_(j, 0) = iRow(first_step_idx_jac_g_.at(j), 0);

            // We should shift n states corresponding to x_0
            jCol_jac_g_(j, 0) = jCol(first_step_idx_jac_g_.at(j), 0) - n_;
         }
      }
      else
      {
         // Each step we shift g constraints
         iRow_jac_g_.block((i - 1) * nnz + first_step_idx_jac_g_.size(), 0, nnz, 1) = (iRow.array() + i * g_).matrix();

         // Each step we shift n states and m inputs
         jCol_jac_g_.block((i - 1) * nnz + first_step_idx_jac_g_.size(), 0, nnz, 1) = (jCol.array() + i * (n_ + m_) - n_).matrix();
      }
   }
}

// Return the structure or values of the Hessian
bool spiritNLP::eval_h(
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
    Number *values)
{
   if (values == NULL)
   {
      Eigen::Map<Eigen::MatrixXi> iRow_matrix(iRow, nele_hess, 1);
      Eigen::Map<Eigen::MatrixXi> jCol_matrix(jCol, nele_hess, 1);

      iRow_matrix = iRow_compact_h_;
      jCol_matrix = jCol_compact_h_;
   }
   else
   {
      Eigen::Map<const Eigen::MatrixXd> w(x, n, 1);

      Eigen::Map<Eigen::MatrixXd> values_compact_matrix(values, nele_hess, 1);
      Eigen::Map<const Eigen::MatrixXd> lambda_matrix(lambda, m, 1);
      Eigen::MatrixXd values_matrix(nnz_h_, 1);

      for (int i = 0; i < N_; ++i)
      {
         Eigen::MatrixXd pk(13, 1);
         pk(0, 0) = dt_;
         pk.block(1, 0, 12, 1) = feet_location_.block(0, i, 12, 1);

         casadi_int sz_arg;
         casadi_int sz_res;
         casadi_int sz_iw;
         casadi_int sz_w;
         eval_hess_g_work_(&sz_arg, &sz_res, &sz_iw, &sz_w);

         const double *arg[sz_arg];
         double *res[sz_res];
         casadi_int iw[sz_iw];
         double _w[sz_w];

         eval_hess_g_incref_();
         int mem;

         Eigen::MatrixXd tmp_arg(2 * n_ + m_, 1), tmp_res(nnz_step_h_ - n_ - m_, 1);
         if (i == 0)
         {
            tmp_arg.topRows(n_) = x_current_;
            tmp_arg.bottomRows(n_ + m_) = w.block(0, 0, n_ + m_, 1);

            arg[0] = tmp_arg.data();
            arg[1] = lambda_matrix.block(i * g_, 0, g_, 1).data();
            arg[2] = pk.data();
            res[0] = tmp_res.data();

            mem = eval_hess_g_checkout_();

            eval_hess_g_(arg, res, iw, _w, mem);

            // The first step only has u_0 and x_1 as decision variables
            for (size_t j = 0; j < first_step_idx_hess_g_.size(); j++)
            {
               values_matrix(j, 0) = tmp_res(first_step_idx_hess_g_.at(j), 0);
            }
         }
         else
         {
            arg[0] = w.block(i * (n_ + m_) - n_, 0, 2 * n_ + m_, 1).data();
            arg[1] = lambda_matrix.block(i * g_, 0, g_, 1).data();
            arg[2] = pk.data();
            res[0] = values_matrix.block((i - 1) * nnz_step_h_ + n_ + m_ + first_step_idx_hess_g_.size(), 0, nnz_step_h_ - n_ - m_, 1).data();

            mem = eval_hess_g_checkout_();

            eval_hess_g_(arg, res, iw, _w, mem);
         }

         eval_hess_g_release_(mem);
         eval_hess_g_decref_();

         values_matrix.block(i * nnz_step_h_ + first_step_idx_hess_g_.size(), 0, m_, 1) = (obj_factor * R_.array()).matrix();
         values_matrix.block(i * nnz_step_h_ + first_step_idx_hess_g_.size() + m_, 0, n_, 1) = (obj_factor * Q_.array()).matrix();
      }

      std::vector<Eigen::Triplet<double>> tripletList;
      tripletList.reserve(nnz_h_);
      for (int i = 0; i < nnz_h_; ++i)
      {
         tripletList.push_back(Eigen::Triplet<double>(iRow_h_(i, 0), jCol_h_(i, 0), values_matrix(i, 0)));
      }
      Eigen::SparseMatrix<double> hess(N_ * (n_ + m_), N_ * (n_ + m_));
      hess.setFromTriplets(tripletList.begin(), tripletList.end());

      int idx_hess = 0;
      for (int k = 0; k < hess.outerSize(); ++k)
      {
         for (Eigen::SparseMatrix<double>::InnerIterator it(hess, k); it; ++it)
         {
            values_compact_matrix(idx_hess, 0) = it.value();
            idx_hess += 1;
         }
      }
   }

   return true;
}

// Return the structure of the Hessian
void spiritNLP::compute_nnz_h()
{
   const casadi_int *sp_i;
   sp_i = eval_hess_g_sparsity_out_(0);
   casadi_int nrow = *sp_i++;
   casadi_int ncol = *sp_i++;
   const casadi_int *colind = sp_i;
   const casadi_int *row = sp_i + ncol + 1;
   casadi_int nnz = sp_i[ncol];

   Eigen::MatrixXi iRow(nnz, 1);
   Eigen::MatrixXi jCol(nnz, 1);
   first_step_idx_hess_g_.clear();
   first_step_idx_hess_g_.reserve(nnz);
   int idx = 0;

   for (int i = 0; i < ncol; ++i)
   {
      for (int j = colind[i]; j < colind[i + 1]; ++j)
      {
         iRow(idx, 0) = row[j];
         jCol(idx, 0) = i;

         // We have the decision variable start from u_0, so we should drop the hessian corresponding to x_0
         if ((iRow(idx, 0) >= n_) && (jCol(idx, 0) >= n_))
         {
            first_step_idx_hess_g_.push_back(idx);
         }

         idx += 1;
      }
   }

   // Hessian from cost
   Eigen::ArrayXi iRow_cost = Eigen::ArrayXi::LinSpaced(n_ + m_, 0, n_ + m_ - 1);
   Eigen::ArrayXi jCol_cost = Eigen::ArrayXi::LinSpaced(n_ + m_, 0, n_ + m_ - 1);

   nnz_h_ = (N_ - 1) * nnz + first_step_idx_hess_g_.size() + N_ * (n_ + m_);
   nnz_step_h_ = nnz + n_ + m_;
   iRow_h_ = Eigen::MatrixXi(nnz_h_, 1);
   jCol_h_ = Eigen::MatrixXi(nnz_h_, 1);

   for (int i = 0; i < N_; ++i)
   {
      if (i == 0)
      {
         // The first step only has u_0 and x_1 as decision variables
         for (size_t j = 0; j < first_step_idx_hess_g_.size(); j++)
         {
            // We should shift n states corresponding to x_0
            iRow_h_(j, 0) = iRow(first_step_idx_hess_g_.at(j), 0) - n_;
            jCol_h_(j, 0) = jCol(first_step_idx_hess_g_.at(j), 0) - n_;
         }
      }
      else
      {
         // Each step we shift n states and m inputs
         iRow_h_.block(i * nnz_step_h_ - nnz + first_step_idx_hess_g_.size(), 0, nnz, 1) = (iRow.array() + i * (n_ + m_) - n_).matrix();
         jCol_h_.block(i * nnz_step_h_ - nnz + first_step_idx_hess_g_.size(), 0, nnz, 1) = (jCol.array() + i * (n_ + m_) - n_).matrix();
      }

      // Hessian from cost
      iRow_h_.block(i * nnz_step_h_ + first_step_idx_hess_g_.size(), 0, n_ + m_, 1) = (iRow_cost + i * (n_ + m_)).matrix();
      jCol_h_.block(i * nnz_step_h_ + first_step_idx_hess_g_.size(), 0, n_ + m_, 1) = (jCol_cost + i * (n_ + m_)).matrix();
   }

   std::vector<Eigen::Triplet<double>> tripletList;
   tripletList.reserve(nnz_h_);
   for (int i = 0; i < nnz_h_; ++i)
   {
      tripletList.push_back(Eigen::Triplet<double>(iRow_h_(i, 0), jCol_h_(i, 0), 1));
   }
   Eigen::SparseMatrix<double> hess(N_ * (n_ + m_), N_ * (n_ + m_));
   hess.setFromTriplets(tripletList.begin(), tripletList.end());

   // We eliminate the overlap nonzero entrances here to get the exact nonzero number
   nnz_compact_h_ = hess.nonZeros();
   iRow_compact_h_ = Eigen::MatrixXi(nnz_compact_h_, 1);
   jCol_compact_h_ = Eigen::MatrixXi(nnz_compact_h_, 1);

   int idx_hess = 0;

   for (int k = 0; k < hess.outerSize(); ++k)
   {
      for (Eigen::SparseMatrix<double>::InnerIterator it(hess, k); it; ++it)
      {
         iRow_compact_h_(idx_hess, 0) = it.row();
         jCol_compact_h_(idx_hess, 0) = it.col();
         idx_hess += 1;
      }
   }
}

// Return the optimization results
void spiritNLP::finalize_solution(
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
    IpoptCalculatedQuantities *ip_cq)
{
   Eigen::Map<const Eigen::MatrixXd> w(x, n, 1);
   w0_ = w;

   Eigen::Map<const Eigen::MatrixXd> z_L_matrix(z_L, n, 1);
   Eigen::Map<const Eigen::MatrixXd> z_U_matrix(z_L, n, 1);
   z_L0_ = z_L_matrix;
   z_U0_ = z_U_matrix;

   Eigen::Map<const Eigen::MatrixXd> lambda_matrix(lambda, m, 1);
   lambda0_ = lambda_matrix;
}

void spiritNLP::shift_initial_guess()
{
   // Shift decision variables for 1 time step
   for (size_t i = 0; i < N_ - 1; i++)
   {
      w0_.block(i * (n_ + m_), 0, (n_ + m_), 1) = w0_.block((i + 1) * (n_ + m_), 0, (n_ + m_), 1);
      z_L0_.block(i * (n_ + m_), 0, (n_ + m_), 1) = z_L0_.block((i + 1) * (n_ + m_), 0, (n_ + m_), 1);
      z_U0_.block(i * (n_ + m_), 0, (n_ + m_), 1) = z_U0_.block((i + 1) * (n_ + m_), 0, (n_ + m_), 1);
      lambda0_.block(i * g_, 0, g_, 1) = lambda0_.block((i + 1) * g_, 0, g_, 1);
   }
}

void spiritNLP::update_solver(
    const Eigen::VectorXd &initial_state,
    const Eigen::MatrixXd &ref_traj,
    const Eigen::MatrixXd &foot_positions,
    const std::vector<std::vector<bool>> &contact_schedule)
{
   // Update foot positions
   // Local planner has row as N horizon and col as position
   feet_location_ = -foot_positions.transpose();

   // Update contact sequence
   // Local planner has outer as N and inner as boolean contact
   for (size_t i = 0; i < contact_schedule.size(); i++)
   {
      for (size_t j = 0; j < contact_schedule.front().size(); j++)
      {
         if (contact_schedule.at(i).at(j))
         {
            contact_sequence_(j, i) = 1;
         }
         else
         {
            contact_sequence_(j, i) = 0;
         }
      }
   }

   // Update initial states
   x_current_ = initial_state;

   // Update reference trajectory
   // Local planner has row as N+1 horizon and col as states
   x_reference_ = ref_traj.bottomRows(N_).transpose();
}

void spiritNLP::update_solver(
    const Eigen::VectorXd &initial_state,
    const Eigen::MatrixXd &ref_traj,
    const Eigen::MatrixXd &foot_positions,
    const std::vector<std::vector<bool>> &contact_schedule,
    const Eigen::MatrixXd &state_traj,
    const Eigen::MatrixXd &control_traj)
{
   this->update_solver(
       initial_state,
       ref_traj,
       foot_positions,
       contact_schedule);

   for (size_t i = 0; i < N_ - 1; i++)
   {
      w0_.block(i * (n_ + m_) + leg_input_start_idx_, 0, 12, 1) = control_traj.row(i).transpose();
      w0_.block(i * (n_ + m_) + m_, 0, 6, 1) = state_traj.block(i, 0, 1, 6).transpose();
      w0_.block(i * (n_ + m_) + m_ + 8, 0, 6, 1) = state_traj.block(i, 6, 1, 6).transpose();
   }

   leg_input_ = control_traj.transpose();

   known_leg_input_ = true;
}