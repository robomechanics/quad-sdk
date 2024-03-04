#ifndef SLEQP_PUB_SCALE_H
#define SLEQP_PUB_SCALE_H

/**
 * @file pub_scale.h
 * @brief Definition of the problem scaling.
 **/

#include "sleqp/export.h"
#include "sleqp/pub_iterate.h"
#include "sleqp/pub_problem.h"

/**
 * @defgroup scaling NLP scaling
 *
 * The scaled version of the basic NLP is given by
 *
 * \f[
 * \begin{aligned}
 * \min \: & f'(x')                \\
 * \st \: & l' \leq c'(x') \leq u' \\
 * & l'_x \leq x' \leq u'_x,
 * \end{aligned}
 * \f]
 *
 * defined by scaling weights \f$ \lambda \in \mathbb{Z} \f$,
 * \f$ \alpha \in \mathbb{Z}^{m} \f$,
 * \f$ \beta \in \mathbb{Z}^{n} \f$.
 *
 * The weights \f$ \alpha, \beta \f$ yield scaling
 * factors \f$ a, b \f$:
 *
 * \f[
 * \begin{aligned}
 * a_i := 2^{\alpha_i}      \\
 * b_i := 2^{\beta_i}       \\
 * \end{aligned}
 * \f]
 *
 * The constraint scales \f$ a \f$ and variable scales
 * \f$ b \f$ yield matrices
 * \f$ A = \operatorname{diag}(a) \f$,
 * \f$ B = \operatorname{diag}(b) \f$.
 *
 * \f[
 * \begin{aligned}
 * f'(\cdot) &:= 2^{\lambda} f(B \cdot)  \\
 * c'(\cdot) &:= A c(B \cdot)            \\
 * l' &:= A l                            \\
 * u' &:= A u                            \\
 * l_x' &:= B l_x                        \\
 * x'   &:= B x                          \\
 * u_x' &:= B u_x                        \\
 * \end{aligned}
 * \f]
 *
 * Note that the scaling is exact (apart from over- / underflows)
 * in the sense that the unscaling is inverse to the scaling
 * even on floating points.
 *
 * In terms of least-squares functions, the objective scaling works
 * by scaling the residual vector \f$ r(x) \f$ by \f$ 2^{\lambda} \f$.
 * Consequently, the objective value is scaled by \f$ \approx 2^{2 \lambda} \f$.
 *
 * @see Functions
 *
 *
 * @{
 **/

typedef struct SleqpScaling SleqpScaling;

SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_scaling_create(SleqpScaling** scaling,
                     int num_variables,
                     int num_constraints);

SLEQP_RETCODE
sleqp_scaling_reset(SleqpScaling* scaling);

SLEQP_EXPORT int
sleqp_scaling_num_vars(const SleqpScaling* scaling);
SLEQP_EXPORT int
sleqp_scaling_num_cons(const SleqpScaling* scaling);

SLEQP_EXPORT int
sleqp_scaling_obj_weight(const SleqpScaling* scaling);

SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_scaling_set_obj_weight(SleqpScaling* scaling, int weight);

SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_scaling_set_obj_weight_from_nominal(SleqpScaling* scaling,
                                          double nominal_value);

SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_scaling_set_var_weight(SleqpScaling* scaling, int index, int weight);

/**
 * Sets variable scaling weights in order for the scaling of the primal values
 * to map all of the given nominal values to [.5, 1.)
 **/
SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_scaling_set_var_weights_from_nominal(SleqpScaling* scaling,
                                           double* nominal_values);

SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_scaling_set_var_weight_from_nominal(SleqpScaling* scaling,
                                          int index,
                                          double nominal_value);

SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_scaling_set_cons_weight(SleqpScaling* scaling, int index, int weight);

SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_scaling_set_cons_weights_from_nominal(SleqpScaling* scaling,
                                            double* nominal_values);

SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_scaling_set_cons_weight_from_nominal(SleqpScaling* scaling,
                                           int index,
                                           double nominal_value);

SLEQP_EXPORT int*
sleqp_scaling_var_weights(SleqpScaling* scaling);

SLEQP_EXPORT int*
sleqp_scaling_cons_weights(SleqpScaling* scaling);

SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_obj_scaling_from_grad(SleqpScaling* scaling,
                            SleqpVec* gradient,
                            double eps);

SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_scaling_from_cons_jac(SleqpScaling* scaling,
                            SleqpMat* cons_jac,
                            double eps);

SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_scaling_capture(SleqpScaling* scaling);

SLEQP_EXPORT SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_scaling_release(SleqpScaling** star);

/**
 * @}
 **/

#endif /* SLEQP_PUB_SCALE_H */
