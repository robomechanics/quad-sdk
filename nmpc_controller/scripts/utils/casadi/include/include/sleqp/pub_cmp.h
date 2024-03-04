#ifndef SLEQP_PUB_CMP_H
#define SLEQP_PUB_CMP_H

/**
 * @file pub_cmp.h
 * @brief Definition of numerical comparison functions.
 **/

#include "sleqp/export.h"
#include "sleqp/pub_types.h"

SLEQP_EXPORT double
sleqp_infinity();

SLEQP_EXPORT bool
sleqp_is_finite(double value);

#endif /* SLEQP_PUB_CMP_H */
