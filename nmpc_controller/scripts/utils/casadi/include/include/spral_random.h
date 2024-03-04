#ifndef SPRAL_RANDOM_H
#define SPRAL_RANDOM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define SPRAL_RANDOM_INITIAL_SEED 486502

/* Generate a sample from Unif(-1,1) or Unif(0,1) */
double spral_random_real(int *state, bool positive);
/* Generate a sample from discrete Unif(1,...,n) */
int spral_random_integer(int *state, int n);
/* Generate a sample from discrete Unif(1,...,n) */
int64_t spral_random_long(int *state, int64_t n);
/* Generate a sample with equal probability of true or false */
bool spral_random_logical(int *state);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif // SPRAL_RANDOM_H
