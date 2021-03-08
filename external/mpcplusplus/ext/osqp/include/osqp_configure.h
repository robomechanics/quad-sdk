#ifndef OSQP_CONFIGURE_H
# define OSQP_CONFIGURE_H

# ifdef __cplusplus
extern "C" {
# endif /* ifdef __cplusplus */

/* DEBUG */
/* #undef DEBUG */

/* Operating system */
#define IS_LINUX
/* #undef IS_MAC */
/* #undef IS_WINDOWS */

/* EMBEDDED */
/* #undef EMBEDDED */

/* PRINTING */
#define PRINTING

/* PROFILING */
#define PROFILING

/* CTRLC */
#define CTRLC

/* DFLOAT */
/* #undef DFLOAT */

/* DLONG */
#define DLONG

/* ENABLE_MKL_PARDISO */
#define ENABLE_MKL_PARDISO

/* MEMORY MANAGEMENT */
/* #undef OSQP_CUSTOM_MEMORY */
#ifdef OSQP_CUSTOM_MEMORY
#include ""
#endif



# ifdef __cplusplus
}
# endif /* ifdef __cplusplus */

#endif /* ifndef OSQP_CONFIGURE_H */
