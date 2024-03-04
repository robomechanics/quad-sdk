/* mumps_compat.h.  Generated from mumps_compat.h.in by configure.  */
/* mumps_compat.h.in. */

#ifndef MUMPS_COMPAT_H
#define MUMPS_COMPAT_H

#ifndef MUMPS_CALL
/* Define Mumps calling convention. */
#define MUMPS_CALL 
#endif

/* tell using codes that we changed mpi.h to mumps_mpi.h */
#define COIN_USE_MUMPS_MPI_H

/* copied from MUMPS' own mumps_compat.h */
#if defined(_WIN32) && ! defined(__MINGW32__)
# define MUMPS_WIN32 1
#endif

#if (__STDC_VERSION__ >= 199901L)
# define MUMPS_INLINE static inline
#else
# define MUMPS_INLINE
#endif


#endif /* MUMPS_COMPAT_H */
