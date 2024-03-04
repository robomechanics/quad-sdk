/* src/Common/config_ipopt.h.  Generated from config_ipopt.h.in by configure.  */
/* src/Common/config_ipopt.h.in. */

#ifndef __CONFIG_IPOPT_H__
#define __CONFIG_IPOPT_H__

/* Version number of project */
#define IPOPT_VERSION "3.14.11"

/* Major Version number of project */
#define IPOPT_VERSION_MAJOR 3

/* Minor Version number of project */
#define IPOPT_VERSION_MINOR 14

/* Release Version number of project */
#define IPOPT_VERSION_RELEASE 11

/* Define to the debug sanity check level (0 is no test) */
#define IPOPT_CHECKLEVEL 0

/* Define to the debug verbosity level (0 is no output) */
#define IPOPT_VERBOSITY 0

/* Define to 1 if using single precision floating point */
/* #undef IPOPT_SINGLE */

/* Define to 1 if Ipopt index type is int64_t */
/* #undef IPOPT_INT64 */

/* Library Visibility Attribute */
#define IPOPTAMPLINTERFACELIB_EXPORT 

/* Library Visibility Attribute */
#define IPOPTLIB_EXPORT 

/* Library Visibility Attribute */
#define SIPOPTLIB_EXPORT 

/** type corresponding to integers in Fortran
 * @deprecated Use ipindex instead.
 */
#define IPOPT_FORTRAN_INTEGER_TYPE ipindex

#endif
