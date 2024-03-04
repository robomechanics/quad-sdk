
#ifndef SLEQP_EXPORT_H
#define SLEQP_EXPORT_H

#ifdef SLEQP_STATIC_DEFINE
#  define SLEQP_EXPORT
#  define SLEQP_NO_EXPORT
#else
#  ifndef SLEQP_EXPORT
#    ifdef sleqp_EXPORTS
        /* We are building this library */
#      define SLEQP_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define SLEQP_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef SLEQP_NO_EXPORT
#    define SLEQP_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef SLEQP_DEPRECATED
#  define SLEQP_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef SLEQP_DEPRECATED_EXPORT
#  define SLEQP_DEPRECATED_EXPORT SLEQP_EXPORT SLEQP_DEPRECATED
#endif

#ifndef SLEQP_DEPRECATED_NO_EXPORT
#  define SLEQP_DEPRECATED_NO_EXPORT SLEQP_NO_EXPORT SLEQP_DEPRECATED
#endif

#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef SLEQP_NO_DEPRECATED
#    define SLEQP_NO_DEPRECATED
#  endif
#endif

#endif /* SLEQP_EXPORT_H */
