
#ifndef TELEOP_TWIST_JOY_EXPORT_H
#define TELEOP_TWIST_JOY_EXPORT_H

#ifdef TELEOP_TWIST_JOY_STATIC_DEFINE
#  define TELEOP_TWIST_JOY_EXPORT
#  define TELEOP_TWIST_JOY_NO_EXPORT
#else
#  ifndef TELEOP_TWIST_JOY_EXPORT
#    ifdef teleop_twist_joy_EXPORTS
        /* We are building this library */
#      define TELEOP_TWIST_JOY_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define TELEOP_TWIST_JOY_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef TELEOP_TWIST_JOY_NO_EXPORT
#    define TELEOP_TWIST_JOY_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef TELEOP_TWIST_JOY_DEPRECATED
#  define TELEOP_TWIST_JOY_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef TELEOP_TWIST_JOY_DEPRECATED_EXPORT
#  define TELEOP_TWIST_JOY_DEPRECATED_EXPORT TELEOP_TWIST_JOY_EXPORT TELEOP_TWIST_JOY_DEPRECATED
#endif

#ifndef TELEOP_TWIST_JOY_DEPRECATED_NO_EXPORT
#  define TELEOP_TWIST_JOY_DEPRECATED_NO_EXPORT TELEOP_TWIST_JOY_NO_EXPORT TELEOP_TWIST_JOY_DEPRECATED
#endif

#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef TELEOP_TWIST_JOY_NO_DEPRECATED
#    define TELEOP_TWIST_JOY_NO_DEPRECATED
#  endif
#endif

#endif /* TELEOP_TWIST_JOY_EXPORT_H */
