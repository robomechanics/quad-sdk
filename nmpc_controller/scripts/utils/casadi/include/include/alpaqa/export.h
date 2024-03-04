
#ifndef ALPAQA_EXPORT_H
#define ALPAQA_EXPORT_H

#ifdef ALPAQA_STATIC_DEFINE
#  define ALPAQA_EXPORT
#  define ALPAQA_NO_EXPORT
#else
#  ifndef ALPAQA_EXPORT
#    ifdef alpaqa_EXPORTS
        /* We are building this library */
#      define ALPAQA_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define ALPAQA_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef ALPAQA_NO_EXPORT
#    define ALPAQA_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef ALPAQA_DEPRECATED
#  define ALPAQA_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef ALPAQA_DEPRECATED_EXPORT
#  define ALPAQA_DEPRECATED_EXPORT ALPAQA_EXPORT ALPAQA_DEPRECATED
#endif

#ifndef ALPAQA_DEPRECATED_NO_EXPORT
#  define ALPAQA_DEPRECATED_NO_EXPORT ALPAQA_NO_EXPORT ALPAQA_DEPRECATED
#endif

#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef ALPAQA_NO_DEPRECATED
#    define ALPAQA_NO_DEPRECATED
#  endif
#endif

#endif /* ALPAQA_EXPORT_H */
