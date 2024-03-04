#pragma once

#include <alpaqa/export.h>

#ifndef DOXYGEN

#ifdef _WIN32
#ifdef __GNUC__ // mingw
#define ALPAQA_EXPORT_EXTERN_TEMPLATE(strcls, name, ...)
#define ALPAQA_EXPORT_TEMPLATE(strcls, name, ...)                              \
    template strcls ALPAQA_EXPORT name<__VA_ARGS__>
#else
#define ALPAQA_EXPORT_EXTERN_TEMPLATE(strcls, name, ...)                       \
    extern template strcls name<__VA_ARGS__>
#define ALPAQA_EXPORT_TEMPLATE(strcls, name, ...)                              \
    template strcls ALPAQA_EXPORT name<__VA_ARGS__>
#endif
#else
#define ALPAQA_EXPORT_EXTERN_TEMPLATE(strcls, name, ...)                       \
    extern template strcls ALPAQA_EXPORT name<__VA_ARGS__>
#define ALPAQA_EXPORT_TEMPLATE(strcls, name, ...)                              \
    template strcls ALPAQA_EXPORT name<__VA_ARGS__>
#endif

#else // DOXYGEN

#define ALPAQA_EXPORT_EXTERN_TEMPLATE(...)
#define ALPAQA_EXPORT_TEMPLATE(...)

#endif // DOXYGEN