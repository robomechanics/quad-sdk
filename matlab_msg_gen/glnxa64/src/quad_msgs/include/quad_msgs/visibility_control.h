#ifndef QUAD_MSGS__VISIBILITY_CONTROL_H_
#define QUAD_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define QUAD_MSGS_EXPORT __attribute__ ((dllexport))
    #define QUAD_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define QUAD_MSGS_EXPORT __declspec(dllexport)
    #define QUAD_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef QUAD_MSGS_BUILDING_LIBRARY
    #define QUAD_MSGS_PUBLIC QUAD_MSGS_EXPORT
  #else
    #define QUAD_MSGS_PUBLIC QUAD_MSGS_IMPORT
  #endif
  #define QUAD_MSGS_PUBLIC_TYPE QUAD_MSGS_PUBLIC
  #define QUAD_MSGS_LOCAL
#else
  #define QUAD_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define QUAD_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define QUAD_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define QUAD_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define QUAD_MSGS_PUBLIC
    #define QUAD_MSGS_LOCAL
  #endif
  #define QUAD_MSGS_PUBLIC_TYPE
#endif
#endif  // QUAD_MSGS__VISIBILITY_CONTROL_H_
// Generated 24-Aug-2025 22:14:39
 