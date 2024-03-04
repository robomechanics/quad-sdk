#ifndef SLEQP_PUB_ERROR_H
#define SLEQP_PUB_ERROR_H

/**
 * @file pub_error.h
 * @brief Error handling.
 **/

#include "pub_types.h"

/**
 * Returns the type of the current error
 **/
SLEQP_EXPORT SLEQP_ERROR_TYPE
sleqp_error_type();

/**
 * Returns the message associated
 * with  the current error
 **/
SLEQP_EXPORT const char*
sleqp_error_msg();

/**
 * Sets the current error. To be used by @ref sleqp_raise
 **/
SLEQP_EXPORT void
sleqp_set_error(const char* file,
                int line,
                const char* func,
                SLEQP_ERROR_TYPE error_type,
                const char* fmt,
                ...) SLEQP_FORMAT_PRINTF(5, 6);

/**
 * Raises an error with the given type and message
 **/
#define sleqp_raise(error_type, fmt, ...)                                      \
  do                                                                           \
  {                                                                            \
    sleqp_set_error(__FILE__,                                                  \
                    __LINE__,                                                  \
                    __PRETTY_FUNCTION__,                                       \
                    error_type,                                                \
                    fmt,                                                       \
                    ##__VA_ARGS__);                                            \
    return SLEQP_ERROR;                                                        \
  } while (false)

#endif /* SLEQP_PUB_ERROR_H */
