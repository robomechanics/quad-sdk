#ifndef SLEQP_PUB_MEM_H
#define SLEQP_PUB_MEM_H

/**
 * @file pub_mem.h
 * @brief Definition of memory (de-)allocation functions.
 **/

#include <stdlib.h>

#include "pub_error.h"
#include "pub_types.h"

#define sleqp_allocate_memory(ptr, size)                                       \
  (*(ptr) = ((size) > 0) ? malloc((size)) : NULL),                             \
    (((size) > 0) && (*(ptr) == NULL))                                         \
      ? (sleqp_set_error(__FILE__,                                             \
                         __LINE__,                                             \
                         __PRETTY_FUNCTION__,                                  \
                         SLEQP_NOMEM,                                          \
                         "Failed to allocate %ld bytes of memory",             \
                         size),                                                \
         SLEQP_ERROR)                                                          \
      : SLEQP_OKAY

#define sleqp_reallocate_memory(ptr, size)                                     \
  (*ptr = realloc(*ptr, size), (((size) > 0) && (*(ptr) == NULL)))             \
    ? (sleqp_set_error(__FILE__,                                               \
                       __LINE__,                                               \
                       __PRETTY_FUNCTION__,                                    \
                       SLEQP_NOMEM,                                            \
                       "Failed to allocate %ld bytes of memory",               \
                       size),                                                  \
       SLEQP_ERROR)                                                            \
    : SLEQP_OKAY

SLEQP_WARNUNUSED SLEQP_RETCODE
sleqp_free(void** ptr);

#define sleqp_malloc(ptr) sleqp_allocate_memory(ptr, sizeof(**ptr))

#define sleqp_alloc_array(ptr, count)                                          \
  sleqp_allocate_memory(ptr, ((count) * sizeof(**ptr)))

#define sleqp_realloc(ptr, count)                                              \
  sleqp_reallocate_memory(ptr, ((count) * sizeof(**ptr)))

#define sleqp_free(ptr)                                                        \
  free(*ptr);                                                                  \
  *ptr = NULL

#endif /* SLEQP_PUB_MEM_H */
