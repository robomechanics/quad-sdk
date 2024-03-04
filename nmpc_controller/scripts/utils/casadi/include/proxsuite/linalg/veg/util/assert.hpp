#include "proxsuite/linalg/veg/internal/assert_impl.hpp"

#ifdef VEG_DEBUG_ASSERT
#undef VEG_DEBUG_ASSERT
#endif
#ifdef VEG_DEBUG_EXPECT
#undef VEG_DEBUG_EXPECT
#endif
#ifdef VEG_DEBUG_ASSERT_ELSE
#undef VEG_DEBUG_ASSERT_ELSE
#endif
#ifdef VEG_DEBUG_EXPECT_ELSE
#undef VEG_DEBUG_EXPECT_ELSE
#endif
#ifdef VEG_DEBUG_ASSERT_ALL_OF
#undef VEG_DEBUG_ASSERT_ALL_OF
#endif
#ifdef VEG_DEBUG_EXPECT_ALL_OF
#undef VEG_DEBUG_EXPECT_ALL_OF
#endif
#ifdef VEG_DEBUG_ASSERT_ALL_OF_ELSE
#undef VEG_DEBUG_ASSERT_ALL_OF_ELSE
#endif
#ifdef VEG_DEBUG_EXPECT_ALL_OF_ELSE
#undef VEG_DEBUG_EXPECT_ALL_OF_ELSE
#endif

#ifdef NDEBUG
#define VEG_DEBUG_ASSERT(...) ((void)(0))
#define VEG_DEBUG_EXPECT(...) ((void)(0))
#define VEG_DEBUG_ASSERT_ELSE(Message, ...) ((void)(0))
#define VEG_DEBUG_EXPECT_ELSE(Message, ...) ((void)(0))
#define VEG_DEBUG_ASSERT_ALL_OF(...) ((void)(0))
#define VEG_DEBUG_EXPECT_ALL_OF(...) ((void)(0))
#define VEG_DEBUG_ASSERT_ALL_OF_ELSE(...) ((void)(0))
#define VEG_DEBUG_EXPECT_ALL_OF_ELSE(...) ((void)(0))
#else
#define VEG_DEBUG_ASSERT(...) VEG_ASSERT(__VA_ARGS__)
#define VEG_DEBUG_EXPECT(...) VEG_EXPECT(__VA_ARGS__)
#define VEG_DEBUG_ASSERT_ELSE(Message, ...)                                    \
  VEG_ASSERT_ELSE(Message, __VA_ARGS__)
#define VEG_DEBUG_EXPECT_ELSE(Message, ...)                                    \
  VEG_EXPECT_ELSE(Message, __VA_ARGS__)
#define VEG_DEBUG_ASSERT_ALL_OF(...) VEG_ASSERT_ALL_OF(__VA_ARGS__)
#define VEG_DEBUG_EXPECT_ALL_OF(...) VEG_EXPECT_ALL_OF(__VA_ARGS__)
#define VEG_DEBUG_ASSERT_ALL_OF_ELSE(...) VEG_ASSERT_ALL_OF_ELSE(__VA_ARGS__)
#define VEG_DEBUG_EXPECT_ALL_OF_ELSE(...) VEG_EXPECT_ALL_OF_ELSE(__VA_ARGS__)
#endif
