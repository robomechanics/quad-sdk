#ifndef VEG_HAS_ASAN_HPP_AWVAMMSFS
#define VEG_HAS_ASAN_HPP_AWVAMMSFS

#if defined(__clang__)
#if __has_feature(address_sanitizer)
#define VEG_HAS_ASAN 1
#else
#define VEG_HAS_ASAN 0
#endif

#elif defined(__SANITIZE_ADDRESS__) && __SANITIZE_ADDRESS__ == 1
#define VEG_HAS_ASAN 1
#else
#define VEG_HAS_ASAN 0
#endif

#endif /* end of include guard VEG_HAS_ASAN_HPP_AWVAMMSFS */
