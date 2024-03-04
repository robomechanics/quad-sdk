#ifndef VEG_TYPEDEFS_HPP_2UKLEQTSS
#define VEG_TYPEDEFS_HPP_2UKLEQTSS

#include <cstdint>
#include <cstddef>

namespace proxsuite {
namespace linalg {
namespace veg {
namespace _detail {
namespace _meta {
template<typename T>
struct make_signed;
template<>
struct make_signed<unsigned char>
{
  using Type = signed char;
};
template<>
struct make_signed<unsigned short>
{
  using Type = signed short;
};
template<>
struct make_signed<unsigned int>
{
  using Type = signed int;
};
template<>
struct make_signed<unsigned long>
{
  using Type = signed long;
};
template<>
struct make_signed<unsigned long long>
{
  using Type = signed long long;
};
} // namespace _meta
} // namespace _detail

using usize = decltype(sizeof(0));
using isize = _detail::_meta::make_signed<usize>::Type;

using i64 = std::int64_t;
using u64 = std::uint64_t;
using i32 = std::int32_t;
using u32 = std::uint32_t;
using i16 = std::int16_t;
using u16 = std::uint16_t;
using u8 = std::uint8_t;
using i8 = std::int8_t;

} // namespace veg
} // namespace linalg
} // namespace proxsuite

#endif /* end of include guard VEG_TYPEDEFS_HPP_2UKLEQTSS */
