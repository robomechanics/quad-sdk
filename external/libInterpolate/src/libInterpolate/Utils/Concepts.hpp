#ifndef Utils_Concepts_hpp
#define Utils_Concepts_hpp

/** @file Concepts.hpp
  * @brief 
  * @author C.D. Clark III
  * @date 01/16/19
  */
#include <boost/type_erasure/any.hpp>
#include <boost/type_erasure/member.hpp>
#include <boost/type_erasure/callable.hpp>
#include <boost/mpl/vector.hpp>

BOOST_TYPE_ERASURE_MEMBER( (has_setData), setData )

namespace _1D {
  template<typename T, typename Sig = void(int,T*,T*)>
  using is_interpolator = boost::mpl::vector< boost::type_erasure::callable<T(T)>, has_setData< Sig > >;
}

namespace _2D {
  template<typename T, typename Sig = void(int,T*,T*)>
  using is_interpolator = boost::mpl::vector< boost::type_erasure::callable<T(T,T)>, has_setData< Sig > >;
}


#endif // include protector
