#ifndef Interpolators__1D_AnyInterpolator_hpp
#define Interpolators__1D_AnyInterpolator_hpp

/** @file AnyInterpolator.hpp
  * @brief 
  * @author C.D. Clark III
  * @date 01/17/19
  */



#include "../../Utils/Concepts.hpp"

namespace _1D {

template<typename T, typename Sig = void(int,T*,T*)>
using AnyInterpolator = boost::type_erasure::any< boost::mpl::vector<
                                                       boost::type_erasure::copy_constructible<>,
                                                       is_interpolator<T,Sig>,
                                                       boost::type_erasure::relaxed >, boost::type_erasure::_self >;


}


#endif // include protector
