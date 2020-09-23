#ifndef Interpolators__1D_InterpolatorBase_hpp
#define Interpolators__1D_InterpolatorBase_hpp

/** @file InterpolatorBase.hpp
  * @brief 
  * @author C.D. Clark III
  * @date 12/24/16
  */

#include <memory>
#include <algorithm>
#include <type_traits>
#include <vector>
#include <Eigen/Dense>


namespace _1D {

/** @class 
  * @brief A base class that provides some useful functions for interpolating data.
  * @author C.D. Clark III
  *
  * This class provides eigen matrices to store the data that is interpolated, a few setData methods
  * to populate the data.
  */

template<typename T>
struct RealTypeOf { using type = double; };
template<template<typename> class T,typename R>
struct RealTypeOf<T<R>> { using type = R; };

template<class Derived, typename Real = typename RealTypeOf<Derived>::type>
class InterpolatorBase
{

  public:
    using VectorType =  Eigen::Matrix<Real,Eigen::Dynamic,1>;
    using MapType =  Eigen::Map<const VectorType>;

  protected:
    // data members
    std::vector<Real> xData, yData;        ///< storage for interpolated data
    std::unique_ptr<MapType> xView, yView; ///< eigen matrix view of the data

  private:
    // making constructors private and the derived class a friend
    // helps to make sure that the derived class actually
    // passes itself as the template argument.
    friend Derived;

    InterpolatorBase(){ }

    // copy constructor
    // we only want to initialize x and y views if the object
    // we are copying from is initialized.
    InterpolatorBase(const InterpolatorBase& rhs)
    :xData(rhs.xData)
    ,yData(rhs.yData)
    ,xView( rhs.xView ? new MapType( xData.data(), xData.size() ) : nullptr )
    ,yView( rhs.xView ? new MapType( yData.data(), yData.size() ) : nullptr )
    { 
    }

    // we use the copy-swap idiom for copy assignment
    friend void swap( InterpolatorBase& lhs, InterpolatorBase& rhs)
    {
      std::swap( lhs.xData, rhs.xData );
      std::swap( lhs.yData, rhs.yData );
      std::swap( lhs.xView, rhs.xView );
      std::swap( lhs.yView, rhs.yView );
    }

    InterpolatorBase& operator=(InterpolatorBase rhs)
    {
      swap(*this,rhs);
      return *this;
    }

  public:

    // methods to get the data
    std::vector<Real> getXData() const { return xData; }
    std::vector<Real> getYData() const { return yData; }


    // methods to set data
    // primary method. this is used by the others
    template<typename I>
    void setData( I n, const Real *x, const Real *y );

    // this template is ambiguous with the pointer template above,
    // so we want to disable it for pointers.
    template<typename X, typename Y>
    typename std::enable_if<!std::is_pointer<Y>::value>::type
    setData( const X &x, const Y &y);

  protected:

    void checkData() const; ///< Check that data has been initialized and throw exception if not.

  private:
    // callSetupInterpolator will call a function named setupInterpolator in the derived class, if
    // it exists. this is just some template magic to detect if the derived class has implemented a
    // setupInterpolator function, and to call it if it does.

    template<typename T>
    struct has_setupInterpolator
    {
      private:
        typedef std::true_type yes;
        typedef std::false_type no;

        template<typename U> static auto test(int) -> decltype(std::declval<U>().setupInterpolator(),yes());
        template<typename  > static no   test(...);

      public:
        static constexpr bool value = std::is_same<decltype(test<T>(0)),yes>::value;
    };
    
    template<typename T>
    typename std::enable_if<has_setupInterpolator<T>::value>::type
    callSetupInterpolator(){ static_cast<Derived*>(this)->setupInterpolator(); }

    template<typename T>
    typename std::enable_if<!has_setupInterpolator<T>::value>::type
    callSetupInterpolator(){ }


};


// implementations


template<class Derived, typename Real>
void
InterpolatorBase<Derived,Real>::checkData() const
{
  if(!this->xView || !this->yView)
    throw std::logic_error("Interpolator data is not initialized. Did you call setData()?");
  if(this->xView->size() == 0 || this->yView->size() == 0)
    throw std::logic_error("Interpolator data is zero size. Did you call setData() with non-zero sized vectors?");
}

template<class Derived, typename Real>
template<typename I>
void
InterpolatorBase<Derived,Real>::setData( I n, const Real *x, const Real *y)
{
  xData.clear();
  yData.clear();
  std::copy( x, x+n, std::back_inserter(xData) );
  std::copy( y, y+n, std::back_inserter(yData) );
    
  this->xView.reset( new MapType( xData.data(), n ) );
  this->yView.reset( new MapType( yData.data(), n ) );

  this->callSetupInterpolator<Derived>();
}

template<class Derived, typename Real>
template<typename X, typename Y>
typename std::enable_if<!std::is_pointer<Y>::value>::type
InterpolatorBase<Derived,Real>::setData( const X &x, const Y &y)
{
  this->setData( x.size(), x.data(), y.data() );
}


}

#endif // include protector
