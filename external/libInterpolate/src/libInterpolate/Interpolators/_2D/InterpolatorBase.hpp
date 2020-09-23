#ifndef Interpolators__2D_InterpolatorBase_hpp
#define Interpolators__2D_InterpolatorBase_hpp

/** @file InterpolatorBase.hpp
  * @author C.D. Clark III
  * @date 12/24/16
  */

#include <memory>
#include <vector>
#include <Eigen/Dense>


namespace _2D {

template<typename T>
struct RealTypeOf { using type = double; };
template<template<typename> class T,typename R>
struct RealTypeOf<T<R>> { using type = R; };

/** @class 
  * @brief A base class for 2D interpolators that provides default implementations of the interface.
  * @author C.D. Clark III
  *
  * This class provides an implementation for the setData method as well as adding a few additional
  * useful methods, including derivative and integral methods.
  */
template<class Derived, typename Real = typename RealTypeOf<Derived>::type>
class InterpolatorBase
{
  public:
    using VectorType = Eigen::Matrix<Real,Eigen::Dynamic,1>;
    using MapType = Eigen::Map<const VectorType>;
    // types used to view data as 2D coordinates
    using MatrixType = typename Eigen::Matrix<Real,Eigen::Dynamic,Eigen::Dynamic>;
    using _2DVectorView = Eigen::Map<const VectorType,Eigen::Unaligned,Eigen::InnerStride<Eigen::Dynamic>>;
    using _2DMatrixView = Eigen::Map<const MatrixType,Eigen::Unaligned,Eigen::Stride<Eigen::Dynamic,Eigen::Dynamic>>;

  protected:

    std::vector<Real> xData, yData, zData;        // data storage
    std::unique_ptr<MapType> xView, yView, zView; // map view of the data

    // these maps are used to view the x,y,z data as two coordinate vectors and a function matrix, instead of three vectors.
    std::unique_ptr<_2DVectorView> X,Y;
    std::unique_ptr<_2DMatrixView> Z;

  private:
    // making constructors private and the derived class a friend
    // helps to make sure that the derived class actually
    // passes itself as the template argument.
    friend Derived;

    InterpolatorBase(){ }

    InterpolatorBase( const InterpolatorBase& rhs )
    :xData(rhs.xData)
    ,yData(rhs.yData)
    ,zData(rhs.zData)
    ,xView( rhs.xView ? new MapType( xData.data(), xData.size() ) : nullptr )
    ,yView( rhs.xView ? new MapType( yData.data(), yData.size() ) : nullptr )
    ,zView( rhs.zView ? new MapType( zData.data(), zData.size() ) : nullptr )
    {
      this->setup2DDataViews();
    }

    // we use the copy-swap idiom for copy assignment
    friend void swap( InterpolatorBase& lhs, InterpolatorBase& rhs)
    {
      std::swap( lhs.xData, rhs.xData );
      std::swap( lhs.yData, rhs.yData );
      std::swap( lhs.zData, rhs.zData );
      std::swap( lhs.xView, rhs.xView );
      std::swap( lhs.yView, rhs.yView );
      std::swap( lhs.zView, rhs.zView );
      std::swap( lhs.X, rhs.X);
      std::swap( lhs.Y, rhs.Y);
      std::swap( lhs.Z, rhs.Z);
    }

    InterpolatorBase& operator=(InterpolatorBase rhs)
    {
      swap(*this,rhs);
      return *this;
    }

  public:

    std::vector<Real> getXData() const { return xData; }
    std::vector<Real> getYData() const { return yData; }
    std::vector<Real> getZData() const { return zData; }

    template<typename I>
    void setData( I n, const Real *x, const Real *y, const Real *z);

    // this template is ambiguous with the pointer template above,
    // wo we want to disable it for pointers
    template<typename XT, typename YT, typename ZT>
    typename std::enable_if<!std::is_pointer<YT>::value>::type
    setData( const XT &x, const YT &y, const ZT &z );

  protected:
    void checkData() const; ///< Check that data has been initialized and throw exception if not.
    void setup2DDataViews(); ///< Setups up 2D views of 1D data arrays

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

template<class Derived, typename Real>
void
InterpolatorBase<Derived,Real>::checkData() const
{
  if(!this->xView || !this->yView || !this->zView)
    throw std::logic_error("Interpolator data is not initialized. Did you call setData()?");
  if(this->xView->size() == 0 || this->yView->size() == 0 || this->zView->size() == 0)
    throw std::logic_error("Interpolator data is zero size. Did you call setData() with non-zero sized vectors?");
}

template<class Derived, typename Real>
void
InterpolatorBase<Derived,Real>::setup2DDataViews()
{
  // make sure data has actually been initialized
  if( !xView || !yView || !zView )
    return;


  // setup 2D view of the data
  // We need to figure out what the x and y dimensions are.
  int N = zView->size();
  int Nx = 0, Ny = 0;
  // Ny will be the number of elements that have the same x coordinate
  Real xlast = (*xView)(0);
  while( Ny < N-1 && fabs((*xView)(Ny)-xlast) < 1e-40 )
    Ny++;
  Nx = N/Ny;

  // consecutive values in the x data are separated by Ny, so this is the inner stride for X
  X.reset( new _2DVectorView( xView->data(), Nx, Eigen::InnerStride<Eigen::Dynamic>(Ny) ) );

  // consecutive values in the y data are next to each other, so the stride is just 1
  Y.reset( new _2DVectorView( yView->data(), Ny, Eigen::InnerStride<Eigen::Dynamic>(1) ) );

  // Eigen defaults to COLUMN MAJOR
  // consecutive elements in a column are separated by Ny (this is the inner stride)
  // consecutive elements in a row are located next to each other (this is the outer stride)
  // Stride object takes outer,inner as arguments.
  Z.reset( new _2DMatrixView( zView->data(), Nx, Ny, Eigen::Stride<Eigen::Dynamic,Eigen::Dynamic>(1,Ny) ) );
}






/**
 * Reads data from x, y, and z arrays. Arrays should all be the same length with each element corresponding to a data point.
 * Basically, x[i], y[i], and z[i] should correspond to the first, second, and third columns of a gnuplot file.
 */
template<class Derived, typename Real>
template<typename I>
void
InterpolatorBase<Derived,Real>::setData( I n, const Real *x, const Real *y, const Real *z )
{
  xData.clear();
  yData.clear();
  zData.clear();
  std::copy( x, x+n, std::back_inserter(xData) );
  std::copy( y, y+n, std::back_inserter(yData) );
  std::copy( z, z+n, std::back_inserter(zData) );

  this->xView.reset( new MapType( xData.data(), n ) );
  this->yView.reset( new MapType( yData.data(), n ) );
  this->zView.reset( new MapType( zData.data(), n ) );

  this->setup2DDataViews();
  this->callSetupInterpolator<Derived>();
}

template<class Derived, typename Real>
template<typename XT, typename YT, typename ZT>
typename std::enable_if<!std::is_pointer<YT>::value>::type
InterpolatorBase<Derived,Real>::setData( const XT &x, const YT &y, const ZT &z )
{
  this->setData( x.size(), x.data(), y.data(), z.data() );
}

}

#endif // include protector
