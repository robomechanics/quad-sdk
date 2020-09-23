#ifndef Interpolators__2D_BilinearInterpolator_hpp
#define Interpolators__2D_BilinearInterpolator_hpp

/** @file BilinearInterpolator.hpp
  * @brief 
  * @author C.D. Clark III
  * @date 12/27/16
  */

#include "InterpolatorBase.hpp"
#include <boost/range/algorithm/lower_bound.hpp>
#include <boost/range/adaptor/strided.hpp>

/** @class 
  * @brief Linear interpolation for 2D functions.
  * @author C.D. Clark III, Aaron Hoffman
  */

namespace _2D {

template<class Real>
class BilinearInterpolator : public InterpolatorBase<BilinearInterpolator<Real>>
{
  public:
    using BASE = InterpolatorBase<BilinearInterpolator<Real>>;
    using VectorType = typename BASE::VectorType;
    using MapType = typename BASE::MapType; 

    // types used to view data as 2D coordinates
    using MatrixType    = typename BASE::MatrixType;
    using _2DVectorView = typename BASE::_2DVectorView;
    using _2DMatrixView = typename BASE::_2DMatrixView;

    // types used for 2x2 matrix algebra
    using Matrix22 = Eigen::Matrix<Real,2,2 >;
    using Matrix22Array = Eigen::Array< Matrix22, Eigen::Dynamic, Eigen::Dynamic >;
    using ColVector2 = Eigen::Matrix<Real,2,1 >;
    using RowVector2 = Eigen::Matrix<Real,1,2 >;

  protected:

    using BASE::xView;
    using BASE::yView;
    using BASE::zView;
    using BASE::X;
    using BASE::Y;
    using BASE::Z;
    
    Matrix22Array Q; // naming convention used by wikipedia article (see Wikipedia https://en.wikipedia.org/wiki/Bilinear_interpolation)


  public:

    template<typename I>
    BilinearInterpolator( I n, Real *x, Real *y, Real *z ) {this->setData(n,x,y,z);}

    template<typename X, typename Y, typename Z>
    BilinearInterpolator( X &x, Y &y, Z &z ) {this->setData(x,y,z);}

    BilinearInterpolator():BASE(){}
    BilinearInterpolator(const BilinearInterpolator& rhs)
    :BASE(rhs)
    ,Q(rhs.Q)
    {}

    // copy-swap idiom
    friend void swap( BilinearInterpolator& lhs, BilinearInterpolator& rhs)
    {
      lhs.Q.swap(rhs.Q);
      swap( static_cast<BASE&>(lhs), static_cast<BASE&>(rhs) );
    }

    BilinearInterpolator& operator=(BilinearInterpolator rhs)
    {
      swap(*this,rhs);
      return *this;
    }

    Real operator()( Real x, Real y ) const;


  private:

    void setupInterpolator();
    friend BASE;

};

template<class Real>
void
BilinearInterpolator<Real>::setupInterpolator()
{

  // Interpolation will be done by multiplying the coordinates by coefficients.

  Q = Matrix22Array( X->size()-1, Y->size()-1 );

  // We are going to pre-compute the interpolation coefficients so
  // that we can interpolate quickly
  for(int i = 0; i < X->size() - 1; i++)
  {
    for( int j = 0; j < Y->size() - 1; j++)
    {
      Real tmp = ( ((*X)(i+1) - (*X)(i) )*( (*Y)(j+1) - (*Y)(j) ) );
      Q(i,j) = Z->block(i,j,2,2)/tmp;
    }
  }
}

template<class Real>
Real
BilinearInterpolator<Real>::operator()( Real x, Real y ) const
{
  BASE::checkData();
  
  // no extrapolation...
  if( x < (*X)(0)
   || x > (*X)(X->size()-1)
   || y < (*Y)(0)
   || y > (*Y)(Y->size()-1) )
  {
    return 0;
  }
  // find the x index that is just to the LEFT of x
  //int i  = Utils::index__last_lt( x, *X, X->size() );
  // NOTE: X data is strided.
  auto xrng = std::make_pair( X->data(), X->data()+X->size()*X->innerStride() ) | boost::adaptors::strided(X->innerStride());
  int i = boost::lower_bound( xrng, x) - boost::begin(xrng) - 1;
  if(i < 0)
    i = 0;

  // find the y index that is just BELOW y
  //int j  = Utils::index__last_lt( y, *Y, Y->size() );
  // NOTE: Y data is NOT strided
  auto yrng = std::make_pair( Y->data(), Y->data()+Y->size() );
  int j = boost::lower_bound( yrng, y) - boost::begin(yrng) - 1;
  if(j < 0)
    j = 0;
  


  // now, create the coordinate vectors (see Wikipedia https://en.wikipedia.org/wiki/Bilinear_interpolation)
  RowVector2 vx;
  ColVector2 vy;
  vx << ((*X)(i+1) - x), (x - (*X)(i));
  vy << ((*Y)(j+1) - y), (y - (*Y)(j));

  // interpolation is just x*Q*y

  return vx*Q(i,j)*vy;
}



}

#endif // include protector
