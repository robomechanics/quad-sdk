#ifndef Interpolators__2D_ThinPlateSplineInterpolator_hpp
#define Interpolators__2D_ThinPlateSplineInterpolator_hpp

/** @file ThinPlateSplineInterpolator.hpp
  * @brief 
  * @author C.D. Clark III
  * @date 12/27/16
  */

#include "InterpolatorBase.hpp"

/** @class 
  * @brief Cubic spline interpolation for for 2D functions.
  * @author C.D. Clark III, Aaron Hoffman
  *
  * This class implements the "Thin Plate Spline" method as derived by David Eberly (https://www.geometrictools.com/Documentation/ThinPlateSplines.pdf)
  * It is essentially the 2D equivalent of cubic splines for 1D. 
  *
  * This method has a performance cost over other methods. Specifically, for N interpolation points, an NxN matrix
  * must be inverted. The matrix inversion is only required during setup, but interpolation will be based on all
  * N points as well, rather than the nearest neighbors. So, it is possible that this method will be slow for
  * large data sets.
  * 
  */

namespace _2D {

template<class Real>
class ThinPlateSplineInterpolator : public InterpolatorBase<ThinPlateSplineInterpolator<Real>>
{
  public:
    using BASE = InterpolatorBase<ThinPlateSplineInterpolator<Real>>;
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
    
    MatrixType a, b;

  public:

    template<typename I>
    ThinPlateSplineInterpolator( I n, Real *x, Real *y, Real *z ) {this->setData(n,x,y,z);}

    template<typename X, typename Y, typename Z>
    ThinPlateSplineInterpolator( X &x, Y &y, Z &z ) {this->setData(x,y,z);}

    ThinPlateSplineInterpolator():BASE()
    { }

    ThinPlateSplineInterpolator(const ThinPlateSplineInterpolator& rhs)
    :BASE(rhs)
    ,a(rhs.a)
    ,b(rhs.b)
    {
    }

    // copy-swap idiom
    friend void swap( ThinPlateSplineInterpolator& lhs, ThinPlateSplineInterpolator& rhs)
    {
      lhs.a.swap(rhs.a);
      lhs.b.swap(rhs.b);
      swap( static_cast<BASE&>(lhs), static_cast<BASE&>(rhs) );
    }

    ThinPlateSplineInterpolator& operator=(ThinPlateSplineInterpolator rhs)
    {
      swap(*this,rhs);
      return *this;
    }

    Real operator()( Real x, Real y ) const;

  protected:

    Real G(Real x, Real y, Real xi, Real yi) const;

    void setupInterpolator();
    friend BASE;




};

template<class Real>
void
ThinPlateSplineInterpolator<Real>::setupInterpolator()
{
  a = MatrixType(this->xView->rows(),1);
  b = MatrixType(this->xView->rows(),3);

  MatrixType M = MatrixType(this->xView->rows(),this->xView->rows());
  MatrixType N = MatrixType(this->xView->rows(),3);

  // rows
  for( int i = 0; i < this->xView->rows(); i++ )
  {
    // N
    N(i,0) = 1;
    N(i,1) = this->xView->coeff(i);
    N(i,2) = this->yView->coeff(i);

    for( int j = 0; j < this->xView->rows(); j++ )
    {
      M(i,j) = G(this->xView->coeff(i), this->yView->coeff(i), this->xView->coeff(j), this->yView->coeff(j) );
    }


  }

  MatrixType Minv = M.inverse();
  MatrixType Ntra = N.transpose();

  b = (Ntra*Minv*N).inverse()*Ntra*Minv*(*this->zView);
  a = Minv*((*this->zView) - N*b);

  return;

}

template<class Real>
Real
ThinPlateSplineInterpolator<Real>::G(Real x1, Real y1, Real x2, Real y2) const
{
  if( x1 == x2 && y1 == y2 )
    return 0;

  Real r = sqrt( (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) );

  return r*r*log(r);
}


template<class Real>
Real
ThinPlateSplineInterpolator<Real>::operator()( Real x, Real y ) const
{
  BASE::checkData();
  
  // no extrapolation...
  if( x < this->xView->minCoeff()
   || x > this->xView->maxCoeff()
   || y < this->yView->minCoeff()
   || y > this->yView->maxCoeff() )
  {
    return 0;
  }
  
  MatrixType Gx( 1, this->xView->rows() );
  for(int i = 0; i < this->xView->rows(); i++)
    Gx(i) = G( x, y, this->xView->coeff(i), this->yView->coeff(i) );

  Real f = (Gx*a)(0,0) + b(0) + b(1)*x + b(2)*y;

  return f;
}





}

#endif // include protector
