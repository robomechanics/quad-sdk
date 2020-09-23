#ifndef Interpolators__1D_MonotonicInterpolator_hpp
#define Interpolators__1D_MonotonicInterpolator_hpp

/** @file MonotonicInterpolator.hpp
  * @brief 
  * @author C.D. Clark III
  * @date 06/06/17
  */

#include "InterpolatorBase.hpp"
#include <boost/range/algorithm/lower_bound.hpp>

namespace _1D {

/** @class MonotonicInterpolator

  * @brief A simple monotonic interpolator based on Steffen, "A simple method for monotonic interpolation in one dimension", 1990.
  * @author C.D. Clark III
  */
template<class Real>
class MonotonicInterpolator : public InterpolatorBase<MonotonicInterpolator<Real>>
{
  public:
    using BASE = InterpolatorBase<MonotonicInterpolator<Real>>;
    using VectorType = typename BASE::VectorType;
    using MapType = typename BASE::MapType;

  protected:
    VectorType a,b,yplow,yphigh;

  public:

    Real operator()( Real x ) const;

    template<typename I>
    MonotonicInterpolator( I n, Real *x, Real *y ) { this->setData(n,x,y); }

    template<typename X, typename Y>
    MonotonicInterpolator( X &x, Y &y ) { this->setData(x,y); }

    MonotonicInterpolator():BASE(){}
    MonotonicInterpolator( const MonotonicInterpolator& rhs )
    :BASE(rhs)
    ,a(rhs.a)
    ,b(rhs.b)
    ,yplow(rhs.yplow)
    ,yphigh(rhs.yphigh)
    {
    }

    // copy-swap idiom
    friend void swap( MonotonicInterpolator& lhs, MonotonicInterpolator& rhs)
    {
      lhs.a.swap(rhs.a);
      lhs.b.swap(rhs.b);
      lhs.yplow.swap(rhs.yplow);
      lhs.yphigh.swap(rhs.yphigh);
      swap( static_cast<BASE&>(lhs), static_cast<BASE&>(rhs) );
    }

    MonotonicInterpolator& operator=(MonotonicInterpolator rhs)
    {
      swap(*this,rhs);
      return *this;
    }



  protected:

    void setupInterpolator();
    friend BASE;

};

template<class Real>
void
MonotonicInterpolator<Real>::setupInterpolator()
{
  const VectorType &X = *(this->xView);
  const VectorType &Y = *(this->yView);

  a      = VectorType(X.size()-1);
  b      = VectorType(X.size()-1);
  yplow  = VectorType(X.size()-1);
  yphigh = VectorType(X.size()-1);


  for(int i = 0; i < X.size()-1; i++)
  {
    // i is the *interval* index
    // the interval spans from X(i) to X(i+1)
    Real xlow  = X(i);
    Real xhigh = X(i+1);
    Real ylow  = Y(i);
    Real yhigh = Y(i+1);
    Real h = xhigh - xlow;

    Real slope = (yhigh - ylow)/h;

    // Determine the first derivative values used at the endpoints of the interval
    // These values may be limited to preserve monotonicity
    if (i==0)
    {
      // first interval does not have an interval to its "left", so just
      // use the slope in the interval.
      yplow[i] = slope;
    }
    else
    {
      // Determine the lower slope value
      Real hlow = xlow - X(i-1);
      Real slope_low = 0.0;
      if (hlow > 0.0)
        slope_low = (ylow - Y(i-1))/hlow;
      if (slope_low* slope <= 0.0)
      {
        // Set derivative as zero
        yplow[i] = 0.0;
      }
      else
      {
        yplow[i] = ((slope_low*h) + (slope*hlow))/(hlow + h);
        if (yplow[i] >= 0.0)
        {
          yplow[i] = (std::min)(yplow[i], 2.0*(std::min)(slope_low, slope));
        }
        else
        {
          yplow[i] = (std::max)(yplow[i], 2.0*(std::max)(slope_low, slope));
        }
      }
    }

  
	if(i == (X.size()-2))
	{
    // last interval does not have an interval to its "right", so just
    // use the slope in the interval.
		yphigh[i] = slope;
	}
	else
	{
		// Determine the upper slope value
		Real hhigh = Y(i+2) - xhigh;
		Real slope_high = 0.0;
		if (hhigh > 0.0)
			slope_high = (Y(i+2) - yhigh)/hhigh;
		if (slope*slope_high <= 0.0)
		{
			// Set derivative as zero
			yphigh[i] = 0.0;
		}
		else
		{
			yphigh[i] = ((slope*hhigh) + (slope_high*h))/(h + hhigh);
			if (yphigh[i] >= 0.0)
			{
				yphigh[i] = (std::min)(yphigh[i], 2.0*(std::min)(slope, slope_high));
			}
			else
			{
				yphigh[i] = (std::max)(yphigh[i], 2.0*(std::max)(slope, slope_high));
			}
		}
	}

  a[i] = (yplow[i] + yphigh[i] - (2.0*slope))/(h*h);
	b[i] = ((3.0*slope) + (-2.0*yplow[i]) - yphigh[i])/h;



  }



}

template<class Real>
Real
MonotonicInterpolator<Real>::operator()( Real x ) const
{
  BASE::checkData();

  // don't extrapolate at all
  if( x < this->xData[0] || x > this->xData[this->xData.size()-1] )
    return 0;

  const VectorType &X = *(this->xView);
  const VectorType &Y = *(this->yView);

  // find the index that is just to the left of the x
  // this will correspond to the "interval index"
  //int i = Utils::index__first_ge( x, X, X.size(), 1);
  auto rng = std::make_pair( X.data()+1, X.data()+X.size() );
  int i = boost::lower_bound( rng, x ) - X.data();
  i--; // we need the interval index, not the right point index.

	// Deal with the degenerate case of xval = xlow = xhigh
	if (X(i+1) <= X(i))
		return 0.5*(Y(i) + Y(i+1));

	Real xm = x - X(i);
	Real xm2 = xm*xm;
	Real f = (a[i]*xm*xm2) + (b[i]*xm2) + (yplow[i]*xm) + Y(i);

  
  return f;
}

}

#endif // include protector
