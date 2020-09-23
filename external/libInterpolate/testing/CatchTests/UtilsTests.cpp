#include "catch.hpp"

#include <libInterpolate/Utils/Indexing.hpp>
#include <libInterpolate/Utils/Concepts.hpp>
#include <Eigen/Dense>
#include <boost/range/adaptor/strided.hpp>
#include <boost/range/algorithm/upper_bound.hpp>
#include <boost/range/algorithm/lower_bound.hpp>
using namespace Eigen;


namespace Utils {
// these functions were originally used to do index searches, but have since been
// replaced. they are defined here so that these legacy unit tests can run.

/**
 * @brief find index of the first value in a sorted array that is greater than a given value.
 *
 * @param val the value to search for.
 * @param vals the array to be searched.
 * @param N the size of the array.
 * @param i the initial index to start searching from.
 */
template<class Val, class Indexable>
int index_first_gt( Val val, const Indexable& vals, size_t N, int i = 0, size_t stride = 1 )
{
  if(i < 0) // don't let the user do harm...
    i = 0;
  // to find first element that is greater
  // we have to keep looking until element is not less
  while( i < (int)N && vals[i] <= val )
  {
    i += stride;
  }

  return i;
}

/**
 * @brief Find index of last element in a sorted array with value less than a given value.
 *
 *
 * @param val the value to search for.
 * @param vals the array to be searched.
 * @param N the size of the array.
 * @param i the initial index to start searching from.
 */
template<class Val, class Indexable>
int index_last_lt( Val val, const Indexable& vals, size_t N, int i = -1, size_t stride = 1 )
{
  // optimization: if val is larger than largest value, just return N-1
  if( vals[N-1] < val )
    return N-1;

  // N is unsigned, so -1 < N will always be false
  // to find the last element that is less than val,
  // we keep looking until the next element is not less than val.
  while( (i < 0 || i < (int)(N-1)) && vals[i+1] < val )
  {
    i += stride;
  }

  return i;
}


template<class Val, class Indexable>
int index_first_ge( Val val, const Indexable& vals, size_t N, int i = 0, size_t stride = 1 )
{
  if(i < 0)
    i = 0;
  while( i < (int)N && vals[i] < val )
  {
    i += stride;
  }

  return i;
}


}


TEST_CASE( "Index Search Utilities", "[utils]" ) {

  std::vector<int> x;
  x.push_back(1);
  x.push_back(2);
  x.push_back(3);


  // original
  CHECK( Utils::index_first_gt( 0  , x, 3 ) == 0 );
  CHECK( Utils::index_first_gt( 0.5, x, 3 ) == 0 );
  CHECK( Utils::index_first_gt( 1  , x, 3 ) == 1 );
  CHECK( Utils::index_first_gt( 1.5, x, 3 ) == 1 );
  CHECK( Utils::index_first_gt( 2  , x, 3 ) == 2 );
  CHECK( Utils::index_first_gt( 2.5, x, 3 ) == 2 );
  CHECK( Utils::index_first_gt( 3  , x, 3 ) == 3 );
  CHECK( Utils::index_first_gt( 3.5, x, 3 ) == 3 );

  CHECK( Utils::index_first_gt( 0  , x, 3, 1 ) == 1 );
  CHECK( Utils::index_first_gt( 0.5, x, 3, 1 ) == 1 );
  CHECK( Utils::index_first_gt( 1  , x, 3, 1 ) == 1 );
  CHECK( Utils::index_first_gt( 1.5, x, 3, 1 ) == 1 );
  CHECK( Utils::index_first_gt( 2  , x, 3, 1 ) == 2 );
  CHECK( Utils::index_first_gt( 2.5, x, 3, 1 ) == 2 );
  CHECK( Utils::index_first_gt( 3  , x, 3, 1 ) == 3 );
  CHECK( Utils::index_first_gt( 3.5, x, 3, 1 ) == 3 );

  // std-based replacement
  CHECK( std::upper_bound( x.data(), x.data()+3, 0  )-x.data() == 0 );
  CHECK( std::upper_bound( x.data(), x.data()+3, 0.5)-x.data() == 0 );
  CHECK( std::upper_bound( x.data(), x.data()+3, 1  )-x.data() == 1 );
  CHECK( std::upper_bound( x.data(), x.data()+3, 1.5)-x.data() == 1 );
  CHECK( std::upper_bound( x.data(), x.data()+3, 2  )-x.data() == 2 );
  CHECK( std::upper_bound( x.data(), x.data()+3, 2.5)-x.data() == 2 );
  CHECK( std::upper_bound( x.data(), x.data()+3, 3  )-x.data() == 3 );
  CHECK( std::upper_bound( x.data(), x.data()+3, 3.5)-x.data() == 3 );

  CHECK( std::upper_bound( x.data()+1, x.data()+3, 0  )-x.data() == 1 );
  CHECK( std::upper_bound( x.data()+1, x.data()+3, 0.5)-x.data() == 1 );
  CHECK( std::upper_bound( x.data()+1, x.data()+3, 1  )-x.data() == 1 );
  CHECK( std::upper_bound( x.data()+1, x.data()+3, 1.5)-x.data() == 1 );
  CHECK( std::upper_bound( x.data()+1, x.data()+3, 2  )-x.data() == 2 );
  CHECK( std::upper_bound( x.data()+1, x.data()+3, 2.5)-x.data() == 2 );
  CHECK( std::upper_bound( x.data()+1, x.data()+3, 3  )-x.data() == 3 );
  CHECK( std::upper_bound( x.data()+1, x.data()+3, 3.5)-x.data() == 3 );

  // boost-based replacement
  auto rng = std::make_pair( x.data(), x.data()+3 );
  CHECK( boost::upper_bound( rng, 0  )-x.data() == 0 );
  CHECK( boost::upper_bound( rng, 0.5)-x.data() == 0 );
  CHECK( boost::upper_bound( rng, 1  )-x.data() == 1 );
  CHECK( boost::upper_bound( rng, 1.5)-x.data() == 1 );
  CHECK( boost::upper_bound( rng, 2  )-x.data() == 2 );
  CHECK( boost::upper_bound( rng, 2.5)-x.data() == 2 );
  CHECK( boost::upper_bound( rng, 3  )-x.data() == 3 );
  CHECK( boost::upper_bound( rng, 3.5)-x.data() == 3 );

  rng = std::make_pair( x.data()+1, x.data()+3 );
  CHECK( boost::upper_bound( rng, 0  )-x.data() == 1 );
  CHECK( boost::upper_bound( rng, 0.5)-x.data() == 1 );
  CHECK( boost::upper_bound( rng, 1  )-x.data() == 1 );
  CHECK( boost::upper_bound( rng, 1.5)-x.data() == 1 );
  CHECK( boost::upper_bound( rng, 2  )-x.data() == 2 );
  CHECK( boost::upper_bound( rng, 2.5)-x.data() == 2 );
  CHECK( boost::upper_bound( rng, 3  )-x.data() == 3 );
  CHECK( boost::upper_bound( rng, 3.5)-x.data() == 3 );



  // original
  CHECK( Utils::index_last_lt( 0  , x, 3 ) == -1 );
  CHECK( Utils::index_last_lt( 0.5, x, 3 ) == -1 );
  CHECK( Utils::index_last_lt( 1  , x, 3 ) == -1 );
  CHECK( Utils::index_last_lt( 1.5, x, 3 ) ==  0 );
  CHECK( Utils::index_last_lt( 2  , x, 3 ) ==  0 );
  CHECK( Utils::index_last_lt( 2.5, x, 3 ) ==  1 );
  CHECK( Utils::index_last_lt( 3  , x, 3 ) ==  1 );
  CHECK( Utils::index_last_lt( 3.5, x, 3 ) ==  2 );

  CHECK( Utils::index_last_lt( 0  , x, 3, 1 ) == 1 );
  CHECK( Utils::index_last_lt( 0.5, x, 3, 1 ) == 1 );
  CHECK( Utils::index_last_lt( 1  , x, 3, 1 ) == 1 );
  CHECK( Utils::index_last_lt( 1.5, x, 3, 1 ) == 1 );
  CHECK( Utils::index_last_lt( 2  , x, 3, 1 ) == 1 );
  CHECK( Utils::index_last_lt( 2.5, x, 3, 1 ) == 1 );
  CHECK( Utils::index_last_lt( 3  , x, 3, 1 ) == 1 );
  CHECK( Utils::index_last_lt( 3.5, x, 3, 1 ) == 2 );

  // std-based replacement
  CHECK( std::lower_bound( x.data(), x.data()+3, 0   )-x.data()-1 == -1 );
  CHECK( std::lower_bound( x.data(), x.data()+3, 0.5 )-x.data()-1 == -1 );
  CHECK( std::lower_bound( x.data(), x.data()+3, 1   )-x.data()-1 == -1 );
  CHECK( std::lower_bound( x.data(), x.data()+3, 1.5 )-x.data()-1 ==  0 );
  CHECK( std::lower_bound( x.data(), x.data()+3, 2   )-x.data()-1 ==  0 );
  CHECK( std::lower_bound( x.data(), x.data()+3, 2.5 )-x.data()-1 ==  1 );
  CHECK( std::lower_bound( x.data(), x.data()+3, 3   )-x.data()-1 ==  1 );
  CHECK( std::lower_bound( x.data(), x.data()+3, 3.5 )-x.data()-1 ==  2 );

  CHECK( std::lower_bound( x.data()+2, x.data()+3, 0   )-x.data()-1 == 1 );
  CHECK( std::lower_bound( x.data()+2, x.data()+3, 0.5 )-x.data()-1 == 1 );
  CHECK( std::lower_bound( x.data()+2, x.data()+3, 1   )-x.data()-1 == 1 );
  CHECK( std::lower_bound( x.data()+2, x.data()+3, 1.5 )-x.data()-1 == 1 );
  CHECK( std::lower_bound( x.data()+2, x.data()+3, 2   )-x.data()-1 == 1 );
  CHECK( std::lower_bound( x.data()+2, x.data()+3, 2.5 )-x.data()-1 == 1 );
  CHECK( std::lower_bound( x.data()+2, x.data()+3, 3   )-x.data()-1 == 1 );
  CHECK( std::lower_bound( x.data()+2, x.data()+3, 3.5 )-x.data()-1 == 2 );

  // boost-based replacement
  rng = std::make_pair( x.data(), x.data()+3 );
  CHECK( boost::lower_bound( rng, 0   )-x.data()-1 == -1 );
  CHECK( boost::lower_bound( rng, 0.5 )-x.data()-1 == -1 );
  CHECK( boost::lower_bound( rng, 1   )-x.data()-1 == -1 );
  CHECK( boost::lower_bound( rng, 1.5 )-x.data()-1 ==  0 );
  CHECK( boost::lower_bound( rng, 2   )-x.data()-1 ==  0 );
  CHECK( boost::lower_bound( rng, 2.5 )-x.data()-1 ==  1 );
  CHECK( boost::lower_bound( rng, 3   )-x.data()-1 ==  1 );
  CHECK( boost::lower_bound( rng, 3.5 )-x.data()-1 ==  2 );

  rng = std::make_pair( x.data()+2, x.data()+3 );
  CHECK( boost::lower_bound( rng, 0   )-x.data()-1 == 1 );
  CHECK( boost::lower_bound( rng, 0.5 )-x.data()-1 == 1 );
  CHECK( boost::lower_bound( rng, 1   )-x.data()-1 == 1 );
  CHECK( boost::lower_bound( rng, 1.5 )-x.data()-1 == 1 );
  CHECK( boost::lower_bound( rng, 2   )-x.data()-1 == 1 );
  CHECK( boost::lower_bound( rng, 2.5 )-x.data()-1 == 1 );
  CHECK( boost::lower_bound( rng, 3   )-x.data()-1 == 1 );
  CHECK( boost::lower_bound( rng, 3.5 )-x.data()-1 == 2 );



  // original
  CHECK( Utils::index_first_ge( 0  , x, 3 ) == 0 );
  CHECK( Utils::index_first_ge( 0.5, x, 3 ) == 0 );
  CHECK( Utils::index_first_ge( 1  , x, 3 ) == 0 );
  CHECK( Utils::index_first_ge( 1.5, x, 3 ) == 1 );
  CHECK( Utils::index_first_ge( 2  , x, 3 ) == 1 );
  CHECK( Utils::index_first_ge( 2.5, x, 3 ) == 2 );
  CHECK( Utils::index_first_ge( 3  , x, 3 ) == 2 );
  CHECK( Utils::index_first_ge( 3.5, x, 3 ) == 3 );

  CHECK( Utils::index_first_ge( 0  , x, 3, 1 ) == 1 );
  CHECK( Utils::index_first_ge( 0.5, x, 3, 1 ) == 1 );
  CHECK( Utils::index_first_ge( 1  , x, 3, 1 ) == 1 );
  CHECK( Utils::index_first_ge( 1.5, x, 3, 1 ) == 1 );
  CHECK( Utils::index_first_ge( 2  , x, 3, 1 ) == 1 );
  CHECK( Utils::index_first_ge( 2.5, x, 3, 1 ) == 2 );
  CHECK( Utils::index_first_ge( 3  , x, 3, 1 ) == 2 );
  CHECK( Utils::index_first_ge( 3.5, x, 3, 1 ) == 3 );

  // std-based replacement
  CHECK( std::lower_bound( x.data(), x.data()+3,  0   )-x.data() == 0 );
  CHECK( std::lower_bound( x.data(), x.data()+3,  0.5 )-x.data() == 0 );
  CHECK( std::lower_bound( x.data(), x.data()+3,  1   )-x.data() == 0 );
  CHECK( std::lower_bound( x.data(), x.data()+3,  1.5 )-x.data() == 1 );
  CHECK( std::lower_bound( x.data(), x.data()+3,  2   )-x.data() == 1 );
  CHECK( std::lower_bound( x.data(), x.data()+3,  2.5 )-x.data() == 2 );
  CHECK( std::lower_bound( x.data(), x.data()+3,  3   )-x.data() == 2 );
  CHECK( std::lower_bound( x.data(), x.data()+3,  3.5 )-x.data() == 3 );

  CHECK( std::lower_bound( x.data()+1, x.data()+3,  0  )-x.data() == 1 );
  CHECK( std::lower_bound( x.data()+1, x.data()+3,  0.5)-x.data() == 1 );
  CHECK( std::lower_bound( x.data()+1, x.data()+3,  1  )-x.data() == 1 );
  CHECK( std::lower_bound( x.data()+1, x.data()+3,  1.5)-x.data() == 1 );
  CHECK( std::lower_bound( x.data()+1, x.data()+3,  2  )-x.data() == 1 );
  CHECK( std::lower_bound( x.data()+1, x.data()+3,  2.5)-x.data() == 2 );
  CHECK( std::lower_bound( x.data()+1, x.data()+3,  3  )-x.data() == 2 );
  CHECK( std::lower_bound( x.data()+1, x.data()+3,  3.5)-x.data() == 3 );

  // boost-based replacement
  rng = std::make_pair( x.data(), x.data()+3 );
  CHECK( boost::lower_bound( rng, 0   )-x.data() == 0 );
  CHECK( boost::lower_bound( rng, 0.5 )-x.data() == 0 );
  CHECK( boost::lower_bound( rng, 1   )-x.data() == 0 );
  CHECK( boost::lower_bound( rng, 1.5 )-x.data() == 1 );
  CHECK( boost::lower_bound( rng, 2   )-x.data() == 1 );
  CHECK( boost::lower_bound( rng, 2.5 )-x.data() == 2 );
  CHECK( boost::lower_bound( rng, 3   )-x.data() == 2 );
  CHECK( boost::lower_bound( rng, 3.5 )-x.data() == 3 );

  rng = std::make_pair( x.data()+1, x.data()+3 );
  CHECK( boost::lower_bound( rng, 0  )-x.data() == 1 );
  CHECK( boost::lower_bound( rng, 0.5)-x.data() == 1 );
  CHECK( boost::lower_bound( rng, 1  )-x.data() == 1 );
  CHECK( boost::lower_bound( rng, 1.5)-x.data() == 1 );
  CHECK( boost::lower_bound( rng, 2  )-x.data() == 1 );
  CHECK( boost::lower_bound( rng, 2.5)-x.data() == 2 );
  CHECK( boost::lower_bound( rng, 3  )-x.data() == 2 );
  CHECK( boost::lower_bound( rng, 3.5)-x.data() == 3 );





}





// an iterator that wraps a pointer with a stride.
// this can be used with std::upper_bound to search through
// strided data.
template <typename T>
struct StridedIterator : public std::iterator<std::forward_iterator_tag, T> {
    StridedIterator(T *t, unsigned stride) : ptr(t), stride(stride) {}
    bool operator==(const StridedIterator<T> &other) const
    {
        return ptr == other.ptr;
    }
    bool operator!=(const StridedIterator<T> &other) const
    {
        return ptr != other.ptr;
    }
    T *operator->() const { return ptr; }
    T &operator*() const { return *ptr; }
    StridedIterator &operator++()
    {
        ptr += stride;
        return *this;
    }
    StridedIterator operator++(int)
    {
        auto ret = *this;
        ++*this;
        return ret;
    }
    StridedIterator operator+(int amt) const
    {
        auto ret = StridedIterator(ptr + amt * stride, stride);
        return ret;
    }
    int operator-(const StridedIterator &right) const
    {
        auto ret = (ptr - right.ptr)/stride;
        return ret;
    }

  private:
    T *ptr;
    unsigned stride;
};


TEST_CASE( "Index Search with Eigen Matricies", "[utils]" ) {

  // matrix
  Matrix<double,Dynamic,1> m(6);
  m(0) = 1;
  m(1) = 2;
  m(2) = 3;
  m(3) = 4;
  m(4) = 5;
  m(5) = 6;

  // strided view of the matrix
  Map<Matrix<double,Dynamic,1>,Unaligned,InnerStride<Dynamic>> v( m.data(), 3, InnerStride<Dynamic>(2) );

  // original
  CHECK( Utils::index_first_gt( 0.0  , m, m.size() ) == 0 );
  CHECK( Utils::index_first_gt( 0.5  , m, m.size() ) == 0 );
  CHECK( Utils::index_first_gt( 1.0  , m, m.size() ) == 1 );
  CHECK( Utils::index_first_gt( 1.5  , m, m.size() ) == 1 );
  CHECK( Utils::index_first_gt( 2.0  , m, m.size() ) == 2 );
  CHECK( Utils::index_first_gt( 2.5  , m, m.size() ) == 2 );
  CHECK( Utils::index_first_gt( 3.0  , m, m.size() ) == 3 );
  CHECK( Utils::index_first_gt( 3.5  , m, m.size() ) == 3 );
  CHECK( Utils::index_first_gt( 4.0  , m, m.size() ) == 4 );
  CHECK( Utils::index_first_gt( 4.5  , m, m.size() ) == 4 );
  CHECK( Utils::index_first_gt( 5.0  , m, m.size() ) == 5 );
  CHECK( Utils::index_first_gt( 5.5  , m, m.size() ) == 5 );
  CHECK( Utils::index_first_gt( 6.0  , m, m.size() ) == 6 );
  CHECK( Utils::index_first_gt( 6.5  , m, m.size() ) == 6 );
  CHECK( Utils::index_first_gt( 7.0  , m, m.size() ) == 6 );

  CHECK( Utils::index_first_gt( 0.0  , v, v.size() ) == 0 );
  CHECK( Utils::index_first_gt( 0.5  , v, v.size() ) == 0 );
  CHECK( Utils::index_first_gt( 1.0  , v, v.size() ) == 1 );
  CHECK( Utils::index_first_gt( 1.5  , v, v.size() ) == 1 );
  CHECK( Utils::index_first_gt( 2.0  , v, v.size() ) == 1 );
  CHECK( Utils::index_first_gt( 2.5  , v, v.size() ) == 1 );
  CHECK( Utils::index_first_gt( 3.0  , v, v.size() ) == 2 );
  CHECK( Utils::index_first_gt( 3.5  , v, v.size() ) == 2 );
  CHECK( Utils::index_first_gt( 4.0  , v, v.size() ) == 2 );
  CHECK( Utils::index_first_gt( 4.5  , v, v.size() ) == 2 );
  CHECK( Utils::index_first_gt( 5.0  , v, v.size() ) == 3 );
  CHECK( Utils::index_first_gt( 5.5  , v, v.size() ) == 3 );
  CHECK( Utils::index_first_gt( 6.0  , v, v.size() ) == 3 );
  CHECK( Utils::index_first_gt( 6.5  , v, v.size() ) == 3 );
  CHECK( Utils::index_first_gt( 7.0  , v, v.size() ) == 3 );

  // replacement
  CHECK( std::upper_bound( m.data(), m.data()+m.size(), 0.0 )-m.data() == 0 );
  CHECK( std::upper_bound( m.data(), m.data()+m.size(), 0.5 )-m.data() == 0 );
  CHECK( std::upper_bound( m.data(), m.data()+m.size(), 1.0 )-m.data() == 1 );
  CHECK( std::upper_bound( m.data(), m.data()+m.size(), 1.5 )-m.data() == 1 );
  CHECK( std::upper_bound( m.data(), m.data()+m.size(), 2.0 )-m.data() == 2 );
  CHECK( std::upper_bound( m.data(), m.data()+m.size(), 2.5 )-m.data() == 2 );
  CHECK( std::upper_bound( m.data(), m.data()+m.size(), 3.0 )-m.data() == 3 );
  CHECK( std::upper_bound( m.data(), m.data()+m.size(), 3.5 )-m.data() == 3 );
  CHECK( std::upper_bound( m.data(), m.data()+m.size(), 4.0 )-m.data() == 4 );
  CHECK( std::upper_bound( m.data(), m.data()+m.size(), 4.5 )-m.data() == 4 );
  CHECK( std::upper_bound( m.data(), m.data()+m.size(), 5.0 )-m.data() == 5 );
  CHECK( std::upper_bound( m.data(), m.data()+m.size(), 5.5 )-m.data() == 5 );
  CHECK( std::upper_bound( m.data(), m.data()+m.size(), 6.0 )-m.data() == 6 );
  CHECK( std::upper_bound( m.data(), m.data()+m.size(), 6.5 )-m.data() == 6 );
  CHECK( std::upper_bound( m.data(), m.data()+m.size(), 7.0 )-m.data() == 6 );

  // WRONG
  CHECK( std::upper_bound( v.data(), v.data()+v.size(), 0.0 )-v.data() == 0 );
  CHECK( std::upper_bound( v.data(), v.data()+v.size(), 0.5 )-v.data() == 0 );
  CHECK( std::upper_bound( v.data(), v.data()+v.size(), 1.0 )-v.data() == 1 );
  CHECK( std::upper_bound( v.data(), v.data()+v.size(), 1.5 )-v.data() == 1 );
  CHECK( std::upper_bound( v.data(), v.data()+v.size(), 2.0 )-v.data() == 2 );
  CHECK( std::upper_bound( v.data(), v.data()+v.size(), 2.5 )-v.data() == 2 );
  CHECK( std::upper_bound( v.data(), v.data()+v.size(), 3.0 )-v.data() == 3 );
  CHECK( std::upper_bound( v.data(), v.data()+v.size(), 3.5 )-v.data() == 3 );
  CHECK( std::upper_bound( v.data(), v.data()+v.size(), 4.0 )-v.data() == 3 );
  CHECK( std::upper_bound( v.data(), v.data()+v.size(), 4.5 )-v.data() == 3 );
  CHECK( std::upper_bound( v.data(), v.data()+v.size(), 5.0 )-v.data() == 3 );
  CHECK( std::upper_bound( v.data(), v.data()+v.size(), 5.5 )-v.data() == 3 );
  CHECK( std::upper_bound( v.data(), v.data()+v.size(), 6.0 )-v.data() == 3 );
  CHECK( std::upper_bound( v.data(), v.data()+v.size(), 6.5 )-v.data() == 3 );
  CHECK( std::upper_bound( v.data(), v.data()+v.size(), 7.0 )-v.data() == 3 );

  // we need an iterator that includes the stride.
  StridedIterator<double> begin(v.data(),2);
  StridedIterator<double> end = begin + v.size();
  CHECK( std::upper_bound( begin, end, 0.0 )-begin == 0 );
  CHECK( std::upper_bound( begin, end, 0.5 )-begin == 0 );
  CHECK( std::upper_bound( begin, end, 1.0 )-begin == 1 );
  CHECK( std::upper_bound( begin, end, 1.5 )-begin == 1 );
  CHECK( std::upper_bound( begin, end, 2.0 )-begin == 1 );
  CHECK( std::upper_bound( begin, end, 2.5 )-begin == 1 );
  CHECK( std::upper_bound( begin, end, 3.0 )-begin == 2 );
  CHECK( std::upper_bound( begin, end, 3.5 )-begin == 2 );
  CHECK( std::upper_bound( begin, end, 4.0 )-begin == 2 );
  CHECK( std::upper_bound( begin, end, 4.5 )-begin == 2 );
  CHECK( std::upper_bound( begin, end, 5.0 )-begin == 3 );
  CHECK( std::upper_bound( begin, end, 5.5 )-begin == 3 );
  CHECK( std::upper_bound( begin, end, 6.0 )-begin == 3 );
  CHECK( std::upper_bound( begin, end, 6.5 )-begin == 3 );
  CHECK( std::upper_bound( begin, end, 7.0 )-begin == 3 );
  
}

#include <vector>
#include <libInterpolate/Interpolators/_1D/InterpolatorBase.hpp>
#include <libInterpolate/Interpolators/_1D/CubicSplineInterpolator.hpp>
#include <type_traits>
TEST_CASE("RealTypeOf tests")
{
  CHECK( std::is_same<_1D::RealTypeOf<double>::type, double>::value );
  CHECK( std::is_same<_1D::RealTypeOf<float>::type, double>::value );
  CHECK( std::is_same<_1D::RealTypeOf<int>::type, double>::value );
  CHECK( std::is_same<_1D::RealTypeOf<_1D::CubicSplineInterpolator<double>>::type, double>::value );
  CHECK( std::is_same<_1D::RealTypeOf<_1D::CubicSplineInterpolator<float>>::type, float>::value );
  CHECK( std::is_same<_1D::RealTypeOf<_1D::CubicSplineInterpolator<int>>::type, int>::value );
}

