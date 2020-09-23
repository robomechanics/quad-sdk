/* @file
 * This file contains some benchmarks to compare the index search
 * algorithm that was originally used by this library with std::upper_bound and boost::upper_bound.
 * The original algorithm was just a straight serial search.
 *
 * results seem to indicate the, on average, boost will perform better.
 */
#include <celero/Celero.h>

#include<algorithm>
#include<cassert>
#include<Eigen/Dense>
#include<boost/range/adaptor/strided.hpp>
#include<boost/range/algorithm/upper_bound.hpp>

CELERO_MAIN


// A naive, unsophisticated algorithm for finding the index of an element in an array
// that is just to the "right" of a given value.
// The interpolators use this to figure out what points in the data array need to be used
// for interpolating to a given value.
template<class Val, class Indexable>
int index_first_gt( Val val, const Indexable& vals, int64_t N, int64_t i = 0, int stride = 1 )
{
  if(i < 0) // don't let the user do harm...
    i = 0;
  // to find first element that is greater
  // we have to keep looking until element is not less
  while( i < N && vals[i] <= val )
  {
    i += stride;
  }

  return i/stride;
}


// fixture to setup some data array.
class Fixture : public celero::TestFixture
{
  public:
    typedef Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> Matrix;
    typedef Eigen::Matrix<double,Eigen::Dynamic,1> ColumnVector;
    typedef Eigen::Matrix<double,1,Eigen::Dynamic> RowVector;

    typedef Eigen::Map<ColumnVector> ColumnVectorMap;
    typedef Eigen::Map<ColumnVector,Eigen::Unaligned,Eigen::InnerStride<Eigen::Dynamic>> StridedColumnVectorMap;

    Fixture()
    {
    }

    virtual std::vector<std::pair<int64_t, uint64_t>> getExperimentValues() const override
    {
      std::vector<std::pair<int64_t, uint64_t>> problemSpace;
      const int runs = 10;
      for( int i = 0; i < runs; i++)
      {
        problemSpace.push_back(std::make_pair(int64_t(pow(2,i+1)),uint64_t(0)));
      }

      return problemSpace;
    }
    virtual void setUp(int64_t N)
    {
      this->size = N;
      this->data = ColumnVector(N);
      for(int64_t i = 0; i < this->size; i++)
        this->data(i) = i;

      data_map.reset( new ColumnVectorMap( this->data.data(), this->size ) );
      strided_data_map.reset( new StridedColumnVectorMap( this->data.data(), this->size/2, Eigen::InnerStride<Eigen::Dynamic>(2) ) );
    }

    int64_t size;
    ColumnVector data;
    std::shared_ptr< ColumnVectorMap > data_map;
    std::shared_ptr< StridedColumnVectorMap > strided_data_map;


};


// ____            _       _   ____        _                _         ____                  ____       _       _            
/// ___|  ___ _ __(_) __ _| | |  _ \  __ _| |_ __ _  __   _(_) __ _  |  _ \ __ ___      __ |  _ \ ___ (_)_ __ | |_ ___ _ __ 
//\___ \ / _ \ '__| |/ _` | | | | | |/ _` | __/ _` | \ \ / / |/ _` | | |_) / _` \ \ /\ / / | |_) / _ \| | '_ \| __/ _ \ '__|
// ___) |  __/ |  | | (_| | | | |_| | (_| | || (_| |  \ V /| | (_| | |  _ < (_| |\ V  V /  |  __/ (_) | | | | | ||  __/ |   
//|____/ \___|_|  |_|\__,_|_| |____/ \__,_|\__\__,_|   \_/ |_|\__,_| |_| \_\__,_| \_/\_/   |_|   \___/|_|_| |_|\__\___|_|   
                                                                                                                          
// these benchmarks compare our "manual" method to the std::upper_bound function.
// three different tests are ran, one that searches for an elment at the beginning of the array,
// one that searches for an element in the middle of an array, and one that searches for an element at the end.


BASELINE_F(SerialDataPtr_Begin, upper_bound_raw_pointer, Fixture, 100, 10000)
{
  int i = std::upper_bound( this->data.data(), this->data.data()+this->size, 0 ) - this->data.data();
  assert(i == 1);
}

BENCHMARK_F(SerialDataPtr_Begin, manual_raw_pointer, Fixture, 100, 10000 )
{
  int i = index_first_gt( 0, this->data.data(), this->size );
  assert(i == 1);
}


BASELINE_F(SerialDataPtr_Middle, upper_bound_raw_pointer, Fixture, 100, 10000)
{
  int i = std::upper_bound( this->data.data(), this->data.data()+this->size, this->size/2 ) - this->data.data();
  assert(i == this->size/2+1);
}

BENCHMARK_F(SerialDataPtr_Middle, manual_raw_pointer, Fixture, 100, 10000 )
{
  int i = index_first_gt( this->size/2, this->data.data(), this->size );
  assert(i == this->size/2+1);
}


BASELINE_F(SerialDataPtr_End, upper_bound_raw_pointer, Fixture, 100, 10000)
{
  int i = std::upper_bound( this->data.data(), this->data.data()+this->size, this->size-2 ) - this->data.data();
  assert(i == this->size-1);
}

BENCHMARK_F(SerialDataPtr_End, manual_raw_pointer, Fixture, 100, 10000 )
{
  int i = index_first_gt( this->size-2, this->data.data(), this->size );
  assert(i == this->size-1);
}





// ____            _       _   ____        _                _        __     __        _             
/// ___|  ___ _ __(_) __ _| | |  _ \  __ _| |_ __ _  __   _(_) __ _  \ \   / /__  ___| |_ ___  _ __ 
//\___ \ / _ \ '__| |/ _` | | | | | |/ _` | __/ _` | \ \ / / |/ _` |  \ \ / / _ \/ __| __/ _ \| '__|
// ___) |  __/ |  | | (_| | | | |_| | (_| | || (_| |  \ V /| | (_| |   \ V /  __/ (__| || (_) | |   
//|____/ \___|_|  |_|\__,_|_| |____/ \__,_|\__\__,_|   \_/ |_|\__,_|    \_/ \___|\___|\__\___/|_|   
                                                                                                  
// these benchmarks test the same thing as the set above, but our "manual" method accesses elements through
// an Eigen matrix instead of the raw pointer. it also compares a boost.range based method.



BASELINE_F(SerialVec_Begin, upper_bound_raw_pointer, Fixture, 100, 10000)
{
  int i = std::upper_bound( this->data.data(), this->data.data()+this->size, 0 ) - this->data.data();
  assert(i == 1);
}

BENCHMARK_F(SerialVec_Begin, manual_eigen_matrix, Fixture, 100, 10000 )
{
  int i = index_first_gt( 0, this->data, this->size );
  assert(i == 1);
}

BENCHMARK_F(SerialVec_Begin, boost_range, Fixture, 100, 10000 )
{
  auto rng = std::make_pair(  this->data.data(),
                              this->data.data()+this->data.size() );
  int i = boost::upper_bound( rng, 0) - this->data.data();
  assert(i == 1);
}


BASELINE_F(SerialVec_Middle, upper_bound_raw_pointer, Fixture, 100, 10000)
{
  int i = std::upper_bound( this->data.data(), this->data.data()+this->size, this->size/2 ) - this->data.data();
  assert(i == this->size/2+1);
}

BENCHMARK_F(SerialVec_Middle, manual_eigen_matrix, Fixture, 100, 10000 )
{
  int i = index_first_gt( this->size/2, this->data, this->size );
  assert(i == this->size/2+1);
}

BENCHMARK_F(SerialVec_Middle, boost_range, Fixture, 100, 10000 )
{
  auto rng = std::make_pair(  this->data.data(),
                              this->data.data()+this->data.size() );
  int i = boost::upper_bound( rng, this->size/2) - this->data.data();
  assert(i == this->size/2+1);
}



BASELINE_F(SerialVec_End, upper_bound_raw_pointer, Fixture, 100, 10000)
{
  int i = std::upper_bound( this->data.data(), this->data.data()+this->size, this->size-2 ) - this->data.data();
  assert(i == this->size-1);
}

BENCHMARK_F(SerialVec_End, manual_eigen_matrix, Fixture, 100, 10000 )
{
  int i = index_first_gt( this->size-2, this->data, this->size );
  assert(i == this->size-1);
}

BENCHMARK_F(SerialVec_End, boost_range, Fixture, 100, 10000 )
{
  auto rng = std::make_pair(  this->data.data(),
                              this->data.data()+this->data.size() );
  int i = boost::upper_bound( rng, this->size-2) - this->data.data();
  assert(i == this->size-1);
}




// ____  _        _     _          _   ____        _                _        __     __        _             
/// ___|| |_ _ __(_) __| | ___  __| | |  _ \  __ _| |_ __ _  __   _(_) __ _  \ \   / /__  ___| |_ ___  _ __ 
//\___ \| __| '__| |/ _` |/ _ \/ _` | | | | |/ _` | __/ _` | \ \ / / |/ _` |  \ \ / / _ \/ __| __/ _ \| '__|
// ___) | |_| |  | | (_| |  __/ (_| | | |_| | (_| | || (_| |  \ V /| | (_| |   \ V /  __/ (__| || (_) | |   
//|____/ \__|_|  |_|\__,_|\___|\__,_| |____/ \__,_|\__\__,_|   \_/ |_|\__,_|    \_/ \___|\___|\__\___/|_|   
                                                                                                          
// these benchmarks compare our manual method and a boost.range method to find the index of an element
// in an array with "strided" data to std::upper_bound. A custom iterator that skips elements
// has to be used with std::upper_bound. the boost.range library provides a strided range adaptor.
// our "manual" method does not have to worry about the stride, because it accesses the elements with
// an eigen map, which handles the striding.


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
    unsigned stride;
    T *ptr;
};



BASELINE_F(StridedVec_Begin, upper_bound_strided_iterator, Fixture, 100, 10000)
{
  StridedIterator<double> begin( this->strided_data_map->data(), this->strided_data_map->innerStride() );
  StridedIterator<double> end = begin + this->strided_data_map->size();
  int i = std::upper_bound( begin, end, 0 ) - begin;
  assert(i == 1);
}

BENCHMARK_F(StridedVec_Begin, manual_eigen_map, Fixture, 100, 10000 )
{
  int i = index_first_gt( 0, *(this->strided_data_map), this->strided_data_map->size() );
  assert(i == 1);
}

BENCHMARK_F(StridedVec_Begin, boost_strided_range, Fixture, 100, 10000 )
{
  auto rng = std::make_pair(  this->strided_data_map->data(),
                              this->strided_data_map->data()+this->strided_data_map->size()*this->strided_data_map->innerStride()
                          ) | boost::adaptors::strided(this->strided_data_map->innerStride());
  int i = boost::upper_bound( rng, 0 ) - boost::begin(rng);
  assert(i == 1);
}



BASELINE_F(StridedVec_Middle, upper_bound_strided_iterator, Fixture, 100, 10000)
{
  StridedIterator<double> begin( this->strided_data_map->data(), this->strided_data_map->innerStride() );
  StridedIterator<double> end = begin + this->strided_data_map->size();
  int i = std::upper_bound( begin, end, this->size/2 ) - begin;
  assert(i == this->size/4+1);
}

BENCHMARK_F(StridedVec_Middle, manual_eigen_map, Fixture, 100, 10000 )
{
  int i = index_first_gt( this->size/2, *(this->strided_data_map), this->strided_data_map->size() );
  assert(i == this->size/4+1);
}

BENCHMARK_F(StridedVec_Middle, boost_strided_range, Fixture, 100, 10000 )
{
  auto rng = std::make_pair(  this->strided_data_map->data(),
                              this->strided_data_map->data()+this->strided_data_map->size()*this->strided_data_map->innerStride()
                          ) | boost::adaptors::strided(this->strided_data_map->innerStride());
  int i = boost::upper_bound( rng, this->size/2 ) - boost::begin(rng);
  assert(i == this->size/4+1);
}



BASELINE_F(StridedVec_End, upper_bound_strided_iterator, Fixture, 100, 10000)
{
  StridedIterator<double> begin( this->strided_data_map->data(), this->strided_data_map->innerStride() );
  StridedIterator<double> end = begin + this->strided_data_map->size();
  int i = std::upper_bound( begin, end, this->size-2 ) - begin;
  assert(i == this->size/2);
}

BENCHMARK_F(StridedVec_End, manual_eigen_map, Fixture, 100, 10000 )
{
  int i = index_first_gt( this->size-2, *(this->strided_data_map), this->strided_data_map->size() );
  assert(i == this->size/2);
}

BENCHMARK_F(StridedVec_End, boost_strided_range, Fixture, 100, 10000 )
{
  auto rng = std::make_pair(  this->strided_data_map->data(),
                              this->strided_data_map->data()+this->strided_data_map->size()*this->strided_data_map->innerStride()
                          ) | boost::adaptors::strided(this->strided_data_map->innerStride());
  int i = boost::upper_bound( rng, this->size-2 ) - boost::begin(rng);
  assert(i == this->size/2);
}
