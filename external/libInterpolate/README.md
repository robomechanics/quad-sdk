# libInterpolate

A C++ interpolation library.

This library provides classes to perform various types of function interpolation (linear, spline, etc).

Currently implemented methods are:

- 1D Interpolation
    - linear (https://en.wikipedia.org/wiki/Linear_interpolation)
    - cubic spline (https://en.wikipedia.org/wiki/Spline_interpolation)
    - monotonic spline interpolation (http://adsabs.harvard.edu/full/1990A%26A...239..443S)
- 2D Interpolation
    - bilinear (https://en.wikipedia.org/wiki/Bilinear_interpolation)
    - bicubic (https://en.wikipedia.org/wiki/Bicubic_interpolation)
    - thin plate spline (https://en.wikipedia.org/wiki/Thin_plate_spline)

## Example

**Note: `libInterpolate` has been renamed from `libInterp`.** There were a few naming inconsistencies, so I decided
to rename the library to `libInterpolate`. I have provided CMake targets with the old names, so CMakeLists.txt
referencing the old names *should* still work. These will be dropped in version 3.

```C++
#include <libInterpolate/Interpolate.hpp>
...
vector<double> x,y;
...
fill x and y with data
...

// Use a cubic spline interpolator to interpolate the data
_1D::CubicSplineInterpolator<double> interp;
interp.setData(x,y);

double val = interp(2.0); // val contains the value of the function y(x) interpolated at x = 2.0

```

The `setData` method is a template that will accept any container that provides `size()` and `data()` methods. Here `std::vector` is used, but you
could also use `Eigen::Matrix`. There is also a low-level `setData` method that takes an integer size and two data pointers, which you can use directly.
This is actually what the `setData` method in the example is calling under the hood.

```C++
interp.setData( x.size(), x,data(), y.data() )
```


## Installing

`libInterpolate` is a header-only C++ library, so you can simply include
the headers you want/need in your source code. If you use `git subrepo`, you
can clone the source into your externals directory and use it from there.

`libInterpolate` depends on `Boost` and `Eigen3`, so you will need to include the directories
containing their header files when compiling.

`libInterpolate` also supports being installed, and will install a `*Config.cmake` file that CMake can detect.
To build and install `libInterpolate`:

```bash
$ git clone https://github.com/CD3/libInterpolate
$ cd libInterpolate
$ mkdir build
$ cd build
$ cmake ..
$ cmake --build .
$ cmake --build . --target install
```

Now you can use the `find_package` command in your CMakeLists.txt file to detect and configure `libInterpolate`.
```CMake
# find libInterpolate
find_package(libInterpolate REQUIRED)
# create your target
add_executabe( myProgram myProgram.cpp )
# add include dirs required for libInterpolate and its dependencies
target_link_libraries( myProgram libInterpolate::Interpolate )
```
Again, `boost` and `Eigen3` need to be installed.

### Conan

You can also install `libInterpolate` with the [Conan package manager](https://docs.conan.io/en/latest/),
which will automatically install its dependencies. Just add https://api.bintray.com/conan/cd3/conan-devel
to your list of remotes and search for the latest release of `libInterpolate`.



## Design

`libInterpolate` uses inheritance for code reuse and implements the "Curiously Recurring Template Pattern" (CRTP).
CRTP allows the base class to implement functions that depend on parts that need to be implemented by the
derived class. For example, `InterpolatorBase` implements a function named `setData` that reads the
interpolated data into the interpolator. This way, derived classes do not need to implement functions to load the
interpolated data. However, different interpolation methods may require some additional setup. For example, they
may compute and store some coefficients that are used during interpolation. The `setData` function therefore calls a
function named `setupInterpolator` that is implemented by the derived class, if needed. This is where the CRTP comes in.

If you write a new interpolation method and derive from the `InterpolatorBase` class, you need to pass the derived class
to the base class as a template parameter.
```C++
class MyInterpolator : _1D::InterpolatorBase<MyInterpolator>
{
...
};
```
If your interpolator needs additional setup *after* the interpolated data has been read in, then you should also
implement the function `void setupInterpolator()`.
```C++
class MyInterpolator : _1D::InterpolatorBase<MyInterpolator>
{
  double operator()(double x); // implement the interpolation
  void setupInterpolator();    // implement for additional setup
};
```

CRTP provides static polymorphism, but not runtime polymorphism, since each derived type derives from a different base class.
`libInterpolate` now provides a type-erased `AnyInterpolator` class that can be used to store any of the interpolators, which allows
runtime binding. The `AnyInterpolator` class uses Boost.TypeErasure, and is not included in the monolithic header, it must be included
seprately.


```C++
#include <libInterpolate/Interpolate.hpp>
#include <libInterpolate/AnyInterpolator.hpp>

...

_1D::AnyInterpolator<double> interp = _1D::CubicSplineInterpolator<double>();

interp.setData( x.size(), x.data(), y.data() );

...
// do some interpolation
...
// now change the interpolator
// you will need to reload the data though
interp = _1D::LinearInterpolator<double>();

interp.setData( x.size(), x.data(), y.data() );
...
// do some more inteprolation

...

```

Currently, the `AnyInterpolator` only provides one `setData` method, which is the low-level version that takes a size an two data pointers. This can
be changed by passing the desired signature as a second template arguments.
```C++
_1D::AnyInterpolator<double, void(std::vector<double>,std::vector<double>)> interp = _1D::CubicSplineInterpolator<double>();

interp.setData( x, y );
```

You can also use `std::function`, but you will have to explicitly cast the function object to the interpolator to call `setData`.

```C++
std::function<double(double)> interp = _1D::CubicSplineInterpolator<double>();
interp.target<_1D::CubicSplineInterpolator<double>>()->setData(x,y)

...

```


### Data Storage

The interpolator classes copy the interpolation data and store them internally. This makes the type much more convenient
to work with since it is completely self contained and you don't have to worry about keeping that data that you are interpolating
alive. However, it does mean that you need to be mindful of copies.


## Interpolation Methods

Each interpolation method is implemented as a class that is templated on the data type. All one-dimensional
interpolators (interpolators for a function of one variable) are in the `_1D` namespace. Two-dimensional interpolators
are in the `_2D` namespace.

All interpolators implement the same interface, so there is no difference in how each method is used.

```C++
#include <libInterpolate/Interpolate.hpp>
...
vector<double> x,y;
...
fill x and y with data
...

// To select a method, use the corresponding class
_1D::LinearInterpolator<double> interp;
_1D::CubicSplineInterpolator<double> interp;
_1D::MonotonicInterpolator<double> interp;

// set the data that will be interpolated.
interp.setData(x,y);

// interpolation is done with the operator() method.
double val = interp(2.0);
```

If you need to select the interpolation method at runtime, you can use `std::function`.

```C++
#include <libInterpolate/Interpolate.hpp>
#include <functional>
...
string method;
vector<double> x,y;
...
fill x and y with data
...

// Create a std::function to store the interpolator
std::function<double(double)> interp;

// select the interpolation method on user input
if( method == "linear")
{
  interp = _1D::LinearInterpolator<double>();
  // need to cast to the interpolation class to set data
  interp.target<_1D::LinearInterpolator<double>>()->setData(x,y);
}
if( method == "cubicspline")
{
  interp = _1D::CubicSplineInterpolator<double>();
  interp.target<_1D::CubicSplineInterpolator<double>>()->setData(x,y);
}
if( method == "monotonic")
{
  interp = _1D::MonotonicInterpolator<double>();
  interp.target<_1D::MonotonicInterpolator<double>>()->setData(x,y);
}

// interpolation is done with the operator() method.
double val = interp(2.0);
```

or the new `AnyInterpolator` class, which doesn't require a cast when calling `setData`.

```C++
#include <libInterpolate/Interpolate.hpp>
#include <libInterpolate/AnyInterpolator.hpp>
...
string method;
vector<double> x,y;
...
fill x and y with data
...

// Create a std::function to store the interpolator
std::function<double(double)> interp;

// select the interpolation method on user input
if( method == "linear")
  interp = _1D::LinearInterpolator<double>();
if( method == "cubicspline")
  interp = _1D::CubicSplineInterpolator<double>();
if( method == "monotonic")
  interp = _1D::MonotonicInterpolator<double>();

interp.setData(x.size(), x.data(), y.data());

// interpolation is done with the operator() method.
double val = interp(2.0);
```


Note that the interpolated data is copied by the interpolator, so it is safe to pass both the `std::function` and `AnyInterpolator` objects
around. The interpolator it stores will be self-contained.

Note that the interpolated data is copied by the interpolator, so it is safe to pass the `std::function` object
around. The interpolator it stores will be self-contained. Just remember that all data will be copied when you pass by value.
