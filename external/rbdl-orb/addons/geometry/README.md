@brief geometry - a set of static tool kits for creating and evaluating curves,
                  surfaces and solids. This addon is maintained by Matthew 
                  Millard, so if you have problems with it email him.

@author Matthew Millard

\copyright 2016 Matthew Millard <millard.matthew@gmail.com>

\b Requirements
This addon is standalone as of its first release

\b Description
  This addon currently contains an optimized library for creating and 
  evaluating 5th order 2D Bezier splines: SegmentedQuinticBezierToolkit.h 
  and SegmentedQuinticBezierToolkit.cc. In addition, there is a nice class
  that can be used to package the memory and functions required to 
  evaluate these curves: SmoothSegmentedFunction.h and 
  SmoothSegmentedFunction.cc. 

\b Future Development
In the near future this library will also contain

1. Geometry tools to represent C2 convex implicit surfaces and enforce 
   contact constraints between two surfaces. This tool kit will be first
   used for simulating foot-ground contact. It could later be used for
   3D muscle wrapping:

   SmoothImplicitSurfaceToolkit
   SmoothImplicitSurface
  
2. Geometry tools to represent quintic Pythagorean Hodograph curves - which are
   special Bezier curves that have an analytic equation for arc-length. This 
   package will also contain code to represent polar Pythagorean Hodographs 
   which will be first used to formulate a foot-ground joint. Later this toolkit
   will be used for a 2D cable transmission system 
   (to simulate muscle wrapping).

   SegmentedQuinticPythagoreanHodographToolkit
   PolarFunctionToolkit


\b Licensing
The following files have been ported over from OpenSim and Simbody and as such
are licenced under the Apache 2.0 Licence:

SmoothSegmentedFunction.h
SmoothSegmentedFunction.cc
SegmentedQuinticBezierToolkit.h
SegmentedQuinticBezierToolkit.cc
Function.h

The Apache License is very similar to the zlib license and is quite liberal.
Licensed under the Apache License, Version 2.0 (the "License"); you may   
not use this file except in compliance with the License. You may obtain a 
copy of the License at http://www.apache.org/licenses/LICENSE-2.0.        

The remaining code has been written from scratch and is licenced under the 
zlib license. See the LICENSE file for details.


