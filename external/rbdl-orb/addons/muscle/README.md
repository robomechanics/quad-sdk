@brief muscle - a set of functions and classes for simulation musculotendon
                dynamics. This addon is maintained by Matthew Millard, so if 
                you have problems with it email him.

@author Matthew Millard

\copyright 2016 Matthew Millard <millard.matthew@gmail.com>

\b Requirements
This addon depends on the geometry addon 

\b Description
  This addon currently contains an optimized library for creating specialized 
  curves that describe phenomenological curves for line-type muscles 
  -fiber active-force-length curve
  -fiber force-velocity curve
  -fiber passive-force-length curve
  -tendon force-length curve
  and torque-type muscles
  -active torque-angle curve
  -active torque-velocity curve
  -passive torque-angle curve
  In addition, there is a class that can be used to package the memory and 
  functions required to model torque muscles: Millard2016TorqueMuscle.

\b Future Development
In the near future this library will also contain

1. Torque-type muscle models with 
  a. Elastic tendons
  b. Short-range-stiffness
  
2. An implementation of the line-type muscle Millard2012Equilibrium muscle model
   which features the option of using rigid and elastic tendons, and a damped/
   undamped formulation.
  
3. An novel implemenation of 2D muscle wrapping using an obstacle set of 
   smooth convex shapes described using Pythagorean hodographs.

\b Licensing
The following files have been ported over from OpenSim and Simbody and as such
are licenced under the Apache 2.0 Licence:

SmoothSegmentedFunctionFactory.h
SmoothSegmentedFunctionFactory.cc

The Apache License is very similar to the zlib license and is quite liberal.
Licensed under the Apache License, Version 2.0 (the "License"); you may   
not use this file except in compliance with the License. You may obtain a 
copy of the License at http://www.apache.org/licenses/LICENSE-2.0.        

The remaining code has been written from scratch and is licenced under the 
zlib license. See the LICENSE file for details.


