Fade2D, v1.93, September 1st. 2021
==================================

Welcome to the Fade2D Library! Two important things first:

1. UPGRADES FROM A PREVIOUS VERSION:

When you upgrade from a previous Fade version, you *also* need
to replace the header files.

2. COMPILING WITH MAKE

+ Linux/Apple: Edit the Makefile and uncomment (only!) the line
with your OS. Make sure you have libgmp installed.





Getting started
===============

Find instructions for compiling in the very first example online
or simply look below:


* Visual Studio:
----------------
Open a solution file 

  examples_2D/vs20xx_exampleProject/examples_2D.sln   
  examples_25D/vs20xx_exampleProject/examples_25D.sln   

For maximum performance compile in x64 Release mode. Find the executable 
in fadeRelease/x64 and run it in a command line window (cmd.exe). When
you link the library with your own software you can use the same settings
that you find in the example solutions. 

VS2010 - version 10 - toolset v100 or Windows7.1SDK
VS2012 - version 11 - toolset v110 
VS2013 - version 12 - toolset v120
VS2015 - version 14 - toolset v140
VS2017 - version 15 - toolset v141
VS2019 - version 16 - toolset v142

* Linux and Mac: 
----------------
cd examples_2D   (or examples_25D)

* In the Makefile, choose your Linux distribution or Apple (if a specific 
Linux distro is not listed use a similar one). Be sure libgmp is installed.

$> make
$> ./examples_2D   (or ./examples_25D)





Documentation
=============

This software package contains C++ example source codes that cover 
the different topics. They are documented here:

  https://www.geom.at/category/fade2d-examples/
  https://www.geom.at/category/fade25d-examples/

Library Documentation can be found here:

  https://www.geom.at/fade2d/html

Supported platforms:
 
- Windows (VS2010, VS2012, VS2013, VS2015, VS2017, VS2019) 
- Linux (gcc) for x86_64 and Raspberry PI (ARMv6, ARMv7)
- Apple (clang)





Directories
===========

* include_fade2d and include_fade25d
Header files of Fade2D and Fade25D

* x64 (Win32)
The *.dll and *.lib files. This is also the output directory for the
example executables compiled with Visual Studio.

* lib_${DISTRO}_${ARCHITECTURE}
The shared libraries (*.so) for Linux and (*.dylib) Apple developers. The 
libraries work for a wide range of Linux distributions. Commercial users 
who need support for a certain additional Linux distribution: Please get 
in contact with the author. 

* examples_2D
Source code of all examples using Fade2D

* examples_25D
Source code of all examples using Fade2.5D

* doc
Documentation





Licensing
=========


Student license
---------------

Fade is not free software. But the student license is free of charge 
for personal non-commercial scientific research and it can be used 
without registration. When you use Fade in your software,

+ Please put a LINK TO FADE on your research page or your project homepage.
+ Describe and cite Fade in scientific publications for which you have used it.
+ Make clear that the software may not be used commercially.

Limits of the student version:
------------------------------
2D triangulation:   1 million points
2.5D triangulation: 100 000 points
Voronoi diagram:    100 000 points
Meshing:            50 000 output triangles
Cut&Fill:           10 000 triangles
SegmentChecker:     50 000 segments

Not enough for evaluation purposes? Ask the author (bkorn@geom.at) for an 
extended evaluation license if your project requires larger limits. Please 
describe your project and include a link to your research homepage. 


Commercial license
------------------

All other applications, including commercial in-house usage, require a 
commercial license which has the advantage of maintenance, error corrections 
and personal support. The commercial license of Fade consists of several parts:

+ The Fade2D base component (mandatory): It covers 2D Delaunay triangulations, 
constrained Delaunay triangulations (insertion of segments) and the SegmentChecker
+ The Quality Mesh Generator (optional): Delaunay Meshing and Grid Meshing refines 
meshes to achieve well shaped triangles. You might have a look at the examples.
+ The Fade2.5D extension (optional): This is for 2.5D point clouds i.e., for 
terrains and other height fields: Points have (x,y,z)-coordinates and there 
is a bunch of additional algorithms like ISO line computation, height queries, 
point cloud simplification, valley-ridge-optimization, 2.5D segment checking. 
Existing triangles can be imported and trimmed.
+ The Cut&Fill software for earthwork calculations (optional): Cut-And-Fill 
computes the volume between two overlapping TIN.

For a commercial license please contact the author (bkorn@geom.at). Please 
describe your software project briefly and tell us which fade components you 
would like to use.




Your feedback is appreciated:

 Geom Software
 Bernhard Kornberger

 https://www.geom.at
 bkorn@geom.at





In no case can Geom Software be made responsible for damages of any kind 
that arise in connection with the use or non-usability of the software 
or information provided on our internet pages. If you can't accept these 
terms, you are not allowed to use the software. Using Fade for military 
research and applications is not accepted.


