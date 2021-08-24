RBDL - Rigid Body Dynamics Library
Copyright (c) 2011-2020 Martin Felis <martin@fysx.org>

Introduction
============

RBDL is a highly efficient C++ library that contains some essential rigid body dynamics algorithms
such as the Articulated Body Algorithm (ABA) for forward dynamics, Recursive Newton-Euler
Algorithm (RNEA) for inverse dynamics and the Composite Rigid Body Algorithm (CRBA) for the
efficient computation of the joint space inertia matrix. It further contains code for Jacobians,
forward and inverse kinematics, handling of external constraints such as contacts and collisions,
and closed-loop models.

The code was originally developed by Martin Felis <martin@fysx.org> at the research group
[Optimization in Robotics and Biomechanics (ORB)](http://orb.iwr.uni-heidelberg.de) of the
[Interdisciplinary Center for Scientific Computing (IWR)](http://www.iwr.uni-heidelberg.de) and
[Institute of Computer Engineering](https://www.ziti.uni-heidelberg.de/ziti/en/) at [Heidelberg
University](http://www.uni-heidelberg.de). The code closely follows the notation used in Roy
Featherstone's book "Rigid Body Dynamics Algorithm".

This repository contains the version of RBDL that is maintained by the members
of the ORB research group. 


Documentation
=============

The documentation is contained in the code and can be extracted with the tool [doxygen](http://www.doxygen.org).

To create the documentation simply run
```
    doxygen Doxyfile
```

which will generate the documentation in the subdirectory ./doc/html. The main page will then
be located in ./doc/html/index.html.

Getting RBDL
============

The official rbdl-orb git repository can be cloned from
```
    https://github.com/ORB-HD/rbdl-orb
```

(See [https://git-scm.com/downloads/guis/](https://git-scm.com/downloads/guis/) for git clients.)

To make sure all submodules are correctly downloaded, clone the repository recursively!

```
git clone --recurive https://github.com/ORB-HD/rbdl-orb
```

Upgrading from an older version of RBDL
=======================================

For convenience there is a script to upgrade to the newest RBDL repository version.

```
./upgrade.sh
```

It pulls the latest commits from master and also checks out the correct version of all sub repositories. Manual upgrading requires doing the following:

```
git pull origin master
git submodule update --init
```

Building and Installation
=========================

## Linux: RBDL

1. Prior to installation update the apt system. Open a terminal and type
```
  sudo apt update
  sudo apt upgrade
```
2. Install git
```
  sudo apt install git-core
```

3. Install cmake 
```
  sudo apt install cmake
```

4. Install Eigen3
  RBDL uses Eigen3 for efficient computations ([http://eigen.tuxfamily.org](http://eigen.tuxfamily.org)). 

```
  sudo apt install libeigen3-dev
```

5. Install a c++ compiler
  The choice of compiler can have a large effect on performance. Consider evaluating a few different compilers, such as Clang, for the best performance.

```
  sudo apt-get install build-essential
```

6. Install cmake-curses *(optional)*
    If you are planning on taking advantage of the many addons and other build options we recommend that you use cmake-curses as it makes the build configuration process faster and less prone to error.
```
  sudo apt install cmake-curses-gui
```

7. Install Catch2 *(optional)*
  Install Catch2 if you want to run RBDL's test code.
  
  At the moment most linux distributions do not have catch2 in their repositories yet. So the recommended install approach is to build it from source.
  ```
  $ git clone --branch v2.x https://github.com/catchorg/Catch2.git
  $ cd Catch2
  $ cmake -Bbuild -H. -DBUILD_TESTING=OFF
  $ sudo cmake --build build/ --target install 
  ```

8. Build RBDL using CMake
([http://www.cmake.org](http://www.cmake.org)). To compile the library in a separate directory in Release mode use:
```
  mkdir /rbdl-build
  cd rbdl-build/
  cmake -D CMAKE_BUILD_TYPE=Release ../rbdl
  make
```
If you have installed cmake-curses-gui you can see all of the available build options by running cmake-curses
```
  mkdir /rbdl-build
  cd rbdl-build/
  ccmake ../rbdl 
```
at which point you will see full list of build options for RBDL. We recommend that you build and run RBDL's test code at least once by building RBDL with 
  ```
  RBDL_BUILD_TESTS                 ON
  RUN_AUTOMATIC_TESTS              ON
  ```

## Linux: RBDL's documentation

1. Install doxygen
```  
    sudo apt install doxygen
```

2. Build the doxygen:
  - Open a terminal in the RBDL source directory and type
  ```
  doxygen Doxyfile
  ```

3. Open the file doc/html/index.html in a web-browser.

## Linux: RBDL's examples

1. Install Boost *(optional)*
  Boost is needed to run many of the example simulations that come with RBDL.
  ```
  sudo apt install libboost-all-dev
  ```    

## Linux: RBDL's addon dependencies

1. luamodel addon: 
  -  If you'd like to load model files written in Lua to RBDL. Without this addon you will need to build models programmatically, or read them in using the URDF addon. To do so:
  - Install Lua51 
  ```
    sudo apt install lua5.1
    sudo apt install liblua5.1-0-dev
  ```
  - Build RBDL with
  ```
    RBDL_BUILD_ADDON_LUAMODEL        ON
  ```
2. urdf addon
  - If you'd like to load model files written in URDF to RBDL. This addon uses the URDF_Parser library which is included as a submodule. You will need to have cloned the repository recursively!
  If you missed doing that you can intialize the submodules (from a terminal within the source directory) after the fact with:
  ```
  git submodule init
  git submodule update
  ```
  - Build RBDL with
  ```
  RBDL_BUILD_ADDON_URDFREADER        ON
  ```
3. muscle addon
  - If you'd like to include muscles in your RBDL muscles, such as those in Millard et al., then build RBDL with
  ```
    RBDL_BUILD_ADDON_GEOMETRY ON
    RBDL_BUILD_ADDON_MUSCLE   ON
  ```
  - The geometry addon is a dependency which cmake will automatically include
  - Millard M, Emonds AL, Harant M, Mombaur K. A reduced muscle model and planar musculoskeletal model fit for the simulation of whole-body movements. Journal of biomechanics. 2019 Apr 10. 
   
4. muscle addon: muscle fitting option
  - If you'd like to make use of the muscle fitting algorithms detailed in Millard et al. 
  - Install Ipopt. One of the easier ways to do this is to follow these instructions from [Ipopt's online documentation](https://www.coin-or.org/Ipopt/documentation/node12.html#SECTION00042300000000000000) which guides you through the process. Instructions to build the code appear in the README located in the Ipopt folder
  - Configure RBDL's cmake file with these flags set to 'On'
  ```
          RBDL_BUILD_ADDON_GEOMETRY        ON                                           
          RBDL_BUILD_ADDON_LUAMODEL        ON                                           
          RBDL_BUILD_ADDON_MUSCLE          ON                                          
          RBDL_BUILD_ADDON_MUSCLE_FITTING  ON  
  ```
    
  - Set the CUSTOM_IPOPT_PATH to the main Ipopt directory.
  - Build RBDL
  - Update your .bashrc file so that Ipopt's lib folder is in LD_LIBRARY_PATH
  ```
        export IPOPT_HOME=/home/mjhmilla/dev/Ipopt-3.12.8
        export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$IPOPT_HOME/lib
  ```
  -  As of March 2019 all of the muscle fitting code has been tested with Ipopt-3.12.8. 
  - 
  - Millard M, Emonds AL, Harant M, Mombaur K. A reduced muscle model and planar musculoskeletal model fit for the simulation of whole-body movements. Journal of biomechanics. 2019 Apr 10.

## Windows

Although RBDL can be installed on Windows, none of the ORB members currently
uses Windows and so we are unable to provide detailed instructions.

Python Bindings
===============

RBDL can also build an experimental python wrapper that works with python 3 and 
python 2. To do this enable the the `RBDL_BUILD_PYTHON_WRAPPER` cmake options. 
This will build the wrapper for python 3, if you want to use python 2 instead
you will also have to enable the `RBDL_USE_PYTHON_2` cmake option. The result 
of this is an extra python directory in the build directory. From within which 
you can install it using setup.py. This is done automatically when using 
`make install`

## Linux: Python wrapper dependencies 
1. Install Python3, NumPy, SciPy, \& Matplotlib *(optional)*
  Most of RBDL is accessible through Python. If you are interested in using the RBDL through Python these instructions:

  - If you are using Ubuntu 18.04 or later python3 comes pre-installed.
  - To check if you have python3, in a command shell type
  ```
  python3 -V
  ```
  - If you already have python3 installed system-wide then you can get the remaining libraries with
  ```
  sudo apt install cython3 python3-numpy python3-scipy python3-matplotlib
  ```
  - If you are not using Ubuntu 18.04, and do not currently have python3, please
    look for instructions online to install these libraries on your system.     
2. Build and install RBDL with the 
  ```
  RBDL_BUILD_PYTHON_WRAPPER : ON
  ```
  (Note: you may need sudo privileges to install the rbdl.egg_info file to usr/local/lib/python directory.)
3. Add RBDL to Python's path
  Update your .bashrc file so that python can find the python version of rbdl. To do this you need to add the path to 'rbdl-build/python' to the PYTHONPATH which can be done by adding the following line to your .bashrc file.
  ```
  export PYTHONPATH=$PYTHONPATH:<path-to-the-RBDL-build-directory>/python
  ``` 

Resources to learn more
======================= 

There are four main ways to learn about anything that appears in RBDL:

1. The examples folder
  - There are a set of deep-dive examples which are accompanied by detailed 
  documentation: if you are new to RBDL start here first.
  - There are also a set of minimalistic examples 
  - The examples cover the basics reasonably well, but currently many advanced 
  items (quaternion joints, custom-joints, custom-constraints, muscle-fitting) 
  do not have examples.
2. The Doxygen documentation   
  - The Doxygen for methods and components that were developed recently are 
  covered in great detail (e.g. the Millard2016TorqueMuscle class in the muscle addon).
  - Doxygen for more well established methods are more sparsely documented.
3. The test code; 
  - A minimalistic example of every command and modeling component can be found 
  in the test code (e.g. in rbdl/tests, addons/geometry/tests, addons/muscle/tests, etc).
  - A specific command can be easily found by using a text editor that can 
  search an entire directory (e.g. sublime text) of text files for a keyword.
4. The literature. 
  - In addition to Featherstone's text and Felis's papers there
are a number of exciting methods and modeling tools which are included in RBDL.
  - The appropriate literature references are mentioned in the doxygen for the 
  method in question.


Citation
========

An overview of the theoretical and implementation details has been published in [https://doi.org/10.1007/s10514-016-9574-0](Felis, M.L. Auton Robot (2017) 41: 495). To cite RBDL in your academic research you can use the following BibTeX entry:

    @Article{Felis2016,
      author="Felis, Martin L.",
      title="RBDL: an efficient rigid-body dynamics library using recursive algorithms",
      journal="Autonomous Robots",
      year="2016",
      pages="1--17",
      issn="1573-7527",
      doi="10.1007/s10514-016-9574-0",
      url="http://dx.doi.org/10.1007/s10514-016-9574-0"
    }

Licensing
=========

The library is published under the very permissive zlib free software license which should allow you to use the software wherever you need. 

This is the full license text (zlib license):

    RBDL - Rigid Body Dynamics Library
    Copyright (c) 2011-2020 Martin Felis <martin@fysx.org>
    
    This software is provided 'as-is', without any express or implied
    warranty. In no event will the authors be held liable for any damages
    arising from the use of this software.
    
    Permission is granted to anyone to use this software for any purpose,
    including commercial applications, and to alter it and redistribute it
    freely, subject to the following restrictions:
    
       1. The origin of this software must not be misrepresented; you must not
       claim that you wrote the original software. If you use this software
       in a product, an acknowledgment in the product documentation would be
       appreciated but is not required.
    
       2. Altered source versions must be plainly marked as such, and must not
       be misrepresented as being the original software.
    
       3. This notice may not be removed or altered from any source
       distribution.

Acknowledgements
================

Work on this library was originally funded by the [Heidelberg Graduate School of Mathematical and Computational Methods for the Sciences (HGS)](http://hgs.iwr.uni-heidelberg.de/hgs.mathcomp/), and the European FP7 projects [ECHORD](http://echord.eu) (grant number 231143) and [Koroibot](http://koroibot.eu) (grant number 611909).

Work on the geometry and muscle addons was completed by Matthew Millard. Financial support from Deutsche Forschungs Gemeinschaft grant no. MI 2109/1-1 and from the European Commission within the H2020 project Spexor (GA 687662) is gratefully acknowledged.
