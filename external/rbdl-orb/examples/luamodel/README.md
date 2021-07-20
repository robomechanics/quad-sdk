luamodel
Copyright (c) 2019 Matthew Millard <matthew.millard@iwr.uni-heidelberg.de>
Licensed under the zlib license. See LICENSE for more details.

# Quick Start

The command-line tool example_luamodel will read in a multibody model described
using a lua script and will print the following high level information to 
the screen. To get started:

1. Make a 'build' folder here

2. Run cmake from the 'build' folder. You will be asked to set CUSTOM_RBDL_PATH,
   which is the path to the install folder of the version of RBDL that you 
   would like to use. 

    If you have a system-wide installation of RBDL this folder is '/usr/local'

3. From a terminal in the build directory call 
  ```
    make
  ```
4. From a terminal in the build directory run
  ```
    example_luamodel ../samplemodel.lua
  ```
5. If everything works you should see a bunch of meta data related to the 
   model printed to the screen: overview of the degrees of freedom, the model
   hierarchy, and results of a call to forward dynamics when q, qd, and tau 
   are set to zero.


#Example Output 1

  When given a call to samplemodel:
  ```
    example_luamodel ../samplemodel.lua    
  ```
  ```
  Degree of freedom overview:
    0: pelvis_TX
    1: pelvis_TY
    2: pelvis_TZ
    3: pelvis_RZ
    4: pelvis_RY
    5: pelvis_RX
    6: thigh_right_RZ
    7: thigh_right_RY
    8: thigh_right_RX
    9: shank_right_RY
   10: thigh_left_RZ
   11: thigh_left_RY
   12: thigh_left_RX
   13: shank_left_RY
  Model Hierarchy:
  ROOT
    pelvis [ TX, TY, TZ, RZ, RY, RX ]
      thigh_right [ RZ, RY, RX ]
        shank_right [ RY ]
          foot_right [fixed]
      thigh_left [ RZ, RY, RX ]
        shank_left [ RY ]
          foot_left [fixed]
  Forward Dynamics with q, qdot, tau set to zero:
   1.07324e-15 -1.59568e-15        -9.81  6.30217e-16  7.05478e-17 -9.99864e-16  8.33789e-16 -3.71464e-16  5.67859e-16 -5.71731e-17  7.76215e-16 -3.35942e-16  5.73777e-16 -3.30172e-17
  ```

#Example Output 2
  When given a call to sampleconstrainedmodel.lua
  ```
  ./example_luamodel ../sampleconstrainedmodel.lua 
  ```
  ```  
  Degree of freedom overview:
    0: base_TX
    1: base_TY
    2: base_TZ
    3: l11_RZ
    4: l12_RZ
    5: l21_RZ
    6: l22_RZ
  Model Hierarchy:
  ROOT
    base [ TX, TY, TZ ]
      l11 [ RZ ]
        l12 [ RZ ]
      l21 [ RZ ]
        l22 [ RZ ]
  Forward Dynamics with q, qdot, tau set to zero:
  0 0 0 0 0 0 0
  ```





