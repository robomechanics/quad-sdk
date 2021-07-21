# Overview

This folder contains a set of minimal examples 

- simple
  - pendulum made using RBDL's API
- luamodel 
  - a model is described in a separate Lua file which RBDL reads in
- urdfreader 
  - an RBDL model is constructed from a urdf file
- python 
  - a model is constructed and manipulated using the python interface

In addition, there are also a set of deep-dive examples of the tools and background literature of RBDL:

- pendulum:
  - Brief tour through RBDL's API and also how to construct your own CMakeLists.txt files.
  - a lua model read into RBDL
  - simulated forward in time using Boost
  - system energy is analyzed
- constrainedDoublePendulum
  - a lua model that uses loop constraints is read into RBDL
  - Constraints vs. joints are discussed as is constraint stabilization
- bouncingBall 
  - Example shows how to apply external forces to a body in RBDL
  - For this example a Hunt-Crossley contact model and a regularized friction model are used to compute forces between a bouncing ball and the ground                       
- walkingInverseDynamicsWithPython
  - Demonstrates an inverse-dynamics pipeline on a walking data and a model

# Getting started

1. Install 
  - RBDL: https://github.com/ORB-HD/rbdl-orb
    - Note: All optional dependencies are required to go through these examples except those related to the muscle addon's muscle-fitting option.  
  - Meshup: https://github.com/ORB-HD/MeshUp
  - Puppeteer: https://github.com/ORB-HD/puppeteer
  -      
2. Each example contains a README.md which contains instructions to run the example, a tour through the code, and literature references (when appropriate, and sparingly used).

If you are trying RBDL for the first time, consider going through the deep-dive examples first (pendulum, constrainedDoublePendulum, bouncingBall, walkingInverseDynamicsWithPython) as these examples have a great deal of documentation to help you get the examples running.

# Contributing

Finally, if you are familiar with these parts of RBDL we'd be happy to receive an example from you on the following topics:

- Quaternion joint
- Custom joint
- Custom constraint
- Muscle fitting


