author Matthew Millard
date   : 27 March 2019
version: 0.1

Before getting started note that typically the install process required to
get this example to work is the most time consuming. The most common place to
get stuck is to have problems installing python3 (particularly if your version
of CMake is out of date) and getting the rbdl-python wrapper to build correctly. 

However the time spent installing python3 there is a big reward: you can now
call RBDL from python. This means you do not have to spend time compiling 
C++ code, nor do you have to write a CMakeLists.txt file. In addition you can
now seamlessly use all of the libraries in python3 and associated libraries 
directly with RBDL: this results in a tremendous timesavings during the 
development phase of a typical project.


# Quick Start

This provides an example of the preprocessing that is typically done on
motion capture data that is recorded in a gait lab.

1. Open a terminal in this folder and enter
  ```
  meshup gait912.lua qIK.csv grf.ff 
  ```
You should see a planar model walking with ground forces appearing below
its feet. This is experimental data, and so if you look closely at the
feet you can see they are not always on the ground.

2. To run the python script, from a terminal in this folder
  ```
  python3 processInverseDynamics.py
  ```
2a. When you do this you will see some output to the terminal 
    (in Example Output) and 5 plots will be generated:

  - Figure 1: Shows the (filtered & interpolated) Q's of the walking data 
              that is in qIK.csv

  - Figure 2: Shows numerically calculated QDot's of the walking data

  - Figure 3: Shows numerically calculated QDDot's of the walking data

  - Figure 4: Shows the measured ground forces

  - Figure 5: Shows the generalized forces that result from the inverse 
  dynamics. Under good conditions, the generalized forces applied to the 
  floating base (Tau0-Tau2, which are Fx, Fz and TqY) will be small while 
  ground forces are measured.

2b. To get an idea of what 'acceptable' residual forces are please visit 

https://simtk-confluence.stanford.edu/display/OpenSim/Simulation+with+OpenSim+-+Best+Practices

and scroll down to the table that appears below 'Evaluating your Results:'

<strong>Question</strong>: Using Figure 5, up until what time are the residual 
forces in the data acceptable?

<strong>Question</strong>: Look at the animation and Figure 5: why does 
Tau 1 (Fz, where z is in the normal direction) spike after 1.0s?


# Detailed Code Tour

1. Open gait912.lua and have a look at it. This model is more complex than
what you've seen previously and was generated with the help of a tool
called 'Model Factory' written by Manish Sreenivasa & Monika Harant. To
learn about, and use this tool please visit:

code : https://github.com/manishsreenivasa/ModelFactory
paper: https://arxiv.org/pdf/1804.03407.pdf

It is useful to note that each joint frame (e.g. the "Pelvis" frame) now
has a set of markers in it (e.g. LASI, RASI, LPSI, and RPSI). These markers
have exactly the same names as markers that appear in a c3d file. This allows
puppeteer to load the model, and the c3d file and to solve an 
inverse-kinematics problem by minimizing the distance between the virtual
and recorded markers.

2. To run puppeteer on this data:

2a. Open a terminal in 'walkingInverseDynamicsWithPython' and type
  ```
  puppeteer gait912.lua gaitDataPlanar.c3d
  ```
  Puppeteer should open, and you should see a view of the data and the model.

2b. If puppeteer complains about not being able to find meshes, 
    copy the 'meshes' folder in this directory into your 'puppeteer' folder.
    This should not be necessary.

2c. Press the 'Fit Animation' button that is located under the 'Animation'
    tab on the right. This will solve the inverse kinematics problem.

2d. Press 'Play' (bottom left) to play an animation of the model and the data.

2e. To perfectly register the virtual and real markers at one point in time:

  - Move the time square to the time of interest (say the beginning)

  - Under the 'Display' tab make sure the 'Model Markers' box and the 
   'Mocap Markers' box is checked

  - Under the 'Markers' tab press the 'Auto Markers' button: you will see
   the grey model markers snap to perfectly align with the blue data markers

  - Now, re-run the IK routine: go to the 'Animation' tab and press the 
   'Fit Animation' button again.

  - Press the 'Save' button and save the IK solution - this will write
   it to 'animation.csv'. The marker errors will be written to 
   'fitting_log.csv'. These errors are extremely important for evaluating
   the quality of the fit, and for reporting it in publication.

  *Note*: the 'qIK.csv' file is just the 'animation.csv' file renamed.

3. Open the 'processInverseDynamics.py' file in a text editor and go through
   through it. This implements a standard inverse dynamics processing 
   pipe line:

3a. The force and mocap data are interpolated to have a common sense of time.
3b. The q's are filtered using a low-pass 2nd order Butterworth filter 
   (signal.butter) applied in the forwards and then backwards directions 
   (filtfilt) so that no phase error is introduced.
3e. The values for qdot and qddot are formed using numerical derivatives

At each point in time
3f. The ground forces are resolved into wrenches in the ROOT frame
3g. The foot wrenches are applied to the appropriate id for each foot
   see above for the command to get idRightFoot and idLeftFoot
3h. The inverse dynamics function in RBDL is called
3i. The generalized force vector tau is copied to a matrix
3j. Plots are generated of the input and output data 



# Example Output

DoF:  9
(74, 19)
