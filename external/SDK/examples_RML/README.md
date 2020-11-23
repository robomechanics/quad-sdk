# Examples_RML

This directory contains all the GR provided examples as well as those written by RML. Please add any custom examples here (and not the `examples/` directory) so that new GR examples will not conflict with our changes.

## Low Level SDK
The examples `FirstStep`, `SpinMotor`, `LimbControl`, use the [low level SDK](http://ghostrobotics.gitlab.io/docs/lowlevel/), and require re-flashing the mainboard to run. You can always flash the default mainboard firmware back to the robot afterwards using the SDK/firmware directory.

See LimbControl/README.md for building the LimbControl example with CMake. The other low-level examples are built with a simple Makefile directly in their directories.

## High Level SDK
The `MAVLink` examples `Sit` and `Stand` use the [high level SDK](http://ghostrobotics.gitlab.io/docs/), and don't require re-flashing the mainboard; they operate with the default mainboard firmware, and just send MAVLink messages to control the built in robot gaits.

## MAVLink speed test
What this do?

