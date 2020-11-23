/**
MIT License (modified)

Copyright (c) 2018 Ghost Robotics
Authors:
Avik De <avik@ghostrobotics.io>
Tom Jacobs <tom.jacobs@ghostrobotics.io>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this **file** (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#include <SDK.h>
#include <stdio.h>

gr::Robot robot;

class SpinMotor : public gr::Behavior {
 public:
  void begin(const gr::Robot *R) {}
  void end() {}

  void update(gr::Robot *R) {
    // Spin all motors
    for (int i = 0; i < R->P.limbs_count; ++i) {
      // Set current mode at 2A
      R->setJointOpenLoops(i, false, 2 * Eigen::Vector3f::Ones());
      // // Or set joint positions
      // R->setJointPositions(i, false, Eigen::Vector3f::Zero(), 10 * Eigen::Vector3f::Ones(), 1 * Eigen::Vector3f::Ones());
    }
  }
};

void debug() {
  printf("%lu\t", robot.millis);
  printf("%d\t%lx\t", robot.status.mode, robot.P.version);
  printf("\n");
}

int main(int argc, char *argv[]) {
  RobotParams_Type rtype =
#ifdef ROBOT_SPIRIT
      RobotParams_Type_SPIRIT;
#else
      RobotParams_Type_NGR;
#endif
#ifdef STARTUP_DELAY
	robot.P.startupDelay = STARTUP_DELAY;
#endif
#ifdef ROBOT_VERSION
	robot.hardwareConfig.versionOverride = ROBOT_VERSION;
#endif
	robot.init(rtype, argc, argv);

  // Hardware configuration
  robot.behaviorConfig.overTemperatureShutoff = false;
  robot.behaviorConfig.softStart = false;
  
  // Remove default behaviors from behaviors vector, create, add, and start ours
  SpinMotor spinMotor;
  robot.behaviors.clear();
  robot.behaviors.push_back(&spinMotor);

	return robot.begin();
}
