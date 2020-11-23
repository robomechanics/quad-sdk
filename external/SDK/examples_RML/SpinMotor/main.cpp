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
#include <Motor.h>
#include <SDK.h>
#include <stdio.h>

class SpinMotor : public Behavior {
 public:
  void begin() {}
  void end() {}

  void update() {
    // Control robot by joints
    C->mode = RobotCommand_Mode_JOINT;

    // Spin all motors
    for (int i = 0; i < 8; i++) {
      // Use current control
      joint[i].setOpenLoopMode(JointMode_CURRENT);

      // Spin at 2 amps
      joint[i].setOpenLoop(2);

      // Or set motor position:
      //joint[i].setGain(10, 1);
      //joint[i].setPosition(0);
    }
  }
};
SpinMotor behavior;

void debug() {
  printf("%lu\t", S->millis);
  printf("%d\t%x\t", S->status.mode, robotInfo.robotVersion);
  //printf("%lu\t%lu\t", S->status.updateATime, S->status.behaviorTime);
  //printf("%.2f\t%.2f\t", S->status.updateARate, S->status.behaviorRate);

  // Joints
  for (uint8_t i = 0; i < P->joints_count; ++i) {
    printf("%.2f\t", joint[i].getPosition());
  }

  // IMU
  //printf("%.2f\t%.2f\t%.2f\t", S->imu.euler.x, S->imu.euler.y, S->imu.euler.z);

  printf("\n");
}

int main(int argc, char *argv[]) {
  RobotParams_Type robot =
#ifdef ROBOT_SPIRIT
      RobotParams_Type_SPIRIT;
#elif defined(ROBOT_MINITAUR_E)
      RobotParams_Type_MINITAUR_E;
#else
      RobotParams_Type_NGR;
#endif
#ifdef STARTUP_DELAY
  STARTUP_DELAY_MS = STARTUP_DELAY;
#endif
#ifdef ROBOT_VERSION
  ROBOT_VERSION_OVERRIDE = ROBOT_VERSION;
#endif
  init(robot, argc, argv);

  // Hardware configuration
  P->behaviorConfig.overTemperatureShutoff = false;
  JoyType joyType = JoyType_FRSKY_XSR;
  ioctl(JOYSTICK_FILENO, IOCTL_CMD_JOYSTICK_SET_TYPE, &joyType);

  // Remove default behaviors from behaviors vector, create, add, and start ours
  behaviors.clear();
  behaviors.push_back(&behavior);
  behavior.begin();

  return begin();
}
