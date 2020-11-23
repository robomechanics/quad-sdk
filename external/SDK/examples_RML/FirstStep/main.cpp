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
/*
Note that commented out items are from previous SDK releases
*/
// #include <Behavior.h>
// #include <Motor.h>
#include <SDK.h>
// #include <math.h>
#include <stdio.h>

gr::Robot robot;

void debug() {
  // Robot status
  // printf("%d %x\t", S->status.mode, robotInfo.robotVersion);
  printf("%d %lx\t", robot.status.mode, robot.P.version);
  // Joints
  // for (int i = 0; i < P->joints_count; ++i) {
  //   //printf("%.2f\t", joint[i].getPosition());
  //   //printf("%.2f\t", joint[i].getRawPosition());
  // }
  // for (int i = 0; i < P->limbs_count; ++i) {
  //   //printf("%.1f,%.1f,%.1f, ", limb[i].getPosition(EXTENSION), limb[i].getPosition(ANGLE), limb[i].getPosition(ABDUCTION));
  // }

  for (int i = 0; i < robot.P.limbs_count; ++i) {
    auto pos = robot.getLimbPosition(i);
    printf("(%.1f,%.1f,%.1f)->", robot.getJointPosition(i, HIPJ), robot.getJointPosition(i, KNEEJ), robot.getJointPosition(i, ABDUCTIONJ));
    printf("(%.1f,%.1f,%.1f), ", pos.x(), pos.y(), pos.z());
  }

  // // IMU
  // // printf("imu %.2f\t%.2f\t%.2f\t", S->imu.euler.x, S->imu.euler.y, S->imu.euler.z);
  // // printf("angrate %.2f\t%.2f\t%.2f\t", S->imu.angular_velocity.x, S->imu.angular_velocity.y, S->imu.angular_velocity.z);
  // // printf("%.2f\t%.2f\t%.2f\t", S->imu.linear_acceleration.x, S->imu.linear_acceleration.y, S->imu.linear_acceleration.z);

  //   // IMU
  // const auto *I = &robot.imu;
  // printf("imu %.2f\t%.2f\t%.2f\t", I->euler.x, I->euler.y, I->euler.z);
  // printf("angrate %.2f\t%.2f\t%.2f\t", I->angular_velocity.x, I->angular_velocity.y,I->angular_velocity.z);
  // printf("%.2f\t%.2f\t%.2f\t", I->linear_acceleration.x, I->linear_acceleration.y, I->linear_acceleration.z);

  printf("\n");
}

int main(int argc, char *argv[]) {
  // RobotParams_Type robot =
  RobotParams_Type rtype =
#ifdef ROBOT_SPIRIT
      RobotParams_Type_SPIRIT;
#else
      RobotParams_Type_NGR;
#endif
#ifdef STARTUP_DELAY
  // STARTUP_DELAY_MS = STARTUP_DELAY;
      robot.P.startupDelay = STARTUP_DELAY;
#endif
#ifdef ROBOT_VERSION
  // ROBOT_VERSION_OVERRIDE = ROBOT_VERSION;
  robot.hardwareConfig.versionOverride = ROBOT_VERSION;
#endif
  // init(robot, argc, argv);
  robot.init(rtype, argc, argv);

  // // Hardware configuration
  // JoyType joyType = JoyType_FRSKY_XSR;
  // ioctl(JOYSTICK_FILENO, IOCTL_CMD_JOYSTICK_SET_TYPE, &joyType);

  // return begin();
  return robot.begin();
}
