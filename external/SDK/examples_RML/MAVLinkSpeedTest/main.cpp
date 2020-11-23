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
// #include <Motor.h>
// #include <Behavior.h>
#include <SDK.h>
// This example allows the mainboard to receive toe/limb positions from the MAVLink ethernet interface mblink,
// to command the robot's limbs to those positions.
// Make this example using CMake, flash it to the robot, plug in an ethernet cable, and run "./stand",
// and the robot will stand up and sit down, from commanded toe/limb positions.
// See README.md for more information.

// Robot class
gr::Robot robot;

// Data received for one limb
struct LimbCmd_t {
  float pos[3], Kp[3], Kd[3];
	// Eigen::Vector3f pos, kp, kd;
};

// Data buffer, data is copied from from incoming MAVLink float array
LimbCmd_t limbCmd[4];

// Last time we got a MAVLink packet
// uint32_t last_mavlink_time_msec = 0;
uint32_t lastMAVLinkTimeMS = 0;

// This is called every time we receive a new float array from a MAVLink UDP packet to mainboard
// void mavlinkRx(const float data[58], uint64_t time_usec, char name[10]) {
//   // Copy the data to a global var to use in a behavior
//   memcpy(limbCmd, data, 4 * sizeof(LimbCmd_t));

//   // Update last received MAVLink packet time (http://ghostrobotics.gitlab.io/SDK/lowlevel/SDKFunctions.html#_CPPv411clockTimeUSv)
//   last_mavlink_time_msec = clockTimeUS() / 1000;

//   // Unused
//   (void)time_usec;
//   (void)name;
// }
void mavlinkRx(const float data[58], uint64_t /* time_usec */, char /* name */[10]) {
  // Copy the data to a global var to use in a behavior
  memcpy(limbCmd, data, 4 * sizeof(LimbCmd_t));

  // Update last received MAVLink packet time (http://ghostrobotics.gitlab.io/SDK/lowlevel/SDKFunctions.html#_CPPv411clockTimeUSv)
  // last_mavlink_time_msec = clockTimeUS() / 1000;
  lastMAVLinkTimeMS = robot.millis;

  // // Unused
  // (void)time_usec;
  // (void)name;
}
class UserLimbCmd : public gr::Behavior
{
public:
	const uint32_t MAVLINK_TIMEOUT_MS = 1000;
	void begin(const gr::Robot *R) {
		// Register the callback
		// mavlinkUserRxCallback(mavlinkRx);
		gr::mavlinkUserRxCallback(mavlinkRx);

		// Zero out for safety before mavlink stuff is received
		memset(limbCmd, 0, 4 * sizeof(LimbCmd_t));
	}
	void end() {}
		// void update()
	void update(gr::Robot *R) {
		disableAllLimbs(R);
		// // All joints in current control mode
		// for (int j = 0; j < P->joints_count; ++j)
		// 	joint[j].setOpenLoopMode(JointMode_CURRENT);
		// Set the limb position command for all limbs

		sendDataBackToComputer(R);
	}
protected:

		// // Limb control mode
		// C->mode = RobotCommand_Mode_LIMB;
		
		// // Stop the robot if no MAVLink message has been received in one second
		// uint32_t time_msec = clockTimeUS() / 1000;
		// if(time_msec > last_mavlink_time_msec + 1000)
		// {
		// 	// No moving for you, robot
		// 	C->mode = RobotCommand_Mode_JOINT;
		// 	for (int j = 0; j < P->joints_count; ++j)
		// 		C->joints[j].mode = JointMode_OFF;
		// }
		
		// // For all limbs
		// for (int i = 0; i < 4; ++i)
		// {
		// 	// For all joints on this limb
		// 	for (int j = 0; j < 3; ++j)
		// 	{
		// 		// Set gains from MAVLink packet
		// 		limb[i].setGain(j, limbCmd[i].Kp[j], limbCmd[i].Kd[j]);
				
		// 		// Set positions from MAVLink packet
		// 		limb[i].setPosition(j, limbCmd[i].pos[j]);
		// 	}
		// }
		
		// // Send back data to computer
		// float userData[10];
		// userData[0] = (float) clockTimeUS();  // Send time in microseconds
		// userData[1] = limbCmd[0].pos[2];      // Echo back commanded toe z position
		// userData[2] = (C->mode == RobotCommand_Mode_LIMB) ? 1 : 0; // Report limbs active/inactive state
		// mavlinkUserTxPack(userData); // Pointer to 10 floats or 40 bytes of anything
	inline void disableAllLimbs(gr::Robot *R) {
		for (int i = 0; i < R->P.limbs_count; ++i) {
			R->C.limbs[i].mode = LimbCmdMode_LIMB_OFF;
		}
	}
	void sendDataBackToComputer(const gr::Robot *R) {
		static float userData[10];
		userData[0] = (float)R->millis;  // Send time in ms
		userData[1] = 6969;      // Echo back commanded toe z position
		userData[2] = R->C.limbs[0].mode == LimbCmdMode_LIMB_OFF ? 0 : 1; // Report limbs active/inactive state
		gr::mavlinkUserTxPack(userData); // Pointer to 10 floats or 40 bytes of anything
	}

	// void update()
	// {

	// 	// No moving for you, robot
	// 	C->mode = RobotCommand_Mode_JOINT;
	// 	for (int j = 0; j < P->joints_count; ++j)
	// 		C->joints[j].mode = JointMode_OFF;


	// 	// // All joints in current control mode
	// 	// for (int j = 0; j < P->joints_count; ++j)
	// 	// 	joint[j].setOpenLoopMode(JointMode_CURRENT);

	// 	// // Limb control mode
	// 	// C->mode = RobotCommand_Mode_LIMB;
		
	// 	// // Stop the robot if no MAVLink message has been received in one second
	// 	// uint32_t time_msec = clockTimeUS() / 1000;
	// 	// if(time_msec > last_mavlink_time_msec + 1000)
	// 	// {
	// 	// }
		
	// 	// // For all limbs
	// 	// for (int i = 0; i < 4; ++i)
	// 	// {
	// 	// 	// For all joints on this limb
	// 	// 	for (int j = 0; j < 3; ++j)
	// 	// 	{
	// 	// 		// Set gains from MAVLink packet
	// 	// 		limb[i].setGain(j, limbCmd[i].Kp[j], limbCmd[i].Kd[j]);
				
	// 	// 		// Set positions from MAVLink packet
	// 	// 		limb[i].setPosition(j, limbCmd[i].pos[j]);
	// 	// 	}
	// 	// }
		
	// 	// Send back data to computer
	// 	float userData[10];
	// 	userData[0] = (float) clockTimeUS();  // Send time in microseconds
	// 	userData[1] = 6969;      // Echo back commanded toe z position (or something else)
	// 	userData[2] = (C->mode == RobotCommand_Mode_LIMB) ? 1 : 0; // Report limbs active/inactive state
	// 	mavlinkUserTxPack(userData); // Pointer to 10 floats or 40 bytes of anything
	// }
};
UserLimbCmd userLimbCmd;

void debug() {
	printf("Mainboard output: %f",limbCmd[0].pos[0]);
		  // printf("%lu\t", robot.millis);
  // printf("%d\t%lx\t", robot.status.mode, robot.P.version);
  // printf("Millis: %lu\t", S->millis);
  // printf("Last MAVLink: %lu\t", last_mavlink_time_msec);
  // printf("%d\t%x\t", S->status.mode, robotInfo.robotVersion);

  // // Joints
  // for (uint8_t i = 0; i < P->joints_count; ++i) {
  //   printf("%.2f\t", joint[i].getPosition());
  // }

  // IMU
  //printf("%.2f\t%.2f\t%.2f\t", S->imu.euler.x, S->imu.euler.y, S->imu.euler.z);

  printf("\n");
}

int main(int argc, char *argv[]) {
	RobotParams_Type rtype =
  // RobotParams_Type robot =
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

  // // Disable built-in soft stand up
  // P->behaviorConfig.softStart = false;

  // // Remove default behaviors from behaviors vector, create, add, and start ours
  // behaviors.clear();
  // behaviors.push_back(&userLimbCmd);
  // userLimbCmd.begin();

  // return begin();
         robot.behaviorConfig.overTemperatureShutoff = false;
  robot.behaviorConfig.softStart = false;
  robot.behaviorConfig.fallRecovery = false;



  // Remove default behaviors from behaviors vector, create, add, and start ours
  // behaviors.clear();
  // behaviors.push_back(&userLimbCmd);
  // userLimbCmd.begin();
    robot.behaviors.clear();
  robot.behaviors.push_back(&userLimbCmd);
  userLimbCmd.begin(&robot);

  // return begin();
  return robot.begin();
}
