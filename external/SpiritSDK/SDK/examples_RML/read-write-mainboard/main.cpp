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


/*
GOAL OF THIS SCRIPT IS TO TAKE IN A CURRENT DESIRED STATE, FEEDFORWARD TERM, AND GAIN AND APPLY THAT TO THE MOTORS OF THE ROBOT.
I THINK WE SHOULD RETURN THE STATE OF THE ROBOT AND THE APPLIED TORQUES/VOLTAGES?
Features
- Maybe we can take in several states and positions and interpoalte?
- If we don't get a new input coming in we hold it at the current state?
- If new input doesn't come when expected several times we shut down 
*/
gr::Robot robot;
int n_bytes = 4;
// Data received for one limb
struct LimbCmd_t {
  // float pos[3], Kp[3], Kd[3];
  // probs add tau
	// float tau[3], pos[3], vel[3], Kp[3], Kd[3];
	Eigen::Vector3f tau, pos, vel,kp,kd;
};

// Data buffer, data is copied from from incoming MAVLink float array
LimbCmd_t limbCmd[1];
float data_local[58];

// Last time we got a MAVLink packet
// uint32_t last_mavlink_time_msec = 0;
uint32_t lastMAVLinkTimeMS = 0;

// This is called every time we receive a new float array from a MAVLink UDP packet to mainboard
// void mavlinkRx(const Eigen::Matrix<float, 58, 1> &data, uint64_t time_usec, char name[10]) {
// void mavlinkRx(const float data[58], uint64_t time_usec, char name[10]) {
void mavlinkRx(const float data[58], uint64_t /* time_usec */, char /* name */[10]) {
  // Copy the data to a global var to use in a behavior
  // memcpy(data_local, data, n_bytes);
	memcpy(data_local, data, 1 * sizeof(LimbCmd_t));

  // Update last received MAVLink packet time (http://ghostrobotics.gitlab.io/SDK/lowlevel/SDKFunctions.html#_CPPv411clockTimeUSv)
  // last_mavlink_time_msec = clockTimeUS() / 1000;
	lastMAVLinkTimeMS = robot.millis;
}

class UserLimbCmd : public gr::Behavior
{
public:
	void begin(const gr::Robot *R)
	{
		// Register the callback
		gr::mavlinkUserRxCallback(mavlinkRx);

		// Zero out for safety before mavlink stuff is received
		memset(data_local, 0, 1 * sizeof(LimbCmd_t));
	}

	void end() {}

	void update(gr::Robot *R)
	{

		// // No moving for you, robot
		disableAllLimbs(R);



		// Send back data to computer (maybe send back joint torques and states?) getCurrent, getTemperature
		// We should probably send a struct back? 
		sendDataBackToComputer(R);
	}
protected:
	inline void disableAllLimbs(gr::Robot *R) {
		for (int i = 0; i < R->P.limbs_count; ++i) {
			R->C.limbs[i].mode = LimbCmdMode_LIMB_OFF;
		}
	}

	void sendDataBackToComputer(const gr::Robot *R) {
		gr::mavlinkUserTxPack(data_local); // Pointer to 10 floats or 40 bytes of anything
	}
};
UserLimbCmd userLimbCmd;

void debug() {
  printf("Current data: %f",data_local[0]);
  // printf("%lu\t", robot.millis);
  // printf("%d\t%lx\t", robot.status.mode, robot.P.version);
  // printf("%d\t%x\t", S->status.mode, robotInfo.robotVersion);

  // // Joints
  // for (uint8_t i = 0; i < P->joints_count; ++i) {
  //   printf("%.2f\t", joint[i].getPosition());
  // }

  // IMU
  //printf("%.2f\t%.2f\t%.2f\t", S->imu.euler.x, S->imu.euler.y, S->imu.euler.z);

  printf("\r\n");
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
  robot.behaviorConfig.fallRecovery = false;

// Remove default behaviors from behaviors vector, create, add, and start ours
  robot.behaviors.clear();
  robot.behaviors.push_back(&userLimbCmd);
  userLimbCmd.begin(&robot);

  return robot.begin();
}