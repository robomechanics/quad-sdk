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

/**
 * @brief This example allows the mainboard to receive toe/limb positions from the MAVLink ethernet interface mblink,
 * to command the robot's limbs to those positions.
 * Make this example using CMake, flash it to the robot, plug in an ethernet cable, and run "./stand",
 * and the robot will stand up and sit down, from commanded toe/limb positions.
 * See README.md for more information.
 */

// Robot class
gr::Robot robot;
// Data for one limb
struct LimbCmd_t {
  Eigen::Vector3f pos, kp, kd;
};
// Data buffer, data is copied from from incoming MAVLink float array
LimbCmd_t limbCmd[4];
// Last time we got a MAVLink packet
uint32_t lastMAVLinkTimeMS = 0;

/**
 * @brief This is called every time we receive a new float array from a MAVLink UDP packet to mainboard
 */
void mavlinkRx(const float data[58], uint64_t /* time_usec */, char /* name */[10]) {
  // Copy the data to a global var to use in a behavior
  memcpy(limbCmd, data, 4 * sizeof(LimbCmd_t));
  // Update last received MAVLink packet time
  lastMAVLinkTimeMS = robot.millis;
}

/**
 * @brief This is a simple Behavior that just commands the limbs using the low-level 
 * SDK with the mavlink-received commmands
 */
class UserLimbCmd : public gr::Behavior
{
public:
	const uint32_t MAVLINK_TIMEOUT_MS = 1000;

	void begin(const gr::Robot *R) {
		// Register the callback
		gr::mavlinkUserRxCallback(mavlinkRx);
		// Zero out for safety before mavlink stuff is received
		memset(limbCmd, 0, 4 * sizeof(LimbCmd_t));
	}

	void end() {}

	void update(gr::Robot *R) {
		// Stop the robot if no MAVLink message has been received in one second
		if (R->millis - lastMAVLinkTimeMS > MAVLINK_TIMEOUT_MS) {
			disableAllLimbs(R);
			return;
		}
		
		// Set the limb position command for all limbs
		for (int i = 0; i < R->P.limbs_count; ++i) {
			R->setLimbPosition(i, limbCmd[i].pos, limbCmd[i].kp, limbCmd[i].kd);
		}
		
		sendDataBackToComputer(R);
	}
protected:
	inline void disableAllLimbs(gr::Robot *R) {
		for (int i = 0; i < R->P.limbs_count; ++i) {
			R->C.limbs[i].mode = LimbCmdMode_LIMB_OFF;
		}
	}

	void sendDataBackToComputer(const gr::Robot *R) {
		static float userData[10];
		userData[0] = (float)R->millis;  // Send time in ms
		userData[1] = limbCmd[0].pos.z();      // Echo back commanded toe z position
		userData[2] = R->C.limbs[0].mode == LimbCmdMode_LIMB_OFF ? 0 : 1; // Report limbs active/inactive state
		gr::mavlinkUserTxPack(userData); // Pointer to 10 floats or 40 bytes of anything
	}
};
UserLimbCmd userLimbCmd;

void debug() {
  printf("%lu\t", robot.millis);
  printf("%d\t%lx\t", robot.status.mode, robot.P.version);

  // Print limb positions
  for (uint8_t i = 0; i < robot.P.limbs_count; ++i) {
		auto pos = robot.getLimbPosition(i);
		printf("(%.1f,%.1f,%.1f), ", pos.x(), pos.y(), pos.z());
  }

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
  robot.behaviorConfig.fallRecovery = false;

  // Remove default behaviors from behaviors vector, create, add, and start ours
  robot.behaviors.clear();
  robot.behaviors.push_back(&userLimbCmd);
  userLimbCmd.begin(&robot);

  return robot.begin();
}
