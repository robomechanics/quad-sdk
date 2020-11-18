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
#include <Behavior.h>

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
// Data received for one limb
struct LimbCmd_t {
  // float pos[3], Kp[3], Kd[3];
  // probs add tau
	float tau[3], pos[3], vel[3], Kp[3], Kd[3];
};

// Data buffer, data is copied from from incoming MAVLink float array
LimbCmd_t limbCmd[4];

// Last time we got a MAVLink packet
uint32_t last_mavlink_time_msec = 0;

// This is called every time we receive a new float array from a MAVLink UDP packet to mainboard
void mavlinkRx(const float data[58], uint64_t time_usec, char name[10]) {
  // Copy the data to a global var to use in a behavior
  memcpy(limbCmd, data, 4 * sizeof(LimbCmd_t));

  // Update last received MAVLink packet time (http://ghostrobotics.gitlab.io/SDK/lowlevel/SDKFunctions.html#_CPPv411clockTimeUSv)
  last_mavlink_time_msec = clockTimeUS() / 1000;

  // Unused
  (void)time_usec;
  (void)name;
}

class UserLimbCmd : public Behavior
{
public:
	void begin()
	{
		// Register the callback
		mavlinkUserRxCallback(mavlinkRx);

		// Zero out for safety before mavlink stuff is received
		memset(limbCmd, 0, 4 * sizeof(LimbCmd_t));
	}

	void end() {}

	void update()
	{

		// // No moving for you, robot
		// C->mode = RobotCommand_Mode_JOINT;
		// for (int j = 0; j < P->joints_count; ++j)
		// 	C->joints[j].mode = JointMode_OFF;


		// All joints in current control mode
		for (int j = 0; j < P->joints_count; ++j)
			joint[j].setOpenLoopMode(JointMode_TORQUE); // might not actually work JointMode_current actually works

		// Limb control mode
		C->mode = RobotCommand_Mode_JOINT;
		
		// Stop the robot if no MAVLink message has been received in one second
		uint32_t time_msec = clockTimeUS() / 1000;
		if(time_msec > last_mavlink_time_msec + 1000)
		{
			// I think this is supposed to be filled? I think we should just make the robot stand
		}
		float norm_joint_torque;
		int current_joint = 0;
		for (int i = 0; i < 4; ++i){
			for (int j = 0; j < 3; ++j){
				// Write our own full state feedback controller for the joints
				float tau_des = limbCmd[i].tau[j];
				float pos_des = limbCmd[i].pos[j];
				float vel_des = limbCmd[i].vel[j];
				float Kp = limbCmd[i].Kp[j];
				float Kd = limbCmd[i].Kd[j];

				// Get the joint data
				float pos_actual = joint[current_joint].getPosition();
				float vel_actual = joint[current_joint].getVelocity();

				float joint_torque = tau_des + Kp*(pos_des- pos_actual) + Kd*(vel_des- vel_actual);
				norm_joint_torque = joint_torque;	
				// Figure out if we need to set a max torque
				// if(std::abs(norm_joint_torque)>MAX_TORQUE){
				// 	norm_joint_torque = norm_joint_torque/std::abs(norm_joint_torque);
				// }
				
				joint[current_joint].setOpenLoop(0); // For now set it to zero for testing
				// joint[current_joint].setOpenLoop(norm_joint_torque);
				current_joint++; // Hopefully we go in the right order. We can just manually set this to be the case.
			}
		}
		
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
		
		// Send back data to computer (maybe send back joint torques and states?) getCurrent, getTemperature
		// We should probably send a struct back? 
		float userData[10];
		userData[0] = (float) clockTimeUS();  // Send time in microseconds
		userData[1] = 6969;      // Echo back commanded toe z position (or something else)
		userData[2] = norm_joint_torque; // send back 1 of the torques
		mavlinkUserTxPack(userData); // Pointer to 10 floats or 40 bytes of anything
	}
};
UserLimbCmd userLimbCmd;

void debug() {
  printf("Millis: %lu\t", S->millis);
  printf("Last MAVLink: %lu\t", last_mavlink_time_msec);
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
  RobotParams_Type robot =
#ifdef ROBOT_SPIRIT
      RobotParams_Type_SPIRIT;
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
  JoyType joyType = JoyType_FRSKY_XSR;
  ioctl(JOYSTICK_FILENO, IOCTL_CMD_JOYSTICK_SET_TYPE, &joyType);

  // Disable built-in soft stand up
  P->behaviorConfig.softStart = false;

  // Remove default behaviors from behaviors vector, create, add, and start ours
  behaviors.clear();
  behaviors.push_back(&userLimbCmd);
  userLimbCmd.begin();

  return begin();
}