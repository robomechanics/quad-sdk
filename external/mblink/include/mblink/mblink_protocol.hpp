/*
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited.
 * Confidential, proprietary and/or trade secret materials.
 * No Distribution without prior written approval.
 *
 * Written by Avik De <avik@ghostrobotics.io>
 *
 * Handles the protocol between the mainboard and any higher-level component
 * 
 * Updated: April 2020
 *
 */
#pragma once

#include <mavlink/common/mavlink.h>
#include <Eigen/Core>
#include <unordered_map>
#include <string>
#include <atomic>

namespace gr
{

class MBLinkProtocol
{
public:
	// RX data can be continuous vectors, discrete vectors, or scalars
	typedef std::unordered_map<std::string, Eigen::VectorXf> RxData_t;

	/**
	 * @brief Construct a new MBLinkProtocol object
	 * 
	 * @param systemId see MAVLink docs
	 * @param componentId see MAVLink docs
	 */
	MBLinkProtocol(uint8_t systemId = 1, uint8_t componentId = 0) : systemId(systemId), componentId(componentId), newRxData(false) {}

	enum ControlMode
	{
		USE_STEPPING_TWIST = 1,
		USE_TWIST = 2,
		USE_STEPPINGZ = 3,
		IGNORE_INPUT = 4,
		IGNORE_MSG = 5
	};
	enum BehaviorID
	{
		SIT = 0,
		STAND = 1,
		WALK = 2
	};
	typedef Eigen::Matrix<float, 1, 1> Vec1_t;
	
	// Debug float array IDs sent back
	constexpr static int DEBUG_FLOAT_ARRAY_ID_FROM_PLANNER = 0;
	constexpr static int DEBUG_FLOAT_ARRAY_ID_USER = 10;
	constexpr static int LANDING_TARGET_FRAME_GAIT_PLAN = 0;
	constexpr static int LANDING_TARGET_FRAME_JOINT_CMD = 1;

	/**
	 * @brief This should be overridden in the derived class
	 * 
	 * @return int 
	 */
	virtual int queueMessage(uint16_t messageLength) { return 0; }

	/**
	* @brief Send a discrete behavior selection command
	* 
	* @param mavId select from SIT/STAND/WALK etc.
	* @param mavArm arm 1 or 0
	* @return Length
	*/
	int setBehavior(BehaviorID mavId, int mavArm);

	/**
	 * @brief Set a parameter value
	 * 
	 * @param name parameter name (16 char or fewer)
	 * @param val value to set to
	 * @return uint16_t 
	 */
	int setParam(const std::string &name, float val);

	/**
	 * @brief Read a parameter value. Should look for a size 1 float in rxdata["param_value"] after calling this
	 * 
	 * @param name parameter name (16 char or fewer)
	 * @return uint16_t 
	 */
	int requestParam(const std::string &name);

	/**
	* @brief Send a desired SE(2) twist
	* 
	* @param twist in units of m/s, rad/s
	* @return Length
	*/
	inline int sendSE2Twist(const Eigen::Vector3f twist) { return sendControl(twist); }

	/**
	* @brief Send a body pose command
	* 
	* @param position xyz in m but only z is used
	* @param orientation Euler angles xyz in radians
	* @return Length
	*/
	int sendPose(const Eigen::Vector3f &position, const Eigen::Vector3f &orientation);

	/**
	 * @brief Send a continuous control signal as the robot is moving
	 * 
	 * @param u Control vector
	 * @param mode Select a control mode
	 * @return int 
	 */
	int sendControl(const Eigen::VectorXf &u, ControlMode mode = USE_TWIST);

	/**
	 * @brief Send user data (58 floats)
	 * 
	 * @param data 
	 * @return int 
	 */
	int sendUser(const Eigen::Matrix<float, 58, 1> &data);

	/**
	 * @brief Communicate information about the environment to the simulation (not used on the robot).
	 * 
	 * @param type Information type
	 * @param start 
	 * @param stop stop-start = # of items. Can send (for example 0--3, 4--7, etc. to send lots of data)
	 * @param data 
	 * @return int 
	 */
	int sendToSim(uint32_t type, uint16_t start, uint16_t stop, const Eigen::Matrix<float, 58, 1> &floatArray);

	/**
	 * @brief Call this if mavlink_parse_char returns true and a valid message is received
	 * 
	 * @param msg Output of mavlink_parse_char
	 * @return true A known message type was received and stored in rxdata
	 * @return false This was an unknown message type - the user should decode it or ignore it
	 */
	bool unpack(const mavlink_message_t *msg);

	/**
	 * @brief Instead of passing mavlink messages, pass raw buffers (for example from recvfrom)
	 * 
	 * @param bytes buffer (from recvfrom, for example)
	 * @param length length
	 * @return int number of messages found
	 */
	int parse(const char *bytes, size_t length);

	int parse(std::string bytes) { return parse(bytes.c_str(), bytes.size()); }

	/**
	 * @brief Get the packed message as a bytes object (useful for python binding)
	 * 
	 * @return std::string 
	 */
	std::string getPackedBytes();

	float MANCON_SCALE_XP = 0.9f, MANCON_SCALE_XM = 0.5f, MANCON_SCALE_Y = 0.5f, MANCON_SCALE_RZ = 0.6f, MANCON_Z_MIN = 0.45f, MANCON_Z_MAX = 0.58f;

protected:
	uint8_t systemId = 1, componentId = 0;

	// Packs TX into here
	mavlink_message_t msg;
	uint8_t txbytes[1024]; // pack into a bytes object
	// For incremental RX (only needed for parseBytes)
	mavlink_message_t rxmsg;
	mavlink_status_t rxstatus;
	uint8_t rxchan = 0;

	/**
	 * @brief Received data are stored here in key-value pairs
	 */
	RxData_t rxdata;
	/**
	 * @brief Flag that is set when there is new data received (should be cleared by whoever is watching).
	 */
	std::atomic_bool newRxData;

	// Data structures for expected messages
	mavlink_wind_cov_t wind_cov;
	mavlink_landing_target_t landing_target;
	mavlink_control_system_state_t control_system_state;
	mavlink_memory_vect_t memory_vect;
	mavlink_debug_float_array_t debug_float_array;
	mavlink_manual_control_t manual_control;
	mavlink_sys_status_t sys_status;
	mavlink_param_value_t param_value;
	mavlink_heartbeat_t heartbeat;
	char paramIdBuf[17];
	void copyParamId(const char *paramId);

	// Single motor controller inputs
	#pragma pack(push,1)
	struct LANI
	{
		float pos, vel;
		int16_t curr;
		uint16_t temperature, voltage, param4;
	}; // 16
	struct MessageFromPlannerHeader_t
	{
		uint32_t type; // goal, steppable, diskList, etc.
		uint16_t start;
		uint16_t stop; // stop-start = # of items. Can send (for example 0--3, 4--7, etc. to send lots of data)
	};
	#pragma pack(pop)

	// Some state
	float lastTime = 0;
	float mvoltage = 0; // mean joint voltage sent in "voltage"
	uint16_t status = 0; // third element of "behavior"
	int nsteps = 0; //< store current step count. This must be packed into landing target commands
	uint32_t behaviorId = 0, behaviorMode = 0; // in "behavior"
	bool contacts[4]; // "contacts"
	uint8_t swingMode[4]; // "swing_mode"
	float t = 0;
	int fl = -1; // for construction the state vector
	Eigen::Matrix<float, 6, 1> ptoe;
	Eigen::Vector3f pcom;

	void constructStateVector();
};


} // namespace gr

