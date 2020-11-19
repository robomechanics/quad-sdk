/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 *
 *
 * This file helps define the interface to the robot using MAVLink.
 *
 */
#pragma once

#include <Peripheral.h>
#include <MAVMessages.h>

typedef uint32_t TagID_t;

#define MAVCMD_MAX_TAGS 10
#define MAVCMD_MAX_PARAMS 20

class MAVCmd : public Peripheral
{
protected:

public:
	bool bRCOverride = true;

	// Different incoming packet types from the high level computer or tablet remote controller
	enum CmdType
	{
		BEHAVIOR_CMD = 0, // default (only send cmd twist)
		PARAM_VEC = 1, // used to be called readGains 
		BEHAVIOR_CMD_TAGS_PARAM_VEC = 2, // tags used for step planning etc. param vec could be used for online tuning
	};
	
	// This is set by the actual packet received (don't write)
	uint32_t cmdType = BEHAVIOR_CMD;
	uint32_t behavModeHighWord = 0;

	// Data chunk formats that constitute the packet ===================

	// Behavior cmd format -----------
	GRMBehaviorCmd behaviorCmd; // points to the location of a behaviorCmd in the incoming command buffer

	// Visual tags format -------------
#pragma pack(push, 1)
	struct TagInfo
	{
		TagID_t id;	  // tag id within family
		Vector3 posE; // location in E frame
	};
#pragma pack(pop)

	// Tags come in as
	// uint8_t numTags; // between 0--MAX_TAGS
	// TagInfo tags[numTags];
	uint32_t numTags;
	TagInfo tags[MAVCMD_MAX_TAGS]; // points to the location of tags in the command buffer

	// Param Vec format ----------------
	// float params[numParams];
	uint32_t numParams;                // number of elements in the following array
	float paramVec[MAVCMD_MAX_PARAMS]; // points to the location of params in the command buffer

	// End (data chunk formats that constitute the packet) ===================

	// Is the RC joystick enabled?
	bool prevJoyRCEnabled = false;
	bool joyRCEnabled = true;

	/**
	 * @brief Translates to SDK behavior stuff from received MAVLink command. 
	 * 
	 * @param mavId 0 = sit, 1 = stand/look, 2 = walk
	 * @param mavMode boolean (ARM or DISARM)
	 */
	void setBehavior(uint32_t receivedId);

	/**
	 * @brief 
	 * 
	 */
	void parseIncomingCmd();

	// Has the stand action completed?
	bool bStandComplete = false;

	float paramArray[2]; // hardcoded for now
	bool overrideStanceGains = false;
	float channel, data;

	uint32_t ethRxUpdated = 0;
	
	// What is the robot doing? This tells us.
	// These are the codes returned in MAVLink HEARTBEAT::custom_mode and SET_MODE::custom_mode https://mavlink.io/en/messages/common.html#SET_MODE
	const uint32_t MAV_ID_SIT = 0;
	const uint32_t MAV_ID_STAND = 1;
	const uint32_t MAV_ID_WALK = 2;

	const uint32_t MAV_ID_FREEZE = 30;    // Robot is in catpounce mode
	const uint32_t MAV_ID_KILL = 50;      // Robot is E-stopped.
	const uint32_t MAV_ID_SELFCHECK = 60; // Robot is in self-check mode

	// MAVLink disarm and armed modes
	const uint32_t MAV_MODE_DISARMED = 0;
	const uint32_t MAV_MODE_ARMED = 1;

	// We assume the robot has two behaviors
	const uint32_t ROBOT_ID_SOFTSTART = 0;
	const uint32_t ROBOT_ID_PCWALK = 1;

	// Is MAVLink control enabled?
	bool bEnabled = false;
	
	// MAV ID and mode reported back to the computer
	uint32_t mavId = 0, mavMode = 0;

	void begin()
	{
		bEnabled = true;
	}

	void update();
};

#ifdef ARCH_mbm
// Need a single global instance
extern MAVCmd mavCmd;
#endif

/**
 * @brief Default callback for populating data that goes back from the MCU
 * 
 * @param hdr 
 * @param bufAfterHdr 
 * @return uint16_t 
 */
uint16_t mavCmdStateCopyCallback(GRMHeader *hdr, uint8_t *bufAfterHdr);

/**
 * @brief Default callback called by ethernet driver (can be replaced by the user)
 * 
 * @param buf 
 * @param dataSize 
 * @param pObj pass global global instance
 */
void mavCmdIncomingCmdCallback(const uint8_t *buf, const uint16_t dataSize, void *pObj);
