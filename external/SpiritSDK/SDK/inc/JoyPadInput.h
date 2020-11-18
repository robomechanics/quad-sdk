/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Tom Jacobs <tom.jacobs@ghostrobotics.io>
 */

#ifndef JoyPadInput_h
#define JoyPadInput_h

#include <gainput/gainput.h>
#include <JoyBase.h>

#if defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#endif

class JoyPadInput: public JoyBase {
public:

	JoyPadInput() {

	}

	virtual void begin();
	virtual void update();
	virtual void toNormalizedBehaviorCmd(BehaviorCmd *b);

	void setSensitivity(float speedSens, float yawSens);

	virtual ~JoyPadInput() {}

	/**
	 * @brief set to true after begin() if a pad was found
	 * 
	 */
	bool padPresent = false;

protected:
	gainput::DeviceId mouseId, padId;
#if defined(_WIN32)
	HWND hWnd;
#endif
};

#endif