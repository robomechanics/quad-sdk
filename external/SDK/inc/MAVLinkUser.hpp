/*
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 */
#pragma once

namespace gr {

typedef void (*DebugFloatArrayCallback_t)(const float[58], uint64_t, char[10]);

/**
 * @brief Register a callback for a MAVlink debug float array message with ID 10 to pass user data to the mainboard
 * 
 * @param cb 
 */
void mavlinkUserRxCallback(DebugFloatArrayCallback_t cb);

/**
 * @brief Pack some custom data to send from the mainboard using mavlink
 * 
 * @param data 10 custom floats (or can pack any other data adding up to 40 bytes)
 */
void mavlinkUserTxPack(float data[10]);
 
}
