/*
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 */
#pragma once

#if defined(_MSC_VER)
typedef int64_t useconds_t;
#define UNUSED_PARAM
// unistd
#define STDIN_FILENO 0
#define STDOUT_FILENO 1
#define STDERR_FILENO 2
// These are POSIX
#define write _write
#define read _read
extern "C" int _write(int file, char *data, int len);
extern "C" int _read(int file, char *ptr, int len);
#else
#include <sys/types.h> // for useconds_t
#include <unistd.h>
// #define UNUSED_PARAM __attribute__((unused_))
#endif

/**
 * @brief Microsecond sleep function. 
 * @details **Warning:** This kind of delay *must not* be used in Behavior::update(). This function
 * works the same in Unix-like systems and the MCU.
 * @param us Delay duration in microseconds
 */
extern "C" int usleep(useconds_t us);

namespace gr {
	
#ifndef bitRead
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))
#endif

/**
 * @brief Interpretation of the IMU status
 */
#define IMU0_CONNECTED_BIT (0)
#define IMU0_DATA_GOOD_BIT (1)
#define IMU1_CONNECTED_BIT (8)
#define IMU1_DATA_GOOD_BIT (9)

}
