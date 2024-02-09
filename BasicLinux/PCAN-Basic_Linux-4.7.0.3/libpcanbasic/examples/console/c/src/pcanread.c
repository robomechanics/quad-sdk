/* SPDX-License-Identifier: LGPL-2.1-only */
/*
 * pcanread.cpp - PCANBasic Example: Simple Read
 *
 * Copyright (C) 2001-2020  PEAK System-Technik GmbH <www.peak-system.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 *
 * Contact:    <linux@peak-system.com>
 * Maintainer: Stephane Grosjean <s.grosjean@peak-system.com>
 * Author:     Thomas Haber <thomas@toem.de>
 */
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <asm/types.h>

#ifndef NO_RT
#include <sys/mman.h>

#ifdef RTAI
#include <rtai_lxrt.h>
#endif

// PCAN-Basic device used to read on
// (RT version doesn't handle USB devices)
#define PCAN_DEVICE	PCAN_PCIBUS1
#else

// PCAN-Basic device used to read on
#define PCAN_DEVICE	PCAN_USBBUS1
#endif

#include "PCANBasic.h"

static void signal_handler(int s)
{
	printf("Interrupted by SIG%u!\n", s);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>	Main entry-point for this application. </summary>
///
/// <remarks>	 </remarks>
///
/// <param name="argc">	The argc. </param>
/// <param name="argv">	[in,out] If non-null, the argv. </param>
///
/// <returns>	. </returns>
////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[])
{
	TPCANMsg Message;
	TPCANTimestamp ts;
	TPCANTimestamp ts_prev;
	TPCANTimestamp ts_diff;
	TPCANStatus Status;
	unsigned int pcan_device = PCAN_DEVICE;
	unsigned long long diff;

#ifndef NO_RT
	mlockall(MCL_CURRENT | MCL_FUTURE);

#ifdef RTAI
	// Initialize LXRT
	RT_TASK *mainr = rt_task_init_schmod(nam2num("MAINR"), 0, 0, 0,
					     SCHED_FIFO, 0xF);
	if (!mainr) {
		printf("pcanread(%xh): unable to setup main RT task\n",
			PCAN_DEVICE);
		return -1;
	}
	rt_make_hard_real_time();
#endif
#endif

	// get the device from the cmd line if provided
	if (argc > 1) {
		char *endptr;
		unsigned long tmp = strtoul(argv[1], &endptr, 0); 
		if (*endptr == '\0')
			pcan_device = tmp;
	}

	// below usleep() will be INTRuptible by user
	signal(SIGINT, signal_handler);

	Status = CAN_Initialize(pcan_device, PCAN_BAUD_500K, 0, 0, 0);
	printf("CAN_Initialize(%xh): Status=0x%x\n", pcan_device, (int)Status);
	if (Status)
		goto lbl_exit;

	while (1) {
		while ((Status=CAN_Read(pcan_device, &Message, &ts)) == PCAN_ERROR_QRCVEMPTY)
			if (usleep(100))
				break;

		if (Status != PCAN_ERROR_OK) {
			printf("CAN_Read(%xh) failure 0x%x\n", pcan_device, (int)Status);
			break;
		}

		ts_diff.millis = ts.millis - ts_prev.millis;
		if (ts.micros > ts_prev.micros) {
			ts_diff.micros = ts.micros - ts_prev.micros;
		} else 	{
			ts_diff.millis--;
			ts_diff.micros = 1000 - ts_prev.micros + ts.micros;
		}
		diff = (ts.millis - ts_prev.millis) * 1000 + (ts.micros - ts_prev.micros);
		printf("  - R ID:%4x LEN:%1x DATA:%02x %02x %02x %02x %02x %02x %02x %02x TS:%010u.%03u Period:%010llu|%010u.%03u\n",
			(int)Message.ID, (int)Message.LEN,
			(int)Message.DATA[0], (int)Message.DATA[1],
			(int)Message.DATA[2], (int)Message.DATA[3],
			(int)Message.DATA[4], (int)Message.DATA[5],
			(int)Message.DATA[6], (int)Message.DATA[7], ts.millis, ts.micros, diff, ts_diff.millis, ts_diff.micros);
		ts_prev = ts;

#ifdef XENOMAI
		// force flush of printf buffers 
		rt_print_flush_buffers();
#endif
	}

	CAN_Uninitialize(pcan_device);

lbl_exit:
#ifdef XENOMAI
#elif defined(RTAI)
	rt_make_soft_real_time();
	rt_task_delete(mainr);
#endif

	return 0;
}
