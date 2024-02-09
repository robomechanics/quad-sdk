/* SPDX-License-Identifier: LGPL-2.1-only */
/*
 * pcanwrite.cpp - PCANBasic Example: Simple Write
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
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <asm/types.h>

#ifndef NO_RT
#include <sys/mman.h>

#ifdef RTAI
#include <rtai_lxrt.h>
#endif

// PCAN-Basic device used to read on
// (RT version doesn't handle USB devices)
#define PCAN_DEVICE		PCAN_PCIBUS2
#else

// PCAN-Basic device used to read on
#define PCAN_DEVICE		PCAN_USBBUS2
#endif

#include <PCANBasic.h>

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
	TPCANStatus Status;
	unsigned long ulIndex = 0;
	unsigned int pcan_device = PCAN_DEVICE;

#ifndef NO_RT
	mlockall(MCL_CURRENT | MCL_FUTURE);

#ifdef RTAI
	// Initialize LXRT
	RT_TASK *mainr = rt_task_init_schmod(nam2num("MAINW"), 0, 0, 0,
					     SCHED_FIFO, 0xF);
	if (!mainr) {
		printf("pcanwrite(%xh): unable to setup main RT task\n",
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

	// be INTRuptible by user
	signal(SIGINT, signal_handler);

	Status = CAN_Initialize(pcan_device, PCAN_BAUD_500K, 0, 0, 0);
	printf("CAN_Initialize(%xh): Status=0x%x\n", pcan_device, (int )Status);
	if (Status)
		goto lbl_exit;

	Message.ID = 0x77;
	Message.LEN = 8;
	Message.MSGTYPE = PCAN_MESSAGE_EXTENDED;
	memset(Message.DATA, '\0', sizeof(Message.DATA));

	while(1) {
		while ((Status = CAN_Write(pcan_device, &Message)) == PCAN_ERROR_OK) {
			// increment data bytes
			for (int i = 0; i < 8; i++)
				if (++Message.DATA[i])
					break;

			ulIndex++;
			if ((ulIndex % 1000) == 0)
				printf("  - T Message %i\n", (int)ulIndex);
		}

		if (Status != PCAN_ERROR_QXMTFULL) {
			printf("CAN_Write(%xh): Error 0x%x\n", pcan_device,
				(int)Status);
			break;
		}

		// Tx queue is full: must wait a bit instad of forever 
		// looping. Handle ^C here too.
		if (usleep(100))
			break;
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
