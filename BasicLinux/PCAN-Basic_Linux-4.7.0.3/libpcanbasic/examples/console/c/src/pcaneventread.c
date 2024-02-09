/* SPDX-License-Identifier: LGPL-2.1-only */
/*
 * pcaneventread.cpp - PCANBasic Example: Event Read
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
#include <string.h>
#include <asm/types.h>
#include <unistd.h>

#ifndef NO_RT
#include <sys/mman.h>

#ifdef RTAI
#include <rtai_lxrt.h>
#include <rtdm/rtdm.h>

/* overload the select() system call with the RTDM one, while it isn't by 
 * RTAI 5.1. Note that this example doesn't care about the timeout
 */
#define select(n, r, w, e, t)	rt_dev_select(n, r, w, e, RTDM_TIMEOUT_INFINITE)
#endif

// PCAN-Basic device used to read on
// (RT version doesn't handle USB devices)
#define PCAN_DEVICE     PCAN_PCIBUS1
#else

// PCAN-Basic device used to read on
#define PCAN_DEVICE     PCAN_USBBUS1
#endif

#include "PCANBasic.h"

static void signal_handler(int s)
{
	printf("Interrupted by SIG%u!\n", s);
}

static int setup_sig_handler(int signum, void (*f)(int))
{
	struct sigaction act;

	memset(&act, 0, sizeof act);
	act.sa_handler = f;

	// note: siagaction() is thread -safe
	return sigaction(signum, &act, NULL);
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
	TPCANStatus Status;
	TPCANTimestamp ts;
	unsigned int rx_count = 0;
	unsigned int pcan_device = PCAN_DEVICE;

#ifndef NO_RT
	mlockall(MCL_CURRENT | MCL_FUTURE);

#ifdef RTAI
	// Initialize LXRT
	RT_TASK *mainr = rt_task_init_schmod(nam2num("EVRD"), 0, 0, 0,
					     SCHED_FIFO, 0xF);
	if (!mainr) {
		printf("pcaneventread(%xh): unable to setup main RT task\n",
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
	setup_sig_handler(SIGINT, signal_handler);

	Status = CAN_Initialize(pcan_device, PCAN_BAUD_500K, 0, 0, 0);
	printf("CAN_Initialize(%xh): Status=0x%x\n", pcan_device, (int)Status);
	if (Status)
		goto lbl_exit;

	int fd;
	Status = CAN_GetValue(pcan_device, PCAN_RECEIVE_EVENT, &fd, sizeof fd);
	printf("CAN_GetValue(%xh): Status=0x%x\n", pcan_device, (int)Status);
	if (Status)
		goto lbl_close;

	printf("Entering infinite loop (reading MSG[ID:0xff LEN:1 DATA[0]:0xff] breaks it)...\n");

	fd_set fds;

	FD_ZERO(&fds);
	FD_SET(fd, &fds);

	// forever loop
	while (1) {

		// blocks on read descriptor
		int err = select(fd+1, &fds, NULL, NULL, NULL);
		if (err != 1 || !FD_ISSET(fd, &fds)) {
			printf("select(%xh) failure: %d\n", pcan_device, err);
			break;
		}

		TPCANMsg Message;
		Status = CAN_Read(pcan_device, &Message, &ts);

		if (Status != PCAN_ERROR_OK) {
			if (Status == PCAN_ERROR_QRCVEMPTY) {
				// nothing to read
				continue;
			}
			printf("CAN_Read(%xh) failure 0x%x\n", pcan_device, (int) Status);
			break;
		}

		// emergency exit...
		if (Message.ID == 0xff && Message.LEN == 1 && Message.DATA[0] == 0xff)
			break;

		rx_count++;
		printf("  - R ID:%4x LEN:%1x DATA:%02x %02x %02x %02x %02x %02x %02x %02x TS:%010u.%03u\n",
				(int) Message.ID, (int) Message.LEN, (int) Message.DATA[0],
				(int) Message.DATA[1], (int) Message.DATA[2],
				(int) Message.DATA[3], (int) Message.DATA[4],
				(int) Message.DATA[5], (int) Message.DATA[6],
				(int) Message.DATA[7], ts.millis, ts.micros);

#ifdef XENOMAI
		// force flush of printf buffers 
		if (!(rx_count % 100))
			rt_print_flush_buffers();
	}

	rt_print_flush_buffers();
#else
	}
#endif

	printf("pcaneventread(%xh): received %u message(s)\n", pcan_device, rx_count);

lbl_close:
	CAN_Uninitialize(pcan_device);

lbl_exit:
#ifdef XENOMAI
#elif defined(RTAI)
	rt_make_soft_real_time();
	rt_task_delete(mainr);
#endif

	return 0;
}
