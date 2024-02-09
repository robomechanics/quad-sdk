/* SPDX-License-Identifier: LGPL-2.1-only */
/*
 * libpcanfd.c
 *
 * the shared library to unify the interface to the PEAK-System CAN[FD] devices
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
 * Contact:       <linux@peak-system.com>
 * Author:        Stephane Grosjean <s.grosjean@peak-system.com>
 */
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

#include "libpcanfd.h"
#include "src/libprivate.h"

/* #define DEBUG */

/* if defined, device id. are 32-bits values instead of old 8-bits format */
#define USES_32BITS_DEVICENO

/* if defined, pcanfd_open() uses O_ACCMODE special value to open() the device
 * without configuring it, knowing that ioctl(SET_INIT) is called next
 */
//#define PCANFD_OPEN_USES_ACCMODE

#ifdef DEBUG
#define stddbg			stderr
#endif

/* define a messages filters list with one element named pcanfd_msg_filters_1 */
struct __array_of_struct(pcanfd_msg_filter, 1);

/*
 * static struct pcan_bittiming *pcanfd_to_bittiming(__u16 btr0btr1,
 *						  struct pcan_bittiming *pbt)
 *
 *	Convert SJA1000 BTR0BTR1 16-bits value into a generic bittiming
 *	representation
 */
static struct pcan_bittiming *pcanfd_to_bittiming(__u16 btr0btr1,
						  struct pcan_bittiming *pbt)
{
	pbt->sjw = 1 + ((btr0btr1 & 0xc000) >> 14);
	pbt->brp = 1 + ((btr0btr1 & 0x3f00) >> 8);
	pbt->tsam = (btr0btr1 & 0x0080) >> 7;
	pbt->tseg2 = 1 + ((btr0btr1 & 0x0070) >> 4);
	pbt->tseg1 = 1 + (btr0btr1 & 0x000f);

	return pbt;
}

static void pcanfd_update_sp(struct pcan_bittiming *pbt)
{
	pbt->sample_point = (10000 * (1 + pbt->tseg1)) /
				(1 + pbt->tseg1 + pbt->tseg2);
}

/*
 * TPCANRdMsg *pcanfd_to_msg(TPCANRdMsg *msg, const struct pcanfd_msg *pf)
 *
 *	Convert new struct pcanfd_msg to old TPCANRdMsg representation.
 */
TPCANRdMsg *pcanfd_to_msg(TPCANRdMsg *msg, const struct pcanfd_msg *pf)
{
	switch (pf->type) {

	case PCANFD_TYPE_STATUS:
		msg->Msg.ID = pf->id;
		msg->Msg.MSGTYPE = MSGTYPE_STATUS;
		msg->Msg.LEN = 4;

		memset(msg->Msg.DATA, CAN_ERR_OK, msg->Msg.LEN);

		switch (pf->id) {
		case PCANFD_ERROR_WARNING:
			msg->Msg.DATA[3] |= CAN_ERR_BUSLIGHT;
			break;
		case PCANFD_ERROR_PASSIVE:
			msg->Msg.DATA[3] |= CAN_ERR_BUSHEAVY;
			break;
		case PCANFD_ERROR_BUSOFF:
			msg->Msg.DATA[3] |= CAN_ERR_BUSOFF;
			break;
		case PCANFD_RX_EMPTY:
			msg->Msg.DATA[3] |= CAN_ERR_QRCVEMPTY;
			break;
		case PCANFD_RX_OVERFLOW:
			msg->Msg.DATA[3] |= CAN_ERR_OVERRUN;
			break;
		case PCANFD_TX_OVERFLOW:
			msg->Msg.DATA[3] |= CAN_ERR_QXMTFULL;
			break;

		default:
		case PCANFD_TX_EMPTY:
			msg->Msg.DATA[3] |= CAN_ERR_RESOURCE;
			break;

		case PCANFD_ERROR_ACTIVE:
			break;
		}
		break;

#if 0
	case PCANFD_TYPE_CANFD_MSG:
#endif
	case PCANFD_TYPE_CAN20_MSG:
		msg->Msg.ID = pf->id;
		msg->Msg.MSGTYPE = (BYTE )(pf->flags & ~MSGTYPE_STATUS);
		msg->Msg.LEN = (pf->data_len > 8) ? 8 : pf->data_len;
		memcpy(&msg->Msg.DATA, pf->data, msg->Msg.LEN);
		break;

	default:
		return NULL;
	}

	/* TODO: should check whether PCANFD_TIMESTAMP is always set */
	if (pf->flags & PCANFD_TIMESTAMP) {
		msg->dwTime = pf->timestamp.tv_sec * 1000;
		msg->dwTime += pf->timestamp.tv_usec / 1000;
		msg->wUsec = pf->timestamp.tv_usec % 1000;
	}

	return msg;
}

/*
 * struct pcanfd_msg *pcanmsg_to_fd(struct pcanfd_msg *pf,
 *						const TPCANRdMsg *msg)
 *
 *	Convert old TPCANRdMsg to new struct pcanfd_msg representation.
 */
struct pcanfd_msg *pcanmsg_to_fd(struct pcanfd_msg *pf, const TPCANRdMsg *msg)
{
	pf->type = (msg->Msg.MSGTYPE & MSGTYPE_STATUS) ?
				PCANFD_TYPE_STATUS : PCANFD_TYPE_CAN20_MSG;
	pf->id = msg->Msg.ID;
	pf->flags = msg->Msg.MSGTYPE & ~MSGTYPE_STATUS;
	pf->data_len = (msg->Msg.LEN > PCAN_MAXDATALEN) ?
				PCAN_MAXDATALEN : msg->Msg.LEN;
	memcpy(pf->data, msg->Msg.DATA, pf->data_len);

	pf->flags |= PCANFD_TIMESTAMP;
	pf->timestamp.tv_sec = msg->dwTime / 1000;
	pf->timestamp.tv_usec = msg->wUsec + (msg->dwTime % 1000) * 1000;

	return pf;
}

/*
 * Level-1 API encapsulates ioctl() calls
 */

/*
 * int pcanfd_set_init(int fd, struct pcanfd_init *pfdi)
 *
 *	Enable to initialize an opened device with bitrate (and data bitrate
 *	if the device is CAN-FD capable) specification.
 *
 * RETURN:
 *
 *	0 if the device has been correctly initialized,
 *	a negative (errno) code otherwise.
 */
int pcanfd_set_init(int fd, struct pcanfd_init *pfdi)
{
#ifdef DEBUG
	__fprintf(stddbg, "%s(fd=%d pfdi=%p)\n", __func__, fd, pfdi);
#endif
	if (!pfdi)
		return -EINVAL;

	pfdi->flags &= OFD_PCANFD_MASK;
	return -__errno_ioctl(fd, PCANFD_SET_INIT, pfdi);
}

/*
 * int pcanfd_get_init(int fd, struct pcanfd_init *pfdi)
 *
 *	Read the initialization settings of an opened device.
 *
 * RETURN:
 *
 *	0 if the device has been correctly initialized,
 *	a negative (errno) code otherwise.
 *
 *	-EINVAL		the pfdi argument is NULL.
 */
int pcanfd_get_init(int fd, struct pcanfd_init *pfdi)
{
#ifdef DEBUG
	__fprintf(stddbg, "%s(fd=%d pfdi=%p)\n", __func__, fd, pfdi);
#endif
	if (!pfdi)
		return -EINVAL;

	return -__errno_ioctl(fd, PCANFD_GET_INIT, pfdi);
}

/*
 * int pcanfd_reset(int fd, unsigned long flags)
 *
 *	Enable to reset things in the driver (see PCAND_RESET_xxx in pcanfd.h)
 *
 * RETURN:
 *
 *	0 if all reset operations were successful,
 *	a negative (errno) code otherwise.
 */
int pcanfd_reset(int fd, unsigned long flags)
{
#ifdef DEBUG
	__fprintf(stddbg, "%s(fd=%d flags=%xh)\n", __func__, fd, flags);
#endif
	return -__errno_ioctl(fd, PCANFD_RESET, flags);
}

/*
 * int pcanfd_get_state(int fd, struct pcanfd_state *pfds)
 *
 *	Read the state of a CAN device/channel.
 *
 * RETURN:
 *
 *	0 if the device has been correctly initialized,
 *	a negative (errno) code otherwise.
 */
int pcanfd_get_state(int fd, struct pcanfd_state *pfds)
{
#ifdef DEBUG
	__fprintf(stddbg, "%s(fd=%d pfds=%p)\n", __func__, fd, pfds);
#endif
	if (!pfds)
		return -EINVAL;

	return -__errno_ioctl(fd, PCANFD_GET_STATE, pfds);
}

/*
 * int pcanfd_add_filters(int fd, const struct pcanfd_msg_filters *pfl)
 *
 *	Add a message filters list into the device's message filters list.
 *
 * RETURN:
 *
 *	0 if the filters list has been correctly added,
 *	a negative (errno) code otherwise.
 *
 *	-EINVAL		the pf argument is NULL.
 */
int pcanfd_add_filters(int fd, const struct pcanfd_msg_filters *pfl)
{
#ifdef DEBUG
	__fprintf(stddbg, "%s(fd=%d)\n", __func__, fd);
#endif
	if (!pfl)
		return -EINVAL;

	return -__errno_ioctl(fd, PCANFD_ADD_FILTERS, pfl);
}

int pcanfd_add_filter(int fd, const struct pcanfd_msg_filter *pf)
{
	struct pcanfd_msg_filters_1 fl1;

#ifdef DEBUG
	__fprintf(stddbg, "%s(fd=%d)\n", __func__, fd);
#endif
	if (!pf)
		return -EINVAL;

	fl1.count = 1;
	memcpy(fl1.list, pf, sizeof(*pf));

	return -__errno_ioctl(fd, PCANFD_ADD_FILTERS, &fl1);
}

int pcanfd_add_filters_list(int fd, int count,
					const struct pcanfd_msg_filter *pf)
{
	int err = -EINVAL;

#ifdef DEBUG
	__fprintf(stddbg, "%s(fd=%d count=%d pf=%p)\n",
			__func__, fd, count, pf);
#endif
	if (!pf)
		return -EINVAL;

	if (count > 0) {
		struct pcanfd_msg_filters *pfl;

		pfl = malloc(sizeof(*pfl) + count * sizeof(*pf));
		if (!pfl) {
#ifdef DEBUG
			__fprintf(stddbg, "%s(): malloc failed\n", __func__);
#endif
			return -ENOMEM;
		}

		pfl->count = count;
		memcpy(pfl->list, pf, count * sizeof(*pf));
		err = -__errno_ioctl(fd, PCANFD_ADD_FILTERS, pfl);
		free(pfl);
	}

	return err;
}

/*
 * int pcanfd_get_filters(int fd, struct pcanfd_msg_filters *pfl)
 *
 *	Copy the message filters list from the device's message filters list.
 *
 * RETURN:
 *
 *	0 if the filters list has been correctly coppied,
 *	a negative (errno) code otherwise.
 *
 *	-EINVAL		the pfl argument is NULL.
 */
int pcanfd_get_filters(int fd, struct pcanfd_msg_filters *pfl)
{
#ifdef DEBUG
	__fprintf(stddbg, "%s(fd=%d pfl=%p)\n", __func__, fd, pfl);
#endif
	if (!pfl)
		return -EINVAL;

	return -__errno_ioctl(fd, PCANFD_GET_FILTERS, pfl);
}

/*
 * int pcanfd_get_filters_list(int fd, int count, struct pcanfd_msg_filter *pf)
 *
 *	Copy the message filters list from the device's message filters list.
 *
 * RETURN:
 *
 *	The number of filters copied into the given list.
 *	a negative (errno) code otherwise.
 */
int pcanfd_get_filters_list(int fd, int count, struct pcanfd_msg_filter *pf)
{
	int err = -EINVAL;

#ifdef DEBUG
	__fprintf(stddbg, "%s(fd=%d count=%d pf=%p)\n",
			__func__, fd, count, pf);
#endif
	if (!pf)
		return -EINVAL;

	if (count > 0) {
		struct pcanfd_msg_filters *pfl;

		pfl = malloc(sizeof(*pfl) + count * sizeof(*pf));
		if (!pfl) {
#ifdef DEBUG
			__fprintf(stddbg, "%s(): malloc failed\n", __func__);
#endif
			return -ENOMEM;
		}

		pfl->count = count;
		err = -__errno_ioctl(fd, PCANFD_GET_FILTERS, pfl);
		if (!err) {
			memcpy(pf, pfl->list, pfl->count * sizeof(*pf));
			err = pfl->count;
		}
		free(pfl);
	}

	return err;
}

/*
 * int pcanfd_del_filters(int fd)
 *
 *	Remove all the message filters from the device's messages filter list.
 *
 * RETURN:
 *
 *	0 if the device has been correctly initialized,
 *	a negative (errno) code otherwise.
 */
int pcanfd_del_filters(int fd)
{
#ifdef DEBUG
	__fprintf(stddbg, "%s(fd=%d)\n", __func__, fd);
#endif
	return -__errno_ioctl(fd, PCANFD_ADD_FILTERS, NULL);
}

/*
 * int pcanfd_send_msg(int fd, struct pcanfd_msg *pfdm)
 *
 *	Enable to write a message to send into the device output queue.
 *
 *	If no space is available in the device output queue, and if the
 *	device is opened in blocking mode, then the calling task goes to sleep,
 *	waiting for some space freed by the driver.
 *	If the device is opened in non-blocking mode, the task doesn't wait and
 *	-EWOULDBLOCK is returned instead.
 *
 * RETURN:
 *
 *	0 if the message has been correctly sent,
 *	a negative (errno) code otherwise.
 */
int pcanfd_send_msg(int fd, const struct pcanfd_msg *pfdm)
{
#ifdef DEBUG
	__fprintf(stddbg, "%s(fd=%d pfdm=%p)\n", __func__, fd, pfdm);
#endif
	if (!pfdm)
		return -EINVAL;

	return -__errno_ioctl(fd, PCANFD_SEND_MSG, pfdm);
}

/*
 * int pcanfd_send_msgs(int fd, struct pcanfd_msgs *pfdml)
 *
 *	Enables to send one or more CANFD messages to the output queue.
 *	User MUST setup pfdml->count to the number of struct pcanfd_msg
 *	items to copied from pfdml->list[] array.
 *
 *	If no space is available in the device output queue, and if the
 *	device is opened in blocking mode, then the calling task goes to sleep,
 *	waiting for space freed by the driver.
 *	If the device is opened in non-blocking mode, the task doesn't wait and
 *	-EWOULDBLOCK is returned instead.
 *
 * RETURN:
 *
 *	0 if at least one message has been correctly sent. In that case,
 *	  pfdml->count is set to the number of messages really copied into the
 *	  output queue.
 *
 *	a negative (errno) code otherwise.
 */
int pcanfd_send_msgs(int fd, struct pcanfd_msgs *pfdml)
{
#ifdef DEBUG
	__fprintf(stddbg, "%s(fd=%d pfdml=%p)\n", __func__, fd, pfdml);
#endif
	if (!pfdml)
		return -EINVAL;

	return -__errno_ioctl(fd, PCANFD_SEND_MSGS, pfdml);
}

/*
 * int pcanfd_send_msgs_list(int fd, int count, struct pcanfd_msg *pfdm)
 *
 * 	Enables to send one or more CANFD messages to the output queue.
 *
 *	If no space is available in the device output queue, and if the
 *	device is opened in blocking mode, then the calling task goes to sleep,
 *	waiting for space freed by the driver.
 *	If the device is opened in non-blocking mode, the task doesn't wait and
 *	-EWOULDBLOCK is returned instead.
 *
 * RETURN:
 *
 *	a positive number indicates how many messages have been written into
 *	the device output queue.
 *
 *	a negative (errno) code otherwise.
 */
int pcanfd_send_msgs_list(int fd, int count, const struct pcanfd_msg *pfdm)
{
	int err = -EINVAL;

#ifdef DEBUG
	__fprintf(stddbg, "%s(fd=%d count=%d pfdm=%p)\n",
				__func__, fd, count, pfdm);
#endif
	if (!pfdm)
		return -EINVAL;

	if (count > 0) {
		struct pcanfd_msgs *pml;

		pml = malloc(sizeof(*pml) + count * sizeof(*pfdm));
		if (!pml) {
#ifdef DEBUG
			__fprintf(stddbg, "%s(): malloc failed\n", __func__);
#endif
			return -ENOMEM;
		}

		pml->count = count;
		memcpy(pml->list, pfdm, count * sizeof(*pfdm));
		err = -__errno_ioctl(fd, PCANFD_SEND_MSGS, pml);
		if (!err)
			err = pml->count;

		free(pml);
	}

	return err;
}

/*
 * int pcanfd_recv_msg(int fd, struct pcanfd_msg *pfdm)
 *
 *	Enable to read one received message from the device input queue.
 *
 *	If no message are to be read from the device input queue, and if the
 *	device is opened in blocking mode, then the calling task goes to sleep,
 *	waiting for any incoming event from the driver.
 *	If the device is opened in non-blocking mode, the task doesn't wait and
 *	-EWOULDBLOCK is returned instead.
 * 
 * RETURN:
 *
 *	0 if a message has been correctly received,
 *	a negative (errno) code otherwise.
 */
int pcanfd_recv_msg(int fd, struct pcanfd_msg *pfdm)
{
#ifdef DEBUG
	__fprintf(stddbg, "%s(fd=%d pfdm=%p)\n", __func__, fd, pfdm);
#endif
	if (!pfdm)
		return -EINVAL;

	return -__errno_ioctl(fd, PCANFD_RECV_MSG, pfdm);
}

/*
 * int pcanfd_recv_msgs(int fd, struct pcanfd_msgs *pfdml)
 *
 *	Enables to read one or more CANFD messages from the input queue.
 *	User MUST setup pfdml->count to the number of struct pcanfd_msg
 *	items allocated in pfdml->list[] array.
 *
 *	If no message are to be read from the device input queue, and if the
 *	device is opened in blocking mode, then the calling task goes to sleep,
 *	waiting for any incoming event from the driver.
 *	If the device is opened in non-blocking mode, the task doesn't wait and
 *	-EWOULDBLOCK is returned instead.
 *
 * RETURN:
 *
 *	0 if at least one message has been correctly received. In that case,
 *	  pfdml->count is set to the number of messages really copied from the
 *	  input queue.
 *
 *	a negative (errno) code otherwise.
 */
int pcanfd_recv_msgs(int fd, struct pcanfd_msgs *pfdml)
{
#ifdef DEBUG
	__fprintf(stddbg, "%s(fd=%d pfdml=%p)\n", __func__, fd, pfdml);
#endif
	if (!pfdml)
		return -EINVAL;

	return -__errno_ioctl(fd, PCANFD_RECV_MSGS, pfdml);
}

/*
 * int pcanfd_recv_msgs_list(int fd, int count, struct pcanfd_msg *pm)
 *
 *	Enables to read one or more CANFD messages from the input queue.
 *	'pm' MUST be an address of a memory buffer large enough to store at
 *	least 'count' consecutive 'struct pcanfd_msg' objects.
 *
 *	If no message are to be read from the device input queue, and if the
 *	device is opened in blocking mode, then the calling task goes to sleep,
 *	waiting for any incoming event from the driver.
 *	If the device is opened in non-blocking mode, the task doesn't wait and
 *	-EWOULDBLOCK is returned instead.
 *
 * RETURN:
 *
 *	a positive number indicates how many messages have been read from the
 *	device input queue.
 *
 *	a negative (errno) code otherwise.
 */
int pcanfd_recv_msgs_list(int fd, int count, struct pcanfd_msg *pm)
{
	int err = -EINVAL;

#ifdef DEBUG
	__fprintf(stddbg, "%s(fd=%d count=%d pm=%p)\n",
			__func__, fd, count, pm);
#endif
	if (!pm)
		return -EINVAL;

	if (count > 0) {
		struct pcanfd_msgs *pml;

		pml = malloc(sizeof(*pml) + count * sizeof(*pm));
		if (!pml) {
#ifdef DEBUG
			__fprintf(stddbg, "%s(): malloc failed\n", __func__);
#endif
			return -ENOMEM;
		}

		pml->count = count;
		err = -__errno_ioctl(fd, PCANFD_RECV_MSGS, pml);
		if (!err) {
			memcpy(pm, pml->list, pml->count * sizeof(*pm));
			err = pml->count;
		}
		free(pml);
	}

	return err;
}


/*
 * int pcanfd_set_device_id(int fd, __u32 devid)
 *
 *	Set a "device id." to a CAN channel (when the corresponding hardware
 *	enables it) so that this CAN channel will be able to be always 
 *	identified by this device id. This is mainly uiseful for USB CAN
 *	interfaces that can be plugged and re-plugged to different USB sockets.
 *	By convention, the value 0xffffffff means "no device id.".
 *
 * RETURN:
 *
 *	O if setting a new device id. to the CAN channel succeeded.
 *
 *	a negative (errno) code otherwise.
 */
int pcanfd_set_device_id(int fd, __u32 devid)
{
	struct pcan_extra_params ep = {
		.nSubFunction = SF_SET_HCDEVICENO,
#ifndef USES_32BITS_DEVICENO
		.func.ucHCDeviceNo = (__u8 )devid,
#else
		.func.dwSerialNumber = devid,
#endif
	};

	return pcan_set_extra_params(fd, &ep);
}

/*
 * int pcanfd_get_device_id(int fd, __u32 *pdevid)
 *
 *	Get the "device id." previously set to a CAN channel or 0xffffffff
 *	if there wasn't any. See "pcanfd_set_device_id()" for more information
 *	about the utility of "device id."
 *
 * RETURN:
 *
 *	O if setting a new device id. to the CAN channel succeeded.
 *
 *	a negative (errno) code otherwise.
 */
int pcanfd_get_device_id(int fd, __u32 *pdevid)
{
	int err;
	struct pcan_extra_params ep = {
		.nSubFunction = SF_GET_HCDEVICENO,
	};

	if (!pdevid)
		return -EINVAL;

	err = pcan_set_extra_params(fd, &ep);
	if (!err)
#ifndef USES_32BITS_DEVICENO
		*pdevid = (__u32 )ep.func.ucHCDeviceNo;
#else
		*pdevid = (__u32 )ep.func.dwSerialNumber;
#endif

	return err;
}

/*
 * int pcanfd_get_available_clocks(int fd, struct pcanfd_available_clocks *pac)
 *
 *	Read clock values available for the CAN[-FD] device.
 *	User MUST setup pac->count to the number of
 *	struct pcanfd_available_clock items allocated in pac->list[] array.
 *
 *	The driver fills pac->list[] with all the clock values that can be
 *	selected in the device and set this number into pac->count (pac->list[0]
 *	always contains the default clock).
 *
 * RETURN:
 *
 *	0 if success (pac->count equals the count of available clocks, and
 *	  pac->list[0] is the default clock value)
 *
 *	a negative (errno) code otherwise.
 */
int pcanfd_get_available_clocks(int fd, struct pcanfd_available_clocks *pac)
{
#ifdef DEBUG
	__fprintf(stddbg, "%s(fd=%d pac=%p)\n", __func__, fd, pac);
#endif
	if (!pac)
		return -EINVAL;

	return -__errno_ioctl(fd, PCANFD_GET_AVAILABLE_CLOCKS, pac);
}

/*
 * int pcanfd_get_bittiming_ranges(int fd,
 *                                 struct pcanfd_bittiming_ranges *pbtr)
 *
 *	Read bittiming ranges available in the CAN-FD device.
 *	User MUST setup pbtr->count to the number of
 *	struct pcanfd_bittiming_range items allocated in pbtr->list[] array.
 *
 *	The driver fills pbtr->list[0] with the [nominal] bitrate bittiming
 *	ranges.	If the device is CAN-FD capable, then the driver also fills
 *	pbtr->list[1] with the data bitrate bittiming ranges. The driver sets
 *	pbtr->count accordingly.
 *
 * RETURN:
 *
 *	0 if at least the [nominal] bitrate bittiming ranges has been copied.
 *	  For CAN 2.0 devices, pbtr->count is set to 1, while it is set to 2
 *	  for CAN-FD capable devices.
 *
 *	a negative (errno) code otherwise.
 */
int pcanfd_get_bittiming_ranges(int fd, struct pcanfd_bittiming_ranges *pbtr)
{
#ifdef DEBUG
	__fprintf(stddbg, "%s(fd=%d pbtr=%p)\n", __func__, fd, pbtr);
#endif
	if (!pbtr)
		return -EINVAL;

	return -__errno_ioctl(fd, PCANFD_GET_BITTIMING_RANGES, pbtr);
}

/*
 * int pcanfd_is_canfd_capable(int fd)
 *
 *	Helper function useful to know whether a CAN device is or is not CAN-FD
 *	capable.
 */
int pcanfd_is_canfd_capable(int fd)
{
	struct __array_of_struct(pcanfd_bittiming_range, 2) btrs = {
		.count = 2,
	};

	return	(!__errno_ioctl(fd, PCANFD_GET_BITTIMING_RANGES, &btrs)) &&
		(btrs.count == 2);
}

/*
 * int pcanfd_get_option(int fd, int name, void *value, int size)
 *
 *	Get an option value from the opened channel.
 *
 * RETURN:
 *
 *	a negative (errno) code in case of error
 *
 *	a count of bytes <= size in case of success
 *
 *	a count of bytes > size in case of to small value buffer: in this case,
 *	this count is the size the buffer should be, to successfully get this
 *	option.
 */
int pcanfd_get_option(int fd, int name, void *value, int size)
{
	struct pcanfd_option opt = {
		.name = name,
		.value = value,
		.size = size,
	};

	int err = -__errno_ioctl(fd, PCANFD_GET_OPTION, &opt);
	if (!err || err == -ENOSPC)
		return opt.size;

	return err;
}

/*
 * int pcanfd_set_option(int fd, int name, void *value, int size)
 *
 *	Set an option value to the opened channel.
 *
 * RETURN:
 *
 *	0 if the option has been set.
 *
 *	a negative (errno) code otherwise.
 */
int pcanfd_set_option(int fd, int name, void *value, int size)
{
	struct pcanfd_option opt = {
		.name = name,
		.value = value,
		.size = size,
	};

	return -__errno_ioctl(fd, PCANFD_SET_OPTION, &opt);
}

/*
 * Old CAN2.0 API entry points with modern design.
 */

/*
 * int pcan_init(int fd, const struct pcan_init *pi)
 *
 *	Enable to initialize an opened device with bitrate specification.
 *
 * RETURN:
 *
 *	0 if the device has been correctly initialized,
 *	a negative (errno) code otherwise.
 */
int pcan_init(int fd, const struct pcan_init *pi)
{
#ifdef DEBUG
	__fprintf(stddbg, "%s(fd=%d pi=%p)\n", __func__, fd, pi);
#endif
	if (!pi)
		return -EINVAL;

	return -__errno_ioctl(fd, PCAN_INIT, pi);
}

/*
 * int pcan_read_msg(int fd, struct pcan_rd_msg *prdm)
 *
 *	Enable to read one received message from the device input queue.
 *
 *	If no message are to be read from the device input queue, and if the
 *	device is opened in blocking mode, then the calling task goes to sleep,
 *	waiting for any incoming event from the driver.
 *	If the device is opened in non-blocking mode, the task doesn't wait and
 *	-EWOULDBLOCK is returned instead.
 * 
 * RETURN:
 *
 *	0 if a message has been correctly received,
 *	a negative (errno) code otherwise.
 */
int pcan_read_msg(int fd, struct pcan_rd_msg *prdm)
{
#ifdef DEBUG
	__fprintf(stddbg, "%s(fd=%d prdm=%p)\n", __func__, fd, prdm);
#endif
	if (!prdm)
		return -EINVAL;

	return -__errno_ioctl(fd, PCAN_READ_MSG, prdm);
}

/*
 * int pcan_write_msg(int fd, const struct pcan_msg *pm)
 *
 *	Enable to write a message to send into the device output queue.
 *
 *	If no space is available in the device output queue, and if the
 *	device is opened in blocking mode, then the calling task goes to sleep,
 *	waiting for some space freed by the driver.
 *	If the device is opened in non-blocking mode, the task doesn't wait and
 *	-EWOULDBLOCK is returned instead.
 *
 * RETURN:
 *
 *	0 if the message has been correctly sent,
 *	a negative (errno) code otherwise.
 */
int pcan_write_msg(int fd, const struct pcan_msg *pm)
{
#ifdef DEBUG
	__fprintf(stddbg, "%s(fd=%d pm=%p)\n", __func__, fd, pm);
#endif
	if (!pm)
		return -EINVAL;

	return -__errno_ioctl(fd, PCAN_WRITE_MSG, pm);
}

/*
 * int pcan_get_status(int fd, struct pcan_status *ps)
 *
 *	Get a struct pcan_status object from the driver, which gives:
 *
 *	wErrorFlag	CAN_ERR_xxx error flags
 *	nLastError	last error code set by the driver
 *
 *	Note that calling this function clears the error flags and the last
 *	error from the driver, for that device.
 */
int pcan_get_status(int fd, struct pcan_status *ps)
{
#ifdef DEBUG
	__fprintf(stddbg, "%s(fd=%d ps=%p)\n", __func__, fd, ps);
#endif
	if (!ps)
		return -EINVAL;

	return -__errno_ioctl(fd, PCAN_GET_STATUS, ps);
}

/*
 * int pcan_get_ext_status(int fd, struct pcan_ext_status *ps)
 *
 *	Get a struct pcan_ext_status object from the driver, which gives:
 *
 *	wErrorFlag	CAN_ERR_xxx error flags
 *	nLastError	last error code set by the driver
 *	nPendingReads	Count of CANFD msgs waiting to be read
 *	nPendingWrites	Count of CANFD message waiting to be sent
 *
 *	Note that calling this function clears the error flags and the last
 *	error from the driver, for that device.
 */
int pcan_get_ext_status(int fd, struct pcan_ext_status *ps)
{
#ifdef DEBUG
	__fprintf(stddbg, "%s(fd=%d ps=%p)\n", __func__, fd, ps);
#endif
	if (!ps)
		return -EINVAL;

	return -__errno_ioctl(fd, PCAN_GET_EXT_STATUS, ps);
}

/*
 * int pcan_get_diag(int fd, struct pcan_diag *pd)
 *
 *	Get a struct pcan_diag object from the driver.
 *
 *	Note that, unlike pcan_get_status(), calling this function DOES NOT
 *	clear the error flags nor the last error from the driver, for that
 *	device.
 */
int pcan_get_diag(int fd, struct pcan_diag *pd)
{
#ifdef DEBUG
	__fprintf(stddbg, "%s(fd=%d pd=%p)\n", __func__, fd, pd);
#endif
	if (!pd)
		return -EINVAL;

	return -__errno_ioctl(fd, PCAN_DIAG, pd);
}

/*
 * int pcan_get_btr0btr1(int fd, struct pcan_btr0btr1 *pb)
 *
 *	Get a bitrate corresponding BTR0BTR1 8 MHz SJA1000 register value.
 */
int pcan_get_btr0btr1(int fd, struct pcan_btr0btr1 *pb)
{
#ifdef DEBUG
	__fprintf(stddbg, "%s(fd=%d pb=%p)\n", __func__, fd, pb);
#endif
	if (!pb)
		return -EINVAL;

	return -__errno_ioctl(fd, PCAN_BTR0BTR1, pb);
}

/*
 * int pcan_set_msg_filter(int fd, const struct pcan_msg_filter *pf)
 *
 *	Set message filter to a device.
 */
int pcan_set_msg_filter(int fd, const struct pcan_msg_filter *pf)
{
#ifdef DEBUG
	__fprintf(stddbg, "%s(fd=%d pf=%p)\n", __func__, fd, pf);
#endif
	if (!pf)
		return -EINVAL;

	return -__errno_ioctl(fd, PCAN_MSG_FILTER, pf);
}

/*
 * int pcan_set_extra_params(int fd, struct pcan_extra_params *pe)
 *
 *	Set/get extra parameters to a device (if supported).
 */
int pcan_set_extra_params(int fd, struct pcan_extra_params *pe)
{
#ifdef DEBUG
	__fprintf(stddbg, "%s(fd=%d pe=%p)\n", __func__, fd, pe);
#endif
	if (!pe)
		return -EINVAL;

	return -__errno_ioctl(fd, PCAN_EXTRA_PARAMS, pe);
}

/*
 * Level-2 API
 */

/*
 * int pcanfd_open(const char *dev_pcan, __u32 flags, ...)
 *
 *	Open a CAN/CANFD device with initialization settings.
 *
 *	dev_pcan	Character string of the name of the device node
 *			representing the CAN/CANFD channel
 *	flags		Bitmask giving option and information settings.
 *			In particular, some of these flags define which
 *			arguments are coming next, respecting the following
 *			order:
 *
 *			OFD_BITRATE	argument next to flag is a numeric value
 *					specifying the [nominal] bitrate in
 *					bpsi (except if OFD_BTR0BTR1 is used).
 *			OFD_DBITRATE	next argument is a numeric value
 *					specifying the data bitrate in bps
 *					(except if OFD_BTR0BTR1 is used), in
 *					case of opening a CANFD device.
 *			OFD_SAMPLEPT	argument next to bitrate is the
 *					minimum sample point value requested for
 *					the [nominal] bitrate expressed in o/oo.
 *			OFD_BTR0BTR1	Bitrate aguments are specified using the
 *					BTR0BTR1 format for SJA1000 CAN
 *					controller (old-school).
 *			OFD_BRPTSEGSJW	Bitrate is specified by the next 4
 *					arguments in the order BRP, TSEG1, TSEG2
 *					and SJW.
 *			OFD_CLOCK	next argument is a numeric value
 *					specifying a clock frequency in Hz to
 *					select (if possible), instead of the
 *					default one.
 *					
 *			See libpcanfd.h for the definition of the other bits.
 *
 *			Note that opening a device with PCANFD_INIT_LISTEN_ONLY
 *			leads to open the system device in O_RDONLY mode too.
 *	...		If OFD_BITRATE flag is set, the 1st argument must be a
 *			numeric value used to specify the nominal bitrate,
 *			in bps. If OFD_BTR0BTR1 flag is set too, then this
 *			numeric value is instead a 16-bits value following the
 *			SJA1000	BTR registers format (500 kps = 0x001c, for ex.)
 *
 *			If OFD_DBITRATE flag is set, the next argument must be a
 *			numeric value used to specify the data bitrate, in bps.
 *			If OFD_BTR0BTR1 flag is set too, then this numeric value
 *			is instead a 16-bits value following the SJA1000 BTR
 *			registers format.
 *
 *			If OFD_SAMPLEPT is set, then next argument to any 
 *			bitrate arg must be a value less than 10000, acting as
 *			an acceptable sample point value. If not set, the
 *			default (CiA recommended) values are internaly used.
 *
 *			If OFD_CLOCK flag is set, the next argument must be a
 *			numeric value used to specify the clock frequency, in
 *			Hz.
 *
 * EXAMPLES:
 *
 *	1/ Opens a CAN 2.0 channel for reading and writing @500k bitrate,
 *	   accepting extended format messages too:
 *
 *	   int fd = pcanfd_open("/dev/pcan0", OFD_BITRATE, 500000);
 *
 *	   - or -
 *
 *	   int fd = pcanfd_open("/dev/pcan0", OFD_BITRATE|OFD_BTR0BTR1, 0x1c);
 *
 *	2/ Opens a CAN-FD ISO channel for reading only @1Mb nominal and 2Mb
 *	   data bitrates, supporting only standard messages:
 *
 *	   int fd = pcanfd_open("/dev/pcan0",
 *	                        OFD_BITRATE|OFD_DBITRATE,
 *	                        PCANFD_INIT_LISTEN_ONLY|
 *	                        PCANFD_INIT_STD_MSG_ONLY|
 *	                        PCANFD_INIT_FD,
 *	                        1000000, 2000000);
 *
 *	3/ [old_timer special] Same as 1/ but with old-school BTR0BTR1 bitrate
 *	   specification:
 *
 *	   int fd = pcanfd_open("/dev/pcan0",
 *				OFD_BITRATE|OFD_BTR0BTR1, 0x001c);
 *
 *	4/ Quite the same as 2/ but with 80 MHz Clock specification:
 *
 *	   int fd = pcanfd_open("/dev/pcan0",
 *	                        OFD_BITRATE|OFD_DBITRATE|OFD_CLOCK,
 *	                        1000000, 2000000, 80000000);
 *
 * RETURN:
 *
 *	A positive integer if open and initialization have succeeded,
 *	a negative value otherwise (-errno).
 */
int pcanfd_open(const char *dev_pcan, __u32 flags, ...)
{
	int fd, o_flags;
	struct pcanfd_init initfd;
	va_list args;

#ifdef DEBUG
	__fprintf(stddbg, "%s(\"%s\", flags=%08x)\n",
				__func__, dev_pcan, flags);
#endif

	o_flags = (flags & PCANFD_INIT_LISTEN_ONLY) ? O_RDONLY : O_RDWR;

	if (flags & OFD_NONBLOCKING)
		o_flags |= O_NONBLOCK;

	va_start(args, flags);

	memset(&initfd, '\0', sizeof(initfd));

	if (flags & OFD_BITRATE) {

		if (flags & OFD_BTR0BTR1) {
			__u16 btr0btr1 = (__u16 )va_arg(args, int);
			pcanfd_to_bittiming(btr0btr1, &initfd.nominal);
			pcanfd_update_sp(&initfd.nominal);
			initfd.clock_Hz = 8000000;
			flags &= ~OFD_CLOCKHZ;

		} else if (flags & OFD_BRPTSEGSJW) {
			initfd.nominal.brp = va_arg(args, __u32);
			initfd.nominal.tseg1 = va_arg(args, __u32);
			initfd.nominal.tseg2 = va_arg(args, __u32);
			initfd.nominal.sjw = va_arg(args, __u32);

		} else {
			initfd.nominal.bitrate = va_arg(args, __u32);
			if (flags & OFD_SAMPLEPT)
				initfd.nominal.sample_point =
						va_arg(args, __u32);
		}
	}

	if (flags & OFD_DBITRATE) {

		flags |= PCANFD_INIT_FD;

		if (flags & OFD_BTR0BTR1) {
			__u16 btr0btr1 = (__u16 )va_arg(args, int);
			pcanfd_to_bittiming(btr0btr1, &initfd.data);
			pcanfd_update_sp(&initfd.data);
			initfd.clock_Hz = 8000000;
			flags &= ~OFD_CLOCKHZ;

		} else if (flags & OFD_BRPTSEGSJW) {
			initfd.data.brp = va_arg(args, __u32);
			initfd.data.tseg1 = va_arg(args, __u32);
			initfd.data.tseg2 = va_arg(args, __u32);
			initfd.data.sjw = va_arg(args, __u32);

		} else {
			initfd.data.bitrate = va_arg(args, __u32);
			if (flags & OFD_SAMPLEPT)
				initfd.data.sample_point = va_arg(args, __u32);
		}
	}

	if (flags & OFD_CLOCKHZ)
		initfd.clock_Hz = va_arg(args, __u32);

	va_end(args);

#ifdef PCANFD_OPEN_USES_ACCMODE
	if (flags & (OFD_BITRATE|OFD_DBITRATE|OFD_CLOCKHZ)) {

		/* Since new bitimings are given, open() won't initialized the
		 * device but ioctl(SET_INIT) will.
		 *
		 * Using here a hack present in the Linux kernel for floppy
		 * (see block/fops.c):
		 *
		 * O_ACCMODE(3) prevents from using read() and write() system
		 * calls but allow FMODE_WRITE_IOCTL special mode.
		 *
		 * The driver handles that special meaning and doesn't configure
		 * the device at all.
		 */
		o_flags |= O_ACCMODE;	/* 0x03 */
	}
#endif

	fd = __open(dev_pcan, o_flags);
	if (fd >= 0) {

		/* if ioctl(SET_INIT) should be called next */
		if (flags & (PCANFD_INIT_LISTEN_ONLY|
		             OFD_BITRATE|OFD_DBITRATE|OFD_CLOCKHZ)) {
			int err;

			initfd.flags = flags & OFD_PCANFD_MASK;

			err = pcanfd_set_init(fd, &initfd);
			if (err) {
#ifdef DEBUG
				__fprintf(stddbg, "%s(%s): pcanfd_set_init() "
						  "failed (err %d)\n",
					__func__, dev_pcan, err);
#endif
				__close(fd);

				fd = err;
			}
		}

	}

	return fd;
}

/*
 * int pcanfd_close(int fd)
 *
 *	Close any opened descriptor.
 *
 * RETURN:
 *
 *	-1
 */
int pcanfd_close(int fd)
{
	__close(fd);
	return -1;
}

#ifdef PCANFD_OLD_STYLE_API

#include "libpcan.c"

/*
 * Old (and ugly) style (kept for compatibility)
 */
static int __errno_to_can_err(int _errno)
{
	switch (_errno) {

	case EBADF:
		return CAN_ERR_ILLHW;
	case ENETDOWN:
		return CAN_ERR_BUSOFF;
	}

	/* default is to give errno code with the sys error mask */
	return (CAN_ERR_ERRNO_MASK|_errno);
}

/*
 * DWORD CAN_InitFD(HANDLE hHandle, TPCANFDInit *pInit)
 *
 *	hHandle		CANFD channel handle returned by CAN_Open()
 *
 * RETURN:
 *
 *	CAN_ERR_OK(0)			Success
 *	CAN_ERR_ILLHW			NULL hHandle
 *	CAN_ERR_ERRNO_MASK | errno	System error (errno in lowest 30 bits)
 */
DWORD CAN_InitFD(HANDLE hHandle, TPCANFDInit *pInit)
{
	struct pcan_handle *desc = (struct pcan_handle *)hHandle;

	if (!desc)
		return __errno_to_can_err(EBADF);

	return __errno_to_can_err(-pcanfd_set_init(desc->fd, pInit));
}

/*
 * DWORD CAN_ReadFD(HANDLE hHandle, TPCANMsgFD *pMsgBuff)
 *
 *	hHandle		CANFD channel handle returned by CAN_Open()
 *
 * RETURN:
 *
 *	CAN_ERR_OK(0)			Success
 *	CAN_ERR_ILLHW			NULL hHandle
 *	CAN_ERR_ERRNO_MASK | errno	System error (errno in lowest 30 bits)
 */
DWORD CAN_ReadFD(HANDLE hHandle, TPCANMsgFD *pMsgBuff)
{
	struct pcan_handle *desc = (struct pcan_handle *)hHandle;

	if (!desc)
		return __errno_to_can_err(EBADF);

	return __errno_to_can_err(-pcanfd_recv_msg(desc->fd, pMsgBuff));
}

/*
 * DWORD CAN_WriteFD(HANDLE hHandle, TPCANMsgFD *pMsgBuff)
 *
 *	hHandle		CANFD channel handle returned by CAN_Open()
 *
 * RETURN:
 *
 *	CAN_ERR_OK(0)			Success
 *	CAN_ERR_ILLHW			NULL hHandle
 *	CAN_ERR_ERRNO_MASK | errno	System error (errno in lowest 30 bits)
 */
DWORD CAN_WriteFD(HANDLE hHandle, TPCANMsgFD *pMsgBuff)
{
	struct pcan_handle *desc = (struct pcan_handle *)hHandle;

	if (!desc)
		return __errno_to_can_err(EBADF);

	return __errno_to_can_err(-pcanfd_send_msg(desc->fd, pMsgBuff));
}

/*
 * Linux old-style FD API
 */

/*
 * DWORD LINUX_CAN_ReadFD_Timeout(HANDLE hHandle, TPCANMsgFD* pMsgBuff,
 *				int nMicroSeconds)
 *
 *	hHandle		CANFD channel handle returned by CAN_Open()
 *
 * RETURN:
 *
 *	CAN_ERR_OK(0)			Success
 *	CAN_ERR_ILLHW			NULL hHandle
 *	CAN_ERR_QRCVEMPTY		No message to read from the input queue
 *	CAN_ERR_ERRNO_MASK | errno	System error (errno in lowest 30 bits)
 */
DWORD LINUX_CAN_ReadFD_Timeout(HANDLE hHandle, TPCANMsgFD* pMsgBuff,
						int nMicroSeconds)
{
	struct pcan_handle *desc = (struct pcan_handle *)hHandle;
	struct timeval tv;
	fd_set fds_read;
	int err;

	if (nMicroSeconds < 0)
		return CAN_ReadFD(hHandle, pMsgBuff);

	if (!desc)
		return __errno_to_can_err(EBADF);

	/* calculate timeout values */
	tv.tv_sec  = nMicroSeconds / 1000000L;
	tv.tv_usec = nMicroSeconds % 1000000L;

	FD_ZERO(&fds_read);
	FD_SET(desc->fd, &fds_read);

	/* wait until timeout or a message is ready to read */
	err = select(desc->fd + 1, &fds_read, NULL, NULL, &tv);

	/* the only one file descriptor is ready for read */
	if (err > 0)
		return CAN_ReadFD(hHandle, pMsgBuff);

	/* nothing is ready, timeout occured */
	if (!err)
		return CAN_ERR_QRCVEMPTY;

	/* anything else is an error */
	return __errno_to_can_err(errno);
}

/*
 * DWORD LINUX_CAN_WriteFD_Timeout(HANDLE hHandle, TPCANMsgFD* pMsgBuff,
 *						int nMicroSeconds)
 *
 *	hHandle		CANFD channel handle returned by CAN_Open()
 *
 * RETURN:
 *
 *	CAN_ERR_OK(0)			Success
 *	CAN_ERR_ILLHW			NULL hHandle
 *	CAN_ERR_QXMTFULL		No more room in the output queue
 *	CAN_ERR_ERRNO_MASK | errno	System error (errno in lowest 30 bits)
 */
DWORD LINUX_CAN_WriteFD_Timeout(HANDLE hHandle, TPCANMsgFD* pMsgBuff,
						int nMicroSeconds)
{
	struct pcan_handle *desc = (struct pcan_handle *)hHandle;
	struct timeval tv;
	fd_set fds_write;
	int err;

	if (nMicroSeconds < 0)
		return CAN_WriteFD(hHandle, pMsgBuff);

	if (!desc)
		return __errno_to_can_err(EBADF);

	/* calculate timeout values */
	tv.tv_sec  = nMicroSeconds / 1000000L;
	tv.tv_usec = nMicroSeconds % 1000000L;

	FD_ZERO(&fds_write);
	FD_SET(desc->fd, &fds_write);

	/* wait until timeout or a message is ready to get written */
	err = select(desc->fd + 1,  NULL, &fds_write, NULL, &tv);

	/* the only one file descriptor is ready for write */
	if (err > 0)
		return CAN_WriteFD(hHandle, pMsgBuff);

	/* nothing is free, timeout occured */
	if (!err)
		return CAN_ERR_QXMTFULL;

	/* anything else is an error */
	return __errno_to_can_err(errno);
}
#endif /* PCANFD_OLD_STYLE_API */
