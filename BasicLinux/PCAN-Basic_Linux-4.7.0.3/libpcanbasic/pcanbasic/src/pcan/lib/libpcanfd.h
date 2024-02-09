/* SPDX-License-Identifier: LGPL-2.1-only */
/*
 * libpcanfd.h - common header to access the functions within libpcanfd.so.x.x.
 *
 * Copyright (C) 2015-2020  PEAK System-Technik GmbH <www.peak-system.com>
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
 * Contact: <linux@peak-system.com>
 * Author:  Stephane Grosjean <s.grosjean@peak-system.com>
 */
#ifndef __LIBPCANFD_H__
#define __LIBPCANFD_H__

#ifdef PCANFD_OLD_STYLE_API
#include <libpcan.h>
#endif

#include <pcanfd.h>

/* CANAPI4 error codes extension */
#define CAN_ERR_ILLHW		0x1400	/* Hardware handle is invalid */
#define CAN_ERR_BUSWARNING	CAN_ERR_BUSHEAVY
#define CAN_ERR_NODRIVER	CAN_ERR_NOVXD
#define CAN_ERR_BUSPASSIVE	0x40000	/* Bus error: CAN is error passive */

#ifdef CAN_ERR_ANYBUSERR
#undef CAN_ERR_ANYBUSERR
#endif

#define CAN_ERR_ANYBUSERR	(CAN_ERR_BUSWARNING | CAN_ERR_BUSPASSIVE | \
				 CAN_ERR_BUSOFF)

/* just like in CANAPI4 for Windows, this mask is used to save system values,
 * that is, errno values under Linux */
#define CAN_ERR_ERRNO_MASK	0xc0000000

/* level-2 API pcanfd_open() additional flags to PCANFD_INIT_xxx flags */
#define OFD_BITRATE		0x80000000	/* (nominal) bitrate */
#define OFD_DBITRATE		0x40000000	/* data bitrate spec */
#define OFD_BTR0BTR1		0x20000000	/* BTR0BTR1 format */
#define OFD_BRPTSEGSJW		0x10000000	/* BRP,TSEG1,TSEG2,SJW */
#define OFD_CLOCKHZ		0x08000000	/* Clock given as next arg */
#define OFD_NONBLOCKING		0x04000000	/* open in non-blocking mode */
#define OFD_SAMPLEPT		0x02000000	/* bitrate sample point */
#define OFD_PCANFD_MASK		(~0xff000000)	/* libpcanfd private bits */

#ifdef PCANFD_OLD_STYLE_API
typedef struct pcanfd_init TPCANFDInit;
typedef struct pcanfd_msg TPCANMsgFD;
#endif

/*
 * new API entry points
 */
#ifdef __cplusplus
extern "C" {
#endif

/*
 * TPCANRdMsg *pcanfd_to_msg(TPCANRdMsg *msg, const struct pcanfd_msg *pf)
 *
 *	Convert new struct pcanfd_msg to old TPCANRdMsg representation.
 */
TPCANRdMsg *pcanfd_to_msg(TPCANRdMsg *msg, const struct pcanfd_msg *pf);

/*
 * struct pcanfd_msg *pcanmsg_to_fd(struct pcanfd_msg *pf,
 *						const TPCANRdMsg *msg)
 *
 *	Convert old TPCANRdMsg to new struct pcanfd_msg representation.
 */
struct pcanfd_msg *pcanmsg_to_fd(struct pcanfd_msg *pf, const TPCANRdMsg *msg);

/* level-1 API */

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
int pcanfd_set_init(int fd, struct pcanfd_init *pfdi);

/*
 * int pcanfd_get_init(int fd, struct pcanfd_init *pfdi)
 *
 *	Read the initialization settings of an opened device.
 *
 * RETURN:
 *
 *	0 if the device has been correctly initialized,
 *	a negative (errno) code otherwise.
 */
int pcanfd_get_init(int fd, struct pcanfd_init *pfdi);

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
int pcanfd_reset(int fd, unsigned long flags);

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
int pcanfd_get_state(int fd, struct pcanfd_state *pfds);

/*
 * int pcanfd_add_filters(int fd, struct pcanfd_msg_filters *pfl)
 * int pcanfd_add_filter(int fd, struct pcanfd_msg_filter *pf)
 * int pcanfd_add_filters_list(int fd, int count, struct pcanfd_msg_filter *pf)
 *
 *	Add a message filters list into the device's message filters list.
 *
 * RETURN:
 *
 *	0 if the filters list has been correctly added,
 *	a negative (errno) code otherwise.
 *
 *	-EINVAL		the pfl argument is NULL.
 */
int pcanfd_add_filters(int fd, const struct pcanfd_msg_filters *pfl);
int pcanfd_add_filter(int fd, const struct pcanfd_msg_filter *pf);
int pcanfd_add_filters_list(int fd, int count,
					const struct pcanfd_msg_filter *pf);

/*
 * int pcanfd_get_filters(int fd, struct pcanfd_msg_filters *pfl)
 * int pcanfd_get_filters_list(int fd, int count, struct pcanfd_msg_filter *pf)
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
int pcanfd_get_filters(int fd, struct pcanfd_msg_filters *pfl);
int pcanfd_get_filters_list(int fd, int count, struct pcanfd_msg_filter *pf);

/*
 * int pcanfd_del_filters(int fd);
 *
 *	Remove all the message filters from the device's messages filter list.
 *
 * RETURN:
 *
 *	0 if the filter has been correctly sent,
 *	a negative (errno) code otherwise.
 */
int pcanfd_del_filters(int fd);

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
 *	a negative (errno) code otherwise:
 *	
 *	-EWOULDBLOCK
 *	-EAGAIN		when device is opened in non-blocking mode, this error
 *	                says that the driver Tx fifo is full.
 *	-ETIMEDOUT	when device is opened in blocking mode, this error
 *	                says that the driver Tx fifo is full and the task does
 *	                not want to wait much more than it did.
 */
int pcanfd_send_msg(int fd, const struct pcanfd_msg *pfdm);

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
 *	0 if the messages have been correctly sent. In that case,
 *	  pfdml->count is set to the number of messages really copied into the
 *	  output queue.
 *
 *	a negative (errno) code otherwise.
 */
int pcanfd_send_msgs(int fd, struct pcanfd_msgs *pfdm);

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
int pcanfd_send_msgs_list(int fd, int count, const struct pcanfd_msg *pfdm);

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
int pcanfd_recv_msg(int fd, struct pcanfd_msg *pfdm);

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
int pcanfd_recv_msgs(int fd, struct pcanfd_msgs *pfdm);

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
int pcanfd_recv_msgs_list(int fd, int count, struct pcanfd_msg *pm);

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
int pcanfd_set_device_id(int fd, __u32 devid);

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
int pcanfd_get_device_id(int fd, __u32 *pdevid);

/*
 * int pcanfd_get_available_clocks(int fd, struct pcanfd_available_clocks *pac)
 *
 *	Read clock values available for the CAN-FD device.
 *	User MUST setup pac->count to the number of
 *	struct pcanfd_available_clock items allocated in pac->list[] array.
 *
 *	The driver fills list[] with all the clock values that can be selected
 *	in the device. list[0] always contains the default clock.
 *
 * RETURN:
 *
 *	0 if success (list[0] is the default clock value)
 *
 *	a negative (errno) code otherwise.
 */
int pcanfd_get_available_clocks(int fd, struct pcanfd_available_clocks *pac);

/*
 * int pcanfd_get_bittiming_ranges(int fd,
 *                                 struct pcanfd_bittiming_ranges *pbtr)
 *
 *	Read bittiming ranges available in the CAN-FD device.
 *	User MUST setup pbtr->count to the number of
 *	struct pcanfd_bittiming_range items allocated in pbtr->list[] array.
 *
 *	The driver fills list[0] with the [nominal] bitrate bittiming ranges.
 *	If the device is CAN-FD capable, then the driver also fills list[1] with
 *	the data bitrate bittiming ranges.
 *
 * RETURN:
 *
 *	0 if at least the [nominal] bitrate bittiming ranges has been copiedr.
 *	  For CAN 2.0 devices, pbtr->count is set to 1, while
 *	  pbtr->count is set to 2 for CAN-FD capable devices.
 *
 *	a negative (errno) code otherwise.
 */
int pcanfd_get_bittiming_ranges(int fd, struct pcanfd_bittiming_ranges *pbtr);

/*
 * int pcanfd_is_canfd_capable(int fd)
 *
 *	Helper function useful to know whether a CAN device is or is not CAN-FD
 *	capable.
 */
int pcanfd_is_canfd_capable(int fd);

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
int pcanfd_get_option(int fd, int name, void *value, int size);

/*
 * int pcanfd_set_option(int fd, int name, const void *value, int size)
 *
 *	Set an option value to the opened channel.
 *
 * RETURN:
 *
 *	0 if the option has been set.
 *
 *	a negative (errno) code otherwise.
 */
int pcanfd_set_option(int fd, int name, void *value, int size);

/*
 * Old CAN2.0 API entry points with modern design.
 */

/*
 * int pcan_init(int fd, struct pcan_init *pi)
 *
 *	Enable to initialize an opened device with bitrate specification.
 *
 * RETURN:
 *
 *	0 if the device has been correctly initialized,
 *	a negative (errno) code otherwise.
 */
int pcan_init(int fd, const struct pcan_init *pi);

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
int pcan_read_msg(int fd, struct pcan_rd_msg *prdm);

/*
 * int pcan_write_msg(int fd, struct pcan_msg *pm)
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
int pcan_write_msg(int fd, const struct pcan_msg *pm);

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
int pcan_get_status(int fd, struct pcan_status *ps);

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
int pcan_get_ext_status(int fd, struct pcan_ext_status *ps);

/*
 * int pcan_get_diag(int fd, struct pcan_diag *pd)
 *
 *	Get a struct pcan_diag object from the driver.
 *
 *	Note that, unlike pcan_get_status(), calling this function DOES NOT
 *	clear the error flags nor the last error from the driver, for that
 *	device.
 */
int pcan_get_diag(int fd, struct pcan_diag *pd);

/*
 * int pcan_get_btr0btr1(int fd, struct pcan_btr0btr1 *pb)
 *
 *	Get a bitrate corresponding BTR0BTR1 8 MHz SJA1000 register value.
 */
int pcan_get_btr0btr1(int fd, struct pcan_btr0btr1 *pb);

/*
 * int pcan_set_msg_filter(int fd, struct pcan_msg_filter *pf)
 *
 *	Set message filter to a device.
 */
int pcan_set_msg_filter(int fd, const struct pcan_msg_filter *pf);

/*
 * int pcan_set_extra_params(int fd, struct pcan_extra_params *pe)
 *
 *	Set/get extra parameters to a device (if supported).
 */
int pcan_set_extra_params(int fd, struct pcan_extra_params *pe);

/*
 * level-2 API
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
 *					bps (except if OFD_BTR0BTR1 is used).
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
 *	2/ Opens a CAN-FD ISO channel for reading only @1Mb nominal and 2Mb
 *	   data bitrates, supporting only standard messages:
 *
 *	   int fd = pcanfd_open("/dev/pcan0",
 *	                        OFD_BITRATE|OFD_DBITRATE|
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
 *	                        OFD_BITRATE|OFD_DBITRATE|OFD_CLOCK|
 *	                        PCANFD_INIT_LISTEN_ONLY|
 *	                        PCANFD_INIT_STD_MSG_ONLY|
 *	                        PCANFD_INIT_FD,
 *	                        1000000, 2000000, 80000000);
 *
 *	5/ Same than 1/ but with specifying BRP, TSEG1, TSEG2 and SJW values
 *	   for a 60 MHz clock:
 *
 *	   int fd = pcanfd_open("/dev/pcan0",
 *	                        OFD_BITRATE|OFD_BRPTSEGSJW|OFD_CLOCK,
 *	                        12, 7, 2, 1, 60000000);
 *
 * RETURN:
 *
 *	A positive integer if open and initialization have succeeded,
 *	a negative value otherwise (-errno).
 */
int pcanfd_open(const char *dev_pcan, __u32 flags, ...);

/*
 * int pcanfd_close(int fd)
 *
 *	Close any opened descriptor.
 *
 * RETURN:
 *
 *	-1
 */
int pcanfd_close(int fd);

#ifdef PCANFD_OLD_STYLE_API
/* old-style API */
DWORD CAN_InitFD(HANDLE hHandle, TPCANFDInit *pInit);
DWORD CAN_ReadFD(HANDLE hHandle, TPCANMsgFD *pMsgBuff);
DWORD CAN_WriteFD(HANDLE hHandle, TPCANMsgFD *pMsgBuff);

/* Linux old-style FD API */
DWORD LINUX_CAN_WriteFD_Timeout(HANDLE hHandle, TPCANMsgFD *pMsgBuff,
						int nMicroSeconds);
DWORD LINUX_CAN_ReadFD_Timeout(HANDLE hHandle, TPCANMsgFD *pMsgBuff,
						int nMicroSeconds);
#endif

#ifdef __cplusplus
};
#endif

#endif /* __LIBPCANFD_H__ */
