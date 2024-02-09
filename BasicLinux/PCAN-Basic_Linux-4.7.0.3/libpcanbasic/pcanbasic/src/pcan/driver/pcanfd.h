/* SPDX-License-Identifier: LGPL-2.1-only */
/*
 * CAN-FD extension to PEAK-System CAN products.
 *
 * Copyright (C) 2014-2020 PEAK System-Technik GmbH <www.peak-system.com>
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
#ifndef __PCANFD_H__
#define __PCANFD_H__

#if 0
#include <linux/types.h>
#include <linux/ioctl.h>

/* ioctls values */
#define PCAN_MAGIC_NUMBER	'z'
#define PCAN_SEQ_START		0x80

#else

/* include the (old) CAN 2.0 API */
#include "pcan.h"

#endif

/* enable to define types including array with 0, 1 or more items */
#define __array_of_struct(_n, _x)					\
	_n##s_##_x {							\
		__u32		count;					\
		struct _n	list[_x];				\
	}

/* Type values */
#define PCANFD_TYPE_NOP		0
#define PCANFD_TYPE_CAN20_MSG	1
#define PCANFD_TYPE_CANFD_MSG	2
#define PCANFD_TYPE_STATUS	3
#define PCANFD_TYPE_ERROR_MSG	4

/* [PCANFD_TYPE_CAN20_MSG]
 * [PCANFD_TYPE_CANFD_MSG]
 *
 * flags bits definition (lowest byte is backward compatible with old MSGTYPE)
 */
#define PCANFD_MSG_STD		0x00000000
#define PCANFD_MSG_RTR		0x00000001
#define PCANFD_MSG_EXT		0x00000002
#define PCANFD_MSG_SLF		0x00000004
#define PCANFD_MSG_SNG		0x00000008
#define PCANFD_MSG_ECHO		0x00000010

/* new bits indicate valid values in the corresponding fields */
#define PCANFD_TIMESTAMP	0x01000000
#define PCANFD_HWTIMESTAMP	0x02000000
#define PCANFD_ERRCNT		0x10000000
#define PCANFD_BUSLOAD		0x20000000
#define PCANFD_OVRCNT		0x40000000

/* CAN-FD */
#define PCANFD_MSG_BRS		0x00100000
#define PCANFD_MSG_ESI		0x00200000

/* [PCANFD_TYPE_STATUS]
 *
 * id field values:
 */
enum pcanfd_status {

	PCANFD_UNKNOWN,

	/* flags & PCANFD_ERROR_BUS.
	 * event id. is the state of the Bus
	 */
	PCANFD_ERROR_ACTIVE,
	PCANFD_ERROR_WARNING,
	PCANFD_ERROR_PASSIVE,		/* receive only state */
	PCANFD_ERROR_BUSOFF,		/* switched off from the bus */

	/* flags & PCANFD_ERROR_CTRLR|PCANFD_ERROR_INTERNAL.
	 * event id. is one of the following error:
	 */
	PCANFD_RX_EMPTY,
	PCANFD_RX_OVERFLOW,
	PCANFD_RESERVED_1,
	PCANFD_TX_OVERFLOW,

	PCANFD_RESERVED_2,
	PCANFD_BUS_LOAD,

	PCANFD_STATUS_COUNT
};

/* Deprecated "enum pcanfd_status": don't use it anymore */
#define PCANFD_BUS_ERROR	9	/* unused (see PCANFD_TYPE_ERROR_MSG) */
#define PCANFD_TX_EMPTY		7	/* unused */

/* [PCANFD_TYPE_STATUS]
 *
 * flags bits definition: indicate the kind of error/status:
 */
#define PCANFD_ERROR_BUS	0x00000080	/* Bus status */
#define PCANFD_ERROR_PROTOCOL	0x00000100	/* Protocol error */
#define PCANFD_ERROR_CTRLR	0x00000200	/* Controller error */
#define PCANFD_ERROR_INTERNAL	0x00000400	/* Internal error */

/* [PCANFD_TYPE_ERROR_MSG]
 *
 * id field values:
 */
enum pcanfd_error {
	PCANFD_ERRMSG_BIT,	/* SJA1000 ECC register format */
	PCANFD_ERRMSG_FORM,	/* (see data[0] for error code) */
	PCANFD_ERRMSG_STUFF,
	PCANFD_ERRMSG_OTHER,

	PCANFD_ERRMSG_COUNT
};

/* [PCANFD_TYPE_ERROR_MSG]
 *
 * flags bits definition: indicate direction
 */
#define PCANFD_ERRMSG_RX	0x00001000	/* err frame received */
#define PCANFD_ERRMSG_GEN	0x00002000	/* triggered by err generator */

#define PCAN_MAXDATALEN		8
#define PCANFD_MAXDATALEN	64

/* indexes describing content of ctrlr_data array */
enum pcanfd_ctrlr_data {
	PCANFD_RXERRCNT,
	PCANFD_ECHOID = PCANFD_RXERRCNT,	/* PCANFD_MSG_ECHO set */
	PCANFD_TXERRCNT,
	PCANFD_BUSLOAD_UNIT,
	PCANFD_BUSLOAD_DEC,
	PCANFD_MAXCTRLRDATALEN
};

/* define new API with CAN-FD support */
struct pcanfd_msg {
	__u16	type;			/* PCANFD_TYPE_xxx */
	__u16	data_len;		/* true length (not the DLC) */
	__u32	id;			/* CAN / STATUS / ERROR Id. */
	__u32	flags;			/* PCANFD_xxx definitions */
	struct timeval	timestamp;	/* timestamp of the event */
	__u8	ctrlr_data[PCANFD_MAXCTRLRDATALEN];
	__u8	data[PCANFD_MAXDATALEN] __attribute__((aligned(8)));
};

/* messages list base type (0 item list) */
struct __array_of_struct(pcanfd_msg, 0);

#define pcanfd_msgs		pcanfd_msgs_0

/* rule: 
 * if "bitrate" is not 0, then it is used,
 * if "bitrate" is 0, then all other 4 fields are used instead if all are not 0
 */
struct pcan_bittiming {
	__u32	brp;
	__u32	tseg1;
	__u32	tseg2;
	__u32	sjw;
	__u32	tsam;		/* triple sampling */

	__u32	bitrate;	/* bps */
	__u32	sample_point;	/* in 1/100 th of % (8750 = 87.50%) */
	__u32	tq;		/* Time quantum in ns. */
	__u32	bitrate_real;	/* info only */
};

/* PCANFD_XXX_INIT arg */

/* flags */
#define PCANFD_INIT_LISTEN_ONLY		0x00000001
#define PCANFD_INIT_STD_MSG_ONLY	0x00000002 /* backward compatibility */
#define PCANFD_INIT_FD			0x00000004 /* setup data bitrate too */
#define PCANFD_INIT_FD_NON_ISO		0x00000008 /* CAN-FD non ISO mode */

/* defines how timestamps should be processed by the driver in the messages
 * it gives to the application
 */
#define PCANFD_INIT_TS_HOST_REL		0x00000000 /* rel. to host init time */
#define PCANFD_INIT_TS_DEV_REL		0x00000010 /* rel. to device init time*/
#define PCANFD_INIT_TS_DRV_REL		0x00000020 /* rel. to driver init time*/
#define PCANFD_INIT_TS_FMT_MASK		0x00000030

/* (time is relative) */
#define PCANFD_INIT_TS_ABS		PCANFD_INIT_TS_HOST_REL

/* ask for being notified with PCANFD_TYPE_STATUS[PCANFD_BUS_LOAD]
 * (if the corresponding hardware is able to)
 */
#define PCANFD_INIT_BUS_LOAD_INFO	0x00000100

struct pcanfd_init {
	__u32	flags;
	__u32	clock_Hz;
	struct pcan_bittiming	nominal;
	struct pcan_bittiming	data;
};

/* PCANFD_GET_STATE arg */

struct pcanfd_state {
	__u16	ver_major, ver_minor, ver_subminor;

	struct timeval	tv_init;	/* time the device was initialized */

	enum pcanfd_status	bus_state;	/* CAN bus state */

	__u32	device_id;		/* device id., ffffffff is unused */

	__u32	open_counter;		/* open() counter */
	__u32	filters_counter;	/* count of message filters */

	__u16	hw_type;		/* pcan device type */
	__u16	channel_number;		/* channel number for the device */

	__u16	can_status;		/* same as wCANStatus but NOT CLEARED */
	__u16	bus_load;		/* bus load value, ffff if not given */

	__u32	tx_max_msgs;		/* Tx fifo size in count of msgs */
	__u32	tx_pending_msgs;	/* msgs waiting to be sent */
	__u32	rx_max_msgs;		/* Rx fifo size in count of msgs */
	__u32	rx_pending_msgs;	/* msgs waiting to be read */
	__u32	tx_frames_counter;	/* Tx frames written on device */
	__u32	rx_frames_counter;	/* Rx frames read on device */
	__u32	tx_error_counter;	/* CAN Tx errors counter */
	__u32	rx_error_counter;	/* CAN Rx errors counter */

	__u64	host_time_ns;		/* host time in nanoseconds as it was */
	__u64	hw_time_ns;		/* when hw_time_ns has been received */
};

/* PCANFD_xxx_FILTERS arg */

struct pcanfd_msg_filter {
	__u32	id_from;		/* msgs ID in range [id_from..id_to] */
	__u32	id_to;			/* and flags == msg_flags */
	__u32	msg_flags;		/* will be passed to applications */
};

/* filters list base type (0 item list) */
struct __array_of_struct(pcanfd_msg_filter, 0);

#define pcanfd_msg_filters		pcanfd_msg_filters_0

/* Device available clocks value */
struct pcanfd_available_clock {
	__u32	clock_Hz;
	__u32	clock_src;
};

/* clocks value list base type (0 item list) */
struct __array_of_struct(pcanfd_available_clock, 0);

#define pcanfd_available_clocks		pcanfd_available_clocks_0

/* default clock always first */
#define AVCLK_DEFAULT	0

/* CAN-FD bittiming capabilities */
struct pcanfd_bittiming_range {
	__u32	brp_min;
	__u32	brp_max;
	__u32	brp_inc;
	__u32	tseg1_min;
	__u32	tseg1_max;
	__u32	tseg2_min;
	__u32	tseg2_max;
	__u32	sjw_min;
	__u32	sjw_max;
};

/* bittimings ranges list base type (0 item list) */
struct __array_of_struct(pcanfd_bittiming_range, 0);

#define pcanfd_bittiming_ranges		pcanfd_bittiming_ranges_0

/* bittiming ranges list sequence order */
enum {
	BR_NOMINAL = 0,
	BR_DATA,

	BR_MAX
};

/* Device options */
struct pcanfd_option {
	int	size;
	int	name;
	void *	value;
};

enum {
	PCANFD_OPT_CHANNEL_FEATURES,
	PCANFD_OPT_DEVICE_ID,
	PCANFD_OPT_AVAILABLE_CLOCKS,		/* supersedes */
	PCANFD_OPT_BITTIMING_RANGES,		/* corresponding */
	PCANFD_OPT_DBITTIMING_RANGES,		/* ioctl() below */
	PCANFD_OPT_ALLOWED_MSGS,		/* see _ALLOWED_xxx_ below */
	PCANFD_OPT_ACC_FILTER_11B,
	PCANFD_OPT_ACC_FILTER_29B,
	PCANFD_OPT_IFRAME_DELAYUS,
	PCANFD_OPT_HWTIMESTAMP_MODE,
	PCANFD_OPT_DRV_VERSION,
	PCANFD_OPT_FW_VERSION,
	PCANFD_IO_DIGITAL_CFG,		/* output mode 1: output active */
	PCANFD_IO_DIGITAL_VAL,		/* digital I/O 32-bit value */
	PCANFD_IO_DIGITAL_SET,		/* multiple dig I/O pins to 1=High */
	PCANFD_IO_DIGITAL_CLR,		/* clr multiple dig I/O pins to 0 */
	PCANFD_IO_ANALOG_VAL,		/* get single analog input pin value */
	PCANFD_OPT_MASS_STORAGE_MODE,	/* all USB FD and some USB devices */
	PCANFD_OPT_FLASH_LED,
	PCANFD_OPT_DRV_CLK_REF,
	PCANFD_OPT_LINGER,

	/* internal use only */
	PCANFD_OPT_SELF_ACK,	/* send ACK to self-written frames */
	PCANFD_OPT_BRS_IGNORE,	/* ignore BRS frames (no error frames sent) */
	PCANFD_OPT_DEFERRED_FRM,

	PCANFD_OPT_MAX
};

/* PCANFD_OPT_CHANNEL_FEATURES option:
 * features of a channel
 */
#define PCANFD_FEATURE_FD		0x00000001
#define PCANFD_FEATURE_IFRAME_DELAYUS	0x00000002
#define PCANFD_FEATURE_BUSLOAD		0x00000004
#define PCANFD_FEATURE_HWTIMESTAMP	0x00000008
#define PCANFD_FEATURE_DEVICEID		0x00000010
#define PCANFD_FEATURE_SELFRECEIVE	0x00000020
#define PCANFD_FEATURE_ECHO		0x00000040
#define PCANFD_FEATURE_MSD		0x00000080
#define PCANFD_FEATURE_TS_SOF		0x00000100
#define PCANFD_FEATURE_SELF_ACK		0x00000200
#define PCANFD_FEATURE_BRS_IGN		0x00000400
#define PCANFD_FEATURE_DEVDATA		0x00000800
#define PCANFD_FEATURE_NEW_FW_AV	0x00001000

/* PCANFD_OPT_ALLOWED_MSGS option:
 * bitmask of allowed message an application is able to receive
 */
#define PCANFD_ALLOWED_MSG_CAN		0x00000001
#define PCANFD_ALLOWED_MSG_RTR		0x00000002
#define PCANFD_ALLOWED_MSG_EXT		0x00000004
#define PCANFD_ALLOWED_MSG_STATUS	0x00000010
#define PCANFD_ALLOWED_MSG_ERROR	0x00000100
#define PCANFD_ALLOWED_MSG_ALL		0xffffffff
#define PCANFD_ALLOWED_MSG_NONE		0x00000000

/* PCANFD_OPT_HWTIMESTAMP_MODE option:
 * 0	off (host timestamp at the time the event has been saved)
 * 1	on (host time base + device raw time offset)
 * 2	on (host time base + device time offset handling clock drift)
 * 3	on (device timestamp in struct timeval format)
 * 4	reserved
 * 5	same as 1 + ts is generated at SOF rather than at EOF (if hw allows it)
 * 6	same as 2 + ts is generated at SOF rather than at EOF (if hw allows it)
 * 7	same as 3 + ts is generated at SOF rather than at EOF (if hw allows it)
 */
enum {
	PCANFD_OPT_HWTIMESTAMP_OFF,
	PCANFD_OPT_HWTIMESTAMP_ON,
	PCANFD_OPT_HWTIMESTAMP_COOKED,
	PCANFD_OPT_HWTIMESTAMP_RAW,
	PCANFD_OPT_HWTIMESTAMP_RESERVED_4,
	PCANFD_OPT_HWTIMESTAMP_SOF_ON,
	PCANFD_OPT_HWTIMESTAMP_SOF_COOKED,
	PCANFD_OPT_HWTIMESTAMP_SOF_RAW,

	PCANFD_OPT_HWTIMESTAMP_MAX
};

/* PCANFD_OPT_XXX_VERSION major, minor and subminor fields */
#define PCANFD_OPT_VER_MAJ(v)		(((v) >> 24) & 0xff)
#define PCANFD_OPT_VER_MIN(v)		(((v) >> 16) & 0xff)
#define PCANFD_OPT_VER_SUB(v)		(((v) >> 8) & 0xff)

/* PCANFD_OPT_LINGER option gives the timeout value used by the driver to
 * wait for the pending data to be transmitted:
 * < 0	the driver estimates itself the maximum necessary time to wait until the
 *	queue is empty
 * 0	the driver does not wait until the queue is empty before closing
 *      everything.
 * > 0	this specifies the timeout period in ms.
 */
#define PCANFD_OPT_LINGER_AUTO		(-1)
#define PCANFD_OPT_LINGER_NOWAIT	0

/* PCANFD_RESET argument flags */
#define	PCANFD_RESET_RXFIFO		0x00000001
#define	PCANFD_RESET_TXFIFO		0x00000002
#define	PCANFD_RESET_CTRLR		0x00000004
#define PCANFD_RESET_ALL		0xffffffff

/* ioctls codes */
#define PCANFD_SEQ_START		0x90

enum {
	PCANFD_SEQ_SET_INIT = PCANFD_SEQ_START,
	PCANFD_SEQ_GET_INIT,
	PCANFD_SEQ_GET_STATE,
	PCANFD_SEQ_ADD_FILTERS,
	PCANFD_SEQ_GET_FILTERS,
	PCANFD_SEQ_SEND_MSG,
	PCANFD_SEQ_RECV_MSG,
	PCANFD_SEQ_SEND_MSGS,
	PCANFD_SEQ_RECV_MSGS,
	PCANFD_SEQ_GET_AVAILABLE_CLOCKS,	/* deprecated, use related */
	PCANFD_SEQ_GET_BITTIMING_RANGES,	/* options above instead */
	PCANFD_SEQ_GET_OPTION,
	PCANFD_SEQ_SET_OPTION,
	PCANFD_SEQ_RESET,
};

#define PCANFD_SET_INIT		_IOW(PCAN_MAGIC_NUMBER, PCANFD_SEQ_SET_INIT,\
					struct pcanfd_init)

#define PCANFD_GET_INIT		_IOR(PCAN_MAGIC_NUMBER, PCANFD_SEQ_GET_INIT,\
					struct pcanfd_init)

#define PCANFD_GET_STATE	_IOR(PCAN_MAGIC_NUMBER, PCANFD_SEQ_GET_STATE,\
					struct pcanfd_state)

#define PCANFD_ADD_FILTERS	_IOW(PCAN_MAGIC_NUMBER,			       \
				     PCANFD_SEQ_ADD_FILTERS,		       \
				     struct pcanfd_msg_filters)

#define PCANFD_GET_FILTERS	_IOWR(PCAN_MAGIC_NUMBER,		       \
				      PCANFD_SEQ_GET_FILTERS,		       \
				      struct pcanfd_msg_filters)

#define PCANFD_SEND_MSG		_IOW(PCAN_MAGIC_NUMBER, PCANFD_SEQ_SEND_MSG,\
					struct pcanfd_msg)

#define PCANFD_RECV_MSG		_IOR(PCAN_MAGIC_NUMBER, PCANFD_SEQ_RECV_MSG,\
					struct pcanfd_msg)

#define PCANFD_SEND_MSGS	_IOWR(PCAN_MAGIC_NUMBER, PCANFD_SEQ_SEND_MSGS,\
					struct pcanfd_msgs)

#define PCANFD_RECV_MSGS	_IOWR(PCAN_MAGIC_NUMBER, PCANFD_SEQ_RECV_MSGS,\
					struct pcanfd_msgs)

#define PCANFD_GET_AVAILABLE_CLOCKS	_IOWR(PCAN_MAGIC_NUMBER,\
					      PCANFD_SEQ_GET_AVAILABLE_CLOCKS,\
					      struct pcanfd_available_clocks)

#define PCANFD_GET_BITTIMING_RANGES	_IOWR(PCAN_MAGIC_NUMBER,\
					      PCANFD_SEQ_GET_BITTIMING_RANGES,\
					      struct pcanfd_bittiming_ranges)

#define PCANFD_GET_OPTION	_IOWR(PCAN_MAGIC_NUMBER, PCANFD_SEQ_GET_OPTION,\
					struct pcanfd_option)
#define PCANFD_SET_OPTION	_IOW(PCAN_MAGIC_NUMBER, PCANFD_SEQ_SET_OPTION,\
					struct pcanfd_option)

#define PCANFD_RESET		_IOW(PCAN_MAGIC_NUMBER, PCANFD_SEQ_RESET,\
					unsigned long)

#endif
