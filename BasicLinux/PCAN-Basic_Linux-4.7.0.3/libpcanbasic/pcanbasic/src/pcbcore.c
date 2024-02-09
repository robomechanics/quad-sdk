/* SPDX-License-Identifier: LGPL-2.1-only */
/*
 * @file pcbcore.c
 * @brief PCAN Basic core
 * $Id: pcbcore.c 15919 2022-12-16 09:28:20Z Fabrice $
 *
 * Copyright (C) 2001-2022  PEAK System-Technik GmbH <www.peak-system.com>
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
 * PCAN is a registered Trademark of PEAK-System Germany GmbH
 *
 * Contact:      <linux@peak-system.com>
 * Maintainer:   Fabrice Vergnaud <f.vergnaud@peak-system.com>
 */
#include "pcbcore.h"
#include "pcbcore_data.h"

#include <stdio.h>		/* snprintf */
#include <stdlib.h>		/* NULL */
#include <string.h>		/* memset, etc. */
#include <sys/queue.h>	/* LIST_ENTRY, etc. */
#include <errno.h>		/* to handle errno returned by libpcanfd */
#include <unistd.h>		/* usleep */
#include <ctype.h>		/* isspace */
#include <fcntl.h>		/* O_RDONLY */
#include <sys/time.h>	/* timespec */

#ifndef CLOCK_MONOTONIC
#include <linux/time.h>	/* CLOCK_MONOTONIC */
#endif

#ifndef TIMESPEC_TO_TIMEVAL
#define TIMESPEC_TO_TIMEVAL(tv, ts) {                               \
	(tv)->tv_sec = (ts)->tv_sec;                                    \
	(tv)->tv_usec = (ts)->tv_nsec / 1000;                           \
}
#endif

#ifndef timespecsub
#define	timespecsub(tsp, usp, vsp)                          \
	do {                                                    \
		(vsp)->tv_sec = (tsp)->tv_sec - (usp)->tv_sec;      \
		(vsp)->tv_nsec = (tsp)->tv_nsec - (usp)->tv_nsec;   \
		if ((vsp)->tv_nsec < 0) {                           \
			(vsp)->tv_sec--;                                \
			(vsp)->tv_nsec += 1000000000L;                  \
		}                                                   \
	} while (0)
#endif

#define STRINGIFY(x)     #x

/* NOTE: the new PCANBasic API uses libpcanfd source code, here is why:
 *  - to avoid code duplication. libpcanfd and pcanbasic both use
 *    [__ioctl, fopen, etc.] functions to control CAN devices.
 * 	- to get updates sooner, libpcanfd gets updated of driver
 * 	  changes before pcanbasic.
 *
 * The API does not link the lib but use its source code instead,
 * because we can't include the old libpcan library as it
 * defines the same prototypes as PCANBasic (like CAN_write, etc.).
 *
 * But via source code, since PCANFD_OLD_STYLE_API is not defined,
 * the libpcan is not included automatically by libpcanfd.
 */
#include "pcanfd.h"	/* force local header */
#include "libpcanfd.h"

#include "PCANBasic.h"	/* PCANBasic types and defines */
#include "pcaninfo.h"		/* discovers PCAN devices */
#include "pcanlog.h"		/* used to debug lib */
#include "resource.h"		/* used to translate error msgs */
#include "pcblog.h"			/* pcanbasic-logger used by get/set_value */
#include "pcbtrace.h"		/* pcanbasic-logger used by get/set_value */
#include "version.h"		/* API version */

#if !defined(LOG_LEVEL)
#if !defined(_DEBUG)
#define LOG_LEVEL		LVL_NORMAL
#else
#define LOG_LEVEL		LVL_NORMAL
#endif
#endif

#if !defined(_DEBUG)
#define API_VERSION_DEBUG	""
#else 
#define API_VERSION_DEBUG	" (debug)"
#endif 

#if !defined(LOG_FILE_DEBUG)
#define LOG_FILE_DEBUG	 "pcanbasic_dbg.log"
#endif 

#if !defined(LOG_SHOW_TIME)
#define LOG_SHOW_TIME	(LOG_LEVEL == LVL_DEBUG)
#endif 

#define CHANNEL_VERSION_EXTRA "Copyright (C) 1995-2021 by\nPEAK-System Technik GmbH, Darmstadt"
/* Uncomment to disable echoing of message via libpcanfd	*/
//#define FORCE_ECHO_STATUS_OFF	1

/* DEFINES	*/

/**
 * Default value for parameter 'bitrate_adapting'
 */
#define DEFAULT_PARAM_BITRATE_ADAPTING	PCAN_PARAMETER_OFF
/**
 * Default value for parameter 'listen_only'
 */
#define DEFAULT_PARAM_LISTEN_ONLY		PCAN_PARAMETER_OFF
/**
 * Default value for parameter 'rcv_status'
 */
#define DEFAULT_PARAM_RCV_STATUS		PCAN_PARAMETER_ON
/**
 * Default value for parameter 'echo_status'
 */
#define DEFAULT_PARAM_ECHO_STATUS		PCAN_PARAMETER_OFF

/**
 * Minimum time elapsed (in µs) before refreshing the struct pcaninfo devices
 */
#define PCANINFO_TIME_REFRESH		100000
#define PCANINFO_TIME_REFRESH_NANO	100000000
 /**
  * Minimum time elapsed (in ns) before refreshing the struct pcaninfo devices
  */
#define UNINITIALIZE_MAX_WAITING_TIME_NANO	500000000

/**
 * Maximum size for hardware name
 */
#define HARDWARE_NAME_MAX_SIZE			32
#define BITRATE_INFO_FD_MAX_SIZE		200

/**
 * @defgroup FD_PARAM_INIT key parameters used in CAN FD initialization string
 *
 * @{
 */
#define FD_PARAM_INIT_CLOCK_MHZ		"f_clock_mhz"
#define FD_PARAM_INIT_CLOCK_HZ		"f_clock"
#define FD_PARAM_INIT_NOM_BRP		"nom_brp"
#define FD_PARAM_INIT_NOM_TSEG1		"nom_tseg1"
#define FD_PARAM_INIT_NOM_TSEG2		"nom_tseg2"
#define FD_PARAM_INIT_NOM_SJW		"nom_sjw"
#define FD_PARAM_INIT_DATA_BRP		"data_brp"
#define FD_PARAM_INIT_DATA_TSEG1	"data_tseg1"
#define FD_PARAM_INIT_DATA_TSEG2	"data_tseg2"
#define FD_PARAM_INIT_DATA_SJW		"data_sjw"
/** @} */

/* extra lookup parameter	*/
#define LOOKUP_HW_HANDLE			__T("hardwarehandle")        /*	Lookup channel by CAN-API hardware handle	*/

/**
 * @def MIN(x,y)
 * @brief A macro that returns the minimum of @a x and @a y.
 */
#define MIN(x,y) ((x) < (y) ? (x) : (y))

/* PRIVATE TYPES	*/

/**
 * Optional features based on driver/firmware capabilities.
 */
struct _pcanbasic_channel_features {
	__u8 initialized;
	TPCANStatus status;
	__u8 echo_frames;
};
typedef struct _pcanbasic_channel_features pcanbasic_channel_features;

/**
 * Stores information on an initialized PCANBasic channel.
 * This structure maps a TPCANHandle to a file descriptor,
 * and implements linked-list feature.
 */
struct _pcanbasic_channel {
	TPCANHandle channel;		/**< CAN channel. */
	TPCANBaudrate btr0btr1; 	/**< Nominal bit rate as BTR0BTR1. */
	TPCANBitrateFD bitratefd;	/**< String configuration for nominal & data bit rates. */
	int fd;						/**< File descriptor, if 0 then channel is not initialized. */
	__u32 fd_flags;				/**< File descriptor's flags for libpcanfd. */
	__u8 bitrate_adapting;		/**< Allows initialization with already opened channels. */
	__u8 busoff_reset;			/**< Automatically resets bus when busoff. */
	__u8 listen_only;			/**< Initializes CAN with the mode listen-only. */
	__u8 rcv_status;			/**< Receive status, if 0 can mutes CAN reception (CAN_Read always returns QRCVEMPTY). */
	__u8 rcv_echo_status;		/**< HW self-reception status (for API user). */
	__u8 echo_status;			/**< HW self-reception status (for trace ordering see PCANFD_MSG_ECHO). */
	struct pcaninfo *pinfo;		/**< Pointer to sysfs info structure. */
	SLIST_ENTRY(_pcanbasic_channel) entries;	/**< Single linked list. */
	__u8 ignore_status_frame;	/**< nb of status frame to ignore	*/ 
	pcanbasic_channel_features features;	/** optional features based on drv/fw capabilities. */

	struct pcbtrace_ctx	tracer;	/**< PCANBasic tracing context. */
};
typedef struct _pcanbasic_channel pcanbasic_channel;
/**
 * PCANBASIC Core persistent data
 */
struct _pcanbasic_core {
	int initialized;				/**< States if structure (SLIST especially) was initialized. */
	struct timespec last_update;	/**< Time of the last pcaninfo hw update (avoid unnecessary updates). */
	struct pcaninfo_list *devices;	/**< Known pcan devices list. */
	SLIST_HEAD(PCANBASIC_channel_SLIST, _pcanbasic_channel) channels;	/* First element of the linked list of initialized channels. */
	int refresh_locked;				/**< If set hw_refresh is disabled. */
};
typedef struct _pcanbasic_core pcanbasic_core;


#define LOOKUP_FLAG_DEV_TYPE	0x01
#define LOOKUP_FLAG_DEV_ID		0x02
#define LOOKUP_FLAG_DEV_CTRL_NB	0x04
#define LOOKUP_FLAG_IP			0x08
#define LOOKUP_FLAG_HW_HANDLE	0x10
struct _pcanbasic_lookup_info {
	__u32 flags;
	TPCANDevice dev_type;
	__u32 dev_id;
	__u32 dev_ctrl_nb;
	char ip_address[50];
	__u32 hw_handle;
};
typedef struct _pcanbasic_lookup_info pcanbasic_lookup_info;


/*	PRIVATE FUNCTIONS DEFINITIONS	*/

/**
* @fn char *pcanbasic_ltrim(char *s)
* @brief Removes leading whitespaces.
*/
static char *pcanbasic_ltrim(char *s);
/**
* @fn char *pcanbasic_rtrim(char *s)
* @brief Removes trailing whitespaces.
*/
static char *pcanbasic_rtrim(char *s);
/**
* @fn char *pcanbasic_trim(char *s)
* @brief Removes leading and trailing whitespaces.
*/
static char *pcanbasic_trim(char *s);

/**
* @fn char *pcanbasic_tolower(char *s)
* @brief Converts a string to upper case.
*/
static char *pcanbasic_tolower(char *s);

/**
* @fn char *pcanbasic_toupper(char *s)
* @brief Converts a string to upper case.
*/
static char *pcanbasic_toupper(char *s);

/**
 * @fn void pcanbasic_init(void)
 * @brief Initializes PCANBasic persistent/private data.
 */
static void pcanbasic_init(void);
/**
 * @fn void pcanbasic_refresh_hw(void)
 * @brief Updates struct pcaninfo device list.
 */
static void pcanbasic_refresh_hw(void);
/**
 * @fn void pcanbasic_atexit(void)
 * @brief Cleans up API's objects.
 */
static void pcanbasic_atexit(void);
/**
 * @fn PCANBASIC_channel * pcanbasic_get_channel(TPCANHandle channel, __u8 opened, TPCANStatus* status)
 * @brief Returns the PCANBasic channel structure based on the channel handle.
 *
 * @param channel The handle of a previously initialized channel.
 * @param opened States if the searched channel must be already initialized (or only pre-initialized).
 * @param status If not NULL specifies the error status.
 * @return An PCANBASIC_channel structure or NULL if the channel was not initialized.
 */
static pcanbasic_channel * pcanbasic_get_channel(TPCANHandle channel, __u8 opened, TPCANStatus* status);
/**
 * @fn struct pcaninfo * pcanbasic_get_device(TPCANHandle channel, __u32 hwtype, __u32 base, __u32 irq, TPCANStatus* status)
 * @brief Returns the struct pcaninfo corresponding to channel handle and more.
 *
 * @param channel Channel handle.
 * @param hwtype Hardware type.
 * @param base I/O port.
 * @param irq Interrupt.
 * @param status If not NULL specifies the error status.
 * @return Pointer to a pcaninfo structure or NULL if not found.
 */
static struct pcaninfo * pcanbasic_get_device(TPCANHandle channel, __u32 hwtype, __u32 base, __u32 irq, TPCANStatus* status);

/**
 * @fn void pcanbasic_get_hw(TPCANHandle channel, enum pcaninfo_hw *hw, __u32 *index)
 * @brief Gets the hardware type and index corresponding to a channel handle.
 *
 * @param[in] channel Channel handle.
 * @param[out] hw Buffer to store the hardware type of the channel.
 * @param[out] index Buffer to store the index of the channel (ex. 3 for PCAN_USBBUS3).
 */
static TPCANStatus pcanbasic_get_hw(TPCANHandle channel, enum pcaninfo_hw *hw, __u32 *index);

/**
 * @fn __u8 pcanbasic_check_hw(TPCANHandle channel, __u8 allow_nonebus)
 * @brief Checks if the channel handle corresponds to a supported hardware.
 *
 * @param[in] channel Channel handle.
 * @param[in] allow_nonebus States if PCAN_NONEBUS is considered valid.
 * @return __u8 1 if handle is valid, 0 otherwise.
 */
static __u8 pcanbasic_check_hw(TPCANHandle channel, __u8 allow_nonebus);

/**
 * @fn TPCANStatus pcanbasic_errno_to_status(int err)
 * @brief Returns the TPCANStatus corresponding to an errno value.
 *
 * @param err An 'errno' value.
 * @return TPCANStatus corresponding to an errno value.
 */
static TPCANStatus pcanbasic_errno_to_status(int err);

/**
 * @fn TPCANStatus pcanbasic_errno_to_status(int err, int ctx)
 * @brief Returns the TPCANStatus corresponding to an errno value.
 *
 * @param err An 'errno' value.
 * @param ctx An 'context' value, see PCB_CTX_xxx defines below.
 * @return TPCANStatus corresponding to an errno value.
 */
static TPCANStatus pcanbasic_errno_to_status_ctx(int err, int ctx);
#define PCB_CTX_READ 1
#define PCB_CTX_WRITE 2
/**
 * @fn pcanbasic_channel* pcanbasic_create_channel(TPCANHandle channel, __u8 add_to_list)
 * @brief Allocates and initializes a pcanbasic_channel structure.
 *
 * @param channel Channel handle.
 * @param add_to_list states if the structure is added to PCANBasic persistent data (g_basiccore) (1=yes, 0=no).
 * @return a  pointer to a PCANBASIC_channel structrue or NULL.
 */
static pcanbasic_channel* pcanbasic_create_channel(TPCANHandle channel, __u8 add_to_list);

/**
 * @fn TPCANStatus pcanbasic_bus_state_to_condition(enum pcanfd_status bus_state, __u8 fd)
 * @brief Returns the channel condition based on a pcanfd_status enum.
 *
 * @param bus_state The bus state to analyze.
 * @param fd States if the bus is CAN-FD initialized.
 * @return The channel condition corresponding to the bus_state.
 */
static TPCANStatus pcanbasic_bus_state_to_condition(enum pcanfd_status bus_state, __u8 fd);

/**
 * @fn int pcanbasic_get_fd(TPCANHandle channel, pcanbasic_channel* pchan, struct pcaninfo** in_out_pinfo, __u8* out_fd_close)
 * @brief Returns the file descriptor of a channel.
 * Depending on the provided parameters, the function will try to retrieve a file descriptor 
 * without having to use libpcanfd_open. If it is not possible, it will open with minimal footprint/ known side-effects.
 *
 * @param channel Channel handle to use if other parameters are useless.
 * @param pchan Channel structure to use (1st param analyzed).
 * @param in_out_pinfo PcanInfo structure buffer (if not NULL, 2nd param analyzed).
 * @param out_fd_close Buffer to store if the returned fd should be closed.
 * @return File descriptor (-1 if not found).
 */
static int pcanbasic_get_fd(TPCANHandle channel, pcanbasic_channel* pchan, struct pcaninfo** in_out_pinfo, __u8* out_fd_close);

/**
 * @fn __u8 pcanbasic_get_filter(pcanbasic_channel * pchan)
 * @brief Returns the filter status of a channel.
 *
 * @param pchan Channel structure get filter status from.
 * @return The filter status (OPENED, CLOSED or CUSTOM).
 */
static __u8 pcanbasic_get_filter(pcanbasic_channel * pchan);

/**
 * @fn TPCANStatus pcanbasic_get_value_device_id(TPCANHandle channel, pcanbasic_channel* pchan, void* buf, __u32 bufsize)
 * @brief Returns the channel's device ID.
 *
 * @param channel Channel handle (used only if pchan is NULL).
 * @param pchan Pointer to an initialized channel or NULL.
 * @param buf Buffer to store the device ID.
 * @param bufsize Size of the buffer.
 * @return A TPCANStatus error code.
 */
static TPCANStatus pcanbasic_get_value_device_id(TPCANHandle channel, pcanbasic_channel* pchan, void* buf, __u32 bufsize);
/**
 * @fn TPCANStatus pcanbasic_get_value_channel_version(TPCANHandle channel, pcanbasic_channel * pchan, void* buf, __u32 bufsize)
 * @brief Returns the channel's driver version.
 *
 * @param channel Channel handle (used only if pchan is NULL).
 * @param pchan Pointer to an initialized channel or NULL.
 * @param buf Buffer to store the value.
 * @param bufsize Size of the buffer.
 * @return A TPCANStatus error code.
 */
static TPCANStatus pcanbasic_get_value_channel_version(TPCANHandle channel, pcanbasic_channel* pchan, void* buf, __u32 bufsize);
/**
 * @fn TPCANStatus pcanbasic_get_value_channel_condition(TPCANHandle channel, pcanbasic_channel* pchan)
 * @brief Returns the channel's condition.
 *
 * @param channel Channel handle (used only if pchan is NULL).
 * @param pchan Pointer to an initialized channel or NULL.
 * @param buf Buffer to store the value.
 * @param bufsize Size of the buffer.
 * @return The channel's condition (AVAILABLE, OCCUPIED, UNAVAILABLE).
 */
static TPCANStatus pcanbasic_get_value_channel_condition(TPCANHandle channel, pcanbasic_channel* pchan);
/**
 * @fn TPCANStatus pcanbasic_get_value_channel_features(TPCANHandle channel, pcanbasic_channel* pchan, void* buf, __u32 bufsize)
 * @brief Returns the channel's features (ex. FD capable or not).
 *
 * @param channel Channel handle (used only if pchan is NULL).
 * @param pchan Pointer to an initialized channel or NULL.
 * @param buf Buffer to store the value.
 * @param bufsize Size of the buffer.
 * @return A TPCANStatus error code.
 */
static TPCANStatus pcanbasic_get_value_channel_features(TPCANHandle channel, pcanbasic_channel* pchan, void* buf, __u32 bufsize);
/**
 * @fn TPCANStatus pcanbasic_get_value_controller_number(TPCANHandle channel, pcanbasic_channel* pchan, void* buf, __u32 bufsize)
 * @brief Returns the channel's controller number.
 *
 * @param channel Channel handle (used only if pchan is NULL).
 * @param pchan Pointer to an initialized channel or NULL.
 * @param buf Buffer to store the value.
 * @param bufsize Size of the buffer.
 * @return A TPCANStatus error code.
 */
static TPCANStatus pcanbasic_get_value_controller_number(TPCANHandle channel, pcanbasic_channel* pchan, void* buf, __u32 bufsize);
/**
 * @fn int pcanbasic_get_value_controller_number(TPCANHandle channel, pcanbasic_channel* pchan, void* buf, __u32 bufsize)
 * @brief Returns the channel's initialization BTR0-BTR1.
 *
 * @param channel Channel handle (used only if pchan is NULL).
 * @param pchan Pointer to an initialized channel or NULL.
 * @param buf Buffer to store the value.
 * @param bufsize Size of the buffer.
 * @return A TPCANStatus error code.
 */
static TPCANStatus pcanbasic_get_value_bitrate_info(TPCANHandle channel, pcanbasic_channel* pchan, void* buf, __u32 bufsize);
/**
 * @fn int pcanbasic_get_value_controller_number(TPCANHandle channel, pcanbasic_channel* pchan, void* buf, __u32 bufsize)
 * @brief Returns the channel's bitrate fd initialization string.
 *
 * @param channel Channel handle (used only if pchan is NULL).
 * @param pchan Pointer to an initialized channel or NULL.
 * @param buf Buffer to store the value.
 * @param bufsize Size of the buffer.
 * @return A TPCANStatus error code.
 */
static TPCANStatus pcanbasic_get_value_bitrate_info_fd(TPCANHandle channel, pcanbasic_channel* pchan, void* buf, __u32 bufsize);

/**
 * @fn TPCANStatus pcanbasic_init_fw_features(pcanbasic_channel* pchan)
 * @brief Initialize channel extra features based on firmware's and driver's versions.
 *
 * @param pchan Pointer to an initialized channel.
 * @return A TPCANStatus error code.
 */
TPCANStatus pcanbasic_init_fw_features(pcanbasic_channel* pchan);

/**
 * @fn TPCANStatus pcanbasic_set_hwtimestamp_mode(pcanbasic_channel* pchan)
 * @brief Change the hwtimestamp mode to be compatible with PCANBasic tracer.
 *
 * @param pchan Pointer to an initialized channel or NULL.
 * @return A TPCANStatus error code.
 */
static TPCANStatus pcanbasic_set_hwtimestamp_mode(pcanbasic_channel* pchan);
/**
 * @fn TPCANStatus pcanbasic_set_value_device_id(TPCANHandle channel, pcanbasic_channel* pchan, void* buf, __u32 bufsize)
 * @brief Sets the channel's device ID.
 *
 * @param channel Channel handle (used only if pchan is NULL).
 * @param pchan Pointer to an initialized channel or NULL.
 * @param buf Buffer storing the value.
 * @param bufsize Size of the buffer.
 * @return A TPCANStatus error code.
 */
static TPCANStatus pcanbasic_set_value_device_id(TPCANHandle channel, pcanbasic_channel* pchan, void* buf, __u32 bufsize);


/**
 * @fn int pcanbasic_parse_fd_init(struct pcanfd_init * pfdi, TPCANBitrateFD fdbitrate)
 * @brief Parses and fills a pcanfd_init struct based on an FD bitrate string
 *
 * @param pfdi Buffer to store the initialization values.
 * @param fdbitrate FD-Bitrate-Initialization string to be parsed.
 * @return 0 if no error, an errno otherwise.
 */
static int pcanbasic_parse_fd_init(struct pcanfd_init * pfdi, TPCANBitrateFD fdbitrate);

/**
 * @fn int pcanbasic_parse_int(const char* value);
 * @brief Parses a string representing an integer (examples: 10, Ah, 0xA)
 *
 * @param buf Buffer to store the parsed value.
 * @param str String representing an integer (examples: "10", "Ah", "0xA").
 * @return 0 if no error, an errno otherwise.
 */
static int pcanbasic_parse_u32(__u32* buf, const char* str);

/**
 * @fn int pcanbasic_parse_ipv4(const char* str);
 * @brief Parses a string representing an IP v4 address (example: 10.1.12.195)
 *
 * @param str String representing an IP v4 address.
 * @return 0 if no error, an errno otherwise.
 */
static int pcanbasic_parse_ipv4(const char* str);
/**
 * @fn int pcanbasic_parse_lookup(pcanbasic_lookup_info* lui, char* param)
 * @brief Parses a PCANBasic lookup-channel string.
 *
 * @param lui Buffer to store the initialization values.
 * @param param String to be parsed.
 * @return 0 if no error, an errno otherwise.
 */
static int pcanbasic_parse_lookup(pcanbasic_lookup_info* lui, char* param);

/**
 * @fn TPCANStatus pcanbasic_parameter_supported(TPCANHandle channel, TPCANParameter parameter, __u8 is_set)
 * @brief States if a parameter is handled by the API.
 *
 * @param channel Channel handle to use the parameter on.
 * @param parameter Parameter to check.
 * @param is_set States if parameter will be used in a SetValue context.
 * @return PCAN_ERROR_OK if handled, PCAN_ERROR_ILLPARAMTYPE otherwise.
 */
static TPCANStatus pcanbasic_parameter_supported(TPCANHandle channel, TPCANParameter parameter, __u8 is_set);

/**
 * @fn TPCANStatus pcanbasic_read_common(pcanbasic_channel *pchan, TPCANMsgFD* message, struct timeval *t)
 * @brief A common function to read CAN messages (CAN20 or CANFD message).
 *
 * @param pchan Channel object to read from.
 * @param[out] message Buffer to store the message (stored in a TPCANMsgFD struct even if it is a CAN20 message).
 * @param[out] t Buffer to store the timestamp of the message (NULL to skip).
 * @return A TPCANStatus error code.
 */
static TPCANStatus pcanbasic_read_common(pcanbasic_channel *pchan, TPCANMsgFD* message, struct timeval *t);
/**
 * @fn TPCANStatus pcanbasic_write_common(pcanbasic_channel *pchan, TPCANMsgFD* message)
 * @brief A common function to write CAN messages (CAN20 or CANFD message).
 *
 * @param pchan Channel object to write to.
 * @param[in] message Pointer to a TPCANMsgFD holding the message to write (either a CAN20 of CANFD message).
 * @return A TPCANStatus error code.
 */
static TPCANStatus pcanbasic_write_common(pcanbasic_channel *pchan, TPCANMsgFD* message);
/**
 * @fn TPCANStatus pcanbasic_wait_bus_active(int fd, int timeout)
 * @brief A function waiting for the CAN file descriptor to be BUS_ACTIVE
 *
 * @param fd CAN file descriptor
 * @param timeout Time to wait in milliseconds.
 * @return A TPCANStatus error code.
 */
static TPCANStatus pcanbasic_wait_bus_active(int fd, int timeout_ms);

/* PRIVATE VARIABLES	*/
/**
 * Stores persistent PCANBasic data
 */
static pcanbasic_core g_basiccore;

/*	PRIVATE FUNCTIONS	*/
char *pcanbasic_ltrim(char *s)
{
	while (isspace(*s)) s++;
	return s;
}

char *pcanbasic_rtrim(char *s)
{
	char* back = s + strlen(s);
	while (isspace(*--back));
	*(back + 1) = '\0';
	return s;
}

char *pcanbasic_trim(char *s)
{
	return pcanbasic_rtrim(pcanbasic_ltrim(s));
}

char *pcanbasic_tolower(char *s)
{
	char* p = s;
	while (*p != 0) {
		*p = tolower(*p);
		p++;
	}
	return s;
}

char *pcanbasic_toupper(char *s)
{
	char* p = s;
	while (*p != 0) {
		*p = toupper(*p);
		p++;
	}
	return s;
}

void pcanbasic_init(void) {
	pcanlog_set(LOG_LEVEL, LOG_FILE_DEBUG, LOG_SHOW_TIME);
	pcanlog_log(LVL_VERBOSE, "Initializing PCAN-Basic API...\n");
	SLIST_INIT(&g_basiccore.channels);
	g_basiccore.devices = NULL;
	g_basiccore.refresh_locked = 0;
	pcanbasic_refresh_hw();
	g_basiccore.initialized = 1;
	atexit(pcanbasic_atexit);
}

void pcanbasic_refresh_hw(void) {
	if (!g_basiccore.refresh_locked) {
		if (g_basiccore.devices) {
			free(g_basiccore.devices);
			g_basiccore.devices = NULL;
		}
		pcanlog_log(LVL_VERBOSE, "Refreshing hardware device list...\n");
		pcaninfo_get(&g_basiccore.devices, 1);
		clock_gettime(CLOCK_MONOTONIC, &g_basiccore.last_update);
	}
}

void pcanbasic_atexit(void) {
	pcanbasic_channel *plist;
//TODO FIXME: recursive call to atexit()
	/* assert API is initialized */
	if (!g_basiccore.initialized) {
		return;
	}
	pcanlog_log(LVL_VERBOSE, "Cleaning up PCAN-Basic API...\n");
	/* look through initialized channels */
	for (plist = g_basiccore.channels.slh_first; plist != NULL; plist = plist->entries.sle_next) {
		pcanbasic_uninitialize(plist->channel);
	}
	if (g_basiccore.devices) {
		free(g_basiccore.devices);
		g_basiccore.devices = NULL;
		memset(&g_basiccore, 0, sizeof(g_basiccore));
	}
}

pcanbasic_channel * pcanbasic_get_channel(TPCANHandle channel, __u8 opened, TPCANStatus* status) {
	pcanbasic_channel *result = NULL;
	TPCANStatus sts = PCAN_ERROR_ILLHANDLE;
	pcanbasic_channel *plist;

	/* assert API is initialized */
	if (!g_basiccore.initialized) {
		pcanbasic_init();
	}
	/* look through initialized channels */
	if (channel != PCAN_NONEBUS) {
		sts = PCAN_ERROR_ILLHW;
		for (plist = g_basiccore.channels.slh_first; plist != NULL; plist = plist->entries.sle_next) {
			if (plist->channel == channel) {
				sts = PCAN_ERROR_OK;
				result = plist;
				if (opened && (plist->fd < 0)) {
					result =  NULL;
					sts = PCAN_ERROR_INITIALIZE;
				}
				break;
			}
		}
		if (sts != PCAN_ERROR_OK) {
			/* check if the handle is valid	*/
			pcanbasic_get_device(channel, 0, 0, 0, &sts);
			/* a valid handle with no channel found is simply not initialized	*/
			if (sts == PCAN_ERROR_OK && opened) {
				sts = PCAN_ERROR_INITIALIZE;
			}
		}
	}

	if (status != NULL)
		*status = sts;
	return result;
}

void pcanbasic_free_channel(pcanbasic_channel * pchan) {
	if (pchan == NULL)
		return;
	if (pchan->fd > -1) {
		/* device is opened, handle pending tx msgs...	*/
		struct pcanfd_state fds;
		int ires;
		struct timespec t, tstart, tsub;
		uint8_t loop = 1;

		/* loop to wait for Tx queue to be cleared (50 ms periodic check up to 500ms)	*/
		clock_gettime(CLOCK_MONOTONIC, &tstart);
		while (loop) {
			ires = pcanfd_get_state(pchan->fd, &fds);
			loop = (ires == 0 && fds.tx_pending_msgs > 0);
			if (loop) {
				/* compute total waiting time	*/
				clock_gettime(CLOCK_MONOTONIC, &t);
				timespecsub(&t, &tstart, &tsub);
				if (tsub.tv_sec > 0 || t.tv_nsec > UNINITIALIZE_MAX_WAITING_TIME_NANO) {
					/* timeout */
					loop = 0;
				}
				else {
					/* wait some time to transmit any pending msgs */
					usleep(50000);
				}
			}
		}
		/* abort any pending Tx frames	*/
		pcanfd_reset(pchan->fd, PCANFD_RESET_TXFIFO);
		pcanfd_close(pchan->fd);
		pchan->fd = -1;
	}
	if (pchan->bitratefd) {
		free(pchan->bitratefd);
		pchan->bitratefd = NULL;
	}
	if (pchan->pinfo) {
		free(pchan->pinfo);
		pchan->pinfo = NULL;
	}
	pcbtrace_close(&pchan->tracer);
	pcbtrace_release(&pchan->tracer);
	free(pchan);
}

TPCANStatus pcanbasic_get_hw(TPCANHandle channel, enum pcaninfo_hw *hw, __u32 *index) {
	TPCANStatus sts = PCAN_ERROR_OK;
	__u32 index_max = 0;

	/* get the device's hardware category and minor/index from the channel */
	if (channel > 0xFF) {
		*hw = ((channel & 0xFF00) >> 8);
		*index = (channel & 0xFF);
	}
	else {
		*hw = ((channel & 0xF0) >> 4);
		*index = (channel & 0x0F);
	}
	switch (*hw) {
		case PCANINFO_HW_PCI:
		case PCANINFO_HW_USB:
		case PCANINFO_HW_LAN:
			index_max = 16;
			break;
		case PCANINFO_HW_ISA:
			index_max = 6;
			break;
		case PCANINFO_HW_DNG:
			index_max = 1;
			break;
		case PCANINFO_HW_PCC:
			index_max = 2;
			break;
		case PCANINFO_HW_PEAKCAN:
			/* not supported */
		case PCANINFO_HW_VIRTUAL:
			/* not supported */
			sts = PCAN_ERROR_ILLHW;
			*hw = PCANINFO_HW_NONE;
			*index = 0;
			break;
		case PCANINFO_HW_NONE:
		default:
			sts = PCAN_ERROR_ILLHANDLE;
			*index = 0;
			break;
	}
	if (sts == PCAN_ERROR_OK && *index > index_max) {
		sts = PCAN_ERROR_ILLHANDLE;
	}
	return sts;
}

__u8 pcanbasic_check_hw(TPCANHandle channel, __u8 allow_nonebus) {
	enum pcaninfo_hw hw;
	__u32 index;

	if (channel == PCAN_NONEBUS) {
		return allow_nonebus;
	}
	pcanbasic_get_hw(channel, &hw, &index);
	return (hw != PCANINFO_HW_NONE);
}

struct pcaninfo* pcanbasic_get_device(TPCANHandle channel, __u32 hwtype, __u32 base, __u32 irq, TPCANStatus* status) {
	int i;
	struct pcaninfo* pinfo;
	struct pcaninfo* result = NULL;
	enum pcaninfo_hw hw;
	__u32 index, count;
	struct timespec t, tsub;
	TPCANStatus sts;

	/* get time to see if device hw info needs an update */
	clock_gettime(CLOCK_MONOTONIC, &t);
	timespecsub(&t, &g_basiccore.last_update, &tsub);
	if (tsub.tv_sec > 0 || tsub.tv_nsec > PCANINFO_TIME_REFRESH_NANO)
		pcanbasic_refresh_hw();
	/* get the device's hardware category and minor/index from the channel */
	sts = pcanbasic_get_hw(channel, &hw, &index);
	if (sts == PCAN_ERROR_OK) {
		/* the handle is supported, change default error assuming channel is not initialized */
		sts = PCAN_ERROR_INITIALIZE;
		/* detection algorithm depends on HW being plug'n play */
		switch(hw) {
		case PCANINFO_HW_PCC:
		case PCANINFO_HW_PCI:
		case PCANINFO_HW_USB:
			count = 0;
			/* loop through all known devices */
			for (i = 0; i < g_basiccore.devices->length; i++) {
				pinfo = &g_basiccore.devices->infos[i];
				/* select only devices with the same category */
				if (pinfo->hwcategory == hw) {
					/* the idea is to match the index of the channel
					* and the "pseudo-detection index" of the device
					* within that category: ex. PCAN_USBBUS3 will match
					* the third USB device detected in sysfs.
					* Remember that in linux each channel of a harware
					* is seen as a device, so a PCAN-USB-PRO device
					* (2 channels) is seen in as two different 2 devices.
					*/
					count++;
					if (count == index) {
						result = pinfo;
						sts = PCAN_ERROR_OK;
						i = g_basiccore.devices->length;
					}
				}
			}
			break;
		case PCANINFO_HW_DNG:
		case PCANINFO_HW_ISA:
			/* loop through all known devices and find matching device */
			for (i = 0; i < g_basiccore.devices->length; i++) {
				pinfo = &g_basiccore.devices->infos[i];
				if (pinfo->hwcategory == hw &&
						pinfo->hwtype == hwtype &&
						pinfo->base == base &&
						pinfo->irq == irq) {
					result = pinfo;
					sts = PCAN_ERROR_OK;
					i = g_basiccore.devices->length;
				}
			}
			break;
		case PCANINFO_HW_NONE:
			sts = PCAN_ERROR_ILLHANDLE;
			break;
		case PCANINFO_HW_LAN:
		case PCANINFO_HW_PEAKCAN:
		case PCANINFO_HW_VIRTUAL:
			sts = PCAN_ERROR_NODRIVER;
			break;
		default:
			sts = PCAN_ERROR_ILLHW;
			break;
		}
	}
	if(status != NULL)
		*status = sts;
	return result;
}

TPCANStatus pcanbasic_errno_to_status(int err) {
	TPCANStatus sts;
	switch(err) {
	case EAGAIN: /* same as case EWOULDBLOCK: */
		sts = PCAN_ERROR_CAUTION;
		break;
	case ENODEV:
	case EBADF:
		sts = PCAN_ERROR_ILLHW;
		break;
	case ENETDOWN:
		sts = PCAN_ERROR_BUSOFF;
		break;
	case EBADMSG:
	case EINVAL:
		sts = PCAN_ERROR_ILLPARAMVAL;
		break;
	case EOPNOTSUPP:
		sts = PCAN_ERROR_ILLOPERATION;
		break;
	case 0:
		sts = PCAN_ERROR_OK;
		break;
	case EBUSY:
		/* system error (pcanfd_set_init: device is opened more than once) */
	case EIO:
		/* system error (with old PCAN cards) */
	case EINTR:
		/* should never occur as it is returned when open mode is blocking */
	case EFAULT:
		/* system error */
	case ENOMEM:
		/* system error */
	case ENOSPC:
		/* overrun or panfd_set_option: wrong size passed for the given parameter */
	case ETIMEDOUT:
		/* system error */
	default:
		pcanlog_log(LVL_NORMAL, "Error unhandled errno (%d / 0x%x).\n", err, err);
		sts = PCAN_ERROR_UNKNOWN;
		break;
	}
	return sts;
}

TPCANStatus pcanbasic_errno_to_status_ctx(int err, int ctx) {
	switch(err) {
	case EAGAIN:
		switch (ctx) {
		case PCB_CTX_READ:
			return PCAN_ERROR_QRCVEMPTY;
		case PCB_CTX_WRITE:
			return PCAN_ERROR_QXMTFULL;
		}
	}
	return pcanbasic_errno_to_status(err);
}

pcanbasic_channel* pcanbasic_create_channel(TPCANHandle channel, __u8 add_to_list) {
	pcanbasic_channel* pchan;

	pchan = (pcanbasic_channel*) calloc(1, sizeof(pcanbasic_channel));
	if (pchan == NULL) {
		return NULL;
	}
	pchan->pinfo = (struct pcaninfo*) calloc(1, sizeof(struct pcaninfo));
	if (pchan->pinfo == NULL) {
		free(pchan);
		return NULL;
	}
	pchan->channel = channel;
	pchan->fd = -1;
	pchan->bitrate_adapting = DEFAULT_PARAM_BITRATE_ADAPTING;
	pchan->listen_only = DEFAULT_PARAM_LISTEN_ONLY;
	pchan->rcv_status = DEFAULT_PARAM_RCV_STATUS;
	pchan->echo_status = DEFAULT_PARAM_ECHO_STATUS;
	pchan->rcv_echo_status = DEFAULT_PARAM_ECHO_STATUS;
	pcbtrace_set_defaults(&pchan->tracer);
	pchan->tracer.pinfo = pchan->pinfo;
	if (add_to_list)
		SLIST_INSERT_HEAD(&g_basiccore.channels, pchan, entries);
	return pchan;
}

TPCANStatus pcanbasic_bus_state_to_condition(enum pcanfd_status bus_state, __u8 fd) {
	TPCANStatus sts = PCAN_ERROR_OK;

	if ((bus_state & PCANFD_ERROR_WARNING) == PCANFD_ERROR_WARNING)
		sts |= PCAN_ERROR_BUSLIGHT;
	if ((bus_state & PCANFD_ERROR_PASSIVE) == PCANFD_ERROR_PASSIVE)
		sts |= (fd == 0) ? PCAN_ERROR_BUSHEAVY : PCAN_ERROR_BUSHEAVY|PCAN_ERROR_BUSPASSIVE;
	if ((bus_state & PCANFD_ERROR_BUSOFF) == PCANFD_ERROR_BUSOFF)
		sts |= PCAN_ERROR_BUSOFF;

	return sts;
}

int pcanbasic_get_fd(TPCANHandle channel, pcanbasic_channel* pchan, struct pcaninfo** in_out_pinfo, __u8* out_fd_close) {
	struct pcaninfo* pinfo = (in_out_pinfo != NULL) ? *in_out_pinfo : NULL;
	int fd = -1;
	__u8 fd_close = 0;

	/* check if channel is not initialized */
	if (pchan == NULL || pchan->fd < 0) {
		/* even if pchan is set, do not rely on pchan->pinfo (data most probably outdated) */
		if (pinfo == NULL) {
			/* get device via sysfs */
			pinfo = pcanbasic_get_device(channel, 0, 0, 0, NULL);
		}
		if (pinfo != NULL) {
			/* simply call open (without libpcanfd to prevent controller initialization with wrong user-settings) */
			int o_flags = O_RDONLY;
#if 0	/* Disabled -> Waiting for PCAN 8.14 tests validation (possible side-effects not fully tested) */
			struct pcaninfo_version pciv = {0, 0, 0, 0, 0};
			if (pcaninfo_parse_version(g_basiccore.devices->version, &pciv) == 0 &&
				pciv.status & PCB_VERSION_DEFINED_MIN) {
				/* with 8.14, we are able to open a device without configuring it */
				if (pciv.major > 8 || (pciv.major == 8 && pciv.minor >= 14)) {
					o_flags = O_ACCMODE;
				}
			}
#endif
			fd = open(pinfo->path, o_flags);
			fd_close = 1;
		}
	}
	else {
		/* channel initialized */
		pinfo = pchan->pinfo;
		fd = pchan->fd;
	}

	if (in_out_pinfo != NULL && *in_out_pinfo == NULL) 
		*in_out_pinfo = pinfo;
	*out_fd_close = fd_close;
	return fd;
}

__u8 pcanbasic_get_filter(pcanbasic_channel * pchan) {
	int i, ires;
	__u8 res, bclosed;
	struct pcanfd_msg_filters *pfml;
	struct pcanfd_state fds;

	pfml = NULL;
	memset(&fds, 0, sizeof(fds));
	ires = pcanfd_get_state(pchan->fd, &fds);
	res = PCAN_FILTER_CLOSE;

	if (ires == 0) {
		if (fds.filters_counter > 0) {
			pfml = calloc(1, sizeof(*pfml) + fds.filters_counter * sizeof(pfml->list[0]));
			pfml->count = fds.filters_counter;
			ires = pcanfd_get_filters(pchan->fd, pfml);
			if (!ires) {
				bclosed = 1;
				/* the driver check for all filters
				* and if one match the message is allowed.
				* So an FILTER_OPEN occurs if there is no filter or
				* if a filter allow all CAN IDs.
				* Likewise it is FILTER_CLOSE if no filter allow any
				* CAN message */
				for (i = 0; i < pfml->count; i++) {
					if (pfml->list[i].id_from == 0) {
						if (pfml->list[i].msg_flags & PCANFD_MSG_EXT) {
							if (pfml->list[i].id_to == CAN_MAX_EXTENDED_ID) {
								res = PCAN_FILTER_OPEN;
								goto pcanbasic_get_filter_free;
							}
						}
						else if (pfml->list[i].id_to == CAN_MAX_STANDARD_ID) {
							res = PCAN_FILTER_OPEN;
							goto pcanbasic_get_filter_free;
						}
					}
					if (pfml->list[i].id_from <= pfml->list[i].id_to) {
						bclosed = 0;
					}
				}
				if (bclosed)
					res = PCAN_FILTER_CLOSE;
				else
					res = PCAN_FILTER_CUSTOM;
			}
		}
		else
			res = PCAN_FILTER_OPEN;
	}

pcanbasic_get_filter_free:
	if (pfml)
		free(pfml);
	return res;
}

TPCANStatus pcanbasic_get_value_device_id(TPCANHandle channel, pcanbasic_channel* pchan, void* buf, __u32 bufsize) {
	TPCANStatus sts = PCAN_ERROR_OK;
	int ires;
	__u32 itmp = 0;
	struct pcaninfo* pinfo = NULL;
	int fd = -1;
	__u8 fd_close = 0;

	if (bufsize < sizeof(itmp)) {
		return PCAN_ERROR_ILLPARAMVAL;
	}

	/* get a file descriptor even if channel is not initialized */
	fd = pcanbasic_get_fd(channel, pchan, &pinfo, &fd_close);
	/* try to get value via sysfs */
	if (pinfo != NULL && (pinfo->availflag & PCANINFO_FLAG_DEVID)) {
		memcpy(buf, &pinfo->devid, sizeof(itmp));
	}
	/* try to get value via libcanfd */
	else if (fd > -1) {
		ires = pcanfd_get_device_id(fd, (__u32*)&itmp);
		if (ires == 0) {
			memcpy(buf, &itmp, sizeof(itmp));
		}
		else {
			sts = pcanbasic_errno_to_status(-ires);
		}
	}
	else {
		sts = PCAN_ERROR_ILLHW;
	}
	if (fd_close)
		pcanfd_close(fd);

	return sts;
}

TPCANStatus pcanbasic_get_value_channel_version(TPCANHandle channel, pcanbasic_channel* pchan, void* buf, __u32 bufsize) {
	TPCANStatus sts = PCAN_ERROR_OK;
	struct pcanfd_state fds = { 0 };
	int ires;
	__u32 size;
	struct pcaninfo* pinfo = NULL;
	int fd = -1;
	__u8 fd_close = 0;
	char buffer[256];

	/* get a file descriptor even if channel is not initialized */
	fd = pcanbasic_get_fd(channel, pchan, &pinfo, &fd_close);
	/* try to get value via libcanfd */
	if (fd > -1) {
		ires = pcanfd_get_state(fd, &fds);
		if (ires == 0) {
			enum pcaninfo_hw hw_type; 
			__u32 hw_index;
			pcanbasic_get_hw(channel, &hw_type, &hw_index);	
			snprintf(buffer, sizeof(buffer), "%s %d.%d.%d\n%s", pcaninfo_hw_to_string(hw_type, 1), fds.ver_major, fds.ver_minor, fds.ver_subminor, CHANNEL_VERSION_EXTRA);
			size = strnlen(buffer, sizeof(buffer));
			if (size + 1 <= bufsize) {
				memcpy(buf, buffer, size + 1);
			} else {
				sts = PCAN_ERROR_ILLPARAMVAL;
			}
		}
		else {
			sts = pcanbasic_errno_to_status(-ires);
		}
	}
	else {
		sts = PCAN_ERROR_ILLHW;
	}
	if (fd_close)
		pcanfd_close(fd);

	return sts;
}

TPCANStatus pcanbasic_get_value_channel_condition(TPCANHandle channel, pcanbasic_channel* pchan) {
	TPCANStatus sts = PCAN_ERROR_UNKNOWN;
	struct pcaninfo * pinfo = NULL;

	/* check if channel is not initialized */
	if (pchan == NULL || pchan->fd < 0) {
		/* get device via sysfs (will refresh 'bus_state' if needed) */
		pinfo = pcanbasic_get_device(channel, 0, 0, 0, NULL);
		pcaninfo_update(pinfo);
		if (pinfo)
			sts = (pinfo->bus_state == PCANFD_UNKNOWN) ? PCAN_CHANNEL_AVAILABLE : PCAN_CHANNEL_OCCUPIED;
		else
			sts = PCAN_CHANNEL_UNAVAILABLE;
	}
	else {
		/* channel is initialized */
		sts = PCAN_CHANNEL_OCCUPIED;
	}
	return sts;
}

TPCANStatus pcanbasic_get_value_channel_features(TPCANHandle channel, pcanbasic_channel* pchan, void* buf, __u32 bufsize) {
	TPCANStatus sts = PCAN_ERROR_OK;
	int ires;
	__u32 itmp = 0;
	__u32 device_features = 0;
	struct pcaninfo* pinfo = NULL;
	int fd = -1;
	__u8 fd_close = 0;

	if (bufsize < sizeof(device_features)) {
		return PCAN_ERROR_ILLPARAMVAL;
	}

	/* get a file descriptor even if channel is not initialized */
	fd = pcanbasic_get_fd(channel, pchan, &pinfo, &fd_close);
	/* try to get value via libcanfd */
	if (fd > -1) {
		/* request libcanfd's features */
		ires = pcanfd_get_option(fd, PCANFD_OPT_CHANNEL_FEATURES, &itmp, sizeof(itmp));
		if (ires >= 0) {
			if (itmp & PCANFD_FEATURE_FD)
				device_features |= FEATURE_FD_CAPABLE;
			if (itmp & PCANFD_FEATURE_IFRAME_DELAYUS)
				device_features |= FEATURE_DELAY_CAPABLE;
		}
		/* IO feature must be retrieved from another libcanfd's parameter */
		ires = pcanfd_get_option(fd, PCANFD_IO_DIGITAL_CFG, &itmp, sizeof(itmp));
		if (ires >= 0)
			device_features |= FEATURE_IO_CAPABLE;
		if (fd_close)
			pcanfd_close(fd);
		memcpy(buf, &device_features, sizeof(device_features));
	}
	else {
		sts = PCAN_ERROR_ILLHW;
	}
	return sts;
}

TPCANStatus pcanbasic_get_value_controller_number(TPCANHandle channel, pcanbasic_channel* pchan, void* buf, __u32 bufsize) {
	TPCANStatus sts = PCAN_ERROR_ILLPARAMTYPE;
	int ires;
	struct pcanfd_state fds = { 0 };
	struct pcaninfo* pinfo = NULL;
	int fd = -1;
	__u8 fd_close = 0;

	if (bufsize < sizeof(fds.channel_number)) {
		return PCAN_ERROR_ILLPARAMVAL;
	}

	/* get a file descriptor even if channel is not initialized */
	fd = pcanbasic_get_fd(channel, pchan, &pinfo, &fd_close);
	/* try to get value via libcanfd */
	if (fd > -1) {
		ires = pcanfd_get_state(fd, &fds);
		if (ires == 0) {
			memcpy(buf, &fds.channel_number, sizeof(fds.channel_number));
			sts = PCAN_ERROR_OK;
		}
		else {
			sts = pcanbasic_errno_to_status(-ires);
		}
	}
	else {
		sts = PCAN_ERROR_ILLHW;
	}
	if (fd_close)
		pcanfd_close(fd);

	return sts;
}

TPCANStatus pcanbasic_get_value_bitrate_info(TPCANHandle channel, pcanbasic_channel* pchan, void* buf, __u32 bufsize) {
	TPCANStatus sts = PCAN_ERROR_ILLPARAMTYPE;
	__u16 value = 0;
	struct pcaninfo* pinfo = NULL;

	if (bufsize < sizeof(value)) {
		return PCAN_ERROR_ILLPARAMVAL;
	}

	/* try to get sysfs info */
	pinfo = (pchan != NULL) ? pchan->pinfo : pcanbasic_get_device(channel, 0, 0, 0, &sts);
	if (pinfo != NULL) {
		/* check if channel is initialized as FD (should use pcanbasic_get_value_bitrate_info_fd) */
		if (pinfo->bus_state != PCANFD_UNKNOWN && (pinfo->init_flags & PCANFD_INIT_FD) == PCANFD_INIT_FD) {
			sts = PCAN_ERROR_ILLOPERATION;
		}
		else {
			value = (__u16)pinfo->btr0btr1;
			memcpy(buf, &value, sizeof(value));
			sts = PCAN_ERROR_OK;
		}
	}
	else {
		/* PCAN_ERROR_INITIALIZE is irrelevant as parameter does not require initialization */
		if (sts == PCAN_ERROR_INITIALIZE)
			sts = PCAN_ERROR_ILLHW;
	}
	return sts;
}
TPCANStatus pcanbasic_get_value_bitrate_info_fd(TPCANHandle channel, pcanbasic_channel* pchan, void* buf, __u32 bufsize) {
	TPCANStatus sts = PCAN_ERROR_ILLPARAMTYPE;
	__u32 size = 140;
	struct pcaninfo* pinfo = NULL;

	if (bufsize < size) {
		return PCAN_ERROR_ILLPARAMVAL;
	}

	/* try to get sysfs info */
	pinfo = (pchan != NULL) ? pchan->pinfo : pcanbasic_get_device(channel, 0, 0, 0, &sts);
	if (pinfo != NULL) {
		/* check if channel is initialized as classic CAN (should use pcanbasic_get_value_bitrate_info) */
		if (pinfo->bus_state != PCANFD_UNKNOWN && (pinfo->init_flags & PCANFD_INIT_FD) == 0) {
			sts = PCAN_ERROR_ILLOPERATION;
		}
		else {
			/* example:
			 *	f_clock=80000000,nom_brp=10,nom_tseg1=5,nom_tseg2=2,nom_sjw=1,
			 *	data_brp=4,data_tseg1=7,data_tseg2=2,data_sjw=1
			 *
			 *	Note: this is similar to pcaninfo_bitrate_to_init_string(...)
			 *	but ensures all string-parameters are present even if
			 *	corresponding PCANINFO_FLAG_XXX is not available.
			 */
			snprintf(buf, bufsize, "%s=%d,%s=%d,%s=%d,%s=%d,%s=%d,%s=%d,%s=%d,%s=%d,%s=%d", PCAN_BR_CLOCK, pinfo->clock,
					PCAN_BR_NOM_BRP, pinfo->nom_brp,
					PCAN_BR_NOM_TSEG1, pinfo->nom_tseg1,
					PCAN_BR_NOM_TSEG2, pinfo->nom_tseg2,
					PCAN_BR_NOM_SJW, pinfo->nom_sjw,
					PCAN_BR_DATA_BRP, pinfo->data_brp,
					PCAN_BR_DATA_TSEG1, pinfo->data_tseg1,
					PCAN_BR_DATA_TSEG2, pinfo->data_tseg2,
					PCAN_BR_DATA_SJW, pinfo->data_sjw);
			sts = PCAN_ERROR_OK;
		}
	}
	else {
		/* PCAN_ERROR_INITIALIZE is irrelevant as parameter does not require initialization */
		if (sts == PCAN_ERROR_INITIALIZE)
			sts = PCAN_ERROR_ILLHW;
	}
	return sts;
}

TPCANStatus pcanbasic_get_value_adapter_part_number(TPCANHandle channel, pcanbasic_channel* pchan, void* buf, __u32 bufsize) {
	TPCANStatus sts = PCAN_ERROR_ILLPARAMTYPE;
	__u32 size = 15; // at least "IPEH-0002021"
	struct pcaninfo* pinfo = NULL;

	if (bufsize < size) {
		return PCAN_ERROR_ILLPARAMVAL;
	}
	memset(buf, 0, bufsize);
	/* try to get sysfs info */
	pinfo = (pchan != NULL) ? pchan->pinfo : pcanbasic_get_device(channel, 0, 0, 0, &sts);
	if (pinfo != NULL) {
		if (pinfo->availflag_ex & PCANINFO_FLAG_EX_ADAPTER_PARTNUM) {
			size = strlen(pinfo->adapter_partnum);
			if (bufsize < size + 1) {	// +1: ensures null-terminated string
				sts = PCAN_ERROR_ILLPARAMVAL;
			} else {
				sts = PCAN_ERROR_OK;
				strcpy(buf, pinfo->adapter_partnum);
			}
		}
	}
	else {
		/* PCAN_ERROR_INITIALIZE is irrelevant as parameter does not require initialization */
		if (sts == PCAN_ERROR_INITIALIZE)
			sts = PCAN_ERROR_ILLHW;
	}
	return sts;
}

__u8 pcanbasic_get_fd_dlc(int len) {
	if (len < 0)
		return 0;
	if (len <= 8)
		return len;
	if (len <= 12)
		return 9;
	if (len <= 16)
		return 10;
	if (len <= 20)
		return 11;
	if (len <= 24)
		return 12;
	if (len <= 32)
		return 13;
	if (len <= 48)
		return 14;
	if (len <= 64)
		return 15;
	return 0x0F;
}

int pcanbasic_get_fd_len(__u8 dlc) {
	dlc = dlc & 0x0F;
	if (dlc <= 8)
		return dlc;
	switch(dlc) {
	case 9:
		return 12;
	case 10:
		return 16;
	case 11:
		return 20;
	case 12:
		return 24;
	case 13:
		return 32;
	case 14:
		return 48;
	case 15:
	default:
		return 64;
	}
}

TPCANStatus pcanbasic_init_fw_features(pcanbasic_channel* pchan) {
	struct pcaninfo_version version_drv;
	struct pcaninfo_version version_fw;
	TPCANStatus sts = PCAN_ERROR_ILLDATA;
	int ires, itmp = 0;
	uint8_t is_devdata = 0;

	/* skip if already initialized */
	if (pchan->features.initialized)
		goto pcanbasic_init_fw_features_exit;
	/* parse versions and asserts data is well-formed */
	if (pcaninfo_parse_version(g_basiccore.devices->version, &version_drv) != 0 || 
		pcaninfo_parse_version(pchan->pinfo->adapter_version, &version_fw) != 0 ||
		!(version_drv.status & PCB_VERSION_DEFINED_MIN) || 
		!(version_fw.status & PCB_VERSION_DEFINED_MIN)) 
	{
		sts = PCAN_ERROR_ILLDATA;
		pcanlog_log(LVL_ALWAYS, "Failed to check PCAN driver %s and %s fw %s information.\n", g_basiccore.devices->version, pchan->pinfo->adapter_name, pchan->pinfo->adapter_version);
		goto pcanbasic_init_fw_features_finalize;
	}
	/* assert PCAN driver >=8.14 */
	if (!(version_drv.major > 8 || (version_drv.major == 8 && version_drv.minor >= 14)))
	{
		sts = PCAN_ERROR_NODRIVER;
		pcanlog_log(LVL_ALWAYS, "PCAN Linux driver %s is obsolete and should be upgraded.\n", g_basiccore.devices->version);
		goto pcanbasic_init_fw_features_finalize;
	}
	/* get supported features from the driver */
	ires = pcanfd_get_option(pchan->fd, PCANFD_OPT_CHANNEL_FEATURES, &itmp, sizeof(itmp));
	if (ires < 0) {
		sts = PCAN_ERROR_ILLOPERATION;
		pcanlog_log(LVL_ALWAYS, "Failed to check channel's features for %s fw %s (%d).\n", pchan->pinfo->adapter_name, pchan->pinfo->adapter_version, ires);
		goto pcanbasic_init_fw_features_finalize;
	}
	sts = PCAN_ERROR_OK;

	/* forward fw_update notice */
	if (((itmp & PCANFD_FEATURE_NEW_FW_AV) == PCANFD_FEATURE_NEW_FW_AV)) {
		pcanlog_log(LVL_ALWAYS, "%s fw %s has an update available and should be upgraded.\n", pchan->pinfo->adapter_name, pchan->pinfo->adapter_version);
	}
	/* fw_feature: devdata */
	if (((itmp & PCANFD_FEATURE_DEVDATA) == PCANFD_FEATURE_DEVDATA)) {
		is_devdata = 1;
		pchan->features.echo_frames = pchan->echo_status;
	}
	/* backward compatibility checks */
	if (!is_devdata) {
		int compat = __PCB_IS_FW_ECHO(pchan->pinfo);
		if (compat > 0) {
			pchan->features.echo_frames = pchan->echo_status;
		}
		else {
			if (compat == 0)
				pcanlog_log(LVL_ALWAYS, "%s fw %s is obsolete and should be upgraded.\n", pchan->pinfo->adapter_name, pchan->pinfo->adapter_version);
			else 
				pcanlog_log(LVL_DEBUG, "Device '%s' with fw %s does not support extended features.\n", pchan->pinfo->adapter_name, pchan->pinfo->adapter_version);
		}
	}

pcanbasic_init_fw_features_finalize:
	pchan->features.initialized = 1;
	pchan->features.status = sts;
pcanbasic_init_fw_features_exit:
	return pchan->features.status;
}

TPCANStatus pcanbasic_set_hwtimestamp_mode(pcanbasic_channel* pchan) {
	__u32 param;
	int ires;

	/* override driver/device settings for HWTIMESTAMP mode */
	param = PCANFD_OPT_HWTIMESTAMP_COOKED;
	ires = pcanfd_set_option(pchan->fd, PCANFD_OPT_HWTIMESTAMP_MODE, &param, sizeof(param));
	if (ires < 0) {
		pcanlog_log(LVL_NORMAL, "Failed to set PCANFD_OPT_HWTIMESTAMP_MODE to PCANFD_OPT_HWTIMESTAMP_COOKED(%d) (sts=%d)\n", param, ires);
		/* fallback to SW timestamps */
		param = PCANFD_OPT_HWTIMESTAMP_OFF;
		ires = pcanfd_set_option(pchan->fd, PCANFD_OPT_HWTIMESTAMP_MODE, &param, sizeof(param));
		pcanlog_log(LVL_DEBUG, "Switching PCANFD_OPT_HWTIMESTAMP_MODE to PCANFD_OPT_HWTIMESTAMP_OFF(%d) (sts=%d)\n", param, ires);
	}
	return pcanbasic_errno_to_status(ires);
}

TPCANStatus pcanbasic_set_value_device_id(TPCANHandle channel, pcanbasic_channel* pchan, void* buf, __u32 bufsize) {
	TPCANStatus sts = PCAN_ERROR_OK;
	int ires;
	__u32 itmp;
	struct pcaninfo* pinfo = NULL;
	int fd = -1;
	__u8 fd_close = 0;

	if (bufsize > sizeof(itmp))
		bufsize = sizeof(itmp);
	memcpy(&itmp, buf, bufsize);

	/* check if channel is not initialized */
	fd = pcanbasic_get_fd(channel, pchan, &pinfo, &fd_close);
	/* try to set value via libcanfd */
	if (fd > -1) {
		ires = pcanfd_set_device_id(fd, itmp);
		if (ires < 0) {
			sts = pcanbasic_errno_to_status(-ires);
		}
		if (pchan != NULL)
			pcaninfo_update(pchan->pinfo);
	}
	else {
		sts = PCAN_ERROR_ILLHW;
	}
	if (fd_close)
		pcanfd_close(fd);

	return sts;
}

#define BITRATE_FD_CHECK_CLOCK_HZ_DEFINED	0x0001
#define BITRATE_FD_CHECK_CLOCK_MHZ_DEFINED	0x0002
#define BITRATE_FD_CHECK_CLOCK_INVALID		(BITRATE_FD_CHECK_CLOCK_HZ_DEFINED|BITRATE_FD_CHECK_CLOCK_MHZ_DEFINED)
#define BITRATE_FD_CHECK_BRP_NOM_DEFINED	0x0004
#define BITRATE_FD_CHECK_BRP_DATA_DEFINED	0x0008
#define BITRATE_FD_CHECK_BRP_DATA_REQUIRED	0x0010
#define BITRATE_FD_CHECK_STRING_INVALID		0x0020
int pcanbasic_parse_fd_init(struct pcanfd_init * pfdi, TPCANBitrateFD fdbitrate) {
	char * sfd_init;
	char * tok, *saveptr1, *saveptr2;
	char * skey, *sval;
	__u32 val;
	__u32 valid = 0; 
	int res = 0;

	if (pfdi == NULL || fdbitrate == NULL) {
		return EINVAL;
	}
	/* Init string example :
	 * f_clock_mhz=20, nom_brp=5, nom_tseg1=2, nom_tseg2=1, nom_sjw=1, data_brp=2, data_tseg1=3, data_tseg2=1, data_sjw=1
	 */
	memset(pfdi, 0, sizeof(*pfdi));
	sfd_init = strndup(fdbitrate, 500);
	if (sfd_init == NULL)
		return ENOMEM;
	pcanlog_log(LVL_DEBUG, "Parsing FD string: '%s'.\n", sfd_init);
	/* Each pair of parameter/value must be separated with a ','.
		 Blank spaces are allowed but are not necessary. */
	tok = strtok_r(sfd_init, ",", &saveptr1);
	if (tok == NULL)
		valid |= BITRATE_FD_CHECK_STRING_INVALID;
	while (tok) {
		pcanlog_log(LVL_DEBUG, "Parsing key/value pair: '%s'.\n", tok);
		/* The value for each parameter must be separated with a '='. */
		skey = strtok_r(tok, "=", &saveptr2);
		if (skey != NULL) {
			sval = strtok_r(NULL, "=", &saveptr2);
			if (sval != NULL) {
				skey = pcanbasic_trim(skey);
				sval = pcanbasic_trim(sval);
				val = strtoul(sval, NULL, 0);
				pcanlog_log(LVL_DEBUG, "Parsing key/value pair: '%s' = '%s'.\n", skey, sval);
				if(strcmp(skey, FD_PARAM_INIT_CLOCK_HZ) == 0) {
					pfdi->clock_Hz = val;
					valid |= BITRATE_FD_CHECK_CLOCK_HZ_DEFINED;
				}
				else if(strcmp(skey, FD_PARAM_INIT_CLOCK_MHZ) == 0) {
					pfdi->clock_Hz = val * 1000000;
					valid |= BITRATE_FD_CHECK_CLOCK_MHZ_DEFINED;
				}
				else if(strcmp(skey, FD_PARAM_INIT_NOM_BRP) == 0) {
					pfdi->nominal.brp = val;
					valid |= BITRATE_FD_CHECK_BRP_NOM_DEFINED;
				}
				else if(strcmp(skey, FD_PARAM_INIT_NOM_TSEG1) == 0) {
					pfdi->nominal.tseg1 = val;
				}
				else if(strcmp(skey, FD_PARAM_INIT_NOM_TSEG2) == 0) {
					pfdi->nominal.tseg2 = val;
				}
				else if(strcmp(skey, FD_PARAM_INIT_NOM_SJW) == 0) {
					pfdi->nominal.sjw = val;
				}
				else if(strcmp(skey, FD_PARAM_INIT_DATA_BRP) == 0) {
					pfdi->data.brp = val;
					valid |= BITRATE_FD_CHECK_BRP_DATA_DEFINED;
				}
				else if(strcmp(skey, FD_PARAM_INIT_DATA_TSEG1) == 0) {
					pfdi->data.tseg1 = val;
					valid |= BITRATE_FD_CHECK_BRP_DATA_REQUIRED;
				}
				else if(strcmp(skey, FD_PARAM_INIT_DATA_TSEG2) == 0) {
					pfdi->data.tseg2 = val;
					valid |= BITRATE_FD_CHECK_BRP_DATA_REQUIRED;
				}
				else if(strcmp(skey, FD_PARAM_INIT_DATA_SJW) == 0) {
					pfdi->data.sjw = val;
					valid |= BITRATE_FD_CHECK_BRP_DATA_REQUIRED;
				}
			}
			else {
				valid |= BITRATE_FD_CHECK_STRING_INVALID;
			}
		}
		tok = strtok_r(NULL, ",", &saveptr1);
	}
	free(sfd_init);

	/* The string must contain only one of the two possible "Clock Frequency" parameters, 
		depending on the unit used (Hz, or MHz). */
	if ((valid & BITRATE_FD_CHECK_STRING_INVALID) ||
		(valid & BITRATE_FD_CHECK_CLOCK_INVALID) == BITRATE_FD_CHECK_CLOCK_INVALID)
		res = EINVAL;
	else {
		/* The frequency to use must be one of the 6 listed within the "Clock Frequency" parameters. */
		switch (pfdi->clock_Hz) {
			case 80000000:
			case 60000000:
			case 40000000:
			case 30000000:
			case 24000000:
			case 20000000:
				/* OK */
				break;
			default:
				res = EINVAL;
				break;
		}
	}
	/* Both Bit rates, or only the nominal one, must be defined within the string (PCAN_BR_DATA_* and PCAN_BR_NOM_*, or only PCAN_BR_NOM_*). */
	if ((valid & BITRATE_FD_CHECK_BRP_NOM_DEFINED) == 0 ||
		((valid & BITRATE_FD_CHECK_BRP_DATA_REQUIRED) &&
		 (valid & BITRATE_FD_CHECK_BRP_DATA_DEFINED) == 0
		))
		res = EINVAL;
	return res;
}

int pcanbasic_parse_u32(__u32* buf, const char* str) {
	char* sval;
	size_t len;
	int base;
	int res = 0;

	sval = strndup(str, 500);
	if (sval != NULL) {		
		len = strlen(sval);
		if (len > 0) {
			base = 10;
			if (tolower(sval[len - 1]) == 'h') {
				base = 16;
				sval[len - 1] = '\0';
			}
			else if (sval[0] == '0' && tolower(sval[1]) == 'x') {
				base = 16;
				strncpy(sval, &str[2], len - 2 + 1);
			}
			else if (sval[0] != '+' && (sval[0] < '0' || sval[0] > '9')) {
				res = EINVAL;
			}
			if (res == 0)
				*buf = strtoul(sval, NULL, base);	
		}
		free(sval);
		sval = NULL;
	}
	else {
		res = ENOMEM;
	}
	return res;
}

int pcanbasic_parse_ipv4(const char* str) {
	int res = 0;
	int count = 0;
	__u32 ibuf = 0;
	char ipv4[30];
	char *tok, *saveptr1; 
	
	memset(ipv4, 0, sizeof(ipv4));
	strncpy(ipv4, str, sizeof(ipv4) - 1);
	tok = strtok_r((char*)ipv4, ".", &saveptr1);
	while (tok) {
		count++;
		if (pcanbasic_parse_u32(&ibuf, tok) == 0 && (ibuf >= 0 && ibuf <= 255))
			tok = strtok_r(NULL, ".", &saveptr1);
		else {
			tok = NULL;
			res = EINVAL;
		}
	}
	if (count != 4)
		res = EINVAL;
	return res;
}

int pcanbasic_parse_lookup(pcanbasic_lookup_info* lui, char* param) {
	char* lookup_str;
	char* tok, * saveptr1, * saveptr2;
	char* skey, * sval;
	__u32 val;
	TPCANStatus sts = PCAN_ERROR_OK;

	if (lui == NULL || param == NULL) {
		return EINVAL;
	}

	/* Init string example :
	 * devicetype=pcan_usb, deviceid=7
	 */
	memset(lui, 0, sizeof(*lui));
	lookup_str = strndup(param, 500);
	if (lookup_str == NULL)
		return ENOMEM;
	pcanlog_log(LVL_DEBUG, "Parsing lookup string: '%s'.\n", lookup_str);
	tok = strtok_r(lookup_str, ",", &saveptr1);
	if (tok == NULL)
		sts = PCAN_ERROR_ILLPARAMVAL;
	while (tok && sts == PCAN_ERROR_OK) {
		pcanlog_log(LVL_DEBUG, "Parsing key/value pair: '%s'.\n", tok);
		skey = strtok_r(tok, "=", &saveptr2);
		if (skey != NULL) {
			sval = strtok_r(NULL, "=", &saveptr2);
			if (sval != NULL) {
				skey = pcanbasic_trim(skey);
				skey = pcanbasic_tolower(skey);
				sval = pcanbasic_trim(sval);
				pcanlog_log(LVL_DEBUG, "Parsing key/value pair: '%s' = '%s'.\n", skey, sval);
				if (strcmp(skey, LOOKUP_DEVICE_TYPE) == 0) {
					sval = pcanbasic_toupper(sval);
					/* order strcmp from most probable to least probable */
					if (strcmp(sval, STRINGIFY(PCAN_PCI)) == 0) {
						lui->flags |= LOOKUP_FLAG_DEV_TYPE;
						lui->dev_type = PCAN_PCI;
					}
					else if (strcmp(sval, STRINGIFY(PCAN_USB)) == 0) {
						lui->flags |= LOOKUP_FLAG_DEV_TYPE;
						lui->dev_type = PCAN_USB;
					}
					else if (strcmp(sval, STRINGIFY(PCAN_ISA)) == 0) {
						lui->flags |= LOOKUP_FLAG_DEV_TYPE;
						lui->dev_type = PCAN_ISA;
					}
					else if (strcmp(sval, STRINGIFY(PCAN_DNG)) == 0) {
						lui->flags |= LOOKUP_FLAG_DEV_TYPE;
						lui->dev_type = PCAN_DNG;
					}
					else if (strcmp(sval, STRINGIFY(PCAN_PCC)) == 0) {
						lui->flags |= LOOKUP_FLAG_DEV_TYPE;
						lui->dev_type = PCAN_PCC;
					}
					else if (strcmp(sval, STRINGIFY(PCAN_VIRTUAL)) == 0) {
						lui->flags |= LOOKUP_FLAG_DEV_TYPE;
						lui->dev_type = PCAN_VIRTUAL;
					}
					else if (strcmp(sval, STRINGIFY(PCAN_LAN)) == 0) {
						lui->flags |= LOOKUP_FLAG_DEV_TYPE;
						lui->dev_type = PCAN_LAN;
					}
					else if (strcmp(sval, STRINGIFY(PCAN_PEAKCAN)) == 0) {
						lui->flags |= LOOKUP_FLAG_DEV_TYPE;
						lui->dev_type = PCAN_PEAKCAN;
					}
					else if (strcmp(sval, STRINGIFY(PCAN_NONE)) == 0) {
						lui->flags |= LOOKUP_FLAG_DEV_TYPE;
						lui->dev_type = PCAN_NONE;
					}
					else {
						sts = PCAN_ERROR_ILLPARAMVAL;
					}	
				}
				else if (strcmp(skey, LOOKUP_DEVICE_ID) == 0) {
					if (pcanbasic_parse_u32(&val, sval) == 0) {
						lui->flags |= LOOKUP_FLAG_DEV_ID;
						lui->dev_id = (__u32)val;				
					}
					else {
						sts = PCAN_ERROR_ILLPARAMVAL;
					}
				}
				else if (strcmp(skey, LOOKUP_CONTROLLER_NUMBER) == 0) {
					if (pcanbasic_parse_u32(&val, sval) == 0) {
						lui->flags |= LOOKUP_FLAG_DEV_CTRL_NB;
						lui->dev_ctrl_nb = (__u32)val;				
					}
					else {
						sts = PCAN_ERROR_ILLPARAMVAL;
					}
				}
				else if (strcmp(skey, LOOKUP_IP_ADDRESS) == 0) {
					if (pcanbasic_parse_ipv4(sval) == 0) {
						lui->flags |= LOOKUP_FLAG_IP;
						strncpy(lui->ip_address, sval, sizeof(lui->ip_address) - 1);
					}
					else {
						sts = PCAN_ERROR_ILLPARAMVAL;
					}
				}
				else if (strcmp(skey, LOOKUP_HW_HANDLE) == 0) {		
					if (pcanbasic_parse_u32(&val, sval) == 0) {
						lui->flags |= LOOKUP_FLAG_HW_HANDLE;
						lui->hw_handle = (__u32)val;				
					}
				}
				else
				{
					sts = PCAN_ERROR_ILLPARAMVAL;
				}
			}
			else {
				sts = PCAN_ERROR_ILLPARAMVAL;
			}
		}
		tok = strtok_r(NULL, ",", &saveptr1);
	}
	free(lookup_str);
	return sts;
}

TPCANStatus pcanbasic_parameter_supported(TPCANHandle channel, TPCANParameter parameter, __u8 is_set)
{
	TPCANStatus result = PCAN_ERROR_ILLPARAMTYPE;
	enum pcaninfo_hw hw;
	__u32 hw_index;
	pcanbasic_get_hw(channel, &hw, &hw_index);
	switch(parameter) {
	case PCAN_DEVICE_ID:
		switch (hw) {
		case PCANINFO_HW_PCI:
		case PCANINFO_HW_USB:
		case PCANINFO_HW_LAN:
			result = PCAN_ERROR_OK;
			break;
		default:
			result = PCAN_ERROR_ILLHANDLE;
			break;
		}
		break;
	case PCAN_5VOLTS_POWER:
		switch (hw) {
		case PCANINFO_HW_PCC:
		case PCANINFO_HW_USB:
			result = PCAN_ERROR_OK;
			break;
		default:
			result = PCAN_ERROR_ILLHANDLE;
			break;
		}
		break;
	case PCAN_RECEIVE_EVENT:
	case PCAN_MESSAGE_FILTER:
		switch (hw) {
		case PCANINFO_HW_NONE:
			result = PCAN_ERROR_ILLHANDLE;
			break;
		default:
			result = PCAN_ERROR_OK;
			break;
		}
		break;
	case PCAN_API_VERSION:
		if (!is_set)
			result = PCAN_ERROR_OK;
		break;
	case PCAN_CHANNEL_VERSION:
		if (!is_set) {
			result = (channel != PCAN_NONEBUS) ? PCAN_ERROR_OK : PCAN_ERROR_ILLHANDLE;
		}
		break;
	case PCAN_BUSOFF_AUTORESET:
		result = (channel != PCAN_NONEBUS) ? PCAN_ERROR_OK : PCAN_ERROR_ILLHANDLE;
		break;
	case PCAN_LISTEN_ONLY:
		result = (channel != PCAN_NONEBUS) ? PCAN_ERROR_OK : PCAN_ERROR_ILLHANDLE;
		break;
	case PCAN_LOG_LOCATION:
	case PCAN_LOG_STATUS:
	case PCAN_LOG_CONFIGURE:
		result = (channel == PCAN_NONEBUS) ? PCAN_ERROR_OK : PCAN_ERROR_ILLHANDLE;
		break;
	case PCAN_LOG_TEXT:
		if (is_set)
			result = (channel == PCAN_NONEBUS) ? PCAN_ERROR_OK : PCAN_ERROR_ILLHANDLE;
		break;
	case PCAN_CHANNEL_CONDITION:
	case PCAN_HARDWARE_NAME:
		if (!is_set)
			result = (channel != PCAN_NONEBUS) ? PCAN_ERROR_OK : PCAN_ERROR_ILLHANDLE;			
		break;
	case PCAN_RECEIVE_STATUS:
		result = (channel != PCAN_NONEBUS) ? PCAN_ERROR_OK : PCAN_ERROR_ILLHANDLE;			
		break;
	case PCAN_CONTROLLER_NUMBER:
		if (!is_set)
			result = (channel != PCAN_NONEBUS) ? PCAN_ERROR_OK : PCAN_ERROR_ILLHANDLE;			
		break;
	case PCAN_TRACE_LOCATION:
	case PCAN_TRACE_STATUS:
	case PCAN_TRACE_SIZE:
	case PCAN_TRACE_CONFIGURE:
		result = (channel != PCAN_NONEBUS) ? PCAN_ERROR_OK : PCAN_ERROR_ILLHANDLE;			
		break;
	case PCAN_CHANNEL_IDENTIFYING:
		switch (hw) {
		case PCANINFO_HW_USB:
			result = PCAN_ERROR_OK;
			break;
		default:
			result = PCAN_ERROR_ILLHANDLE;
			break;
		}
		break;
	case PCAN_CHANNEL_FEATURES:
		if (!is_set)
			result = (channel != PCAN_NONEBUS) ? PCAN_ERROR_OK : PCAN_ERROR_ILLHANDLE;			
		break;
	case PCAN_BITRATE_ADAPTING:
		result = (channel != PCAN_NONEBUS) ? PCAN_ERROR_OK : PCAN_ERROR_ILLHANDLE;			
		break;
	case PCAN_BITRATE_INFO:
	case PCAN_BITRATE_INFO_FD:
	case PCAN_BUSSPEED_NOMINAL:
	case PCAN_BUSSPEED_DATA:
		if (!is_set)
			result = (channel != PCAN_NONEBUS) ? PCAN_ERROR_OK : PCAN_ERROR_ILLHANDLE;			
		break;
	case PCAN_IP_ADDRESS:
		switch (hw) {
		case PCANINFO_HW_LAN:
			if (!is_set)
				result = PCAN_ERROR_OK;
			break;
		default:
			result = PCAN_ERROR_ILLHANDLE;
			break;
		}
		break;
	case PCAN_LAN_SERVICE_STATUS:
		if (is_set)
			result = (channel == PCAN_NONEBUS) ? PCAN_ERROR_OK : PCAN_ERROR_ILLHANDLE;			
		break;
	case PCAN_ALLOW_STATUS_FRAMES:
	case PCAN_ALLOW_RTR_FRAMES:
	case PCAN_ALLOW_ERROR_FRAMES:
	case PCAN_ALLOW_ECHO_FRAMES:
	case PCAN_INTERFRAME_DELAY:
	case PCAN_ACCEPTANCE_FILTER_11BIT:
	case PCAN_ACCEPTANCE_FILTER_29BIT:
		result = (channel != PCAN_NONEBUS) ? PCAN_ERROR_OK : PCAN_ERROR_ILLHANDLE;
		break;
	case PCAN_IO_DIGITAL_CONFIGURATION:
	case PCAN_IO_DIGITAL_VALUE:
		switch (hw) {
		case PCANINFO_HW_USB:
			result = PCAN_ERROR_OK;
			break;
		default:
			result = PCAN_ERROR_ILLHANDLE;
			break;
		}
		break;
	case PCAN_IO_DIGITAL_SET:
	case PCAN_IO_DIGITAL_CLEAR:
		if (is_set) {
			switch (hw) {
			case PCANINFO_HW_USB:
				result = PCAN_ERROR_OK;
				break;
			default:
				result = PCAN_ERROR_ILLHANDLE;
				break;
			}
		}
		break;
	case PCAN_IO_ANALOG_VALUE:
		if (!is_set) {
			switch (hw) {
			case PCANINFO_HW_USB:
				result = PCAN_ERROR_OK;
				break;
			default:
				result = PCAN_ERROR_ILLHANDLE;
				break;
			}
		}
		break;
	case PCAN_FIRMWARE_VERSION:
		if (!is_set)
			result = (channel != PCAN_NONEBUS) ? PCAN_ERROR_OK : PCAN_ERROR_ILLHANDLE;
		break;
	case PCAN_ATTACHED_CHANNELS_COUNT:
	case PCAN_ATTACHED_CHANNELS:
		if (!is_set)
			result = (channel == PCAN_NONEBUS) ? PCAN_ERROR_OK : PCAN_ERROR_ILLHANDLE;
		break;
	case PCAN_DEVICE_PART_NUMBER:
		if (!is_set)
			result = (channel != PCAN_NONEBUS) ? PCAN_ERROR_OK : PCAN_ERROR_ILLHANDLE;
		break;
	}
	return result;
}
TPCANStatus pcanbasic_read_common(
		pcanbasic_channel *pchan,
		TPCANMsgFD* message,
		struct timeval *t) {
	TPCANStatus sts;
	struct pcanfd_msg msg;
	int ires;
	__u8 check_filter = 0;
	__u8 msg_echoed = 0;

	if (message == NULL || pchan == NULL) {
		sts = PCAN_ERROR_ILLPARAMVAL;
		goto pcanbasic_read_common_exit;
	}
	/* read msg via libpcanfd */
	ires = pcanfd_recv_msg(pchan->fd, &msg);
	/* SGr Notes: move return code test next to the function call */
	if (ires < 0) {
		sts = pcanbasic_errno_to_status_ctx(-ires, PCB_CTX_READ);
		goto pcanbasic_read_common_exit;
	}
	/* discard message if rcv_status is OFF */
	if (pchan->rcv_status == PCAN_PARAMETER_OFF) {
		sts = PCAN_ERROR_QRCVEMPTY;
		goto pcanbasic_read_common_exit;
	}
	sts = PCAN_ERROR_OK;
	/* convert msg to PCANBasic structure */
	message->MSGTYPE = 0;
	memset(message->DATA, 0, sizeof(message->DATA));
	message->ID = msg.id;
	message->DLC = pcanbasic_get_fd_dlc(msg.data_len);
	switch (msg.type) {
	case PCANFD_TYPE_CANFD_MSG:
		message->MSGTYPE |= PCAN_MESSAGE_FD;
		/* no break */
	case PCANFD_TYPE_CAN20_MSG:
		if (msg.data_len > sizeof(message->DATA))
			pcanlog_log(LVL_ALWAYS, "Received malformed CAN message (data_len=%d)", msg.data_len);
		memcpy(message->DATA, msg.data, MIN(sizeof(message->DATA), msg.data_len));
		/* standard or extended CAN msg */
		if((msg.flags & PCANFD_MSG_EXT) == PCANFD_MSG_EXT)
			message->MSGTYPE |= PCAN_MESSAGE_EXTENDED;
		else
			message->MSGTYPE |= PCAN_MESSAGE_STANDARD;
		/* RTR msg ? */
		if((msg.flags & PCANFD_MSG_RTR) == PCANFD_MSG_RTR) {
			message->MSGTYPE |= PCAN_MESSAGE_RTR;
			/* mark to check filtering option */
			check_filter = 1;
		}
		/* FD flags */
		if((msg.flags & PCANFD_MSG_BRS) == PCANFD_MSG_BRS)
			message->MSGTYPE |= PCAN_MESSAGE_BRS;
		if((msg.flags & PCANFD_MSG_ESI) == PCANFD_MSG_ESI)
			message->MSGTYPE |= PCANFD_MSG_ESI;
		/* echo flags */
		if ((msg.flags & PCANFD_MSG_ECHO) == PCANFD_MSG_ECHO) {
			msg_echoed = 1;
			message->MSGTYPE |= PCAN_MESSAGE_ECHO;
		}
		break;
	case PCANFD_TYPE_STATUS:
		if (pchan->busoff_reset && (msg.flags & PCANFD_ERROR_BUS) && msg.id == PCANFD_ERROR_BUSOFF) {
			/* auto-reset */
			pcanbasic_reset(pchan->channel);
			sts = PCAN_ERROR_BUSOFF;
			goto pcanbasic_read_common_exit;
		}
		message->MSGTYPE = PCAN_MESSAGE_STATUS;
		message->DLC = 4;
		switch (msg.id) {
		case PCANFD_ERROR_WARNING:
			message->DATA[3] |= CAN_ERR_BUSLIGHT;
			break;
		case PCANFD_ERROR_PASSIVE:
			message->DATA[3] |= CAN_ERR_BUSHEAVY;
			break;
		case PCANFD_ERROR_BUSOFF:
			message->DATA[3] |= CAN_ERR_BUSOFF;
			break;
		case PCANFD_RX_EMPTY:
			message->DATA[3] |= CAN_ERR_QRCVEMPTY;
			break;
		case PCANFD_RX_OVERFLOW:
			message->DATA[3] |= CAN_ERR_OVERRUN;
			break;
		case PCANFD_TX_OVERFLOW:
			message->DATA[3] |= CAN_ERR_QXMTFULL;
			break;

		default:
		case PCANFD_TX_EMPTY:
			message->DATA[3] |= CAN_ERR_RESOURCE;
			break;

		case PCANFD_ERROR_ACTIVE:
			break;
		}
		/* mark to check filtering option */
		check_filter = 1;
		break;
	case PCANFD_TYPE_ERROR_MSG:
		message->ID = 1 << msg.id;
		message->MSGTYPE = PCAN_MESSAGE_ERRFRAME;
		message->DATA[0] = (msg.flags & PCANFD_ERRMSG_RX) == PCANFD_ERRMSG_RX ? 1 : 0;
		message->DATA[1] = msg.data[0];
		message->DATA[2] = msg.ctrlr_data[0];
		message->DATA[3] = msg.ctrlr_data[1];
		/* mark to check filtering option */
		check_filter = 1;
		break;
	}

	/*	workaround to ignore messages that should have been ignored by libpcanfd. */
	if (check_filter != 0) {
		int ignore = 0;
		__u32 opt_msgs = 0;
		ires = pcanfd_get_option(pchan->fd, PCANFD_OPT_ALLOWED_MSGS, &opt_msgs, sizeof(opt_msgs));
		if (ires >= 0) {
			switch (msg.type) {
			case PCANFD_TYPE_CAN20_MSG:
				ignore = ((opt_msgs & PCANFD_ALLOWED_MSG_RTR) == 0) && ((msg.flags & PCANFD_MSG_RTR) == PCANFD_MSG_RTR);
				break;
			case PCANFD_TYPE_ERROR_MSG:
				ignore = ((opt_msgs & PCANFD_ALLOWED_MSG_ERROR) == 0);
				break;
			case PCANFD_TYPE_STATUS:
				ignore = ((opt_msgs & PCANFD_ALLOWED_MSG_STATUS) == 0);
#if !defined(KEEP_STATUS_FRAME_ON_RESET)
				if (pchan->ignore_status_frame > 0) {
					pchan->ignore_status_frame--;
					ignore = 1;
				}
#endif				
				break;
			}
			if (ignore) {
				/* ignore message and read again with a tail recursion */
				pcanlog_log(LVL_DEBUG, "Ignored message: ID=0x%04x; TYPE=0x%02x; FLAGS=0x%02x; DATA=[0x%02x...].\n",
						msg.id, msg.type, msg.flags, msg.data[0]);
				return pcanbasic_read_common(pchan, message, t);
			}
		}
	}
	/* copy timestamp */
	if (t != NULL)
		*t = msg.timestamp;
	pcanlog_log(LVL_VERBOSE, "Read message: ID=0x%04x; TYPE=0x%02x; FLAGS=0x%02x; DATA=[0x%02x...].\n",
		msg.id, msg.type, msg.flags, msg.data[0]);
	/* trace message: echoed message is considered a Tx */
	pcbtrace_write_msg(&pchan->tracer, message, msg.data_len, &msg.timestamp, !msg_echoed);
	/* handle echoed message */
	if (msg_echoed && !pchan->rcv_echo_status) {
		/* ignore message and read again with a tail recursion */
		return pcanbasic_read_common(pchan, message, t);
	}

pcanbasic_read_common_exit:
	return sts;
}

TPCANStatus pcanbasic_write_common(
		pcanbasic_channel *pchan,
		TPCANMsgFD* message) {
	TPCANStatus sts;
	struct pcanfd_msg msg;
	struct timespec ts;
	struct timeval tv;
	int ires;

	if (message == NULL || pchan == NULL) {
		sts = PCAN_ERROR_ILLPARAMVAL;
		goto pcanbasic_write_exit;
	}
	/* convert message and send it */
	memset(&msg, 0, sizeof(msg));
	msg.id = message->ID;
	msg.data_len = pcanbasic_get_fd_len(message->DLC);
	memcpy(msg.data, message->DATA, MIN(sizeof(message->DATA), msg.data_len));
	/* set message FD type */
	if ((message->MSGTYPE & PCAN_MESSAGE_FD) == PCAN_MESSAGE_FD)
		msg.type = PCANFD_TYPE_CANFD_MSG;
	else
		msg.type = PCANFD_TYPE_CAN20_MSG;
	/* set message type */
	if ((message->MSGTYPE & PCAN_MESSAGE_EXTENDED) == PCAN_MESSAGE_EXTENDED)
		msg.flags = PCANFD_MSG_EXT;
	else
		msg.flags = PCANFD_MSG_STD;
	/* set extra flags */
	if ((message->MSGTYPE & PCAN_MESSAGE_RTR) == PCAN_MESSAGE_RTR)
		msg.flags |= PCANFD_MSG_RTR;
	if ((message->MSGTYPE & PCAN_MESSAGE_BRS) == PCAN_MESSAGE_BRS)
		msg.flags |= PCANFD_MSG_BRS;
	/* handle echo frame to order trace file in chronological order */
	if (pchan->echo_status)
		msg.flags |= PCANFD_MSG_ECHO;
	pcanlog_log(LVL_VERBOSE, "Writing message: ID=0x%04x; TYPE=0x%02x; FLAGS=0x%02x; DATA=[0x%02x...].\n",
		msg.id, msg.type, msg.flags, msg.data[0]);
	ires = pcanfd_send_msg(pchan->fd, &msg);
	if (ires < 0) {
		sts = pcanbasic_errno_to_status_ctx(-ires, PCB_CTX_WRITE);
		if (sts == PCAN_ERROR_QXMTFULL) {
			struct pcanfd_state fds;
			ires = pcanfd_get_state(pchan->fd, &fds);
			if (ires == 0 && fds.can_status & CAN_ERR_XMTFULL)
				sts = PCAN_ERROR_XMTFULL;
		}
		/* check busoff auto reset */
		if(sts == PCANFD_ERROR_BUSOFF && pchan->busoff_reset)
			pcanbasic_reset(pchan->channel);
		goto pcanbasic_write_exit;
	}
	clock_gettime(pchan->pinfo->clk_ref, &ts);
	/* handle trace ordering via echoed messages */
	if (!pchan->echo_status) {
		/* echoing is not supported: trace file will not be in chronological order */
		TIMESPEC_TO_TIMEVAL(&tv, &ts);
		pcbtrace_write_msg(&pchan->tracer, message, msg.data_len, &tv, 0);
	}
	sts = PCAN_ERROR_OK;

pcanbasic_write_exit:
	return sts;

}

static TPCANStatus pcanbasic_wait_bus_active(int fd, int timeout_ms) 
{
	struct pcanfd_state fds;
	enum pcanfd_status bs = PCANFD_UNKNOWN;
	struct timespec tstart, tnow, tsub;
	
	clock_gettime(CLOCK_MONOTONIC, &tstart);
	do {
		int res = pcanfd_get_state(fd, &fds); 
		if (res == 0) {
			bs = fds.bus_state;
		}
		clock_gettime(CLOCK_MONOTONIC, &tnow);
		timespecsub(&tnow, &tstart, &tsub);
	}
	while (bs == PCANFD_UNKNOWN && tsub.tv_nsec <= (timeout_ms * 1000000));
	
	return (bs == PCANFD_UNKNOWN) ? PCAN_ERROR_CAUTION : PCAN_ERROR_OK; 
}

/* GLOBAL FUNCTIONS */
struct pcaninfo * pcanbasic_get_info(TPCANHandle channel) {
	pcanbasic_channel * pcbch;

	pcbch = pcanbasic_get_channel(channel, 1, NULL);
	if (pcbch)
		return pcbch->pinfo;
	return NULL;
}

TPCANHandle pcanbasic_get_handle(char * device, struct pcaninfo_list * plist) {
	TPCANHandle result = PCAN_NONEBUS;
	struct pcaninfo_list * pcil;
	struct pcaninfo * pinfo;
	enum pcaninfo_hw hw_count[PCANINFO_HW_COUNT];
	int i;

	if (device == NULL)
		return PCAN_NONEBUS;
	/* auto-load devices information if it is not provided */
	if (plist == NULL) {
		if (pcaninfo_get(&pcil, 1) != 0)
			return result;
	}
	else
		pcil = plist;
	/* loop through PCANInfo devices and count their number by category:
	 * 	ex. PCAN_PCIBUS3 = the third PCAN PCI device */
	memset(&hw_count, 0, sizeof(hw_count));
	for (i = 0; i < pcil->length; i++) {
		pinfo = &pcil->infos[i];
		/* load pci information if it was not done yet */
		if (!(pinfo->availflag & PCANINFO_FLAG_INITIALIZED))
			pcaninfo_update(pinfo);
		hw_count[pinfo->hwcategory]++;

		/* SGr Notes: "dev" is not part of sysfs in RT world: do the
		 * cmp on the device path instead */
		//if (strcmp(pci->dev, device) == 0)
		if (strcmp(pinfo->path, device) == 0)
		{
			/* device found, will exit for loop */
			i = pcil->length;
			/* compute handle */
			switch(pinfo->hwcategory) {
			case PCANINFO_HW_DNG:
				result = PCAN_DNGBUS1 - 1 + hw_count[pinfo->hwcategory];
				break;
			case PCANINFO_HW_ISA:
				result = PCAN_ISABUS1 - 1 + hw_count[pinfo->hwcategory];
				break;
			case PCANINFO_HW_PCC:
				result = PCAN_PCCBUS1 - 1 + hw_count[pinfo->hwcategory];
				break;
			case PCANINFO_HW_PCI:
				if (hw_count[pinfo->hwcategory] < 9)
					result = PCAN_PCIBUS1 - 1 + hw_count[pinfo->hwcategory];
				else
					result = PCAN_PCIBUS9 - 9 + hw_count[pinfo->hwcategory];
				break;
			case PCANINFO_HW_USB:
				if (hw_count[pinfo->hwcategory] < 9)
					result = PCAN_USBBUS1 - 1 + hw_count[pinfo->hwcategory];
				else
					result = PCAN_USBBUS9 - 9 + hw_count[pinfo->hwcategory];
				break;
			case PCANINFO_HW_LAN:
				result = PCAN_LANBUS1 - 1 + hw_count[pinfo->hwcategory];
				break;
			case PCANINFO_HW_PEAKCAN:
			case PCANINFO_HW_VIRTUAL:
			case PCANINFO_HW_NONE:
				result = PCAN_NONEBUS;
				break;
			}
		}
	}
	/* release local allocation */
	if (plist == NULL && pcil)
		free(pcil);

	return result;
}

/* PCANBASIC API	*/
TPCANStatus pcanbasic_initialize(
		TPCANHandle channel,
		TPCANBaudrate btr0btr1,
		TPCANType hwtype,
		DWORD base,
		WORD irq) {
	TPCANStatus sts;
	pcanbasic_channel *pchan;
	struct pcaninfo * pinfo;
	__u8 inserted;
	int itmp, ires;

	inserted = 0;
	/* check if channel exists */
	pchan = pcanbasic_get_channel(channel, 0, &sts);
	if (pchan != NULL) {
		/* channel is already initialized */
		if (pchan->fd > -1) {
			sts = PCAN_ERROR_INITIALIZE;
			/* initialized with different bitrate */
			if (pchan->btr0btr1 != btr0btr1 && pchan->bitrate_adapting)
				sts = PCAN_ERROR_CAUTION;
			goto pcanbasic_initialize_exit;
		}
		/* channel is not initialized */
		else {
			/* this means that flags were set for initialization
			 * jump directly after malloc */
			inserted = 1;
			goto pcanbasic_initialize_malloc_post;
		}
	}
	/* allocate and initialize a new PCANBASIC_channel */
	pchan = pcanbasic_create_channel(channel, 0);
	if (pchan == NULL) {
		/* failed to create channel... get a more precise status code */
		pcanbasic_get_device(channel, 0, 0, 0, &sts);
		if (sts == PCAN_ERROR_OK)
			sts = PCAN_ERROR_UNKNOWN;	/* unexpected use-case */
		goto pcanbasic_initialize_exit;
	}

pcanbasic_initialize_malloc_post:
	sts = PCAN_ERROR_OK;
	/* find a corresponding device */
	pinfo = pcanbasic_get_device(channel, hwtype, base, irq, &sts);
	if (pinfo == NULL) {
		/* PCAN_ERROR_INITIALIZE is irrelevant in initialize function */
		if (sts == PCAN_ERROR_INITIALIZE)
			sts = PCAN_ERROR_ILLHW;
		if (inserted)
			SLIST_REMOVE(&g_basiccore.channels, pchan, _pcanbasic_channel, entries);
		pcanbasic_free_channel(pchan);
		goto pcanbasic_initialize_exit;
	}
	/* complete struct initialization */
	pchan->btr0btr1 = btr0btr1;
	pcaninfo_update(pinfo);
	memcpy(pchan->pinfo, pinfo, sizeof(*pinfo));
	/* check if CAN channel is used by another application */
	if (pchan->pinfo->bus_state != PCANFD_UNKNOWN) {
		/* check bitrate adaptation */
		if (!pchan->bitrate_adapting) {
			/* not allowed, compare btr0btr1 */
			if (pchan->pinfo->btr0btr1 != btr0btr1) {
				sts = PCAN_ERROR_INITIALIZE;
				if (inserted)
					SLIST_REMOVE(&g_basiccore.channels, pchan, _pcanbasic_channel, entries);
				pcanbasic_free_channel(pchan);
				goto pcanbasic_initialize_exit;
			}
		}
		else {
			sts = PCAN_ERROR_CAUTION;
		}
	}

	/* initialize time_start before opening channel */
	clock_gettime(pchan->pinfo->clk_ref, &pchan->tracer.time_start);
	/* open file descriptor */
	pchan->fd_flags = OFD_BITRATE | OFD_BTR0BTR1 | OFD_NONBLOCKING;
	if (pchan->listen_only == PCAN_PARAMETER_ON)
		pchan->fd_flags |= PCANFD_INIT_LISTEN_ONLY;
	pchan->fd = pcanfd_open(pchan->pinfo->path, pchan->fd_flags, pchan->btr0btr1);
	if (pchan->fd < 0) {
		sts = pcanbasic_errno_to_status(-pchan->fd);
		if (sts == PCAN_ERROR_UNKNOWN)
			sts = PCAN_ERROR_ILLOPERATION;
		if (inserted)
			SLIST_REMOVE(&g_basiccore.channels, pchan, _pcanbasic_channel, entries);
		pcanbasic_free_channel(pchan);
		goto pcanbasic_initialize_exit;
	}
	/* check support of message echoing to order tracing */
#if !(defined(FORCE_ECHO_STATUS_OFF) && FORCE_ECHO_STATUS_OFF == 1)
	itmp = 0;
	pcanfd_get_option(pchan->fd, PCANFD_OPT_CHANNEL_FEATURES, &itmp, sizeof(itmp));
	pchan->echo_status = ((itmp & PCANFD_FEATURE_ECHO) == PCANFD_FEATURE_ECHO);
#endif
	/* check firmware capabilities */
	pcanbasic_init_fw_features(pchan);
	/* override driver/device settings for HWTIMESTAMP mode */
	pcanbasic_set_hwtimestamp_mode(pchan);
	/* initialize default allowed messages */
	itmp = PCANFD_ALLOWED_MSG_ALL & ~(PCANFD_ALLOWED_MSG_ERROR);
	ires = pcanfd_set_option(pchan->fd, PCANFD_OPT_ALLOWED_MSGS, &itmp, sizeof(itmp));
	if (ires != 0)
		pcanlog_log(LVL_NORMAL, "Failed to configure default allowed messages (%d)...\n", ires);
#if !defined(KEEP_STATUS_FRAME_ON_RESET)
	/* Windows driver does not produce a status frame upon CAN initialization */
	pchan->ignore_status_frame = 1;
#endif
	/* insert new channel info in list */
	if (!inserted)
		SLIST_INSERT_HEAD(&g_basiccore.channels, pchan, entries);
	/* remove previously set filters */
	pcanfd_del_filters(pchan->fd);
	if (sts == PCAN_ERROR_OK) {
		pcanbasic_wait_bus_active(pchan->fd, 1);
		/* refresh pcaninfo struct to update bitrate information */
		pcaninfo_update(pchan->pinfo);
	}

pcanbasic_initialize_exit:
	return sts;
}

TPCANStatus pcanbasic_initialize_fd(
	TPCANHandle channel,
	TPCANBitrateFD bitratefd) {
	TPCANStatus sts;
	pcanbasic_channel *pchan;
	struct pcaninfo * pinfo;
	__u8 inserted;
	struct pcanfd_init fdi;
	int itmp, ires;

	inserted = 0;
	/* check if channel exists */
	pchan = pcanbasic_get_channel(channel, 0, &sts);
	if (pchan != NULL) {
		/* channel is already initialized */
		if (pchan->fd > -1) {
			sts = PCAN_ERROR_INITIALIZE;
			/* initialized with different bitrate */
			if (pchan->bitrate_adapting && strcmp(pchan->bitratefd, bitratefd) == 0)
				sts = PCAN_ERROR_CAUTION;
			goto pcanbasic_initialize_fd_exit;
		}
		/* channel is not initialized */
		else {
			/* this means that flags were set for initialization
			 * jump directly after malloc */
			inserted = 1;
			goto pcanbasic_initialize_malloc_post;
		}
	}
	/* allocate and initialize a new PCANBASIC_channel */
	pchan = pcanbasic_create_channel(channel, 0);
	if (pchan == NULL) {
		/* failed to create channel... get a more precise status code */
		pcanbasic_get_device(channel, 0, 0, 0, &sts);
		if (sts == PCAN_ERROR_OK)
			sts = PCAN_ERROR_UNKNOWN;	/* unexpected use-case */
		goto pcanbasic_initialize_fd_exit;
	}

pcanbasic_initialize_malloc_post:
	sts = PCAN_ERROR_OK;
	/* find a corresponding device */
	pinfo = pcanbasic_get_device(channel, 0, 0, 0, &sts);
	if (pinfo == NULL) {
		/* PCAN_ERROR_INITIALIZE is irrelevant in initialize function */
		if (sts == PCAN_ERROR_INITIALIZE)
			sts = PCAN_ERROR_ILLHW;
		if (inserted)
			SLIST_REMOVE(&g_basiccore.channels, pchan, _pcanbasic_channel, entries);
		pcanbasic_free_channel(pchan);
		goto pcanbasic_initialize_fd_exit;
	}
	/* complete struct initialization */
	if (pcanbasic_parse_fd_init(&fdi, bitratefd) != 0) {
		sts = PCAN_ERROR_ILLPARAMVAL;
		if (inserted)
			SLIST_REMOVE(&g_basiccore.channels, pchan, _pcanbasic_channel, entries);
		pcanbasic_free_channel(pchan);
		goto pcanbasic_initialize_fd_exit;
	}
	if (pchan->bitratefd)
		free(pchan->bitratefd);
	pchan->bitratefd = strdup(bitratefd);
	pcaninfo_update(pinfo);
	memcpy(pchan->pinfo, pinfo, sizeof(*pinfo));
	/* check if CAN channel is used by another application */
	if (pchan->pinfo->bus_state != PCANFD_UNKNOWN) {
		/* check bitrate adaptation */
		if (!pchan->bitrate_adapting) {
			/* not allowed, compare bitratefd */
			if (pchan->pinfo->nom_bitrate != fdi.nominal.bitrate &&
				pchan->pinfo->data_bitrate != fdi.data.bitrate) {
				sts = PCAN_ERROR_INITIALIZE;
				if (inserted)
					SLIST_REMOVE(&g_basiccore.channels, pchan, _pcanbasic_channel, entries);
				pcanbasic_free_channel(pchan);
				goto pcanbasic_initialize_fd_exit;
			}
		}
		else {
			sts = PCAN_ERROR_CAUTION;
		}
	}

	/* initialize time_start before opening channel */
	clock_gettime(pchan->pinfo->clk_ref, &pchan->tracer.time_start);
	/* open file descriptor */
	pchan->fd_flags = OFD_BITRATE | OFD_DBITRATE | OFD_BRPTSEGSJW | OFD_CLOCKHZ | OFD_NONBLOCKING;
	if (pchan->listen_only == PCAN_PARAMETER_ON)
		pchan->fd_flags |= PCANFD_INIT_LISTEN_ONLY;
	pchan->fd = pcanfd_open(pchan->pinfo->path, pchan->fd_flags,
			fdi.nominal.brp, fdi.nominal.tseg1, fdi.nominal.tseg2, fdi.nominal.sjw,
			fdi.data.brp, fdi.data.tseg1, fdi.data.tseg2, fdi.data.sjw,
			fdi.clock_Hz);
	if (pchan->fd < 0) {
		sts = pcanbasic_errno_to_status(-pchan->fd);
		if (sts == PCAN_ERROR_UNKNOWN)
			sts = PCAN_ERROR_ILLOPERATION;
		if (inserted)
			SLIST_REMOVE(&g_basiccore.channels, pchan, _pcanbasic_channel, entries);
		pcanbasic_free_channel(pchan);
		goto pcanbasic_initialize_fd_exit;
	}
	/* check support of message echoing to order tracing */
	itmp = 0;
	pcanfd_get_option(pchan->fd, PCANFD_OPT_CHANNEL_FEATURES, &itmp, sizeof(itmp));
	pchan->echo_status = ((itmp & PCANFD_FEATURE_ECHO) == PCANFD_FEATURE_ECHO);
	/* check firmware capabilities */
	pcanbasic_init_fw_features(pchan);
	/* override driver/device settings for HWTIMESTAMP mode */
	pcanbasic_set_hwtimestamp_mode(pchan);
	/* initialize default allowed messages */
	itmp = PCANFD_ALLOWED_MSG_ALL & ~(PCANFD_ALLOWED_MSG_ERROR);
	ires = pcanfd_set_option(pchan->fd, PCANFD_OPT_ALLOWED_MSGS, &itmp, sizeof(itmp));
	if (ires != 0)
		pcanlog_log(LVL_NORMAL, "Failed to configure default allowed messages (%d)...\n", ires);

	/* insert new channel info in list */
	if (!inserted)
		SLIST_INSERT_HEAD(&g_basiccore.channels, pchan, entries);
	/* remove previously set filters */
	pcanfd_del_filters(pchan->fd);
	if (sts == PCAN_ERROR_OK) {
		pcanbasic_wait_bus_active(pchan->fd, 1);
		/* refresh pcaninfo struct to update bitrate information */
		pcaninfo_update(pchan->pinfo);
	}

pcanbasic_initialize_fd_exit:
	return sts;
}

TPCANStatus pcanbasic_uninitialize(
		TPCANHandle channel) {
	TPCANStatus sts;
	pcanbasic_channel *pchan;

	/* PCAN_NONEBUS clears all channels */
	if (channel == PCAN_NONEBUS) {
		pcanbasic_channel *plist;
		/* uninitialize channels */
		for (plist = g_basiccore.channels.slh_first; plist != NULL; plist = plist->entries.sle_next) {
			pcanbasic_uninitialize(plist->channel);
		}
		sts = PCAN_ERROR_OK;
		goto pcanbasic_uninitialize_exit;
	}
	/* get channel */
	pchan = pcanbasic_get_channel(channel, 1, &sts);
	if (pchan == NULL) {
		goto pcanbasic_uninitialize_exit;
	}
	/* close and remove channel from initialized channels' list */
	SLIST_REMOVE(&g_basiccore.channels, pchan, _pcanbasic_channel, entries);
	pcanbasic_free_channel(pchan);
	sts = PCAN_ERROR_OK;

pcanbasic_uninitialize_exit:
	return sts;
}

TPCANStatus pcanbasic_reset(
		TPCANHandle channel) {
	TPCANStatus sts;
	pcanbasic_channel *pchan;
	int ires;
	
	/* get channel */
	pchan = pcanbasic_get_channel(channel, 1, &sts);
	if (pchan == NULL) {
		goto pcanbasic_reset_exit;
	}

	/* try to use PCAN_8.13 feature */
	ires = pcanfd_reset(pchan->fd, PCANFD_RESET_RXFIFO|PCANFD_RESET_TXFIFO);
	if (ires < 0) {
		/* fallback to a reset mimicked by an uninitialization and init */
		struct pcanfd_init pfdinit;

		/* get fd initialization to restore it later */
		pcanfd_get_init(pchan->fd, &pfdinit);
		if (pchan->listen_only)
			pfdinit.flags |= PCANFD_INIT_LISTEN_ONLY;
		/* close and open file descriptor */
		pcanfd_close(pchan->fd);
		pchan->fd = pcanfd_open(pchan->pinfo->path, OFD_NONBLOCKING);	/* no flag as we will use set_init */
		if (pchan->fd < 0) {
			sts = PCAN_ERROR_ILLOPERATION;
			goto pcanbasic_reset_exit;
		}
		/* re-set config */
		if (pcanfd_set_init(pchan->fd, &pfdinit) < 0) {
			sts = PCAN_ERROR_ILLOPERATION;
			goto pcanbasic_reset_exit;
		}
	#if !defined(KEEP_STATUS_FRAME_ON_RESET)
		pchan->ignore_status_frame = 1;
	#endif
	}

	sts = PCAN_ERROR_OK;
pcanbasic_reset_exit:
	return sts;
}

TPCANStatus pcanbasic_get_status(
		TPCANHandle channel) {
	TPCANStatus sts;
	pcanbasic_channel *pchan;
	struct pcanfd_state fds;
	int ires;

	/* get channel */
	pchan = pcanbasic_get_channel(channel, 1, &sts);
	if (pchan == NULL) {
		goto pcanbasic_get_status_exit;
	}
	/* read status and convert result */
	ires = pcanfd_get_state(pchan->fd, &fds);
	if (ires < 0) {
		sts = pcanbasic_errno_to_status(-ires);
		goto pcanbasic_get_status_exit;
	}
	sts = pcanbasic_bus_state_to_condition(fds.bus_state, (pchan->fd_flags & OFD_BITRATE) == OFD_BITRATE);

pcanbasic_get_status_exit:
	return sts;
}

TPCANStatus pcanbasic_read(
		TPCANHandle channel,
		TPCANMsg* message,
		TPCANTimestamp* timestamp) {
	TPCANStatus sts;
	pcanbasic_channel *pchan;
	TPCANMsgFD msgfd;
	struct timeval t;

	if (message == NULL) {
		sts = PCAN_ERROR_ILLPARAMVAL;
		goto pcanbasic_read_exit;
	}
	/* get initialized channel */
	pchan = pcanbasic_get_channel(channel, 1, &sts);
	if (pchan == NULL) {
		goto pcanbasic_read_exit;
	}
	if ((pchan->fd_flags & OFD_BTR0BTR1) != OFD_BTR0BTR1) {
		/* channel is FD initialized, use CAN_ReadFD */
		sts = PCAN_ERROR_ILLOPERATION;
		goto pcanbasic_read_exit;
	}
	/* use common fd read function */
	sts = pcanbasic_read_common(pchan, &msgfd, &t);
	if (sts == PCAN_ERROR_OK) {
		/* convert TPCANMsgFD message to TPCANMsg */
		message->ID = msgfd.ID;
		memcpy(message->DATA, msgfd.DATA, sizeof(message->DATA));
		message->LEN = msgfd.DLC;
		message->MSGTYPE = msgfd.MSGTYPE;
		if(message->MSGTYPE == PCAN_MESSAGE_STATUS) {
			/* status code matches status frame data */
			sts = (TPCANStatus)message->DATA[3];
			sts |= (TPCANStatus)message->DATA[2] << 8;
			sts |= (TPCANStatus)message->DATA[1] << 16;
			sts |= (TPCANStatus)message->DATA[0] << 24;
		}
		/* convert timestamp */
		if (timestamp != NULL) {
			/* overflow: idea is to get millis in 64bit
			 * and see if it match the 32b version
			 * if not then an overflow occured */
			__u64 millis = ((__u64) t.tv_sec) * 1000 + (t.tv_usec / 1000);
			timestamp->micros = t.tv_usec % 1000;
			timestamp->millis = millis;
			if (timestamp->millis != millis)
				timestamp->millis_overflow = (millis - timestamp->millis) >> (sizeof(timestamp->millis)*8);
			else
				timestamp->millis_overflow = 0;
		}
	}

pcanbasic_read_exit:
	return sts;
}

TPCANStatus pcanbasic_read_fd(
	TPCANHandle channel,
	TPCANMsgFD* message,
	TPCANTimestampFD *timestamp) {
	TPCANStatus sts;
	pcanbasic_channel *pchan;
	struct timeval t;

	/* get initialized channel */
	pchan = pcanbasic_get_channel(channel, 1, &sts);
	if (pchan == NULL) {
		goto pcanbasic_read_fd_exit;
	}
	if ((pchan->fd_flags & OFD_BTR0BTR1) == OFD_BTR0BTR1) {
		/* channel is NOT FD initialized, use CAN_Read */
		sts = PCAN_ERROR_ILLOPERATION;
		goto pcanbasic_read_fd_exit;
	}

	sts = pcanbasic_read_common(pchan, message, &t);
	if (sts == PCAN_ERROR_OK) {
		if (timestamp != NULL) {
			*timestamp = ((__u64) t.tv_sec) * 1000000 + t.tv_usec;
		}		
		if(message->MSGTYPE == PCAN_MESSAGE_STATUS) {
			/* status code matches status frame data */
			sts = (TPCANStatus)message->DATA[3];
			sts |= (TPCANStatus)message->DATA[2] << 8;
			sts |= (TPCANStatus)message->DATA[1] << 16;
			sts |= (TPCANStatus)message->DATA[0] << 24;
		}
	}

pcanbasic_read_fd_exit:
	return sts;
}

TPCANStatus pcanbasic_write(
		TPCANHandle channel,
		TPCANMsg* message) {
	TPCANStatus sts;
	pcanbasic_channel *pchan;
	TPCANMsgFD msgfd;

	if (message == NULL) {
		sts = PCAN_ERROR_ILLPARAMVAL;
		goto pcanbasic_write_exit;
	}

	/* get initialized channel */
	pchan = pcanbasic_get_channel(channel, 1, &sts);
	if (pchan == NULL) {
		goto pcanbasic_write_exit;
	}
	if ((pchan->fd_flags & OFD_BTR0BTR1) != OFD_BTR0BTR1) {
		/* channel is FD initialized, use CAN_WriteFD */
		sts = PCAN_ERROR_ILLOPERATION;
		goto pcanbasic_write_exit;
	}
	/* convert std message to a FD message and use a single common function */
	memset(msgfd.DATA, 0, sizeof(msgfd.DATA));
	msgfd.ID = message->ID;
	msgfd.DLC = message->LEN;
	msgfd.MSGTYPE = message->MSGTYPE;
	memcpy(msgfd.DATA, message->DATA, MIN(sizeof(message->DATA), message->LEN));
	sts = pcanbasic_write_common(pchan, &msgfd);

pcanbasic_write_exit:
	return sts;
}

TPCANStatus pcanbasic_write_fd(
	TPCANHandle channel,
	TPCANMsgFD* message) {
	TPCANStatus sts;
	pcanbasic_channel *pchan;

	/* get initialized channel */
	pchan = pcanbasic_get_channel(channel, 1, &sts);
	if (pchan == NULL) {
		goto pcanbasic_write_fd_exit;
	}
	if ((pchan->fd_flags & OFD_BTR0BTR1) == OFD_BTR0BTR1) {
		/* channel is NOT FD initialized, use CAN_Write */
		sts = PCAN_ERROR_ILLOPERATION;
		goto pcanbasic_write_fd_exit;
	}
	sts = pcanbasic_write_common(pchan, message);

pcanbasic_write_fd_exit:
	return sts;
}

TPCANStatus pcanbasic_filter(
		TPCANHandle channel,
		DWORD from,
		DWORD to,
		TPCANMode mode) {
	TPCANStatus sts;
	pcanbasic_channel *pchan;
	struct pcanfd_msg_filter filter;
	int ires;

	/* get channel */
	pchan = pcanbasic_get_channel(channel, 1, &sts);
	if (pchan == NULL) {
		goto pcanbasic_filter_exit;
	}

	filter.id_from = from;
	filter.id_to = to;
	switch (mode) {
	case PCAN_MODE_EXTENDED:
		filter.msg_flags = PCAN_MESSAGE_EXTENDED;
		break;
	case PCAN_MODE_STANDARD:
	default:
		filter.msg_flags = PCAN_MESSAGE_STANDARD;
		break;
	}
	ires = pcanfd_add_filter(pchan->fd, &filter);
	if (ires < 0) {
		sts = pcanbasic_errno_to_status(-ires);
		goto pcanbasic_filter_exit;
	}
	sts = PCAN_ERROR_OK;

pcanbasic_filter_exit:
	return sts;
}

TPCANStatus pcanbasic_get_value(
		TPCANHandle channel,
		TPCANParameter parameter,
		void* buffer,
		DWORD len) {
	TPCANStatus sts;
	TPCANStatus sts_param;
	pcanbasic_channel *pchan = NULL;
	struct pcaninfo *pinfo;
	char sversion[MAX_LENGTH_VERSION_STRING];
	struct __array_of_struct(pcanfd_msg_filter, 1);
	int ires;
	size_t size;
	__u8 ctmp;
	__u32 itmp;

	/* check output parameter */
	if (buffer == NULL || len <= 0) {
		sts = PCAN_ERROR_ILLPARAMVAL;
		goto pcanbasic_get_value_exit;
	}

	ctmp = 0;
	itmp = 0;
	/* clear buffer to avoid memcpy side effects
	 * with a type bigger than the one required */
	memset(buffer, 0, len);

	/* check channel handle */
	if (channel != PCAN_NONEBUS) {
		/* get initialized (or pre-initialized) channel if any (also ensures API is correctly initialized) */
		pchan = pcanbasic_get_channel(channel, 0, &sts);
		if (sts & PCAN_ERROR_ILLHANDLE) {
			if (parameter == PCAN_CHANNEL_CONDITION && pcanbasic_check_hw(channel, 0)) {
				/* this parameter allow unavailable handles */
			}
			else {
				/* fail even if some parameters could work with an invalid handle: */
				/* 	API behave as the Windows version */
				goto pcanbasic_get_value_exit;
			}
		}
	}
	/* check parameter usage */
	sts_param = pcanbasic_parameter_supported(channel, parameter, 0);
	if (sts_param != PCAN_ERROR_OK) {
		sts = sts_param;
		goto pcanbasic_get_value_exit;
	}

	/* handle parameters that do not require a channel */
	switch (parameter) {
	case PCAN_DEVICE_ID:
		sts = pcanbasic_get_value_device_id(channel, pchan, buffer, len);
		goto pcanbasic_get_value_exit;
		break;
	case PCAN_5VOLTS_POWER:
		/* since 4.4.0: can be called with an uninitialized channel */
		/* not supported by driver */
		sts = PCAN_ERROR_ILLPARAMTYPE;
		goto pcanbasic_get_value_exit;
		break;
	case PCAN_API_VERSION:
		size = sizeof(sversion);
		snprintf(sversion, size, "%d.%d.%d.%d" API_VERSION_DEBUG, VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH, VERSION_BUILD);
		size = strlen(sversion) * sizeof(sversion[0]);
		if (len < size + 1) {	/* +1: ensures null-terminated string */
			sts = PCAN_ERROR_ILLPARAMVAL;
			goto pcanbasic_get_value_exit;
		}
		memcpy(buffer, &sversion, size);
		goto pcanbasic_get_value_exit_ok;
		break;
	case PCAN_CHANNEL_VERSION:
		sts = pcanbasic_get_value_channel_version(channel, pchan, buffer, len);
		goto pcanbasic_get_value_exit;
		break;
	case PCAN_LISTEN_ONLY:
		/* can be called with an uninitialized channel */
		size = sizeof(ctmp);
		if (len < size) {
			sts = PCAN_ERROR_ILLPARAMVAL;
			goto pcanbasic_get_value_exit;
		}
		/* channel can be pre-initialized */
		ctmp = (pchan != NULL) ? pchan->listen_only : DEFAULT_PARAM_LISTEN_ONLY;
		memcpy(buffer, &ctmp, size);
		goto pcanbasic_get_value_exit_ok;
		break;
	case PCAN_LOG_LOCATION:
		if (len < 1) {
			sts = PCAN_ERROR_ILLPARAMVAL;
			goto pcanbasic_get_value_exit;
		}
		pcblog_get_location(buffer, len);
		goto pcanbasic_get_value_exit_ok;
		break;
	case PCAN_LOG_STATUS:
		size = sizeof(ctmp);
		if (len < size) {
			sts = PCAN_ERROR_ILLPARAMVAL;
			goto pcanbasic_get_value_exit;
		}
		ctmp = pcblog_get_status();
		memcpy(buffer, &ctmp, size);
		goto pcanbasic_get_value_exit_ok;
		break;
	case PCAN_LOG_CONFIGURE:
		size = sizeof(itmp);
		if (len < size) {
			sts = PCAN_ERROR_ILLPARAMVAL;
			goto pcanbasic_get_value_exit;
		}
		itmp = pcblog_get_config();
		memcpy(buffer, &itmp, size);
		goto pcanbasic_get_value_exit_ok;
		break;
	case PCAN_LOG_TEXT:
		/* not possible */
		sts = PCAN_ERROR_ILLPARAMTYPE;
		goto pcanbasic_get_value_exit;
		break;
	case PCAN_CHANNEL_CONDITION:
		/* can be called with an uninitialized channel */
		size = sizeof(sts);
		if (len < size) {
			sts = PCAN_ERROR_ILLPARAMVAL;
			goto pcanbasic_get_value_exit;
		}
		sts = pcanbasic_get_value_channel_condition(channel, pchan);
		memcpy(buffer, &sts, size);
		goto pcanbasic_get_value_exit_ok;
		break;
	case PCAN_HARDWARE_NAME:
		/* fetch channel's information */
		pinfo = (pchan != NULL) ? pchan->pinfo : pcanbasic_get_device(channel, 0, 0, 0, &sts);
		if (pinfo != NULL) {
			size = strnlen(pinfo->adapter_name, MAX_LENGTH_HARDWARE_NAME);
			if (len < size + 1) {	/*	+1: ensures null-terminated string	*/
				sts = PCAN_ERROR_ILLPARAMVAL;
				goto pcanbasic_get_value_exit;
			}
			sts = PCAN_ERROR_OK;	/*	pcanbasic_get_device may have modified sts	*/
			memcpy(buffer, pinfo->adapter_name, size);
			goto pcanbasic_get_value_exit_ok;
		}
		else {
			/* PCAN_ERROR_INITIALIZE is irrelevant as parameter does not require initialization */
			if (sts == PCAN_ERROR_INITIALIZE)
				sts = PCAN_ERROR_ILLHW;
			goto pcanbasic_get_value_exit;
		}
		break;
	case PCAN_RECEIVE_STATUS:
		/* can be called with an uninitialized channel */
		size = sizeof(ctmp);
		if (len < size) {
			sts = PCAN_ERROR_ILLPARAMVAL;
			goto pcanbasic_get_value_exit;
		}
		/* try to get uninitialized channel */
		ctmp = (pchan != NULL) ? pchan->rcv_status : DEFAULT_PARAM_RCV_STATUS;
		memcpy(buffer, &ctmp, size);
		goto pcanbasic_get_value_exit_ok;
		break;
	case PCAN_CONTROLLER_NUMBER:
		/* can be called with an uninitialized channel */
		sts = pcanbasic_get_value_controller_number(channel, pchan, buffer, len);
		goto pcanbasic_get_value_exit;
		break;
	case PCAN_CHANNEL_IDENTIFYING:
		/* not implemented in PCAN Linux driver */
		sts = PCAN_ERROR_ILLPARAMTYPE;
		break;
	case PCAN_CHANNEL_FEATURES:
		/* can be called with an uninitialized channel */
		sts = pcanbasic_get_value_channel_features(channel, pchan, buffer, len);
		goto pcanbasic_get_value_exit;
		break;
	case PCAN_BITRATE_ADAPTING:
		/* can be called with an uninitialized channel */
		size = sizeof(ctmp);
		if (len < size) {
			sts = PCAN_ERROR_ILLPARAMVAL;
			goto pcanbasic_get_value_exit;
		}
		ctmp = (pchan != NULL) ? pchan->bitrate_adapting : DEFAULT_PARAM_BITRATE_ADAPTING;
		memcpy(buffer, &ctmp, size);
		goto pcanbasic_get_value_exit_ok;
		break;

	case PCAN_BITRATE_INFO:
		/* can be called with an uninitialized channel */
		sts = pcanbasic_get_value_bitrate_info(channel, pchan, buffer, len);
		goto pcanbasic_get_value_exit;
		break;
	case PCAN_BITRATE_INFO_FD:
		/* can be called with an uninitialized channel */
		sts = pcanbasic_get_value_bitrate_info_fd(channel, pchan, buffer, len);
		goto pcanbasic_get_value_exit;
		break;
	case PCAN_ATTACHED_CHANNELS_COUNT:
		/*  force HW refresh */
		pcanbasic_refresh_hw();
		size = sizeof(g_basiccore.devices->length);
		if (len < size) {
			sts = PCAN_ERROR_ILLPARAMVAL;
			goto pcanbasic_get_value_exit;
		}
		memcpy(buffer, &g_basiccore.devices->length, size);
		goto pcanbasic_get_value_exit_ok;
		break;
	case PCAN_ATTACHED_CHANNELS:
		/*  force HW refresh */
		pcanbasic_refresh_hw();
		size = sizeof(TPCANChannelInformation) * g_basiccore.devices->length;
		if (len < size) {
			sts = PCAN_ERROR_ILLPARAMVAL;
			goto pcanbasic_get_value_exit;
		}
		else {
			TPCANChannelInformation* chi;
			TPCANHandle lchannel;
			pcanbasic_channel* lpchan;
			int fd;
			__u8 fd_close;
			int i;

			g_basiccore.refresh_locked = 1;	/* disable hw_refresh */
			for (i = 0; i < g_basiccore.devices->length; ++i) {
				chi = &(((TPCANChannelInformation*)buffer)[i]);
				pinfo = &g_basiccore.devices->infos[i];
				lchannel = pcanbasic_get_handle(pinfo->path, g_basiccore.devices);
				lpchan = pcanbasic_get_channel(lchannel, 1, NULL);

				chi->channel_condition = (pinfo->bus_state == PCANFD_UNKNOWN) ? PCAN_CHANNEL_AVAILABLE : PCAN_CHANNEL_OCCUPIED;
				chi->channel_handle = lchannel;
				chi->controller_number = pinfo->ctrlnb;
				chi->device_id = pinfo->devid;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstringop-truncation"
				strncpy(chi->device_name, pinfo->adapter_name, MAX_LENGTH_HARDWARE_NAME);
#pragma GCC diagnostic pop
				chi->device_type = pinfo->hwcategory;
				/* device feature is not available in sysfs */
				chi->device_features = 0;
				/* get a file descriptor even if channel is not initialized */
				fd = pcanbasic_get_fd(chi->channel_handle, lpchan, &pinfo, &fd_close);
				if (fd > -1) {
					ires = pcanfd_get_option(fd, PCANFD_OPT_CHANNEL_FEATURES, &itmp, sizeof(itmp));
					if (ires >= 0) {
						if (itmp & PCANFD_FEATURE_FD)
							chi->device_features |= FEATURE_FD_CAPABLE;
						if (itmp & PCANFD_FEATURE_IFRAME_DELAYUS)
							chi->device_features |= FEATURE_DELAY_CAPABLE;
					}
					ires = pcanfd_get_option(fd, PCANFD_IO_DIGITAL_CFG, &itmp, sizeof(itmp));
					if (ires >= 0)
						chi->device_features |= FEATURE_IO_CAPABLE;
					if (fd_close)
						pcanfd_close(fd);
				}
			}
			g_basiccore.refresh_locked = 0;
		}
		goto pcanbasic_get_value_exit_ok;
		break;
	case PCAN_IO_DIGITAL_SET:
	case PCAN_IO_DIGITAL_CLEAR:
		sts = PCAN_ERROR_ILLPARAMTYPE;
		goto pcanbasic_get_value_exit;
		break;
	case PCAN_DEVICE_PART_NUMBER:
		/* can be called with an uninitialized channel */
		sts = pcanbasic_get_value_adapter_part_number(channel, pchan, buffer, len);
		goto pcanbasic_get_value_exit;
		break;
	}

	/* assert channel is initialized */
	if (pchan == NULL || pchan->fd < 0) {
		/* assert status is not OK  */
		if (sts == PCAN_ERROR_OK)
			sts = PCAN_ERROR_INITIALIZE;
		goto pcanbasic_get_value_exit;
	}
	/* handle parameters that require a channel */
	switch(parameter) {
	case PCAN_RECEIVE_EVENT:
		size = sizeof(itmp);
		if (len < size) {
			sts = PCAN_ERROR_ILLPARAMVAL;
			goto pcanbasic_get_value_exit;
		}
		memcpy(buffer, &pchan->fd, size);
		break;
	case PCAN_MESSAGE_FILTER:
		size = sizeof(ctmp);
		if (len < size) {
			sts = PCAN_ERROR_ILLPARAMVAL;
			goto pcanbasic_get_value_exit;
		}
		ctmp = pcanbasic_get_filter(pchan);
		memcpy(buffer, &ctmp, size);
		break;
	case PCAN_BUSOFF_AUTORESET:
		size = sizeof(pchan->busoff_reset);
		if (len < size) {
			sts = PCAN_ERROR_ILLPARAMVAL;
			goto pcanbasic_get_value_exit;
		}
		memcpy(buffer, &pchan->busoff_reset, size);
		break;
	case PCAN_TRACE_LOCATION:
		if (pchan->tracer.directory != NULL) {
			size = strlen(pchan->tracer.directory);
			if (len < size + 1) {	/* +1: ensures null-terminated string */
				sts = PCAN_ERROR_ILLPARAMVAL;
				goto pcanbasic_get_value_exit;
			}
			memcpy(buffer, pchan->tracer.directory, size);
		}
		else {
			size = 1;	/* ensures null-terminated string */
			if (len < size) {
				sts = PCAN_ERROR_ILLPARAMVAL;
				goto pcanbasic_get_value_exit;
			}
			memset(buffer, 0, 1);
		}
		break;
	case PCAN_TRACE_STATUS:
		size = sizeof(pchan->tracer.status);
		if (len < size) {
			sts = PCAN_ERROR_ILLPARAMVAL;
			goto pcanbasic_get_value_exit;
		}
		memcpy(buffer, &pchan->tracer.status, size);
		break;
	case PCAN_TRACE_SIZE:
		size = sizeof(pchan->tracer.maxsize);
		if (len < size) {
			sts = PCAN_ERROR_ILLPARAMVAL;
			goto pcanbasic_get_value_exit;
		}
		memcpy(buffer, &pchan->tracer.maxsize, size);
		break;
	case PCAN_TRACE_CONFIGURE:
		size = sizeof(pchan->tracer.flags);
		if (len < size) {
			sts = PCAN_ERROR_ILLPARAMVAL;
			goto pcanbasic_get_value_exit;
		}
		memcpy(buffer, &pchan->tracer.flags, size);
		break;
	case PCAN_BUSSPEED_NOMINAL:
		size = sizeof(pchan->pinfo->nom_bitrate);
		if (len < size) {
			sts = PCAN_ERROR_ILLPARAMVAL;
			goto pcanbasic_get_value_exit;
		}
		memcpy(buffer, &pchan->pinfo->nom_bitrate, size);
		break;
	case PCAN_BUSSPEED_DATA:
		size = sizeof(pchan->pinfo->data_bitrate);
		if (len < size) {
			sts = PCAN_ERROR_ILLPARAMVAL;
			goto pcanbasic_get_value_exit;
		}
		memcpy(buffer, &pchan->pinfo->data_bitrate, size);
		break;
	case PCAN_IP_ADDRESS:
	case PCAN_LAN_SERVICE_STATUS:
		/* not supported by driver */
		sts = PCAN_ERROR_NODRIVER;
		goto pcanbasic_get_value_exit;
		break;
	case PCAN_ALLOW_ERROR_FRAMES:
		size = sizeof(__u8);
		if (len < size) {
			sts = PCAN_ERROR_ILLPARAMVAL;
			goto pcanbasic_get_value_exit;
		}
		ires = pcanfd_get_option(pchan->fd, PCANFD_OPT_ALLOWED_MSGS, &itmp, sizeof(itmp));
		if (ires < 0) {
			sts = pcanbasic_errno_to_status(-ires);
			goto pcanbasic_get_value_exit;
		}
		itmp = (itmp & PCANFD_ALLOWED_MSG_ERROR) == PCANFD_ALLOWED_MSG_ERROR ?
				PCAN_PARAMETER_ON : PCAN_PARAMETER_OFF;
		memcpy(buffer, &itmp, size);
		break;
	case PCAN_ALLOW_RTR_FRAMES:
		size = sizeof(__u8);
		if (len < size) {
			sts = PCAN_ERROR_ILLPARAMVAL;
			goto pcanbasic_get_value_exit;
		}
		ires = pcanfd_get_option(pchan->fd, PCANFD_OPT_ALLOWED_MSGS, &itmp, sizeof(itmp));
		if (ires < 0) {
			sts = pcanbasic_errno_to_status(-ires);
			goto pcanbasic_get_value_exit;
		}
		itmp = (itmp & PCANFD_ALLOWED_MSG_RTR) == PCANFD_ALLOWED_MSG_RTR ?
				PCAN_PARAMETER_ON : PCAN_PARAMETER_OFF;
		memcpy(buffer, &itmp, size);
		break;
	case PCAN_ALLOW_STATUS_FRAMES:
		size = sizeof(__u8);
		if (len < size) {
			sts = PCAN_ERROR_ILLPARAMVAL;
			goto pcanbasic_get_value_exit;
		}
		ires = pcanfd_get_option(pchan->fd, PCANFD_OPT_ALLOWED_MSGS, &itmp, sizeof(itmp));
		if (ires < 0) {
			sts = pcanbasic_errno_to_status(-ires);
			goto pcanbasic_get_value_exit;
		}
		itmp = (itmp & PCANFD_ALLOWED_MSG_STATUS) == PCANFD_ALLOWED_MSG_STATUS ?
				PCAN_PARAMETER_ON : PCAN_PARAMETER_OFF;
		memcpy(buffer, &itmp, size);
		break;
	case PCAN_INTERFRAME_DELAY:
		size = sizeof(__u32);
		if (len < size) {
			sts = PCAN_ERROR_ILLPARAMVAL;
			goto pcanbasic_get_value_exit;
		}
		ires = pcanfd_get_option(pchan->fd, PCANFD_OPT_IFRAME_DELAYUS, buffer, len);
		if (ires < 0) {
			sts = pcanbasic_errno_to_status(-ires);
			goto pcanbasic_get_value_exit;
		}
		break;
	case PCAN_ACCEPTANCE_FILTER_11BIT:
		size = sizeof(__u64);
		if (len < size) {
			sts = PCAN_ERROR_ILLPARAMVAL;
			goto pcanbasic_get_value_exit;
		}
		ires = pcanfd_get_option(pchan->fd, PCANFD_OPT_ACC_FILTER_11B, buffer, len);
		if (ires < 0) {
			sts = pcanbasic_errno_to_status(-ires);
			goto pcanbasic_get_value_exit;
		}
		break;
	case PCAN_ACCEPTANCE_FILTER_29BIT:
		size = sizeof(__u64);
		if (len < size) {
			sts = PCAN_ERROR_ILLPARAMVAL;
			goto pcanbasic_get_value_exit;
		}
		ires = pcanfd_get_option(pchan->fd, PCANFD_OPT_ACC_FILTER_29B, buffer, len);
		if (ires < 0) {
			sts = pcanbasic_errno_to_status(-ires);
			goto pcanbasic_get_value_exit;
		}
		break;
	case PCAN_IO_DIGITAL_CONFIGURATION:
		size = sizeof(__u32);
		if (len < size) {
			sts = PCAN_ERROR_ILLPARAMVAL;
			goto pcanbasic_get_value_exit;
		}
		ires = pcanfd_get_option(pchan->fd, PCANFD_IO_DIGITAL_CFG, buffer, len);
		if (ires < 0) {
			if (-ires == EOPNOTSUPP)
				sts = PCAN_ERROR_ILLPARAMTYPE;
			else 
				sts = pcanbasic_errno_to_status(-ires);
			goto pcanbasic_get_value_exit;
		}
		break;
	case PCAN_IO_DIGITAL_VALUE:
		size = sizeof(__u32);
		if (len < size) {
			sts = PCAN_ERROR_ILLPARAMVAL;
			goto pcanbasic_get_value_exit;
		}
		ires = pcanfd_get_option(pchan->fd, PCANFD_IO_DIGITAL_VAL, buffer, len);
		if (ires < 0) {
			if (-ires == EOPNOTSUPP)
				sts = PCAN_ERROR_ILLPARAMTYPE;
			else 
				sts = pcanbasic_errno_to_status(-ires);
			goto pcanbasic_get_value_exit;
		}
		break;
	case PCAN_IO_ANALOG_VALUE:
		size = sizeof(__u32);
		if (len < size) {
			sts = PCAN_ERROR_ILLPARAMVAL;
			goto pcanbasic_get_value_exit;
		}
		ires = pcanfd_get_option(pchan->fd, PCANFD_IO_ANALOG_VAL, buffer, len);
		if (ires < 0) {
			if (-ires == EOPNOTSUPP)
				sts = PCAN_ERROR_ILLPARAMTYPE;
			else 
				sts = pcanbasic_errno_to_status(-ires);
			goto pcanbasic_get_value_exit;
		}
		break;
	case PCAN_FIRMWARE_VERSION:
		size = strnlen(pchan->pinfo->adapter_version, sizeof(pchan->pinfo->adapter_version) - 1);
		if (len < size + 1) {	/* +1: ensures null-terminated string */
			sts = PCAN_ERROR_ILLPARAMVAL;
			goto pcanbasic_get_value_exit;
		}
		memcpy(buffer, &pchan->pinfo->adapter_version, size);
		break;
	case PCAN_ALLOW_ECHO_FRAMES:
		size = sizeof(pchan->rcv_echo_status);
		if (len < size) {
			sts = PCAN_ERROR_ILLPARAMVAL;
			goto pcanbasic_get_value_exit;
		}
		memcpy(buffer, &pchan->rcv_echo_status, size);
		break;
	default:
		sts = PCAN_ERROR_ILLPARAMTYPE;
		goto pcanbasic_get_value_exit;
		break;
	}
pcanbasic_get_value_exit_ok:
	sts = PCAN_ERROR_OK;
pcanbasic_get_value_exit:
	return sts;
}

TPCANStatus pcanbasic_set_value(
		TPCANHandle channel,
		TPCANParameter parameter,
		void* buffer,
		DWORD len) {
	TPCANStatus sts;
	TPCANStatus sts_param;
	pcanbasic_channel *pchan = NULL;
	int ires;
	size_t size;
	__u8 ctmp;
	__u32 itmp = 0;
	int fd;
	__u8 fd_close;

	/* check output parameter */
	if (buffer == NULL || len <= 0) {
		uint8_t param_ok = 0;
		/* only a few parameters allow (NULL+0) */
		if (buffer == NULL && len == 0) {
			switch(parameter) {
				case PCAN_TRACE_LOCATION:
				case PCAN_LOG_LOCATION:
					param_ok = 1;
					break;
			}
		}
		if (!param_ok) {
			sts = PCAN_ERROR_ILLPARAMVAL;
			goto pcanbasic_set_value_exit;
		}
	}
	/* check channel handle */
	if (channel != PCAN_NONEBUS) {
		/* get initialized (or pre-initialized!) channel if any (also ensures API is correctly initialized) */
		pchan = pcanbasic_get_channel(channel, 0, &sts);
		if (sts & PCAN_ERROR_ILLHANDLE) {
			/* even if some parameters could work with an invalid handle */
			/* behave as the Windows version and exit */
			goto pcanbasic_set_value_exit;
		}
	}
	/* check parameter usage */
	sts_param = pcanbasic_parameter_supported(channel, parameter, 1);
	if (sts_param != PCAN_ERROR_OK) {
		sts = sts_param;
		goto pcanbasic_set_value_exit;
	}

	/* handle parameters that do not require a channel */
	switch (parameter) {
	case PCAN_LISTEN_ONLY:
		/* can be called with an uninitialized channel */
		size = sizeof(ctmp);
		/* allow only ON/OFF values */
		ctmp = *(__u8*)buffer;
		if (len > size)
			len = size;
		if (ctmp != PCAN_PARAMETER_ON &&
				ctmp != PCAN_PARAMETER_OFF) {
			sts = PCAN_ERROR_ILLPARAMVAL;
			goto pcanbasic_set_value_exit;
		}
		/* channel can be initialized or not */
		if (pchan == NULL) {
			/* create channel to store INIT. value */
			pchan = pcanbasic_create_channel(channel, 1);
			if (pchan == NULL) {
				sts = PCAN_ERROR_UNKNOWN;
				goto pcanbasic_set_value_exit;
			}
		}
		pchan->listen_only = ctmp;
		if (pchan->fd > -1) {
			sts = pcanbasic_reset(channel);
			goto pcanbasic_set_value_exit;
		}
		goto pcanbasic_set_value_exit_ok;
		break;
	case PCAN_LOG_LOCATION:
		if (!pcblog_set_location(buffer)) {
			sts = PCAN_ERROR_ILLPARAMVAL;
			goto pcanbasic_set_value_exit;
		}
		pcblog_set_status(1);
		goto pcanbasic_set_value_exit_ok;
		break;
	case PCAN_LOG_STATUS:
		size = sizeof(ctmp);
		/* allow only ON/OFF values */
		if (len > size)
			len = size;
		ctmp = *(__u8*)buffer;
		if (ctmp != PCAN_PARAMETER_ON &&
				ctmp != PCAN_PARAMETER_OFF) {
			sts = PCAN_ERROR_ILLPARAMVAL;
			goto pcanbasic_set_value_exit;
		}
		pcblog_set_status(ctmp);
		goto pcanbasic_set_value_exit_ok;
		break;
	case PCAN_LOG_CONFIGURE:
		size = sizeof(itmp);
		if (len > size)
			len = size;
		itmp = *(__u32*)buffer;
		itmp &= ((1UL << (len * 8)) - 1UL);
		pcblog_set_config(itmp);
		goto pcanbasic_set_value_exit_ok;
		break;
	case PCAN_LOG_TEXT:
		pcblog_set_status(1);
		if (pcblog_write(buffer, strnlen(buffer, len)) < 0) {
			pcanlog_log(LVL_ALWAYS, "Failed to write to 'PCANBasic.log' file.\n");
			sts = PCAN_ERROR_RESOURCE;
			goto pcanbasic_set_value_exit;
		}
		goto pcanbasic_set_value_exit_ok;
		break;
	case PCAN_RECEIVE_STATUS:
		/* can be called with an uninitialized channel */
		size = sizeof(ctmp);
		/* allow only ON/OFF values */
		ctmp = *(__u8*)buffer;
		if (len > size)
			len = size;
		if (ctmp != PCAN_PARAMETER_ON &&
				ctmp != PCAN_PARAMETER_OFF) {
			sts = PCAN_ERROR_ILLPARAMVAL;
			goto pcanbasic_set_value_exit;
		}
		/* channel can be initialized or not */
		if (pchan == NULL) {
			/* create channel to store INIT. value */
			pchan = pcanbasic_create_channel(channel, 1);
			if (pchan == NULL) {
				sts = PCAN_ERROR_UNKNOWN;
				goto pcanbasic_set_value_exit;
			}
		}
		pchan->rcv_status =  ctmp;
		goto pcanbasic_set_value_exit_ok;
		break;
	case PCAN_CHANNEL_IDENTIFYING:
		itmp = *(__u32*) buffer;
		itmp &= ((1 << (len * 8)) - 1);
		/* pcan linux driver limitation:
		 * option PCANFD_OPT_FLASH_LED is blocking,
		 * limit blinking time to 5 seconds
		 */
		if (itmp != PCAN_PARAMETER_OFF)
			itmp = 5000;
		/* channel can be initialized or not */
		fd = pcanbasic_get_fd(channel, pchan, NULL, &fd_close);
		ires = pcanfd_set_option(fd, PCANFD_OPT_FLASH_LED, &itmp, sizeof(itmp));
		if (fd_close)
			pcanfd_close(fd);
		if (ires < 0) {
			sts = pcanbasic_errno_to_status(-ires);
			goto pcanbasic_set_value_exit;
		}
		goto pcanbasic_set_value_exit_ok;
		break;
	case PCAN_BITRATE_ADAPTING:
		/* can be called with an uninitialized channel */
		size = sizeof(ctmp);
		/* allow only ON/OFF values */
		ctmp = *(__u8*)buffer;
		if (len > size)
			len = size;
		if (ctmp != PCAN_PARAMETER_ON &&
				ctmp != PCAN_PARAMETER_OFF) {
			sts = PCAN_ERROR_ILLPARAMVAL;
			goto pcanbasic_set_value_exit;
		}
		/* channel can be pre-initialized or not */
		if (pchan == NULL) {
			/* create channel to store INIT. value */
			pchan = pcanbasic_create_channel(channel, 1);
			if (pchan == NULL) {
				sts = PCAN_ERROR_UNKNOWN;
				goto pcanbasic_set_value_exit;
			}
		}
		else if (pchan->fd > -1){ 
			/* channel cannot be initialized */
			sts = PCAN_ERROR_ILLOPERATION;
			goto pcanbasic_set_value_exit;
		}
		pchan->bitrate_adapting = ctmp;
		goto pcanbasic_set_value_exit_ok;
		break;
	case PCAN_CHANNEL_CONDITION:
		/* set is not allowed */
		sts = PCAN_ERROR_ILLPARAMTYPE;
		goto pcanbasic_set_value_exit;
		break;
	}

	/* assert channel is initialized */
	if (pchan == NULL || pchan->fd < 0) {
		/* assert status is not OK  */
		if (sts == PCAN_ERROR_OK)
			sts = PCAN_ERROR_INITIALIZE;
		goto pcanbasic_set_value_exit;
	}
	/* handle parameters that require a channel */
	switch(parameter) {
	case PCAN_5VOLTS_POWER:
		/* not supported by driver */
		sts = PCAN_ERROR_ILLPARAMTYPE;
		goto pcanbasic_set_value_exit;
		break;
	case PCAN_DEVICE_ID:
		sts = pcanbasic_set_value_device_id(channel, pchan, buffer, len);
		goto pcanbasic_set_value_exit;
		break;
	case PCAN_RECEIVE_EVENT:
		/* always enabled via pchan->fd, mark not supported */
		sts = PCAN_ERROR_ILLOPERATION;
		goto pcanbasic_set_value_exit;
		break;
	case PCAN_MESSAGE_FILTER:
		size = sizeof(ctmp);
		/* allow only ON/OFF values */
		ctmp = *(__u8*)buffer;
		if (len > size)
			len = size;
		if (ctmp != PCAN_FILTER_CLOSE &&
				ctmp != PCAN_FILTER_OPEN) {
			sts = PCAN_ERROR_ILLPARAMVAL;
			goto pcanbasic_set_value_exit;
		}
		/* deleting all, opens everything */
		pcanfd_del_filters(pchan->fd);
		if (ctmp == PCAN_FILTER_CLOSE) {
			/* if (id_from > id_to) everything is closed */
			struct pcanfd_msg_filter filter;
			filter.id_from = 1;
			filter.id_to = 0;
			filter.msg_flags = 0;
			pcanfd_add_filter(pchan->fd, &filter);
		}
		break;
	case PCAN_BUSOFF_AUTORESET:
		size = sizeof(pchan->busoff_reset);
		if (len > size)
			len = size;
		memcpy(&pchan->busoff_reset, buffer, len);
		break;
	case PCAN_TRACE_LOCATION:
		if (pcbtrace_set_path(&pchan->tracer, (char *)buffer) != 0) {
			sts = PCAN_ERROR_ILLPARAMVAL;
			goto pcanbasic_set_value_exit;
		}
		if (pchan->tracer.status == PCAN_PARAMETER_ON) {
			enum pcaninfo_hw hw;
			__u32 idx;
			pcbtrace_close(&pchan->tracer);
			pcanbasic_get_hw(pchan->channel, &hw, &idx);
			ires = pcbtrace_open(&pchan->tracer, hw, idx);
			if (ires < 0) {
				sts = pcanbasic_errno_to_status(-ires);
				goto pcanbasic_set_value_exit;
			}
		}
		break;
	case PCAN_TRACE_STATUS:
		size = sizeof(pchan->tracer.status);
		if (len > size)
			len = size;
		memcpy(&pchan->tracer.status, buffer, len);
		if (pchan->tracer.status == PCAN_PARAMETER_ON) {
			enum pcaninfo_hw hw;
			__u32 idx;
			pcanbasic_get_hw(pchan->channel, &hw, &idx);
			ires = pcbtrace_open(&pchan->tracer, hw, idx);
			if (ires < 0) {
				pchan->tracer.status = PCAN_PARAMETER_OFF;
				pcbtrace_close(&pchan->tracer);
				sts = pcanbasic_errno_to_status(-ires);
				goto pcanbasic_set_value_exit;
			}
		}
		else {
			pcbtrace_close(&pchan->tracer);
		}
		break;
	case PCAN_TRACE_SIZE:
		if (pchan->tracer.status == PCAN_PARAMETER_ON) {
			sts = PCAN_ERROR_ILLOPERATION;
			goto pcanbasic_set_value_exit;
		}
		size = sizeof(pchan->tracer.maxsize);
		if (len > size)
			len = size;
		memcpy(&pchan->tracer.maxsize, buffer, len);
		break;
	case PCAN_TRACE_CONFIGURE:
		if (pchan->tracer.status == PCAN_PARAMETER_ON) {
			sts = PCAN_ERROR_ILLOPERATION;
			goto pcanbasic_set_value_exit;
		}
		size = sizeof(pchan->tracer.flags);
		if (len > size)
			len = size;
		memcpy(&pchan->tracer.flags, buffer, len);
		break;

	case PCAN_ALLOW_ERROR_FRAMES:
		size = sizeof(__u8);
		if (len < size) {
			sts = PCAN_ERROR_ILLPARAMVAL;
			goto pcanbasic_set_value_exit;
		}
		/* get options to retrieve all flags */
		ires = pcanfd_get_option(pchan->fd, PCANFD_OPT_ALLOWED_MSGS, &itmp, sizeof(itmp));
		if (ires < 0) {
			sts = pcanbasic_errno_to_status(-ires);
			goto pcanbasic_set_value_exit;
		}
		/* change flag */
		if (*(__u8 *)buffer == PCAN_PARAMETER_ON)
			itmp |= PCANFD_ALLOWED_MSG_ERROR;
		else
			itmp &= (__u32)~PCANFD_ALLOWED_MSG_ERROR;
		ires = pcanfd_set_option(pchan->fd, PCANFD_OPT_ALLOWED_MSGS, &itmp, sizeof(itmp));
		if (ires < 0) {
			sts = pcanbasic_errno_to_status(-ires);
			goto pcanbasic_set_value_exit;
		}
		break;
	case PCAN_ALLOW_RTR_FRAMES:
		size = sizeof(__u8);
		if (len < size) {
			sts = PCAN_ERROR_ILLPARAMVAL;
			goto pcanbasic_set_value_exit;
		}
		/* get options to retrieve all flags */
		ires = pcanfd_get_option(pchan->fd, PCANFD_OPT_ALLOWED_MSGS, &itmp, sizeof(itmp));
		if (ires < 0) {
			sts = pcanbasic_errno_to_status(-ires);
			goto pcanbasic_set_value_exit;
		}
		/* change flag */
		if (*(__u8 *)buffer == PCAN_PARAMETER_ON)
			itmp |= PCANFD_ALLOWED_MSG_RTR;
		else
			itmp &= (__u32)~PCANFD_ALLOWED_MSG_RTR;
		ires = pcanfd_set_option(pchan->fd, PCANFD_OPT_ALLOWED_MSGS, &itmp, sizeof(itmp));
		if (ires < 0) {
			sts = pcanbasic_errno_to_status(-ires);
			goto pcanbasic_set_value_exit;
		}
		break;
	case PCAN_ALLOW_STATUS_FRAMES:
		size = sizeof(__u8);
		if (len < size) {
			sts = PCAN_ERROR_ILLPARAMVAL;
			goto pcanbasic_set_value_exit;
		}
		/* get options to retrieve all flags */
		ires = pcanfd_get_option(pchan->fd, PCANFD_OPT_ALLOWED_MSGS, &itmp, sizeof(itmp));
		if (ires < 0) {
			sts = pcanbasic_errno_to_status(-ires);
			goto pcanbasic_set_value_exit;
		}
		/* change flag */
		if (*(__u8 *)buffer == PCAN_PARAMETER_ON)
			itmp |= PCANFD_ALLOWED_MSG_STATUS;
		else
			itmp &= (__u32)~PCANFD_ALLOWED_MSG_STATUS;
		ires = pcanfd_set_option(pchan->fd, PCANFD_OPT_ALLOWED_MSGS, &itmp, sizeof(itmp));
		if (ires < 0) {
			sts = pcanbasic_errno_to_status(-ires);
			goto pcanbasic_set_value_exit;
		}
		break;
	case PCAN_INTERFRAME_DELAY:
		size = sizeof(__u32);
		if (len < size) {
			sts = PCAN_ERROR_ILLPARAMVAL;
			goto pcanbasic_set_value_exit;
		}
		ires = pcanfd_set_option(pchan->fd, PCANFD_OPT_IFRAME_DELAYUS, buffer, len);
		if (ires < 0) {
			sts = pcanbasic_errno_to_status(-ires);
			goto pcanbasic_set_value_exit;
		}
		break;
	case PCAN_ACCEPTANCE_FILTER_11BIT:
		size = sizeof(__u64);
		if (len < size) {
			sts = PCAN_ERROR_ILLPARAMVAL;
			goto pcanbasic_set_value_exit;
		}
		ires = pcanfd_set_option(pchan->fd, PCANFD_OPT_ACC_FILTER_11B, buffer, len);
		if (ires < 0) {
			sts = pcanbasic_errno_to_status(-ires);
			goto pcanbasic_set_value_exit;
		}
		break;
	case PCAN_ACCEPTANCE_FILTER_29BIT:
		size = sizeof(__u64);
		if (len < size) {
			sts = PCAN_ERROR_ILLPARAMVAL;
			goto pcanbasic_set_value_exit;
		}
		ires = pcanfd_set_option(pchan->fd, PCANFD_OPT_ACC_FILTER_29B, buffer, len);
		if (ires < 0) {
			sts = pcanbasic_errno_to_status(-ires);
			goto pcanbasic_set_value_exit;
		}
		break;
	case PCAN_IO_DIGITAL_CONFIGURATION:
		size = sizeof(__u32);
		if (len < size) {
			sts = PCAN_ERROR_ILLPARAMVAL;
			goto pcanbasic_set_value_exit;
		}
		ires = pcanfd_set_option(pchan->fd, PCANFD_IO_DIGITAL_CFG, buffer, len);
		if (ires < 0) {
			if (-ires == EOPNOTSUPP)
				sts = PCAN_ERROR_ILLPARAMTYPE;
			else
				sts = pcanbasic_errno_to_status(-ires);
			goto pcanbasic_set_value_exit;
		}
		break;
	case PCAN_IO_DIGITAL_VALUE:
		size = sizeof(__u32);
		if (len < size) {
			sts = PCAN_ERROR_ILLPARAMVAL;
			goto pcanbasic_set_value_exit;
		}
		ires = pcanfd_set_option(pchan->fd, PCANFD_IO_DIGITAL_VAL, buffer, len);
		if (ires < 0) {
			if (-ires == EOPNOTSUPP)
				sts = PCAN_ERROR_ILLPARAMTYPE;
			else
				sts = pcanbasic_errno_to_status(-ires);
			goto pcanbasic_set_value_exit;
		}
		break;
	case PCAN_IO_DIGITAL_SET:
		size = sizeof(__u32);
		if (len < size) {
			sts = PCAN_ERROR_ILLPARAMVAL;
			goto pcanbasic_set_value_exit;
		}
		ires = pcanfd_set_option(pchan->fd, PCANFD_IO_DIGITAL_SET, buffer, len);
		if (ires < 0) {
			if (-ires == EOPNOTSUPP)
				sts = PCAN_ERROR_ILLPARAMTYPE;
			else
				sts = pcanbasic_errno_to_status(-ires);
			goto pcanbasic_set_value_exit;
		}
		break;
	case PCAN_IO_DIGITAL_CLEAR:
		size = sizeof(__u32);
		if (len < size) {
			sts = PCAN_ERROR_ILLPARAMVAL;
			goto pcanbasic_set_value_exit;
		}
		ires = pcanfd_set_option(pchan->fd, PCANFD_IO_DIGITAL_CLR, buffer, len);
		if (ires < 0) {
			if (-ires == EOPNOTSUPP)
				sts = PCAN_ERROR_ILLPARAMTYPE;
			else
				sts = pcanbasic_errno_to_status(-ires);
			goto pcanbasic_set_value_exit;
		}
		break;
	case PCAN_ALLOW_ECHO_FRAMES:
		if (!pchan->features.echo_frames) {
			sts = PCAN_ERROR_ILLPARAMTYPE;
			goto pcanbasic_set_value_exit;
		}
		size = sizeof(pchan->rcv_echo_status);
		if (len < size) {
			sts = PCAN_ERROR_ILLPARAMVAL;
			goto pcanbasic_set_value_exit;
		}
		pchan->rcv_echo_status = (*(__u8 *)buffer == PCAN_PARAMETER_ON);
		break;
	default:
		sts = PCAN_ERROR_ILLPARAMTYPE;
		goto pcanbasic_set_value_exit;
		break;
	}
pcanbasic_set_value_exit_ok:
	sts = PCAN_ERROR_OK;
pcanbasic_set_value_exit:
	return sts;
}

TPCANStatus pcanbasic_get_error_text(
		TPCANStatus error,
		WORD language,
		LPSTR buffer) {
	TPCANStatus sts;

	sts = PCAN_ERROR_OK;

	switch (language) {
	case 0x00:
		language = IDS_STR_IND_LANG_EN;
		break;
	case 0x07:
		language = IDS_STR_IND_LANG_DE;
		break;
	case 0x09:
		language = IDS_STR_IND_LANG_EN;
		break;
	case 0x0A:
		language = IDS_STR_IND_LANG_ES;
		break;
	case 0x0C:
		language = IDS_STR_IND_LANG_FR;
		break;
	case 0x10:
		language = IDS_STR_IND_LANG_IT;
		break;
	default:
		language = IDS_STR_IND_LANG_EN;
		break;
	}

	switch (error) {
	case PCAN_ERROR_OK:
		strcpy(buffer, resource[language][IDS_STR_IND_ERR_OK]);
		break;
	case PCAN_ERROR_XMTFULL:
		strcpy(buffer, resource[language][IDS_STR_IND_ERR_XMTFULL]);
		break;
	case PCAN_ERROR_OVERRUN:
		strcpy(buffer, resource[language][IDS_STR_IND_ERR_OVERRUN]);
		break;
	case PCAN_ERROR_BUSLIGHT:
		strcpy(buffer, resource[language][IDS_STR_IND_ERR_BUSLIGHT]);
		break;
	case PCAN_ERROR_BUSHEAVY:
		strcpy(buffer, resource[language][IDS_STR_IND_ERR_BUSHEAVY]);
		break;
	case PCAN_ERROR_BUSOFF:
		strcpy(buffer, resource[language][IDS_STR_IND_ERR_BUSOFF]);
		break;
	case PCAN_ERROR_ANYBUSERR:
		strcpy(buffer, resource[language][IDS_STR_IND_ERR_ANYBUSERR]);
		break;
	case PCAN_ERROR_QRCVEMPTY:
		strcpy(buffer, resource[language][IDS_STR_IND_ERR_QRCVEMPTY]);
		break;
	case PCAN_ERROR_QOVERRUN:
		strcpy(buffer, resource[language][IDS_STR_IND_ERR_QOVERRUN]);
		break;
	case PCAN_ERROR_QXMTFULL:
		strcpy(buffer, resource[language][IDS_STR_IND_ERR_QXMTFULL]);
		break;
	case PCAN_ERROR_REGTEST:
		strcpy(buffer, resource[language][IDS_STR_IND_ERR_REGTEST]);
		break;
	case PCAN_ERROR_NODRIVER:
		strcpy(buffer, resource[language][IDS_STR_IND_ERR_NODRIVER]);
		break;
	case PCAN_ERROR_RESOURCE:
		strcpy(buffer, resource[language][IDS_STR_IND_ERR_RESOURCE]);
		break;
	case PCAN_ERROR_ILLPARAMTYPE:
		strcpy(buffer, resource[language][IDS_STR_IND_ERR_ILLPARAMTYPE]);
		break;
	case PCAN_ERROR_ILLPARAMVAL:
		strcpy(buffer, resource[language][IDS_STR_IND_ERR_ILLPARAMVAL]);
		break;
#if PCAN_ERROR_ILLCLIENT != PCAN_ERROR_ILLHANDLE
	case PCAN_ERROR_ILLHANDLE:
		strcpy(buffer, resource[language][IDS_STR_IND_ERR_ILLHANDLE]);
		break;
#endif
	case PCAN_ERROR_INITIALIZE:
		strcpy(buffer, resource[language][IDS_STR_IND_ERR_INITIALIZE]);
		break;
	case PCAN_ERROR_UNKNOWN:
		strcpy(buffer, resource[language][IDS_STR_IND_ERR_UNKNOW]);
		break;
	case PCAN_ERROR_HWINUSE:
		strcpy(buffer, resource[language][IDS_STR_IND_ERR_HWINUSE]);
		break;
	case PCAN_ERROR_NETINUSE:
		strcpy(buffer, resource[language][IDS_STR_IND_ERR_NETINUSE]);
		break;
	case PCAN_ERROR_ILLHW:
		strcpy(buffer, resource[language][IDS_STR_IND_ERR_ILLHW]);
		break;
	case PCAN_ERROR_ILLNET:
		strcpy(buffer, resource[language][IDS_STR_IND_ERR_ILLNET]);
		break;
	case PCAN_ERROR_ILLCLIENT:
		strcpy(buffer, resource[language][IDS_STR_IND_ERR_ILLCLIENT]);
		break;
	case PCAN_ERROR_ILLDATA:
		strcpy(buffer, resource[language][IDS_STR_IND_ERR_ILLDATA]);
		break;
	case PCAN_ERROR_ILLOPERATION:
		strcpy(buffer, resource[language][IDS_STR_IND_ERR_ILLOPERATION]);
		break;
	case PCAN_ERROR_BUSPASSIVE:
		strcpy(buffer, resource[language][IDS_STR_IND_ERR_BUSPASSIVE]);
		break;
	case PCAN_ERROR_CAUTION:
		strcpy(buffer, resource[language][IDS_STR_IND_ERR_CAUTION]);
		break;
	case PCAN_ERROR_ILLMODE:
		strcpy(buffer, resource[language][IDS_STR_IND_ERR_ILLMODE]);
		break;
	default:
		sprintf(buffer, "Undefined (0x%x)", error);
		sts = PCAN_ERROR_ILLPARAMVAL;
		break;
	}

	return sts;
}

TPCANStatus pcanbasic_lookup_channel(
	LPSTR Parameters,
	TPCANHandle* FoundChannel) {
	TPCANStatus sts;
	pcanbasic_lookup_info lui;

	if (FoundChannel == NULL) {
		return PCAN_ERROR_ILLPARAMVAL;
	}
	sts = pcanbasic_parse_lookup(&lui, Parameters);
	if (sts == PCAN_ERROR_OK) {
		struct pcaninfo* pinfo;
		int i;

		*FoundChannel = PCAN_NONEBUS;
		/*  force HW refresh */
		pcanbasic_refresh_hw();
		g_basiccore.refresh_locked = 1;
		for (i = 0; i < g_basiccore.devices->length; ++i) {
			pinfo = &g_basiccore.devices->infos[i];

			if (((lui.flags & LOOKUP_FLAG_DEV_TYPE) == LOOKUP_FLAG_DEV_TYPE) &&
				(pinfo->hwcategory != lui.dev_type)) {
				continue;
			}
			if (((lui.flags & LOOKUP_FLAG_DEV_ID) == LOOKUP_FLAG_DEV_ID) && 
				(pinfo->devid != lui.dev_id)) {
				continue;
			}
			if (((lui.flags & LOOKUP_FLAG_DEV_CTRL_NB) == LOOKUP_FLAG_DEV_CTRL_NB) && 
				(pinfo->ctrlnb != lui.dev_ctrl_nb)) {
				continue;
			}
			if (((lui.flags & LOOKUP_FLAG_IP) == LOOKUP_FLAG_IP)) {
				/* not supported by driver */
				sts = PCAN_ERROR_NODRIVER;
			}
			if (((lui.flags & LOOKUP_FLAG_HW_HANDLE) == LOOKUP_FLAG_HW_HANDLE) && 
				(pinfo->hwindex != (17 - lui.hw_handle))) {
				continue;
			}
			if (sts == PCAN_ERROR_OK) {
				/* this handle matches requirements (save and exit)  */
				*FoundChannel = pcanbasic_get_handle(pinfo->path, g_basiccore.devices);
			}
			i = g_basiccore.devices->length;
		}
		g_basiccore.refresh_locked = 0;
	}
	else {
		sts = PCAN_ERROR_ILLPARAMVAL;
	}
	return sts;
}
