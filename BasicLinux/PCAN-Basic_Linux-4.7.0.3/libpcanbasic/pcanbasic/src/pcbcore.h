/* SPDX-License-Identifier: LGPL-2.1-only */
/*
 * @file pcbcore.h
 * @brief Function prototypes for the core functions of Linux PCANBasic
 * $Id: pcbcore.h 15919 2022-12-16 09:28:20Z Fabrice $
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

#ifndef SRC_PCBCORE_H_
#define SRC_PCBCORE_H_

#include "PCANBasic.h"
#include "pcaninfo.h"



/* Extra functions to ease PCANBasic API usage on linux */
/**
 * @fn PCANINFO * pcanbasic_get_info(TPCANHandle channel)
 * @brief Returns the PCANINFO corresponding to an initialized channel handle.
 *
 * @param channel The handle of a previously initialized channel
 * @return Pointer to a PCANINFO structure or NULL if not found.
 */
struct pcaninfo * pcanbasic_get_info(TPCANHandle channel);

/**
 * @fn TPCANHandle pcanbasic_get_handle(char * device, struct pcaninfo_list * pcil)
 * @brief Returns the handle corresponding to a file path (like "/dev/pcanusb1").
 *
 * @param device a buffer holding the device path (with a NULL-termination char.)
 * @param pcil An initialized list of PCANINFO (if NULL, scanning is done internally)
 * @return A Channel handle corresponding to the device or PCAN_NONEBUS if not found.
 */
TPCANHandle pcanbasic_get_handle(char * device, struct pcaninfo_list * pcil);

/**
 * @fn __u8 pcanbasic_get_fd_dlc(int len)
 * @brief Returns the DLC associated to a length.
 *
 * @param len the length of the CAN message to convert to DLC
 * @return the DLC corresponding to the length.
 */
__u8 pcanbasic_get_fd_dlc(int len);
/**
 * @fn int pcanbasic_get_fd_len(__u8 dlc)
 * @brief Returns the length associated to a DLC.
 *
 * @param dlc the CAN Data Length Code to convert
 * @return the length associated to the DLC.
 */
int pcanbasic_get_fd_len(__u8 dlc);


/* Standard PCANBasic API functions (see pcanbasic.h) */

TPCANStatus pcanbasic_initialize(
		TPCANHandle channel,
		TPCANBaudrate btr0btr1,
		TPCANType type,
		DWORD base,
		WORD irq);

TPCANStatus pcanbasic_initialize_fd(
		TPCANHandle channel,
		TPCANBitrateFD bitratefd);

TPCANStatus pcanbasic_uninitialize(
		TPCANHandle channel);

TPCANStatus pcanbasic_reset(
		TPCANHandle channel);

TPCANStatus pcanbasic_get_status(
		TPCANHandle channel);

TPCANStatus pcanbasic_read(
		TPCANHandle channel,
		TPCANMsg* message,
		TPCANTimestamp* timestamp);

TPCANStatus pcanbasic_read_fd(
		TPCANHandle channel,
		TPCANMsgFD* message,
		TPCANTimestampFD *timestamp);

TPCANStatus pcanbasic_write(
		TPCANHandle channel,
		TPCANMsg* message);

TPCANStatus pcanbasic_write_fd(
		TPCANHandle channel,
		TPCANMsgFD* message);

TPCANStatus pcanbasic_filter(
		TPCANHandle channel,
		DWORD from,
		DWORD to,
		TPCANMode mode);
TPCANStatus pcanbasic_get_value(
		TPCANHandle channel,
		TPCANParameter parameter,
		void* buffer,
		DWORD length);

TPCANStatus pcanbasic_set_value(
		TPCANHandle channel,
		TPCANParameter parameter,
		void* buffer,
		DWORD length);

TPCANStatus pcanbasic_get_error_text(
		TPCANStatus error,
		WORD language,
		LPSTR buffer);

TPCANStatus pcanbasic_lookup_channel(
	LPSTR Parameters,
	TPCANHandle* FoundChannel);

#endif /* SRC_PCBCORE_H_ */
