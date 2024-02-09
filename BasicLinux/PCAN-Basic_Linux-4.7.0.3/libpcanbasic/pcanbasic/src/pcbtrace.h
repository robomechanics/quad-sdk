/* SPDX-License-Identifier: LGPL-2.1-only */
/*
 * @file pcbtrace.h
 * @brief Function's prototypes for the tracer functions of Linux PCANBasic
 * $Id: pcbtrace.h 15919 2022-12-16 09:28:20Z Fabrice $
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

#ifndef __PCBTRACE_H__
#define __PCBTRACE_H__

/*
 * INCLUDES
 */
#include <sys/types.h>
#include <sys/time.h>
#include <stdio.h>
#include "PCANBasic.h"
#include "pcaninfo.h"

/*
 * DEFINES
 */
#define PCBTRACE_MAX_CHAR_SIZE 256	/**< Max buffer size used in struct pcbtrace */

#ifndef PCAN_MESSAGE_ECHO
/**
 * Internal parameter to support libpcanfd flag: PCANFD_MSG_ECHO
 */
#define PCAN_MESSAGE_ECHO			0x20
#endif
/**
 * Supported versions of trace (.trc) files.
 */
enum pcbtrace_version {
	V1_1,
	V2_0,
};

/**
 * A structure to hold the context information for a PCANBasic tracer
 */
struct pcbtrace_ctx {
	char* directory;			/**< path to the trace directory */
	char chname[50];			/**< short name of the TPANHANDLE */
	char filename_chunk[PCBTRACE_MAX_CHAR_SIZE+2];	/**< base name of the segmented trace's file */
	char filename[50+2*PCBTRACE_MAX_CHAR_SIZE];		/**< current name of the trace's file */
	unsigned int idx;			/**< index of the file (for segmented traces) */
	unsigned short status;		/**< status of the tracer */
	unsigned short maxsize;		/**< maximum size of the trace file in MB */
	unsigned int flags;			/**< trace configuration (see TRACE_FILE_xxx in PCANBasic.h) */
	FILE *pfile;				/**< file descriptor */
	unsigned long msg_cnt;		/**< count the number of CAN messages traced */
	struct timespec time_start;	/**< time of the channel's initialisation */
	struct pcaninfo *pinfo;		/**< channel's information */
	unsigned int clk_ref;		/**< clock reference used by PCAN driver */
};

/**
 * @fn void pcbtrace_set_defaults(struct pcbtrace_ctx *ctx)
 * @brief Initializes a PCANBasic tracer context with default values.
 *
 * @param ctx pointer to a context information of the PCANBasic tracer
 */
void pcbtrace_set_defaults(struct pcbtrace_ctx *ctx);

/**
 * @fn uint8_t pcbtrace_set_path(struct pcbtrace_ctx *ctx, const char* path)
 * @brief Initializes the path of a PCANBasic tracer context.
 *
 * @param ctx pointer to a context information of the PCANBasic tracer
 * @param path path to store trace file
 * @return 0 on success or an errno otherwise
 */
uint8_t pcbtrace_set_path(struct pcbtrace_ctx *ctx, const char* path);


/**
 * @fn void pcbtrace_release(struct pcbtrace_ctx *ctx)
 * @brief Releases a PCANBasic tracer context.
 *
 * @param ctx pointer to a context information of the PCANBasic tracer
 */
void pcbtrace_release(struct pcbtrace_ctx *ctx);

/**
 * @fn int pcbtrace_open(struct pcbtrace_ctx *ctx, enum pcaninfo_hw hw, uint ch_idx)
 * @brief Opens a trace file based on the context information.
 *
 * @param ctx pointer to a context information of the PCANBasic tracer
 * @param hw type of the PCAN hardware
 * @param ch_idx channel index
 * @return 0 on success or an errno otherwise
 */
int pcbtrace_open(struct pcbtrace_ctx *ctx, enum pcaninfo_hw hw, unsigned int ch_idx);

/**
 * @fn int pcbtrace_close(struct pcbtrace_ctx *ctx)
 * @brief Closes a PCANBasic tracer.
 *
 * @param ctx pointer to a context information of the PCANBasic tracer
 * @return 0 on success or an errno otherwise
 */
int pcbtrace_close(struct pcbtrace_ctx *ctx);

/**
 * @fn int pcbtrace_write_msg(struct pcbtrace_ctx *ctx, TPCANMsgFD *msg, int data_len, struct timeval *tv, int rx)
 * @brief Writes a CAN FD message to the trace file.
 *
 * @param ctx pointer to a context information of the PCANBasic tracer
 * @param msg pointer to the CAN FD message to output
 * @param data_len the real data length of the message
 * @param tv timestamp of the message
 * @param rx 1 if the message was received or 0 if it was transmitted
 * @return 0 on success or an errno otherwise
 */
int pcbtrace_write_msg(struct pcbtrace_ctx *ctx, TPCANMsgFD *msg, int data_len, struct timeval *tv, int rx);

/**
 * @fn int pcbtrace_write(struct pcbtrace_ctx *ctx, const char * buffer, uint size)
 * @brief Writes a string message to the trace file.
 *
 * @param ctx pointer to a context information of the PCANBasic tracer
 * @param buffer pointer to the string to output
 * @param size size of the buffer
 * @return 0 on success or an errno otherwise
 */
int pcbtrace_write(struct pcbtrace_ctx *ctx, const char * buffer, unsigned int size);

#endif
