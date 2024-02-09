/* SPDX-License-Identifier: LGPL-2.1-only */
/*
 * @file pcblog.c
 * @brief Logger for PCANBasic API
 *
 * $Id: pcblog.c 15919 2022-12-16 09:28:20Z Fabrice $
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

#include "pcblog.h"

/*
 * INCLUDES
 */
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

#include "PCANBasic.h"
#include "pcbcore.h" /* to get API version without triggering a log entry */

/*
 * DEFINES
 */
/** Max length of a log string */
#define MAX_LOG		256
/** Default log file name */
#define LOG_FILE	"PCANBasic.log"

#define PCBLOG_DEFAULT_PATH	"."

/*
 * STRUCTURES AND TYPES
 */
/** Structure to keep track of the logger's context */
struct pcanbasic_logger {
	int initialized;	/**< states if logger was initialized */
	char* path;			/**< directory path to store logs */
	int enabled; 		/**< logger's status (0 disabled; 1 enabled) */
	int flags;			/**< logger's configuration (see PCAN_LOG_xxx) */
	int fd; 			/**< file descriptor */
};


/** PRIVATE VARIABLES */
/** stores the context of the PCANBasic logger */
static struct pcanbasic_logger g_pcblog = {0, NULL, 0, LOG_FUNCTION_DEFAULT, -1};

/** PRIVATE FUNCTIONS DECLARATIONS */
/**
 * @fn void pcblog_close(void)
 * @brief Closes the PCANBasic log file.
 */
static void pcblog_close(void);
/**
 * @fn void pcblog_check(int init_only)
 * @brief Checks if the PCANBasic log file is opened and do so if required.
 */
static void pcblog_check(int init_only);
/**
 * @fn void pcblog_atexit(void)
 * @brief A callback function to close log file when application stops.
 */
static void pcblog_atexit(void);
/**
 * @fn int pcblog_write_opened(void)
 * @brief Appends a default header when a log file is opened.
 */
static int pcblog_write_opened(void);
/**
 * @fn int pcblog_write_closed(void)
 * @brief Appends a default footer when a log file is closed.
 */
static int pcblog_write_closed(void);
/**
 * @fn int pcblog_write_unsafe(const char * s)
 * @brief Appends a message to the PCANBasic log.
 *
 * @param[in] s message to append
 */
static int pcblog_write_unsafe(const char * msg);
/**
 * @fn int pcblog_write_unsafe(const char * s, unsigned long len)
 * @brief Appends a message to the PCANBasic log.
 *
 * @param[in] s message to append
 * @param[in] len size of the message
 */
static int pcblog_nwrite_unsafe(const char * s, unsigned long len);

/* PRIVATE FUNCTIONS */
void pcblog_close() {
	if (g_pcblog.fd >= 0) {
		pcblog_write_closed();
		close(g_pcblog.fd);
		g_pcblog.fd = -1;
	}
}

void pcblog_check(int init_only) {
	/* check/initialize logger */
	if (!g_pcblog.initialized) {
		g_pcblog.initialized = 1;
		atexit(pcblog_atexit);
		if (g_pcblog.path == NULL)
			pcblog_set_location(NULL);
	}
	/* open file */
	if (!init_only && g_pcblog.enabled && g_pcblog.fd < 0 && g_pcblog.path != NULL) {
		char szFileName[PCAN_LOG_MAX_PATH + 16];
		sprintf(szFileName, "%s/%s", g_pcblog.path, LOG_FILE);
		g_pcblog.fd = open(szFileName, O_WRONLY | O_APPEND | O_CREAT, 0644);
		if (g_pcblog.fd >= 0)
			pcblog_write_opened();
	}
}

void pcblog_atexit(void) {
	pcblog_close();
	if (g_pcblog.path != NULL) {
		free(g_pcblog.path);
		g_pcblog.path = NULL;
	}
	g_pcblog.enabled = 0;
}

int pcblog_write_opened() {
	int res = 0;
	char buf[MAX_LENGTH_VERSION_STRING] = { 0 };
	char szMessage[MAX_LOG+MAX_LENGTH_VERSION_STRING];

	pcanbasic_get_value(PCAN_NONEBUS, PCAN_API_VERSION, buf, sizeof(buf));
	sprintf(szMessage, "«           API v%-20s»", buf);

	res += pcblog_write_unsafe("«____________________________________»");
	res += pcblog_write_unsafe("«           PCAN-Basic Log           »");
	res += pcblog_write_unsafe(szMessage);
	res += pcblog_write_unsafe("«____________________________________»");
	return res;
}

int pcblog_write_closed() {
	int res = 0;
	res += pcblog_write_unsafe("«____________________________________»");
	res += pcblog_write_unsafe("«            ############            »");
	res += pcblog_write_unsafe("«____________________________________»");
	return res;
}

int pcblog_write_unsafe(const char * s) {
	return pcblog_nwrite_unsafe(s, strlen(s));
}

int pcblog_nwrite_unsafe(const char * s, unsigned long len) {
	time_t t;
	struct tm * ptm;
	char *stime;
	ssize_t n;

	if (g_pcblog.fd < 0)
		return -1;
	/* create time info */
	time(&t);
	ptm = localtime(&t);
	stime = asctime(ptm);
	/* removes trailing '\n' */
	stime[strlen(stime) - 1] = 0;
	/* write time */
	n = 0;
	n += write(g_pcblog.fd, stime, strlen(stime));
	n += write(g_pcblog.fd, " - ", 3);
	/* write message */
	if (s)
		n += write(g_pcblog.fd, s, len);
	n += write(g_pcblog.fd, "\n", 1);
	return n;
}

/* PUBLIC FUNCTIONS */
int pcblog_write(const char * msg, unsigned long len) {
	pcblog_check(0);
	return pcblog_nwrite_unsafe(msg, len);
}

int pcblog_write_entry(const char *sfunc) {
	int res = 0;
	if (g_pcblog.enabled && (g_pcblog.flags & LOG_FUNCTION_ENTRY)) {
		char szMessage[MAX_LOG];
		sprintf(szMessage, "ENTRY      '%s'", sfunc);
		res += pcblog_write(szMessage, strlen(szMessage));
	}
	return res;
}

int pcblog_write_param(const char *sfunc, const char *sparam) {
	int res = 0;
	if (g_pcblog.enabled && (g_pcblog.flags & LOG_FUNCTION_PARAMETERS)) {
		char szMessage[MAX_LOG];
		sprintf(szMessage, "  PARAMETERS of %s: %s", sfunc, sparam);
		res += pcblog_write(szMessage, strlen(szMessage));
	}
	return res;
}

int pcblog_write_exit(const char *sfunc, TPCANStatus sts) {
	int res = 0;
	if (g_pcblog.enabled && (g_pcblog.flags & LOG_FUNCTION_LEAVE)) {
		char szMessage[MAX_LOG];
		sprintf(szMessage, "EXIT       '%s' -   RESULT: 0x%02X", sfunc,
				(unsigned int) sts);
		res += pcblog_write(szMessage, strlen(szMessage));
	}
	return res;
}

int pcblog_write_exception(const char *sfunc) {
	int res = 0;
	if (g_pcblog.enabled) {
		char szMessage[MAX_LOG];
		sprintf(szMessage, "EXCEPTION FOUND IN '%s'", sfunc);
		res += pcblog_write(szMessage, strlen(szMessage));
	}
	return res;
}

int pcblog_write_can_msg(TPCANHandle channel, int direction, TPCANMsg* pmsg) {
	int res = 0;
	if (g_pcblog.enabled && (g_pcblog.flags & direction) && pmsg) {
		TPCANMsgFD msg;
		msg.ID = pmsg->ID;
		msg.DLC = pmsg->LEN;
		msg.MSGTYPE = pmsg->MSGTYPE;
		memcpy(msg.DATA, pmsg->DATA, sizeof(pmsg->DATA));
		res += pcblog_write_canfd_msg(channel, direction, &msg);
	}
	return res;
}
int pcblog_write_canfd_msg(TPCANHandle channel, int direction, TPCANMsgFD* pmsg) {
	int res = 0;
	if (g_pcblog.enabled && (g_pcblog.flags & direction) && pmsg) {
		char szId[50];
		char szFormat[MAX_LOG];
		char szMessage[2 * MAX_LOG];
		int i;
		int len = pcanbasic_get_fd_len(pmsg->DLC);
		
		/* format CAN ID */
		if (pmsg->MSGTYPE & PCAN_MESSAGE_EXTENDED)
			sprintf(szId, "0x%08X", pmsg->ID);
		else 
			sprintf(szId, "0x%04X", pmsg->ID);
		/* format MSGTYPE */
		if (pmsg->MSGTYPE == PCAN_MESSAGE_STANDARD) {
			szFormat[0] = '\0';
		}
		else {
			sprintf(szFormat, "[");
			if (pmsg->MSGTYPE & PCAN_MESSAGE_ERRFRAME) {
				strcat(szFormat, "ERR|");
			}
			if (pmsg->MSGTYPE & PCAN_MESSAGE_STATUS) {
				strcat(szFormat, "STS|");
			}
			if (pmsg->MSGTYPE & PCAN_MESSAGE_ESI) {
				strcat(szFormat, "ESI|");
			}
			if (pmsg->MSGTYPE & PCAN_MESSAGE_RTR) {
				strcat(szFormat, "RTR|");
			}
			if (pmsg->MSGTYPE & PCAN_MESSAGE_FD) {
				strcat(szFormat, "FD|");
				if (pmsg->MSGTYPE & PCAN_MESSAGE_BRS) {
					strcat(szFormat, "BRS|");
				}
			}
			size_t formatLen = strlen(szFormat);
			if (formatLen > 1) {
				szFormat[formatLen - 1] = 0;
				strcat(szFormat, "]");
			} 
			else {
				szFormat[0] = '\0';
			}
		}	
		sprintf(
				szMessage,
				"    CHANNEL    0x%02X (%s) %s ID=%s DLC=%d Len=%u, Data=",
				(unsigned int) channel, 
				direction == LOG_FUNCTION_WRITE ? "OUT" : "IN", 
				szFormat, szId, pmsg->DLC, len);
		
		for (i = 0; i < len; ++i) {
			sprintf(szFormat, "0x%02X ", pmsg->DATA[i]);
			strcat(szMessage, szFormat);
		}
		res += pcblog_write(szMessage, strlen(szMessage));
	}
	return res;
}

/* PUBLIC FUNCTIONS: getters/setters */
void pcblog_get_location(char *buffer, unsigned int size) {
	if (buffer == NULL || size == 0)
		return;
	/* check if path was never initialized */
	if (g_pcblog.path == NULL || strcmp(g_pcblog.path, PCBLOG_DEFAULT_PATH) == 0) {
		/* convert to absolute path */
		pcblog_set_location(NULL);
	}
	snprintf(buffer, size, "%s", g_pcblog.path);
}
uint8_t pcblog_set_location(const char *buffer) {
	const char * stmp;
	uint8_t result;

	if (buffer == NULL)
		stmp = PCBLOG_DEFAULT_PATH;
	else if (strnlen(buffer, PCAN_LOG_MAX_PATH) > 0)
		stmp = buffer;
	else
		stmp = PCBLOG_DEFAULT_PATH;

	if (g_pcblog.path != NULL)
		free(g_pcblog.path);
	g_pcblog.path = realpath(stmp, NULL);
	result = (g_pcblog.path != NULL);
	if (!result) {
		/* given path is invalid, revert to the default path */
		g_pcblog.path = realpath(PCBLOG_DEFAULT_PATH, NULL);
		if (g_pcblog.path  == NULL) {
			/* absolute path failed... revert to relative path */
			g_pcblog.path = strdup(PCBLOG_DEFAULT_PATH);
		}
	}
	/* close log if any,
	 * it will be reopened automatically with the new path */
	pcblog_close();
	/* assert logger is initialized, in order to free allocated path */
	pcblog_check(1);
	return result;
}

int pcblog_get_status(void) {
	return g_pcblog.enabled;
}
void pcblog_set_status(int status) {
	g_pcblog.enabled = status;
	if (g_pcblog.enabled)
		pcblog_check(0);
}

int pcblog_get_config(void) {
	return g_pcblog.flags;
}
void pcblog_set_config(int flags) {
	g_pcblog.flags = flags;
}
