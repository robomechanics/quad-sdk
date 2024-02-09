/* SPDX-License-Identifier: LGPL-2.1-only */
/*
 * @file libpcanbasic.cpp
 * @brief PCAN Basic for LINUX API with PCAN Driver <8.0
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
 * Contact:    <linux@peak-system.com>
 * Maintainer: Stephane Grosjean <s.grosjean@peak-system.com>
 * Maintainer: Fabrice Vergnaud <f.vergnaud@peak-system.com>
 * Author:     Thomas Haber <thomas@toem.de>
 */
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <time.h>

#define TPCANMsg _TPCANMsg
#include <pcan.h>
#undef TPCANMsg

#define LPSTR  char*
#include "PCANBasic.h"
#include "resource.h"

#ifndef NO_RT
#include <rtdm/rtdm.h>
#endif

#define VERSION_MAJOR		2
#define VERSION_MINOR		0
#define VERSION_PATCH		4
#define VERSION_BUILD		6
#define STR_HELPER(x)		#x
#define STR(x) 				STR_HELPER(x)

////////////////////////////////////////////////////////////////////////////////////////////////////
// DEFINES

#define PROCFILE "/proc/pcan"                        // where to get information
#define MAX_LINE_LEN 255                             // to store a line of text
#define DEVICE_PATH "/dev/pcan"                      // + Minor = real device path
#define LOCAL_STRING_LEN 64                          // length of internal used strings
#ifndef NO_RT
#define __ioctl(x, y, z) rt_dev_ioctl(x, y, z)
#define __close(fp)      rt_dev_close(fp)
#else
#define __ioctl(x, y, z) ioctl(x, y, z)
#define __close(fp)      close(fp)
//#define __close(fp) 
#endif

#define MINOR_FIRST_PCI 0                            // First minor for PCI
#define MINOR_FIRST_USB 32                           // First minor for USB
#define MINOR_FIRST_PCC 40                           // First minor for PCC
#define API_VERSION  STR(VERSION_MAJOR) "." STR(VERSION_MINOR) "." STR(VERSION_PATCH) "." STR(VERSION_BUILD) // Version
#define MAX_LOG 256                                  // Max length of a log string
#define LOG_FILE "PCANBasic.log"                     // LOG file name
////////////////////////////////////////////////////////////////////////////////////////////////////
// GLOBALS

typedef struct {
	char szDevicePath[LOCAL_STRING_LEN]; // Path of the device
	int nFileNo; // File number
	TPCANBaudrate Btr0Btr1; // Baud rate
	int nFilter; // Filter status
	int nListenOnly; // Listen mode
} PCAN_DESCRIPTOR;
PCAN_DESCRIPTOR** ppDescriptors = NULL;

char szLogLocationPath[LOCAL_STRING_LEN]; // LOG path
int nLogPathStatus = -1; // Path status (-1 not set; 0 set; 1 changed)
int nLogFileNo = -1; // LOG file number
int nLogEnabled = 0; // LOG enablement
int nLogFunction = LOG_FUNCTION_DEFAULT; // LOG configuration


////////////////////////////////////////////////////////////////////////////////////////////////////
// LOG Functions

static void LOG_Print(const char *sz1);

////////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>	Writes inital LOG message. </summary>
///
/// <remarks>	Thomas, 15.06.2010. </remarks>
////////////////////////////////////////////////////////////////////////////////////////////////////

static void LOG_PrintOpen() {
	LOG_Print("«____________________________________»");
	LOG_Print("«           PCAN-Basic Log           »");
	LOG_Print("«____________________________________»");
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>	Writes final LOG message. </summary>
///
/// <remarks>	Thomas, 15.06.2010. </remarks>
////////////////////////////////////////////////////////////////////////////////////////////////////

static void LOG_PrintClose() {
	LOG_Print("«____________________________________»");
	LOG_Print("«            ############            »");
	LOG_Print("«____________________________________»");
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>	Checks the log file und opens it if needed. </summary>
///
/// <remarks>	Thomas, 15.06.2010. </remarks>
////////////////////////////////////////////////////////////////////////////////////////////////////

static void LOG_CheckFile() {

	// check if file needs to be closed
	if (nLogPathStatus == 1 && nLogFileNo >= 0) {
		nLogPathStatus = 0;
		LOG_PrintClose();
		close(nLogFileNo);
		nLogFileNo = -1;
	}
	// open on specific path
	if (nLogPathStatus == 0 && nLogFileNo < 0) {
		char szFileName[LOCAL_STRING_LEN + 16];
		sprintf(szFileName, "%s/%s", szLogLocationPath, LOG_FILE);
		nLogFileNo = open(szFileName, O_WRONLY | O_APPEND, 0644);
		LOG_PrintOpen();
	}
	// open on default path
	if (nLogPathStatus == -1 && nLogFileNo < 0) {
		nLogFileNo = open(LOG_FILE, O_WRONLY | O_CREAT | O_APPEND, 0644);
		LOG_PrintOpen();
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>	Prints a log message. </summary>
///
/// <remarks>	 </remarks>
///
/// <param name="sz1">	The message. </param>
////////////////////////////////////////////////////////////////////////////////////////////////////

static void LOG_Print(const char *sz1) {
	LOG_CheckFile();
	if (nLogFileNo >= 0) {

		time_t Rawtime;
		struct tm * pTimeinfo;
		int nResult = 0;

		// create time info
		time(&Rawtime);
		pTimeinfo = localtime(&Rawtime);
		char *szTimeInfo = asctime(pTimeinfo);
		szTimeInfo[strlen(szTimeInfo) - 1] = 0;

		// write time
		nResult += write(nLogFileNo, szTimeInfo, strlen(szTimeInfo));
		nResult += write(nLogFileNo, " - ", 3);

		// write message
		if (sz1)
			nResult += write(nLogFileNo, sz1, strlen(sz1));
		nResult += write(nLogFileNo, "\n", 1);

	}

}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>	LOG for entering a function. </summary>
///
/// <remarks>	 </remarks>
///
/// <param name="szFunction">	The function. </param>
////////////////////////////////////////////////////////////////////////////////////////////////////

static void LOG_EnteringTo(const char *szFunction) {
	if (nLogEnabled && (nLogFunction & LOG_FUNCTION_ENTRY)) {
		char szMessage[MAX_LOG];
		sprintf(szMessage, "ENTRY      '%s'", szFunction);
		LOG_Print(szMessage);
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>	Logs the parameters. </summary>
///
/// <remarks>	 </remarks>
///
/// <param name="szFunction">	The function. </param>
/// <param name="szParameters">	The parameters. </param>
////////////////////////////////////////////////////////////////////////////////////////////////////

static void LOG_Parameters(const char *szFunction, const char *szParameters) {
	if (nLogEnabled && (nLogFunction & LOG_FUNCTION_PARAMETERS)) {
		char szMessage[MAX_LOG];
		sprintf(szMessage, "PARAMETERS of %s: %s", szFunction, szParameters);
		LOG_Print(szMessage);
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>	LOG for leaving a function.. </summary>
///
/// <remarks>	 </remarks>
///
/// <param name="szFunction">	The function. </param>
/// <param name="Result">		The result. </param>
////////////////////////////////////////////////////////////////////////////////////////////////////

static void LOG_LeavingFrom(const char *szFunction, TPCANStatus Result) {
	if (nLogEnabled && (nLogFunction & LOG_FUNCTION_LEAVE)) {
		char szMessage[MAX_LOG];
		sprintf(szMessage, "EXIT       '%s' -   RESULT: 0x%02X", szFunction,
				(unsigned int) Result);
		LOG_Print(szMessage);
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>	Logs an exception. </summary>
///
/// <remarks>	 </remarks>
///
/// <param name="szFunction">	The function. </param>
////////////////////////////////////////////////////////////////////////////////////////////////////

static void LOG_Exception(const char *szFunction) {
	if (nLogEnabled) {
		char szMessage[MAX_LOG];
		sprintf(szMessage, "EXCEPTION FOUND IN '%s'", szFunction);
		LOG_Print(szMessage);
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>	Logs a channel read/write operation. </summary>
///
/// <remarks>	 </remarks>
///
/// <param name="Channel">			The channel. </param>
/// <param name="nFunction">		The function. </param>
/// <param name="MessageBuffer">	[in] If non-null, buffer for message data. </param>
////////////////////////////////////////////////////////////////////////////////////////////////////

static void LOG_Channel(TPCANHandle Channel, int nFunction,
		TPCANMsg* MessageBuffer) {
	if (nLogEnabled && (nLogFunction & nFunction) && MessageBuffer) {
		char szMessage[MAX_LOG];
		sprintf(
				szMessage,
				"CHANNEL    0x%02X (%s) ID=0x%X Len=%u, Data=0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
				(unsigned int) Channel, nFunction == LOG_FUNCTION_WRITE ? "OUT"
						: "IN", (unsigned int) MessageBuffer->ID,
				(unsigned int) MessageBuffer->LEN,
				(unsigned int) MessageBuffer->DATA[0],
				(unsigned int) MessageBuffer->DATA[1],
				(unsigned int) MessageBuffer->DATA[2],
				(unsigned int) MessageBuffer->DATA[3],
				(unsigned int) MessageBuffer->DATA[4],
				(unsigned int) MessageBuffer->DATA[5],
				(unsigned int) MessageBuffer->DATA[6],
				(unsigned int) MessageBuffer->DATA[7]);
		LOG_Print(szMessage);
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// CAN helper

static int CAN_IsExisting(TPCANHandle Channel, TPCANType HwType, DWORD IOPort,
		WORD Interrupt);

static int CAN_Get_Device_Minor(TPCANHandle Channel, TPCANType HwType,
		DWORD IOPort, WORD Interrupt);

static int CAN_Lookup_Device_Minor(WORD wHardwareType, DWORD dwReqPort,
		WORD wReqIrq);

static int CAN_Parse_Proc(char *buffer, int *nType, unsigned long *dwPort,
		unsigned short *wIrq, int *nMajor, int *nMinor);

static char *CAN_Get_Device_Name(TPCANHandle Channel, TPCANType HwType,
		DWORD IOPort, WORD Interrupt);

////////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>	Checks if device is existing. </summary>
///
/// <remarks>	 </remarks>
///
/// <param name="Channel">		The channel. </param>
/// <param name="HwType">		Type of the hardware. </param>
/// <param name="IOPort">		The i/o port. </param>
/// <param name="Interrupt">	The interrupt. </param>
///
/// <returns>	>= 0 if existing </returns>
////////////////////////////////////////////////////////////////////////////////////////////////////
static int CAN_IsExisting(TPCANHandle Channel, TPCANType HwType, DWORD IOPort,
		WORD Interrupt) {

	// Call lookupDevice to find out if device exists
	if (Channel >= PCAN_ISABUS1 && Channel <= PCAN_ISABUS8)
		return CAN_Lookup_Device_Minor(HwType != 0 ? HwType : HW_ISA_SJA,
				IOPort, Interrupt);
	if (Channel >= PCAN_DNGBUS1 && Channel <= PCAN_DNGBUS1)
		return CAN_Lookup_Device_Minor(HwType != 0 ? HwType : HW_DONGLE_SJA,
				IOPort, Interrupt);
	if (Channel >= PCAN_PCIBUS1 && Channel <= PCAN_PCIBUS8)
		return CAN_Lookup_Device_Minor(HwType != 0 ? HwType : HW_PCI, Channel
				- PCAN_PCIBUS1, Interrupt);
	if (Channel >= PCAN_USBBUS1 && Channel <= PCAN_USBBUS8)
		return CAN_Lookup_Device_Minor(HwType != 0 ? HwType : HW_USB, Channel
				- PCAN_USBBUS1, Interrupt);
	if (Channel >= PCAN_PCCBUS1 && Channel <= PCAN_PCCBUS2)
		return CAN_Lookup_Device_Minor(HwType != 0 ? HwType : HW_PCI, Channel
				- PCAN_PCCBUS1, Interrupt);
	return -1;

}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>	Returns the device minor. </summary>
///
/// <remarks>	 </remarks>
///
/// <param name="Channel">		The channel. </param>
/// <param name="HwType">		Type of the hardware. </param>
/// <param name="IOPort">		The i/o port. </param>
/// <param name="Interrupt">	The interrupt. </param>
///
/// <returns>	The minor value </returns>
////////////////////////////////////////////////////////////////////////////////////////////////////

static int CAN_Get_Device_Minor(TPCANHandle Channel, TPCANType HwType,
		DWORD IOPort, WORD Interrupt) {

	// for non-pnp call lookupDevice
	if (Channel >= PCAN_ISABUS1 && Channel <= PCAN_ISABUS8)
		return CAN_Lookup_Device_Minor(HwType != 0 ? HwType : HW_ISA_SJA,
				IOPort, Interrupt);
	if (Channel >= PCAN_DNGBUS1 && Channel <= PCAN_DNGBUS1)
		return CAN_Lookup_Device_Minor(HwType != 0 ? HwType : HW_DONGLE_SJA,
				IOPort, Interrupt);
	// for pnp, map to minor number
	// warning: a 1:1 mapping is not accurate, if module 'usbhid' is activated
	// then MINOR_FIRST_USB is 33 and not 32.
	if (Channel >= PCAN_PCIBUS1 && Channel <= PCAN_PCIBUS8)
		return Channel - PCAN_PCIBUS1 + MINOR_FIRST_PCI;
	if (Channel >= PCAN_USBBUS1 && Channel <= PCAN_USBBUS8)
		return Channel - PCAN_USBBUS1 + MINOR_FIRST_USB;
	if (Channel >= PCAN_PCCBUS1 && Channel <= PCAN_PCCBUS2)
		return Channel - PCAN_PCCBUS1 + MINOR_FIRST_PCC;
	return -1;

}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>	Lookups the device minor from the proc entries. </summary>
///
/// <remarks>	 </remarks>
///
/// <param name="wHardwareType">	Type of the w hardware. </param>
/// <param name="dwReqPort">		The dw request port. </param>
/// <param name="wReqIrq">			The w request irq. </param>
///
/// <returns>	The minor value. </returns>
////////////////////////////////////////////////////////////////////////////////////////////////////

static int CAN_Lookup_Device_Minor(WORD wHardwareType, DWORD dwReqPort,
		WORD wReqIrq) {

	FILE *f = NULL;
	char *m = NULL;
	char *p = NULL;
	char *ptr;
	int found = 0;
	int nMinor = 0;
	int nMajor = 0;
	int hwMatch = 0;
	unsigned long dwPort;
	unsigned short wIrq;
	int nType;
	errno = ENODEV;

	// open proc
	if ((f = fopen(PROCFILE, "r")) == NULL)
		goto fail;
	if ((m = (char*) malloc(MAX_LINE_LEN)) == NULL)
		goto fail;

	// read an interpret proc entry contents
	do {
		ptr = m;
		p = fgets(ptr, MAX_LINE_LEN, f);
		if (p) {

			// parse proc line
			if (!CAN_Parse_Proc(p, &nType, &dwPort, &wIrq, &nMajor, &nMinor)) {
				/* wHardwareType was though as group, but with USB-PRO and FD release */
				/* this is not the case anymore, we need to search HW matching */
				hwMatch = (wHardwareType == nType);
				if (hwMatch == 0 && nType != -1) {
					switch (wHardwareType) {
					case HW_USB:
						hwMatch = (nType == HW_USB ||
								nType == HW_USB_PRO ||
								nType == HW_USB_FD ||
								nType == HW_USB_PRO_FD);
						break;
					case HW_PCI:

						hwMatch = (nType == HW_PCI);
#ifdef HW_PCI_FD
						hwMatch |= (nType == HW_PCI_FD);
#endif

						break;
					}
				}

				if (hwMatch) {
					switch (wHardwareType) {

					// check port and irq
					case HW_DONGLE_SJA:
					case HW_DONGLE_SJA_EPP:
					case HW_ISA_SJA:
						if (((dwReqPort == dwPort) && (wReqIrq == wIrq))
								|| ((dwReqPort == 0) && (wReqIrq == 0))) // use default
						{
							found = 1;
							break;
						}
						break;

						// check if pnp devices map to minor
					case HW_PCI:
#ifdef HW_PCI_FD
					case HW_PCI_FD:
#endif
						if (((dwReqPort - MINOR_FIRST_PCI)
								== (unsigned int) nMinor) || (dwReqPort == 0)) // use 1st port as default
						{
							found = 1;
							break;
						}
						break;
					case HW_USB:
					case HW_USB_PRO:
					case HW_USB_FD:
					case HW_USB_PRO_FD:
						if (((dwReqPort + MINOR_FIRST_USB)
								== (unsigned int) nMinor) || (dwReqPort == 0)) // use 1st port as default
						{
							found = 1;
							break;
						}
						break;
					case HW_PCCARD:
						if (((dwReqPort + MINOR_FIRST_PCC)
								== (unsigned int) nMinor) || (dwReqPort == 0)) // use 1st port as default
						{
							found = 1;
							break;
						}
						break;
					}
				}
			}
		}
	}
	while ((p) && (!found));

	// -1 if not found
	if (!found)
		nMinor = -1;

fail:
	if (m)
		free(m);
	if (f)
		fclose(f);

	return nMinor;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>	Parse proc entries. </summary>
///
/// <remarks>	 </remarks>
///
/// <param name="buffer">	[in]  If non-null, the buffer. </param>
/// <param name="nType">	[out] If non-null, the type. </param>
/// <param name="dwPort">	[out] If non-null, the port. </param>
/// <param name="wIrq">		[out] If non-null, the irq. </param>
/// <param name="nMajor">	[out] If non-null, the major. </param>
/// <param name="nMinor">	[out] If non-null, the minor. </param>
///
/// <returns>	. </returns>
////////////////////////////////////////////////////////////////////////////////////////////////////

static int CAN_Parse_Proc(char *buffer, int *nType, unsigned long *dwPort,
		unsigned short *wIrq, int *nMajor, int *nMinor) {
	static int m_nMajor = 0;
	char *ptr = buffer;
	char *t;


	if (*ptr == '\n')
		return -1;

	if (*ptr == '*') {
		// search for nMajor
		if ((ptr = strstr(ptr, "major"))) {
			t = strtok(ptr, " ");
			t = strtok(NULL, " ");
			m_nMajor = strtoul(t, NULL, 10);
		}
	} else {
		// skip leading blank
		if (*ptr == ' ')
			ptr++;

		// get minor
		t = strtok(ptr, " ");
		*nMinor = strtoul(ptr, NULL, 10);

		// get type string
		t = strtok(NULL, " ");
		if (!strcmp(t, "pci"))
			*nType = HW_PCI;
#ifdef HW_PCI_FD
		else if (!strcmp(t, "pcifd"))
			*nType = HW_PCI_FD;
#endif
		else if (!strcmp(t, "epp"))
			*nType = HW_DONGLE_SJA_EPP;
		else if (!strcmp(t, "isa"))
			*nType = HW_ISA_SJA;
		else if (!strcmp(t, "sp"))
			*nType = HW_DONGLE_SJA;
		else if (!strcmp(t, "usb"))
			*nType = HW_USB;
		else if (!strcmp(t, "usbpro"))
			*nType = HW_USB_PRO;
		else if (!strcmp(t, "usbfd"))
			*nType = HW_USB_FD;
		else if (!strcmp(t, "usbpfd"))
			*nType = HW_USB_PRO_FD;
		else
			*nType = -1;

		// jump over ndev
		while (*t++ == ' ')
			;
		t = strtok(NULL, " ");

		// get port
		t = strtok(NULL, " ");
		*dwPort = strtoul(t, NULL, 16);

		// get irq
		t = strtok(NULL, " ");
		*wIrq = (unsigned short) strtoul(t, NULL, 10);

		// set major
		*nMajor = m_nMajor;

		return 0;
	}

	return -1;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>	Returns the device name. </summary>
///
/// <remarks>	 </remarks>
///
/// <param name="Channel">		The channel. </param>
/// <param name="HwType">		Type of the hardware. </param>
/// <param name="IOPort">		The i/o port. </param>
/// <param name="Interrupt">	The interrupt. </param>
///
/// <returns>	The device name. </returns>
////////////////////////////////////////////////////////////////////////////////////////////////////

static char *CAN_Get_Device_Name(TPCANHandle Channel, TPCANType HwType,
		DWORD IOPort, WORD Interrupt) {

	static char path[LOCAL_STRING_LEN];
	path[0] = 0;

	// get minor and check
	int nMinor = CAN_Get_Device_Minor(Channel, HwType, IOPort, IOPort);
	if (nMinor > 64 || nMinor < 0)
		return path;

	// create
	sprintf(path, "%s%d", DEVICE_PATH, nMinor);

	return path;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// API

////////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>	Initializes a PCAN Channel. </summary>
///
/// <remarks>	 </remarks>
///
/// <param name="Channel">		"The handle of a PCAN Channel". </param>
/// <param name="Btr0Btr1">		"The speed for the communication (BTR0BTR1 code)". </param>
/// <param name="HwType">		"NON PLUG&PLAY: The type of hardware and operation mode". </param>
/// <param name="IOPort">		"NON PLUG&PLAY: The I/O address for the parallel port". </param>
/// <param name="Interrupt">	The interrupt. </param>
///
/// <returns>	"A TPCANStatus error code". </returns>
///
/// ### <param name="Interupt">	"NON PLUG&PLAY: Interrupt number of the parallel port". </param>
////////////////////////////////////////////////////////////////////////////////////////////////////

TPCANStatus __stdcall CAN_Initialize(TPCANHandle Channel,
		TPCANBaudrate Btr0Btr1, TPCANType HwType, DWORD IOPort, WORD Interrupt) {

	PCAN_DESCRIPTOR *desc = NULL;
	errno = 0;
	TPCANStatus Result = PCAN_ERROR_OK;

	try {

		// logging
		char szLog[MAX_LOG];
		LOG_EnteringTo("CAN_Initialize");
		sprintf(
				szLog,
				"Channel: 0x%02X, Btr0Btr1: %d, HwType: 0x%08X, IOPort: 0x%08X, Interrupt: 0x%08X",
				Channel, Btr0Btr1, IOPort, HwType, Interrupt);
		LOG_Parameters("CAN_Initialize", szLog);

		// device name and falgs
		char *szDeviceName = CAN_Get_Device_Name(Channel, HwType, IOPort,
				IOPort);
		int nFlag = O_RDWR | O_NONBLOCK;

		// allocate for descriptors
		if (ppDescriptors == NULL) {
			if ((ppDescriptors = (PCAN_DESCRIPTOR **) malloc(
					sizeof(PCAN_DESCRIPTOR *) * 0x100)) == NULL) {
				Result = PCAN_ERROR_RESOURCE;
				goto fail;
			}
			memset(ppDescriptors, 0, sizeof(PCAN_DESCRIPTOR *) * 0x100);
		}

		// check if descriptor allready existing
		desc = ppDescriptors[Channel];
		if (ppDescriptors[Channel] != NULL) {
			Result = PCAN_ERROR_INITIALIZE;
			goto fail;
		}

		// create desciptor
		if ((ppDescriptors[Channel] = (PCAN_DESCRIPTOR *) malloc(
				sizeof(PCAN_DESCRIPTOR))) == NULL) {
			Result = PCAN_ERROR_RESOURCE;
			goto fail;
		}

		// fill descriptor
		desc = ppDescriptors[Channel];
		desc->szDevicePath[0] = 0;
		desc->nFileNo = -1;
		desc->nFilter = PCAN_FILTER_OPEN;
		desc->nListenOnly = 0;
		desc->Btr0Btr1 = Btr0Btr1;
		strncpy(desc->szDevicePath, szDeviceName, LOCAL_STRING_LEN);

#ifndef NO_RT
		char DeviceName[256];
		sscanf(szDeviceName, "/dev/%s", DeviceName);
		if ((desc->nFileNo = rt_dev_open(DeviceName, nFlag)) == -1)
#else
		if ((desc->nFileNo = open(szDeviceName, nFlag)) == -1)
#endif
		{
			Result = PCAN_ERROR_NODRIVER;
			goto fail;
		}

		// initialize
		TPCANInit init;
		init.wBTR0BTR1 = desc->Btr0Btr1; // combined BTR0 and BTR1 register of the SJA100
		init.ucCANMsgType = MSGTYPE_EXTENDED; // 11 or 29 bits
		init.ucListenOnly = desc->nListenOnly; // listen only mode when != 0
		if (__ioctl(desc->nFileNo, PCAN_INIT, &init) < 0) {
			Result = PCAN_ERROR_NODRIVER;
			goto fail;
		}
		// reset filters
		nFlag = PCAN_FILTER_OPEN;
		CAN_SetValue(Channel, PCAN_MESSAGE_FILTER, &nFlag, sizeof(nFlag));
		goto leave;

		// error handling
		fail: if (desc) {
			if (desc->nFileNo > -1)
				__close(desc->nFileNo);
			free(desc);
			ppDescriptors[Channel] = NULL;
		}

		leave: LOG_LeavingFrom("CAN_Initialize", Result);
		return Result;
	} catch (...) {
		Result = PCAN_ERROR_UNKNOWN;
		LOG_Exception("CAN_Initialize");
	}
	return Result;

}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>	Uninitializes one or all PCAN Channels initialized by CAN_Initialize. </summary>
///
/// <remarks>
/// Giving the TPCANHandle value "PCAN_NONEBUS", uninitialize all initialized channels.
/// </remarks>
///
/// <param name="Channel">	"The handle of a PCAN Channel". </param>
///
/// <returns>	"A TPCANStatus error code". </returns>
////////////////////////////////////////////////////////////////////////////////////////////////////

TPCANStatus CAN_Uninitialize(TPCANHandle Channel) {

	PCAN_DESCRIPTOR *desc = NULL;
	TPCANStatus Result = PCAN_ERROR_OK;
	int index = 0;

	try {

		// logging
		char szLog[MAX_LOG];
		LOG_EnteringTo("CAN_Uninitialize");
		sprintf(szLog, "Channel: 0x%02X", Channel);
		LOG_Parameters("CAN_Uninitialize", szLog);

		// descriptor
		if (ppDescriptors == NULL) {
			Result = PCAN_ERROR_INITIALIZE;
			goto leave;
		}
		if (ppDescriptors[Channel] == NULL && Channel != PCAN_NONEBUS) {
			Result = PCAN_ERROR_INITIALIZE;
			goto leave;
		}
		desc = ppDescriptors[Channel];


		if (Channel == PCAN_NONEBUS) {
			// uninit all
			for (index = 1; index < 0x100; index++)
				CAN_Uninitialize((TPCANHandle) index);
		} else {
			// close file
			if (desc->nFileNo > -1) {
				__close(desc->nFileNo);
				desc->nFileNo = -1;
			}
			// remove descriptor
			free(desc);
			ppDescriptors[Channel] = NULL;
		}

		leave: LOG_LeavingFrom("CAN_Uninitialize", Result);
		return Result;
	} catch (...) {
		Result = PCAN_ERROR_UNKNOWN;
		LOG_Exception("CAN_Uninitialize");
	}
	return Result;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>	Resets the receive and transmit queues of the PCAN Channel. </summary>
///
/// <remarks>	A reset of the CAN controller is not performed. </remarks>
///
/// <param name="Channel">	"The handle of a PCAN Channel". </param>
///
/// <returns>	"A TPCANStatus error code". </returns>
////////////////////////////////////////////////////////////////////////////////////////////////////

TPCANStatus CAN_Reset(TPCANHandle Channel) {

	PCAN_DESCRIPTOR *desc;
	TPCANStatus Result = PCAN_ERROR_OK;

	try {

		// logging
		char szLog[MAX_LOG];
		LOG_EnteringTo("CAN_Reset");
		sprintf(szLog, "Channel: 0x%02X", Channel);
		LOG_Parameters("CAN_Reset", szLog);

		// descriptor
		if (ppDescriptors == NULL) {
			Result = PCAN_ERROR_INITIALIZE;
			goto leave;
		}
		if (ppDescriptors[Channel] == NULL) {
			Result = PCAN_ERROR_INITIALIZE;
			goto leave;
		}
		desc = ppDescriptors[Channel];

		// call init
		TPCANInit init;
		init.wBTR0BTR1 = desc->Btr0Btr1; // combined BTR0 and BTR1 register of the SJA100
		init.ucCANMsgType = MSGTYPE_EXTENDED; // 11 or 29 bits
		init.ucListenOnly = desc->nListenOnly; // listen only mode when != 0
		if (__ioctl(desc->nFileNo, PCAN_INIT, &init) < 0) {
			Result = PCAN_ERROR_UNKNOWN;
			goto leave;
		}

		leave: LOG_LeavingFrom("CAN_Reset", Result);
		return Result;
	} catch (...) {
		Result = PCAN_ERROR_UNKNOWN;
		LOG_Exception("CAN_Reset");
	}
	return Result;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>	Gets the current status of a PCAN Channel. </summary>
///
/// <remarks>	 </remarks>
///
/// <param name="Channel">	"The handle of a PCAN Channel". </param>
///
/// <returns>	"A TPCANStatus error code". </returns>
////////////////////////////////////////////////////////////////////////////////////////////////////

TPCANStatus CAN_GetStatus(TPCANHandle Channel) {

	PCAN_DESCRIPTOR *desc;
	TPCANStatus Result = PCAN_ERROR_OK;

	try {

		// logging
		char szLog[MAX_LOG];
		LOG_EnteringTo("CAN_GetStatus");
		sprintf(szLog, "Channel: 0x%02X", Channel);
		LOG_Parameters("CAN_GetStatus", szLog);

		// descriptor
		if (ppDescriptors == NULL) {
			Result = PCAN_ERROR_INITIALIZE;
			goto leave;
		}
		if (ppDescriptors[Channel] == NULL) {
			Result = PCAN_ERROR_INITIALIZE;
			goto leave;
		}
		desc = ppDescriptors[Channel];

		// get status
		errno = EBADF;
		TPSTATUS status;
		if (__ioctl(desc->nFileNo, PCAN_GET_STATUS, &status) < 0) {
			{
				Result = PCAN_ERROR_INITIALIZE;
				goto leave;
			}
		}

		// convert result
		if (status.wErrorFlag & CAN_ERR_BUSOFF)
			Result = PCAN_ERROR_BUSOFF;
		else if (status.wErrorFlag & CAN_ERR_BUSHEAVY)
			Result = PCAN_ERROR_BUSHEAVY;
		else if (status.wErrorFlag & CAN_ERR_BUSLIGHT)
			Result = PCAN_ERROR_BUSLIGHT;
		else if (status.wErrorFlag & CAN_ERR_OVERRUN)
			Result = CAN_ERR_OVERRUN;
		else
			Result = PCAN_ERROR_OK;

		leave: LOG_LeavingFrom("CAN_GetStatus", Result);
		return Result;
	} catch (...) {
		Result = PCAN_ERROR_UNKNOWN;
		LOG_Exception("CAN_GetStatus");
	}
	return Result;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>	Reads a CAN message from the receive queue of a PCAN Channel. </summary>
///
/// <remarks>	 </remarks>
///
/// <param name="Channel">			"The handle of a PCAN Channel". </param>
/// <param name="MessageBuffer">	[in,out] "A TPCANMsg structure buffer to store the CAN
/// 								message". </param>
/// <param name="TimestampBuffer">	[in,out] "A TPCANTimestamp structure buffer to get the
/// 								reception time of the message. If this value is not desired,
/// 								this parameter should be passed as NULL". </param>
///
/// <returns>	"A TPCANStatus error code". </returns>
////////////////////////////////////////////////////////////////////////////////////////////////////

TPCANStatus CAN_Read(TPCANHandle Channel, TPCANMsg* MessageBuffer,
		TPCANTimestamp* TimestampBuffer) {

	PCAN_DESCRIPTOR *desc;
	TPCANStatus Result = PCAN_ERROR_OK;
	_TPCANMsg *m = NULL;
	TPCANRdMsg rdm;

	try {

		// logging
		char szLog[MAX_LOG];
		LOG_EnteringTo("CAN_Read");
		sprintf(
				szLog,
				"Channel: 0x%02X, MessageBuffer: 0x%p, TimestampBuffer: 0x%p",
				Channel, MessageBuffer, TimestampBuffer);
		LOG_Parameters("CAN_Read", szLog);

		// descriptor
		if (ppDescriptors == NULL) {
			Result = PCAN_ERROR_INITIALIZE;
			goto leave;
		}
		if (ppDescriptors[Channel] == NULL) {
			Result = PCAN_ERROR_INITIALIZE;
			goto leave;
		}
		desc = ppDescriptors[Channel];

		// read message
		errno = EBADF;
		if (__ioctl(desc->nFileNo, PCAN_READ_MSG, &rdm) < 0) {
			Result = PCAN_ERROR_QRCVEMPTY;
			goto leave;
		}

		// copy and convert values
		m = &rdm.Msg;
		MessageBuffer->ID = m->ID;
		MessageBuffer->LEN = m->LEN;
		if( (m->MSGTYPE & MSGTYPE_EXTENDED) == MSGTYPE_EXTENDED)
			MessageBuffer->MSGTYPE = PCAN_MESSAGE_EXTENDED;
		else
			MessageBuffer->MSGTYPE = PCAN_MESSAGE_STANDARD;

		if( (m->MSGTYPE & MSGTYPE_RTR) == MSGTYPE_RTR)
			MessageBuffer->MSGTYPE |= MSGTYPE_RTR;
		if( (m->MSGTYPE & MSGTYPE_STATUS) == MSGTYPE_STATUS)
			MessageBuffer->MSGTYPE |= MSGTYPE_STATUS;

		memcpy(MessageBuffer->DATA, m->DATA, 8);


		// timestamp
		if (TimestampBuffer != NULL){
			TimestampBuffer->millis = rdm.dwTime;
			TimestampBuffer->millis_overflow = 0;
			TimestampBuffer->micros = rdm.wUsec;
		}

		// log
		LOG_Channel(Channel, LOG_FUNCTION_READ, MessageBuffer);

		leave: LOG_LeavingFrom("CAN_Read", Result);
		return Result;
	} catch (...) {
		Result = PCAN_ERROR_UNKNOWN;
		LOG_Exception("CAN_Read");
	}
	return Result;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>	Transmits a CAN message. </summary>
///
/// <remarks>	 </remarks>
///
/// <param name="Channel">			"The handle of a PCAN Channel". </param>
/// <param name="MessageBuffer">	[in,out] "A TPCANMsg buffer with the message to be sent". </param>
///
/// <returns>	"A TPCANStatus error code". </returns>
////////////////////////////////////////////////////////////////////////////////////////////////////

TPCANStatus CAN_Write(TPCANHandle Channel, TPCANMsg* MessageBuffer) {

	PCAN_DESCRIPTOR *desc;
	TPCANStatus Result = PCAN_ERROR_OK;

	try {

		// logging
		char szLog[MAX_LOG];
		LOG_EnteringTo("CAN_Write");
		sprintf(szLog, "Channel: 0x%02X, MessageBuffer: 0x%p", Channel,
				MessageBuffer);
		LOG_Parameters("CAN_Write", szLog);

		// descriptor
		if (ppDescriptors == NULL) {
			Result = PCAN_ERROR_INITIALIZE;
			goto leave;
		}
		if (ppDescriptors[Channel] == NULL) {
			Result = PCAN_ERROR_INITIALIZE;
			goto leave;
		}
		desc = ppDescriptors[Channel];

		// copy and convert values
		errno = EBADF;
		_TPCANMsg wrm;
		wrm.ID = MessageBuffer->ID;
		wrm.LEN = MessageBuffer->LEN;
		if( (MessageBuffer->MSGTYPE & MSGTYPE_EXTENDED) == MSGTYPE_EXTENDED)
			wrm.MSGTYPE = PCAN_MESSAGE_EXTENDED;
		else
			wrm.MSGTYPE = PCAN_MESSAGE_STANDARD;

		if( (MessageBuffer->MSGTYPE & MSGTYPE_RTR) == MSGTYPE_RTR)
			wrm.MSGTYPE |= MSGTYPE_RTR;
		memcpy(wrm.DATA, MessageBuffer->DATA, 8);

		// write message
		if (__ioctl(desc->nFileNo, PCAN_WRITE_MSG, &wrm) < 0) {
			Result = PCAN_ERROR_XMTFULL;
			goto leave;
		}

		// log
		LOG_Channel(Channel, LOG_FUNCTION_WRITE, MessageBuffer);

		leave: LOG_LeavingFrom("CAN_Write", Result);
		return Result;
	} catch (...) {
		Result = PCAN_ERROR_UNKNOWN;
		LOG_Exception("CAN_Write");
	}
	return Result;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>	Configures the reception filter. </summary>
///
/// <remarks>
/// The message filter will be expanded with every call to this function. If it is desired to
/// reset the filter, please use the CAN_SetParameter function.
/// </remarks>
///
/// <param name="Channel">	"The handle of a PCAN Channel". </param>
/// <param name="FromID">	"The lowest CAN ID to be received". </param>
/// <param name="ToID">		"The highest CAN ID to be received". </param>
/// <param name="Mode">		"Message type, Standard (11-bit identifier) or Extended (29-bit
/// 						identifier)". </param>
///
/// <returns>	"A TPCANStatus error code". </returns>
////////////////////////////////////////////////////////////////////////////////////////////////////

TPCANStatus CAN_FilterMessages(TPCANHandle Channel, DWORD FromID, DWORD ToID,
		TPCANMode Mode) {

	PCAN_DESCRIPTOR *desc;
	TPCANStatus Result = PCAN_ERROR_OK;
	int ibuf;

	try {

		// logging
		char szLog[MAX_LOG];
		LOG_EnteringTo("CAN_FilterMessages");
		sprintf(szLog,
				"Channel: 0x%02X, FromID: 0x%08X, ToID: 0x%08X, Mode: 0x%08X",
				Channel, FromID, ToID, Mode);
		LOG_Parameters("CAN_FilterMessages", szLog);

		// descriptor
		if (ppDescriptors == NULL) {
			Result = PCAN_ERROR_INITIALIZE;
			goto leave;
		}
		if (ppDescriptors[Channel] == NULL) {
			Result = PCAN_ERROR_INITIALIZE;
			goto leave;
		}
		desc = ppDescriptors[Channel];

		// convert parameters
		TPMSGFILTER Filter;
		Filter.FromID = FromID;
		Filter.ToID = ToID;
		switch (Mode) {
		case PCAN_MESSAGE_STANDARD:
			Filter.MSGTYPE = MSGTYPE_STANDARD;
			break;
		case PCAN_MESSAGE_EXTENDED:
			Filter.MSGTYPE = MSGTYPE_EXTENDED;
			break;
		default:
			Filter.MSGTYPE = MSGTYPE_EXTENDED;
		}

		// if channel is fully opened, close channel before adding a filter
		ibuf = 0;
		Result = CAN_GetValue(Channel, PCAN_MESSAGE_FILTER, &ibuf, sizeof(ibuf));
		if (Result == PCAN_ERROR_OK && ibuf == PCAN_FILTER_OPEN) {
			ibuf = PCAN_FILTER_CLOSE;
			Result = CAN_SetValue(Channel, PCAN_MESSAGE_FILTER, &ibuf, sizeof(ibuf));
		}

		// set filter
		if (__ioctl(desc->nFileNo, PCAN_MSG_FILTER, &Filter) < 0) {
			Result = PCAN_ERROR_UNKNOWN;
			goto leave;
		}
		desc->nFilter = PCAN_FILTER_CUSTOM;

		leave: LOG_LeavingFrom("CAN_FilterMessages", Result);
		return Result;
	} catch (...) {
		Result = PCAN_ERROR_UNKNOWN;
		LOG_Exception("CAN_FilterMessages");
	}
	return Result;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>	Retrieves a PCAN Channel value. </summary>
///
/// <remarks>
/// Parameters can be present or not according with the kind of Hardware (PCAN Channel) being
/// used. If a parameter is not available, a PCAN_ERROR_ILLPARAMTYPE error will be returned.
/// </remarks>
///
/// <param name="Channel">		"The handle of a PCAN Channel". </param>
/// <param name="Parameter">	"The TPCANParameter parameter to get". </param>
/// <param name="Buffer">		[in,out] "Buffer for the parameter value". </param>
/// <param name="BufferLength">	"Size in bytes of the buffer". </param>
///
/// <returns>	"A TPCANStatus error code". </returns>
////////////////////////////////////////////////////////////////////////////////////////////////////

TPCANStatus CAN_GetValue(TPCANHandle Channel, TPCANParameter Parameter,
		void* Buffer, DWORD BufferLength) {

	PCAN_DESCRIPTOR *desc;
	TPCANStatus Result = PCAN_ERROR_OK;

	try {

		// logging
		char szLog[MAX_LOG];
		LOG_EnteringTo("CAN_GetValue");
		sprintf(
				szLog,
				"Channel: 0x%02X, Parameter: 0x%08X, Buffer: 0x%p, BufferLength: 0x%08X",
				Channel, Parameter, Buffer, BufferLength);
		LOG_Parameters("CAN_GetValue", szLog);

		// parameters
		if (Buffer == NULL) {
			Result = PCAN_ERROR_ILLPARAMVAL;
			goto leave;
		}

		DWORD l;

		// parameters that don't need a descriptor
		switch (Parameter) {
		case PCAN_API_VERSION:
			l = strlen(API_VERSION);
			if (BufferLength <= l) {
				Result = PCAN_ERROR_ILLPARAMVAL;
				goto leave;
			}
			strncpy((char*) Buffer, API_VERSION, BufferLength);
			goto leave;
		case PCAN_CHANNEL_CONDITION:
			if (BufferLength < sizeof(int)) {
				Result = PCAN_ERROR_ILLPARAMVAL;
				goto leave;
			}
			// check if existing
			if (CAN_IsExisting(Channel, 0, 0, 0) >= 0) {
				// cehck if descriptor available
				//if (ppDescriptors == NULL || ppDescriptors[Channel] == NULL)
					*((int*) Buffer) = PCAN_CHANNEL_AVAILABLE;
				//else
				//	*((int*) Buffer) = PCAN_CHANNEL_OCCUPIED;
			} else
				*((int*) Buffer) = PCAN_CHANNEL_ILLEGAL;
			goto leave;

		case PCAN_LOG_LOCATION:
			// check if path has been set
			if (nLogPathStatus != -1) {
				l = strlen(szLogLocationPath);
				if (BufferLength <= l) {
					Result = PCAN_ERROR_ILLPARAMVAL;
					goto leave;
				}
				strncpy((char*) Buffer, szLogLocationPath, BufferLength);
			} else
				*(char*) Buffer = '\0';
			goto leave;

		case PCAN_LOG_STATUS:
			if (BufferLength < sizeof(int)) {
				Result = PCAN_ERROR_ILLPARAMVAL;
				goto leave;
			}
			*((int*) Buffer) = nLogEnabled;
			goto leave;

		case PCAN_LOG_CONFIGURE:
			if (BufferLength < sizeof(int)) {
				Result = PCAN_ERROR_ILLPARAMVAL;
				goto leave;
			}
			*((int*) Buffer) = nLogFunction;
			goto leave;
		}
		// descriptor
		if (ppDescriptors == NULL) {
			Result = PCAN_ERROR_INITIALIZE;
			goto leave;
		}
		if (ppDescriptors[Channel] == NULL) {
			Result = PCAN_ERROR_INITIALIZE;
			goto leave;
		}
		desc = ppDescriptors[Channel];

		// now parameters that need a descriptor
		switch (Parameter) {
		case PCAN_MESSAGE_FILTER:
			if (BufferLength < sizeof(int)) {
				Result = PCAN_ERROR_ILLPARAMVAL;
				goto leave;
			}
			*((int*) Buffer) = desc->nFilter;
			break;
		case PCAN_DEVICE_NUMBER:
			if (BufferLength < sizeof(int)) {
				Result = PCAN_ERROR_ILLPARAMVAL;
				goto leave;
			}
			TPEXTRAPARAMS Params;
			Params.nSubFunction = SF_GET_HCDEVICENO;
			if (__ioctl(desc->nFileNo, PCAN_EXTRA_PARAMS, &Params) < 0) {
				Result = PCAN_ERROR_UNKNOWN;
				goto leave;
			}
			*((int*) Buffer) = Params.func.ucHCDeviceNo;
			break;

		case PCAN_CHANNEL_VERSION:
			// reaad diag
			TPDIAG Diag;
			if (__ioctl(desc->nFileNo, PCAN_DIAG, &Diag) < 0) {
				Result = PCAN_ERROR_UNKNOWN;
				goto leave;
			}
			if (Diag.szVersionString != NULL) {
				l = strlen(Diag.szVersionString);
				if (BufferLength <= l) {
					Result = PCAN_ERROR_ILLPARAMVAL;
					goto leave;
				}

				strncpy((char*) Buffer, Diag.szVersionString,
					BufferLength);
			} else {
				if (BufferLength < 1) {
					Result = PCAN_ERROR_ILLPARAMVAL;
					goto leave;
				}

				*((char*) Buffer) = '\0';
			}
			break;

		case PCAN_LISTEN_ONLY:
			if (BufferLength < sizeof(int)) {
				Result = PCAN_ERROR_ILLPARAMVAL;
				goto leave;
			}
			*((int*) Buffer) = desc->nListenOnly;
			break;

		case PCAN_RECEIVE_EVENT:
			if (BufferLength < sizeof(int)) {
				Result = PCAN_ERROR_ILLPARAMVAL;
				goto leave;
			}
			*((int*) Buffer) = desc->nFileNo;
			break;

		default: {
			Result = PCAN_ERROR_ILLPARAMTYPE;
			goto leave;
		}
		}
		leave: LOG_LeavingFrom("CAN_GetValue", Result);
		return Result;
	} catch (...) {
		Result = PCAN_ERROR_UNKNOWN;
		LOG_Exception("CAN_GetValue");
	}
	return Result;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>	Configures or sets a PCAN Channel value. </summary>
///
/// <remarks>
/// Parameters can be present or not according with the kind of Hardware (PCAN Channel) being
/// used. If a parameter is not available, a PCAN_ERROR_ILLPARAMTYPE error will be returned.
/// </remarks>
///
/// <param name="Channel">		"The handle of a PCAN Channel". </param>
/// <param name="Parameter">	"The TPCANParameter parameter to set". </param>
/// <param name="Buffer">		[in,out] "Buffer with the value to be set". </param>
/// <param name="BufferLength">	"Size in bytes of the buffer". </param>
///
/// <returns>	"A TPCANStatus error code". </returns>
////////////////////////////////////////////////////////////////////////////////////////////////////

TPCANStatus CAN_SetValue(TPCANHandle Channel, TPCANParameter Parameter,
		void* Buffer, DWORD BufferLength) {

	PCAN_DESCRIPTOR *desc;
	TPCANStatus Result = PCAN_ERROR_OK;

	try {

		// logging
		char szLog[MAX_LOG];
		LOG_EnteringTo("CAN_SetValue");
		sprintf(
				szLog,
				"Channel: 0x%02X, Parameter: 0x%08X, Buffer: 0x%p, BufferLength: 0x%08X",
				Channel, Parameter, Buffer, BufferLength);
		LOG_Parameters("CAN_SetValue", szLog);

		// parameters
		if (Buffer == NULL) {
			Result = PCAN_ERROR_ILLPARAMVAL;
			goto leave;
		}

		// parameters that don't need a descriptor
		switch (Parameter) {
		case PCAN_LOG_LOCATION:
			if (BufferLength < 1) {
				Result = PCAN_ERROR_ILLPARAMVAL;
				goto leave;
			}

			strncpy(szLogLocationPath, (char*) Buffer,
				LOCAL_STRING_LEN-1);
			if (strlen(szLogLocationPath) > 0)
				nLogPathStatus = 1;
			else
				nLogPathStatus = -1;
			goto leave;

		case PCAN_LOG_STATUS:
			if (BufferLength < sizeof(int)) {
				Result = PCAN_ERROR_ILLPARAMVAL;
				goto leave;
			}
			nLogEnabled = *((int*) Buffer);
			goto leave;

		case PCAN_LOG_CONFIGURE:
			if (BufferLength < sizeof(int)) {
				Result = PCAN_ERROR_ILLPARAMVAL;
				goto leave;
			}
			nLogFunction = *((int*) Buffer);
			goto leave;
		}

		// descriptor
		if (ppDescriptors == NULL) {
			Result = PCAN_ERROR_INITIALIZE;
			goto leave;
		}
		if (ppDescriptors[Channel] == NULL) {
			Result = PCAN_ERROR_INITIALIZE;
			goto leave;
		}
		desc = ppDescriptors[Channel];

		// now parameters that need a descritpor
		switch (Parameter) {
		case PCAN_MESSAGE_FILTER:
			if (BufferLength < sizeof(int)) {
				Result = PCAN_ERROR_ILLPARAMVAL;
				goto leave;
			}
			if (*((int*) Buffer) != PCAN_FILTER_OPEN && *((int*) Buffer)
					!= PCAN_FILTER_CLOSE) {
				Result = PCAN_ERROR_ILLPARAMVAL;
				goto leave;
			}
			desc->nFilter = *((int*) Buffer);
			if (desc->nFilter == PCAN_FILTER_OPEN) {
				TPMSGFILTER Filter;
				Filter.FromID = 0;
				Filter.ToID = CAN_MAX_EXTENDED_ID;
				Filter.MSGTYPE = MSGTYPE_EXTENDED;
				if (__ioctl(desc->nFileNo, PCAN_MSG_FILTER, &Filter) < 0) {
					Result = PCAN_ERROR_UNKNOWN;
					goto leave;
				}
			}
			if (desc->nFilter == PCAN_FILTER_CLOSE) {
#if 0
				TPMSGFILTER Filter;
				Filter.FromID = -1;
				Filter.ToID = -1;
				Filter.MSGTYPE = MSGTYPE_STANDARD;
#endif
				if (__ioctl(desc->nFileNo, PCAN_MSG_FILTER, NULL) < 0) {
					Result = PCAN_ERROR_UNKNOWN;
					goto leave;
				}
			}
			break;

#ifdef SF_SET_SERIALNUMBER
		case PCAN_DEVICE_NUMBER:
			if (BufferLength < sizeof(int)) {
				Result = PCAN_ERROR_ILLPARAMVAL;
				goto leave;
			}
			TPEXTRAPARAMS Params;
			Params.nSubFunction = SF_SET_HCDEVICENO;
			Params.func.ucHCDeviceNo = *((int*) Buffer);
			if (__ioctl(desc->nFileNo, PCAN_EXTRA_PARAMS, &Params) < 0) {
				Result = PCAN_ERROR_UNKNOWN;
				goto leave;
			}
			break;
#endif
		case PCAN_LISTEN_ONLY:
			if (BufferLength < sizeof(int)) {
				Result = PCAN_ERROR_ILLPARAMVAL;
				goto leave;
			}
			desc->nListenOnly = *((int*) Buffer);
			TPCANInit init;
			init.wBTR0BTR1 = desc->Btr0Btr1; // combined BTR0 and BTR1 register of the SJA100
			init.ucCANMsgType = MSGTYPE_EXTENDED; // 11 or 29 bits
			init.ucListenOnly = desc->nListenOnly; // listen only mode when != 0
			if (__ioctl(desc->nFileNo, PCAN_INIT, &init) < 0) {
				Result = PCAN_ERROR_UNKNOWN;
				goto leave;
			}
			break;
		default: {
			Result = PCAN_ERROR_ILLPARAMTYPE;
			goto leave;
		}
		}

		leave: LOG_LeavingFrom("CAN_GetErrorText", Result);
		return Result;
	} catch (...) {
		Result = PCAN_ERROR_UNKNOWN;
		LOG_Exception("CAN_GetErrorText");
	}
	return Result;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>
/// Returns a descriptive text of a given TPCANStatus error code, in any desired language.
/// </summary>
///
/// <remarks>
/// The current languages available for translation are: Neutral (0x00), German (0x07), English
/// (0x09), Spanish (0x0A), Italian (0x10) and French (0x0C)
/// </remarks>
///
/// <param name="Error">	"A TPCANStatus error code". </param>
/// <param name="Language">	"Indicates a 'Primary language ID'". </param>
/// <param name="Buffer">	"Buffer for a null terminated char array". </param>
///
/// <returns>	"A TPCANStatus error code". </returns>
////////////////////////////////////////////////////////////////////////////////////////////////////

TPCANStatus CAN_GetErrorText(TPCANStatus Error, WORD Language, LPSTR Buffer) {
	TPCANStatus Result = PCAN_ERROR_OK;

	try {

		// logging
		char szLog[MAX_LOG];
		LOG_EnteringTo("CAN_GetErrorText");
		sprintf(szLog, "Error: 0x%08X, Language: 0x%08X, Buffer: 0x%p",
				Error, Language, Buffer);
		LOG_Parameters("CAN_GetErrorText", szLog);

		switch (Language) {
		case 0x00:
			Language = IDS_STR_IND_LANG_EN;
			break;
		case 0x07:
			Language = IDS_STR_IND_LANG_DE;
			break;
		case 0x09:
			Language = IDS_STR_IND_LANG_EN;
			break;
		case 0x0A:
			Language = IDS_STR_IND_LANG_ES;
			break;
		case 0x10:
			Language = IDS_STR_IND_LANG_IT;
			break;
		case 0x0C:
			Language = IDS_STR_IND_LANG_FR;
			break;
		default:
			Language = IDS_STR_IND_LANG_EN;
			break;
		}
		switch (Error) {
		case PCAN_ERROR_OK:
			strcpy(Buffer, resource[Language][IDS_STR_IND_ERR_OK]);
			break;
		case PCAN_ERROR_XMTFULL:
			strcpy(Buffer, resource[Language][IDS_STR_IND_ERR_XMTFULL]);
			break;
		case PCAN_ERROR_OVERRUN:
			strcpy(Buffer, resource[Language][IDS_STR_IND_ERR_OVERRUN]);
			break;
		case PCAN_ERROR_BUSLIGHT:
			strcpy(Buffer, resource[Language][IDS_STR_IND_ERR_BUSLIGHT]);
			break;
		case PCAN_ERROR_BUSHEAVY:
			strcpy(Buffer, resource[Language][IDS_STR_IND_ERR_BUSHEAVY]);
			break;
		case PCAN_ERROR_BUSOFF:
			strcpy(Buffer, resource[Language][IDS_STR_IND_ERR_BUSOFF]);
			break;
		case PCAN_ERROR_ANYBUSERR:
			strcpy(Buffer, resource[Language][IDS_STR_IND_ERR_ANYBUSERR]);
			break;
		case PCAN_ERROR_QRCVEMPTY:
			strcpy(Buffer, resource[Language][IDS_STR_IND_ERR_QRCVEMPTY]);
			break;
		case PCAN_ERROR_QOVERRUN:
			strcpy(Buffer, resource[Language][IDS_STR_IND_ERR_QOVERRUN]);
			break;
		case PCAN_ERROR_QXMTFULL:
			strcpy(Buffer, resource[Language][IDS_STR_IND_ERR_QXMTFULL]);
			break;
		case PCAN_ERROR_REGTEST:
			strcpy(Buffer, resource[Language][IDS_STR_IND_ERR_REGTEST]);
			break;
		case PCAN_ERROR_NODRIVER:
			strcpy(Buffer, resource[Language][IDS_STR_IND_ERR_NODRIVER]);
			break;
		case PCAN_ERROR_RESOURCE:
			strcpy(Buffer, resource[Language][IDS_STR_IND_ERR_RESOURCE]);
			break;
		case PCAN_ERROR_ILLPARAMTYPE:
			strcpy(Buffer, resource[Language][IDS_STR_IND_ERR_ILLPARAMTYPE]);
			break;
		case PCAN_ERROR_ILLPARAMVAL:
			strcpy(Buffer, resource[Language][IDS_STR_IND_ERR_ILLPARAMVAL]);
			break;
#if PCAN_ERROR_ILLCLIENT != PCAN_ERROR_ILLHANDLE
		case PCAN_ERROR_ILLHANDLE:
			strcpy(Buffer, resource[Language][IDS_STR_IND_ERR_ILLHANDLE]);
			break;
#endif
		case PCAN_ERROR_INITIALIZE:
			strcpy(Buffer, resource[Language][IDS_STR_IND_ERR_INITIALIZE]);
			break;
		case PCAN_ERROR_UNKNOWN:
			strcpy(Buffer, resource[Language][IDS_STR_IND_ERR_UNKNOW]);
			break;
		case PCAN_ERROR_HWINUSE:
			strcpy(Buffer, resource[Language][IDS_STR_IND_ERR_HWINUSE]);
			break;
		case PCAN_ERROR_NETINUSE:
			strcpy(Buffer, resource[Language][IDS_STR_IND_ERR_NETINUSE]);
			break;
		case PCAN_ERROR_ILLHW:
			strcpy(Buffer, resource[Language][IDS_STR_IND_ERR_ILLHW]);
			break;
		case PCAN_ERROR_ILLNET:
			strcpy(Buffer, resource[Language][IDS_STR_IND_ERR_ILLNET]);
			break;
		case PCAN_ERROR_ILLCLIENT:
			strcpy(Buffer, resource[Language][IDS_STR_IND_ERR_ILLCLIENT]);
			break;
		case PCAN_ERROR_ILLDATA:
			strcpy(Buffer, resource[Language][IDS_STR_IND_ERR_ILLDATA]);
			break;
		case PCAN_ERROR_ILLOPERATION:
			strcpy(Buffer, resource[Language][IDS_STR_IND_ERR_ILLOPERATION]);
			break;
		case PCAN_ERROR_BUSPASSIVE:
			strcpy(Buffer, resource[Language][IDS_STR_IND_ERR_BUSPASSIVE]);
			break;
		case PCAN_ERROR_CAUTION:
			strcpy(Buffer, resource[Language][IDS_STR_IND_ERR_CAUTION]);
			break;
		default:
			sprintf(Buffer, "Undefined (0x%x)", Error);
			break;
		}

		LOG_LeavingFrom("CAN_GetErrorText", Result);
		return Result;
	} catch (...) {
		Result = PCAN_ERROR_UNKNOWN;
		LOG_Exception("CAN_GetErrorText");
	}
	return Result;
}

