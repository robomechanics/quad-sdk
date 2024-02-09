/* SPDX-License-Identifier: LGPL-2.1-only */
/*
 * 09_TraceFiles.cpp - PCANBasic Example: TraceFiles
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
 * Maintainer:  Fabrice Vergnaud <f.vergnaud@peak-system.com>
 * 	    	    Romain Tissier <r.tissier@peak-system.com>
 */
#include "09_TraceFiles.h"

TraceFiles::TraceFiles()
{
	ShowConfigurationHelp(); // Shows information about this sample
	ShowCurrentConfiguration(); // Shows the current parameters configuration

	TPCANStatus stsResult;
	// Initialization of the selected channel
	if (IsFD)
		stsResult = CAN_InitializeFD(PcanHandle, BitrateFD);
	else
		stsResult = CAN_Initialize(PcanHandle, Bitrate);

	if (stsResult != PCAN_ERROR_OK)
	{
		std::cout << "Can not initialize. Please check the defines in the code.\n";
		ShowStatus(stsResult);
		std::cout << "\n";
		std::cout << "Closing...\n";
		std::cout << "Press any key to continue...\n";
		_getch();
		return;
	}

	// Trace messages.
	std::cout << "Successfully initialized.\n";
	std::cout << "Starting trace: ";
	std::cout << "Press any key to continue...\n";
	_getch();
	if (ConfigureTrace())
	{
		if (StartTrace())
		{
			m_ThreadRun = true;
			m_ReadThread = new std::thread(&TraceFiles::ThreadExecute, this);
			std::cout << "Messages are being traced.\n";
			std::cout << "Stop trace and quit: ";
			std::cout << "Press any key to continue...\n";
			_getch();
			StopTrace();
			return;
		}
	}
	std::cout << "\n";
	std::cout << "For close: ";
	std::cout << "Press any key to continue...\n";
	_getch();
}

TraceFiles::~TraceFiles()
{
	m_ThreadRun = false;
	m_ReadThread->join();
	CAN_Uninitialize(PCAN_NONEBUS);
}

void TraceFiles::ThreadExecute()
{
	while (m_ThreadRun)
	{
		Sleep(1); //Use Sleep to reduce the CPU load
		ReadMessages();
	}
}

void TraceFiles::ReadMessages()
{
	// We read at least one time the queue looking for messages. If a message is found, we look again trying to
	// find more. If the queue is empty or an error occurr, we get out from the dowhile statement.
	TPCANStatus stsResult;
	TPCANMsgFD CANMsgFD;
	TPCANMsg CANMsg;
	UINT64 TimestampFD;
	TPCANTimestamp Timestamp;
	do
	{
		stsResult = IsFD ? CAN_ReadFD(PcanHandle, &CANMsgFD, &TimestampFD) : CAN_Read(PcanHandle, &CANMsg, &Timestamp);
		if (stsResult != PCAN_ERROR_OK && stsResult != PCAN_ERROR_QRCVEMPTY)
		{
			ShowStatus(stsResult);
			return;
		}
	} while ((!(stsResult & PCAN_ERROR_QRCVEMPTY)));
}

void TraceFiles::StopTrace()
{
	UINT32 iStatus = PCAN_PARAMETER_OFF;

	TPCANStatus stsResult = CAN_SetValue(PcanHandle, PCAN_TRACE_STATUS, &iStatus, sizeof(UINT32)); // We stop the tracing by setting the parameter.

	if (stsResult != PCAN_ERROR_OK)
		ShowStatus(stsResult);
}

bool TraceFiles::ConfigureTrace()
{
	UINT32 iSize = TraceFileSize;

	// Sets path to store files
	TPCANStatus stsResult = CAN_SetValue(PcanHandle, PCAN_TRACE_LOCATION, (void*)TracePath, sizeof(TracePath));

	if (stsResult == PCAN_ERROR_OK)
	{
		// Sets the maximum size of a tracefile
		stsResult = CAN_SetValue(PcanHandle, PCAN_TRACE_SIZE, &iSize, sizeof(UINT32));

		if (stsResult == PCAN_ERROR_OK)
		{
			UINT32 config;
			if (TraceFileSingle)
				config = TRACE_FILE_SINGLE; // Creats one file
			else
				config = TRACE_FILE_SEGMENTED; // Creats more files

			// Activate overwriting existing tracefile
			if (TraceFileOverwrite)
				config = config | TRACE_FILE_OVERWRITE;

			// Uses Data Length instead of Data Length Code
			if (TraceFileDataLength)
				config = config | TRACE_FILE_DATA_LENGTH;

			// Adds date to tracefilename
			if (TraceFileDate)
				config = config | TRACE_FILE_DATE;

			// Adds time to tracefilename
			if (TraceFileTime)
				config = config | TRACE_FILE_TIME;

			// Sets the config
			stsResult = CAN_SetValue(PcanHandle, PCAN_TRACE_CONFIGURE, &config, sizeof(int));

			if (stsResult == PCAN_ERROR_OK)
				return true;
		}
	}
	ShowStatus(stsResult);
	return false;
}

bool TraceFiles::StartTrace()
{
	UINT32 iStatus = PCAN_PARAMETER_ON;

	TPCANStatus stsResult = CAN_SetValue(PcanHandle, PCAN_TRACE_STATUS, &iStatus, sizeof(UINT32)); // We activate the tracing by setting the parameter.

	if (stsResult != PCAN_ERROR_OK)
	{
		ShowStatus(stsResult);
		return false;
	}
	return true;
}

void TraceFiles::ShowConfigurationHelp()
{
	std::cout << "=========================================================================================\n";
	std::cout << "|                           PCAN-Basic TraceFiles Example                                |\n";
	std::cout << "=========================================================================================\n";
	std::cout << "Following parameters are to be adjusted before launching, according to the hardware used |\n";
	std::cout << "                                                                                         |\n";
	std::cout << "* PcanHandle: Numeric value that represents the handle of the PCAN-Basic channel to use. |\n";
	std::cout << "              See 'PCAN-Handle Definitions' within the documentation                     |\n";
	std::cout << "* IsFD: Boolean value that indicates the communication mode, CAN (false) or CAN-FD (true)|\n";
	std::cout << "* Bitrate: Numeric value that represents the BTR0/BR1 bitrate value to be used for CAN   |\n";
	std::cout << "           communication                                                                 |\n";
	std::cout << "* BitrateFD: String value that represents the nominal/data bitrate value to be used for  |\n";
	std::cout << "             CAN-FD communication                                                        |\n";
	std::cout << "* TraceFileSingle: Boolean value that indicates if tracing ends after one file (true) or |\n";
	std::cout << "                   continues                                                             |\n";
	std::cout << "* TraceFileDate: Boolean value that indicates if the date will be added to filename      |\n";
	std::cout << "* TraceFileTime: Boolean value that indicates if the time will be added to filename      |\n";
	std::cout << "* TraceFileOverwrite: Boolean value that indicates if existing tracefiles should be      |\n";
	std::cout << "                      overwritten                                                        |\n";
	std::cout << "* TraceFileDataLength: Boolean value that indicates if the column 'Data Length' is used  |\n";
	std::cout << "                       instead of the column 'Data Length Code'                          |\n";
	std::cout << "* TraceFileSize: Numeric value that represents the size of a tracefile in meagabytes     |\n";
	std::cout << "* TracePath: string value that represents a valid path to an existing directory          |\n";
	std::cout << "* TimerInterval: The time, in milliseconds, to wait before trying to write a message     |\n";
	std::cout << "=========================================================================================\n";
	std::cout << "\n";
}

void TraceFiles::ShowCurrentConfiguration()
{
	std::cout << "Parameter values used\n";
	std::cout << "----------------------\n";
	char buffer[MAX_PATH];
	FormatChannelName(PcanHandle, buffer, IsFD);
	std::cout << "* PCANHandle: " << buffer << "\n";
	if (IsFD)
		std::cout << "* IsFD: True\n";
	else
		std::cout << "* IsFD: False\n";
	ConvertBitrateToString(Bitrate, buffer);
	std::cout << "* Bitrate: " << buffer << "\n";
	std::cout << "* BitrateFD: " << BitrateFD << "\n";
	if (TraceFileSingle)
		std::cout << "* TraceFileSingle: True\n";
	else
		std::cout << "* TraceFileSingle: False\n";
	if (TraceFileDate)
		std::cout << "* TraceFileDate: True\n";
	else
		std::cout << "* TraceFileDate: False\n";
	if (TraceFileTime)
		std::cout << "* TraceFileTime: True\n";
	else
		std::cout << "* TraceFileTime: False\n";
	if (TraceFileOverwrite)
		std::cout << "* TraceFileOverwrite: True\n";
	else
		std::cout << "* TraceFileOverwrite: False\n";
	std::cout << "* TraceFileSize: " << TraceFileSize << " MB\n";
	if (TracePath == "")
		std::cout << "* TracePath: (calling application path)" << "\n";
	else
		std::cout << "* TracePath: " << TracePath << "\n";
	std::cout << "* TimerInterval: " << TimerInterval << "\n";
	std::cout << "\n";
}

void TraceFiles::ShowStatus(TPCANStatus status)
{
	std::cout << "=========================================================================================\n";
	char buffer[MAX_PATH];
	GetFormattedError(status, buffer);
	std::cout << buffer << "\n";
	std::cout << "=========================================================================================\n";
}

void TraceFiles::FormatChannelName(TPCANHandle handle, LPSTR buffer, bool isFD)
{
	TPCANDevice devDevice;
	BYTE byChannel;

	// Gets the owner device and channel for a PCAN-Basic handle
	if (handle < 0x100)
	{
		devDevice = (TPCANDevice)(handle >> 4);
		byChannel = (BYTE)(handle & 0xF);
	}
	else
	{
		devDevice = (TPCANDevice)(handle >> 8);
		byChannel = (BYTE)(handle & 0xFF);
	}

	// Constructs the PCAN-Basic Channel name and return it
	char handleBuffer[MAX_PATH];
	GetTPCANHandleName(handle, handleBuffer);
	if (isFD)
		sprintf_s(buffer, MAX_PATH, "%s:FD %d (%Xh)", handleBuffer, byChannel, handle);
	else
		sprintf_s(buffer, MAX_PATH, "%s %d (%Xh)", handleBuffer, byChannel, handle);
}

void TraceFiles::GetTPCANHandleName(TPCANHandle handle, LPSTR buffer)
{
	strcpy_s(buffer, MAX_PATH, "PCAN_NONE");
	switch (handle)
	{
	case PCAN_PCIBUS1:
	case PCAN_PCIBUS2:
	case PCAN_PCIBUS3:
	case PCAN_PCIBUS4:
	case PCAN_PCIBUS5:
	case PCAN_PCIBUS6:
	case PCAN_PCIBUS7:
	case PCAN_PCIBUS8:
	case PCAN_PCIBUS9:
	case PCAN_PCIBUS10:
	case PCAN_PCIBUS11:
	case PCAN_PCIBUS12:
	case PCAN_PCIBUS13:
	case PCAN_PCIBUS14:
	case PCAN_PCIBUS15:
	case PCAN_PCIBUS16:
		strcpy_s(buffer, MAX_PATH, "PCAN_PCI");
		break;

	case PCAN_USBBUS1:
	case PCAN_USBBUS2:
	case PCAN_USBBUS3:
	case PCAN_USBBUS4:
	case PCAN_USBBUS5:
	case PCAN_USBBUS6:
	case PCAN_USBBUS7:
	case PCAN_USBBUS8:
	case PCAN_USBBUS9:
	case PCAN_USBBUS10:
	case PCAN_USBBUS11:
	case PCAN_USBBUS12:
	case PCAN_USBBUS13:
	case PCAN_USBBUS14:
	case PCAN_USBBUS15:
	case PCAN_USBBUS16:
		strcpy_s(buffer, MAX_PATH, "PCAN_USB");
		break;

	case PCAN_LANBUS1:
	case PCAN_LANBUS2:
	case PCAN_LANBUS3:
	case PCAN_LANBUS4:
	case PCAN_LANBUS5:
	case PCAN_LANBUS6:
	case PCAN_LANBUS7:
	case PCAN_LANBUS8:
	case PCAN_LANBUS9:
	case PCAN_LANBUS10:
	case PCAN_LANBUS11:
	case PCAN_LANBUS12:
	case PCAN_LANBUS13:
	case PCAN_LANBUS14:
	case PCAN_LANBUS15:
	case PCAN_LANBUS16:
		strcpy_s(buffer, MAX_PATH, "PCAN_LAN");
		break;

	default:
		strcpy_s(buffer, MAX_PATH, "UNKNOWN");
		break;
	}
}

void TraceFiles::GetFormattedError(TPCANStatus error, LPSTR buffer)
{
	// Gets the text using the GetErrorText API function. If the function success, the translated error is returned.
	// If it fails, a text describing the current error is returned.
	if (CAN_GetErrorText(error, 0x09, buffer) != PCAN_ERROR_OK)
		sprintf_s(buffer, MAX_PATH, "An error occurred. Error-code's text (%Xh) couldn't be retrieved", error);
}

void TraceFiles::ConvertBitrateToString(TPCANBaudrate bitrate, LPSTR buffer)
{
	switch (bitrate)
	{
	case PCAN_BAUD_1M:
		strcpy_s(buffer, MAX_PATH, "1 MBit/sec");
		break;
	case PCAN_BAUD_800K:
		strcpy_s(buffer, MAX_PATH, "800 kBit/sec");
		break;
	case PCAN_BAUD_500K:
		strcpy_s(buffer, MAX_PATH, "500 kBit/sec");
		break;
	case PCAN_BAUD_250K:
		strcpy_s(buffer, MAX_PATH, "250 kBit/sec");
		break;
	case PCAN_BAUD_125K:
		strcpy_s(buffer, MAX_PATH, "125 kBit/sec");
		break;
	case PCAN_BAUD_100K:
		strcpy_s(buffer, MAX_PATH, "100 kBit/sec");
		break;
	case PCAN_BAUD_95K:
		strcpy_s(buffer, MAX_PATH, "95,238 kBit/sec");
		break;
	case PCAN_BAUD_83K:
		strcpy_s(buffer, MAX_PATH, "83,333 kBit/sec");
		break;
	case PCAN_BAUD_50K:
		strcpy_s(buffer, MAX_PATH, "50 kBit/sec");
		break;
	case PCAN_BAUD_47K:
		strcpy_s(buffer, MAX_PATH, "47,619 kBit/sec");
		break;
	case PCAN_BAUD_33K:
		strcpy_s(buffer, MAX_PATH, "33,333 kBit/sec");
		break;
	case PCAN_BAUD_20K:
		strcpy_s(buffer, MAX_PATH, "20 kBit/sec");
		break;
	case PCAN_BAUD_10K:
		strcpy_s(buffer, MAX_PATH, "10 kBit/sec");
		break;
	case PCAN_BAUD_5K:
		strcpy_s(buffer, MAX_PATH, "5 kBit/sec");
		break;
	default:
		strcpy_s(buffer, MAX_PATH, "Unknown Bitrate");
		break;
	}
}
