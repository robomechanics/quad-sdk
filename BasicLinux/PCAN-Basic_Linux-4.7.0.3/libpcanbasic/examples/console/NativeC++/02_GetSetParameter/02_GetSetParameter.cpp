/* SPDX-License-Identifier: LGPL-2.1-only */
/*
 * 02_GetSetParameter.cpp - PCANBasic Example: GetSetParameter
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
#include "02_GetSetParameter.h"
#ifdef __linux__
#include <inttypes.h>
#else
#ifndef PRIX64
#define PRIX64 "%llX"
#endif
#endif

GetSetParameter::GetSetParameter()
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

	std::cout << "Successfully initialized.\n";
	std::cout << "Get/set parameter\n";
	std::cout << "Press any key to continue...\n";
	_getch();
	std::cout << "\n";

	RunSelectedCommands();

	std::cout << "\n";
	std::cout << "Closing...\n";
	std::cout << "Press any key to continue...\n";
	_getch();
}

GetSetParameter::~GetSetParameter()
{
	CAN_Uninitialize(PCAN_NONEBUS);
}

void GetSetParameter::RunSelectedCommands()
{
	// Fill commands here
	std::cout << "Fill \"RunSelectedCommands\"-function with parameter functions from \"Parameter commands\"-Region in the code.\n";
	//GetPCAN_DEVICE_ID();
	//GetPCAN_ATTACHED_CHANNELS();
	//GetPCAN_CHANNEL_CONDITION();
	//GetPCAN_CHANNEL_IDENTIFYING();
	//GetPCAN_CHANNEL_FEATURES();
	//GetPCAN_BITRATE_ADAPTING();
	//GetPCAN_ALLOW_STATUS_FRAMES();
	//GetPCAN_ALLOW_RTR_FRAMES();
	//GetPCAN_ALLOW_ERROR_FRAMES();
	//GetPCAN_ACCEPTANCE_FILTER_11BIT();
	//GetPCAN_ACCEPTANCE_FILTER_29BIT();
	//GetPCAN_MESSAGE_FILTER();
	
	//SetPCAN_DEVICE_ID(0);
	//SetPCAN_CHANNEL_IDENTIFYING(1);

	//SetPCAN_BITRATE_ADAPTING(1);	 
	//SetPCAN_ALLOW_STATUS_FRAMES(1);
	//SetPCAN_ALLOW_RTR_FRAMES(1);
	//SetPCAN_ALLOW_ERROR_FRAMES(1);
	//SetPCAN_ACCEPTANCE_FILTER_11BIT(0x00000000000007FF);
	//SetPCAN_ACCEPTANCE_FILTER_29BIT(0x000000001FFFFFFF);
	//SetPCAN_MESSAGE_FILTER(0);
}

void GetSetParameter::GetPCAN_DEVICE_ID()
{
	UINT32 iDeviceID;
	TPCANStatus stsResult = CAN_GetValue(PcanHandle, PCAN_DEVICE_ID, &iDeviceID, sizeof(UINT32));

	if (stsResult == PCAN_ERROR_OK)
	{
		std::cout << "-----------------------------------------------------------------------------------------\n";
		std::cout << "Get PCAN_DEVICE_ID: " << iDeviceID << "\n";
		std::cout << "\n";
	}
	else
		ShowStatus(stsResult);
}

void GetSetParameter::SetPCAN_DEVICE_ID(UINT32 iDeviceID)
{
	TPCANStatus stsResult = CAN_SetValue(PcanHandle, PCAN_DEVICE_ID, &iDeviceID, sizeof(UINT32));

	if (stsResult == PCAN_ERROR_OK)
	{
		std::cout << "-----------------------------------------------------------------------------------------\n";
		std::cout << "Set PCAN_DEVICE_ID: " << iDeviceID << "\n";
		std::cout << "\n";
	}
	else
		ShowStatus(stsResult);
}

void GetSetParameter::GetPCAN_ATTACHED_CHANNELS()
{
	UINT32 iChannelsCount;
	TPCANStatus stsResult = CAN_GetValue(PCAN_NONEBUS, PCAN_ATTACHED_CHANNELS_COUNT, &iChannelsCount, sizeof(UINT32));

	if (stsResult == PCAN_ERROR_OK)
	{
		TPCANChannelInformation* ciChannelInformation = new TPCANChannelInformation[iChannelsCount];

		stsResult = CAN_GetValue(PCAN_NONEBUS, PCAN_ATTACHED_CHANNELS, ciChannelInformation, iChannelsCount * sizeof(TPCANChannelInformation));

		if (stsResult == PCAN_ERROR_OK)
		{
			std::cout << "-----------------------------------------------------------------------------------------\n";
			std::cout << "Get PCAN_ATTACHED_CHANNELS:\n";

			for (int i = 0; i < (int)iChannelsCount; i++)
			{
				std::cout << "---------------------------\n";
				char buffer[MAX_PATH];
				GetTPCANHandleName(ciChannelInformation[i].channel_handle, buffer);
				if (IsFD)
					std::cout << "channel_handle:      " << buffer << "BUS" << (ciChannelInformation[i].channel_handle & 0xFF) << "\n";
				else
					std::cout << "channel_handle:      " << buffer << "BUS" << (ciChannelInformation[i].channel_handle & 0xF) << "\n";
				std::cout << "device_type:         " << ConvertDeviceTypeToString(ciChannelInformation[i].device_type) << "\n";
				std::cout << "controller_number:   " << (int)ciChannelInformation[i].controller_number << "\n";
				std::cout << "device_features:     " << ConvertToChannelFeatures(ciChannelInformation[i].device_features) << "\n";
				std::cout << "device_name:         " << ciChannelInformation[i].device_name << "\n";
				std::cout << "device_id:           " << ciChannelInformation[i].device_id << "\n";
				std::cout << "channel_condition:   " << ConvertToChannelCondition(ciChannelInformation[i].channel_condition) << "\n";
			}
			std::cout << "\n";
		}
		delete [] ciChannelInformation;
	}
	if (stsResult != PCAN_ERROR_OK)
		ShowStatus(stsResult);

}

void GetSetParameter::GetPCAN_CHANNEL_CONDITION()
{
	UINT32 iChannelCondition;
	TPCANStatus stsResult = CAN_GetValue(PcanHandle, PCAN_CHANNEL_CONDITION, &iChannelCondition, sizeof(UINT32));

	if (stsResult == PCAN_ERROR_OK)
	{
		std::cout << "-----------------------------------------------------------------------------------------\n";
		std::cout << "Get PCAN_CHANNEL_CONDITION: " << ConvertToChannelCondition(iChannelCondition) << "\n";
		std::cout << "\n";
	}
	else
		ShowStatus(stsResult);
}

void GetSetParameter::GetPCAN_CHANNEL_IDENTIFYING()
{
	UINT32 iChannelIdentifying;
#if 0
	TPCANStatus stsResult = CAN_GetValue(PcanHandle, PCAN_CHANNEL_IDENTIFYING, &iChannelIdentifying, sizeof(UINT32));

	if (stsResult == PCAN_ERROR_OK)
	{
		std::cout << "-----------------------------------------------------------------------------------------\n";
		std::cout << "Get PCAN_CHANNEL_IDENTIFYING: " << ConvertToParameterOnOff(iChannelIdentifying) << "\n";
		std::cout << "\n";
	}
	else
		ShowStatus(stsResult);
#else
	std::cout << "Get PCAN_CHANNEL_IDENTIFYING: not available on linux, see PCANBasic_enu_linux_addenda.txt\n";
#endif
}

void GetSetParameter::SetPCAN_CHANNEL_IDENTIFYING(bool value)
{
	UINT32 ciChannelIdentifying;
	if (value)
		ciChannelIdentifying = PCAN_PARAMETER_ON;
	else
		ciChannelIdentifying = PCAN_PARAMETER_OFF;

	TPCANStatus stsResult = CAN_SetValue(PcanHandle, PCAN_CHANNEL_IDENTIFYING, &ciChannelIdentifying, sizeof(UINT32));

	if (stsResult == PCAN_ERROR_OK)
	{

		std::cout << "-----------------------------------------------------------------------------------------\n";
		std::cout << "Set PCAN_CHANNEL_IDENTIFYING: " << ConvertToParameterOnOff(ciChannelIdentifying) << "\n";
		std::cout << "\n";
	}
	else
		ShowStatus(stsResult);
}

void GetSetParameter::GetPCAN_CHANNEL_FEATURES()
{
	UINT32 iChannelFeatures;
	TPCANStatus stsResult = CAN_GetValue(PcanHandle, PCAN_CHANNEL_FEATURES, &iChannelFeatures, sizeof(UINT32));

	if (stsResult == PCAN_ERROR_OK)
	{
		std::cout << "-----------------------------------------------------------------------------------------\n";
		std::cout << "Get PCAN_CHANNEL_FEATURES: " << ConvertToChannelFeatures(iChannelFeatures) << "\n";
		std::cout << "\n";
	}
	else
		ShowStatus(stsResult);
}

void GetSetParameter::GetPCAN_BITRATE_ADAPTING()
{
	UINT32 iBitrateAdapting;
	TPCANStatus stsResult = CAN_GetValue(PcanHandle, PCAN_BITRATE_ADAPTING, &iBitrateAdapting, sizeof(UINT32));

	if (stsResult == PCAN_ERROR_OK)
	{
		std::cout << "-----------------------------------------------------------------------------------------\n";
		std::cout << "Get PCAN_BITRATE_ADAPTING: " << ConvertToParameterOnOff(iBitrateAdapting) << "\n";
		std::cout << "\n";
	}
	else
		ShowStatus(stsResult);
}

void GetSetParameter::SetPCAN_BITRATE_ADAPTING(bool value)
{
	UINT32 iBitrateAdapting;

    // Note: SetPCAN_BITRATE_ADAPTING requires an uninitialized channel, 
    //
    CAN_Uninitialize(PCAN_NONEBUS);

	if (value)
		iBitrateAdapting = PCAN_PARAMETER_ON;
	else
		iBitrateAdapting = PCAN_PARAMETER_OFF;

	TPCANStatus stsResult = CAN_SetValue(PcanHandle, PCAN_BITRATE_ADAPTING, &iBitrateAdapting, sizeof(UINT32));

	if (stsResult == PCAN_ERROR_OK)
	{
		std::cout << "-----------------------------------------------------------------------------------------\n";
		std::cout << "Set PCAN_BITRATE_ADAPTING: " << ConvertToParameterOnOff(iBitrateAdapting) << "\n";
		std::cout << "\n";
	}
	else
		ShowStatus(stsResult);

    // Channel will be connected again
    if (IsFD)
        stsResult = CAN_InitializeFD(PcanHandle, BitrateFD);
    else
        stsResult = CAN_Initialize(PcanHandle, Bitrate);

    if (stsResult != PCAN_ERROR_OK)
    {
        std::cout << "Error while re-initializing the channel.\n";
        ShowStatus(stsResult);
    }
}

void GetSetParameter::GetPCAN_ALLOW_STATUS_FRAMES()
{
	UINT32 iAllowStatusFrames;
	TPCANStatus stsResult = CAN_GetValue(PcanHandle, PCAN_ALLOW_STATUS_FRAMES, &iAllowStatusFrames, sizeof(UINT32));

	if (stsResult == PCAN_ERROR_OK)
	{
		std::cout << "-----------------------------------------------------------------------------------------\n";
		std::cout << "Get PCAN_ALLOW_STATUS_FRAMES: " << ConvertToParameterOnOff(iAllowStatusFrames) << "\n";
		std::cout << "\n";
	}
	else
		ShowStatus(stsResult);
}

void GetSetParameter::SetPCAN_ALLOW_STATUS_FRAMES(bool value)
{
	UINT32 iAllowStatusFrames;

	if (value)
		iAllowStatusFrames = PCAN_PARAMETER_ON;
	else
		iAllowStatusFrames = PCAN_PARAMETER_OFF;

	TPCANStatus stsResult = CAN_SetValue(PcanHandle, PCAN_ALLOW_STATUS_FRAMES, &iAllowStatusFrames, sizeof(UINT32));

	if (stsResult == PCAN_ERROR_OK)
	{
		std::cout << "-----------------------------------------------------------------------------------------\n";
		std::cout << "Set PCAN_ALLOW_STATUS_FRAMES: " << ConvertToParameterOnOff(iAllowStatusFrames) << "\n";
		std::cout << "\n";
	}
	else
		ShowStatus(stsResult);
}

void GetSetParameter::GetPCAN_ALLOW_RTR_FRAMES()
{
	UINT32 iAllowRTRFrames;
	TPCANStatus stsResult = CAN_GetValue(PcanHandle, PCAN_ALLOW_RTR_FRAMES, &iAllowRTRFrames, sizeof(UINT32));

	if (stsResult == PCAN_ERROR_OK)
	{
		std::cout << "-----------------------------------------------------------------------------------------\n";
		std::cout << "Get PCAN_ALLOW_RTR_FRAMES: " << ConvertToParameterOnOff(iAllowRTRFrames) << "\n";
		std::cout << "\n";
	}
	else
		ShowStatus(stsResult);
}

void GetSetParameter::SetPCAN_ALLOW_RTR_FRAMES(bool value)
{
	UINT32 iAllowRTRFrames;

	if (value)
		iAllowRTRFrames = PCAN_PARAMETER_ON;
	else
		iAllowRTRFrames = PCAN_PARAMETER_OFF;

	TPCANStatus stsResult = CAN_SetValue(PcanHandle, PCAN_ALLOW_RTR_FRAMES, &iAllowRTRFrames, sizeof(UINT32));

	if (stsResult == PCAN_ERROR_OK)
	{
		std::cout << "-----------------------------------------------------------------------------------------\n";
		std::cout << "Set PCAN_ALLOW_RTR_FRAMES: " << ConvertToParameterOnOff(iAllowRTRFrames) << "\n";
		std::cout << "\n";
	}
	else
		ShowStatus(stsResult);
}

void GetSetParameter::GetPCAN_ALLOW_ERROR_FRAMES()
{
	UINT32 iAllowErrorFrames;
	TPCANStatus stsResult = CAN_GetValue(PcanHandle, PCAN_ALLOW_ERROR_FRAMES, &iAllowErrorFrames, sizeof(UINT32));

	if (stsResult == PCAN_ERROR_OK)
	{
		std::cout << "-----------------------------------------------------------------------------------------\n";
		std::cout << "Get PCAN_ALLOW_ERROR_FRAMES: " << ConvertToParameterOnOff(iAllowErrorFrames) << "\n";
		std::cout << "\n";
	}
	else
		ShowStatus(stsResult);
}

void GetSetParameter::SetPCAN_ALLOW_ERROR_FRAMES(bool value)
{
	UINT32 iAllowErrorFrames;

	if (value)
		iAllowErrorFrames = PCAN_PARAMETER_ON;
	else
		iAllowErrorFrames = PCAN_PARAMETER_OFF;

	TPCANStatus stsResult = CAN_SetValue(PcanHandle, PCAN_ALLOW_ERROR_FRAMES, &iAllowErrorFrames, sizeof(UINT32));

	if (stsResult == PCAN_ERROR_OK)
	{
		std::cout << "-----------------------------------------------------------------------------------------\n";
		std::cout << "Set PCAN_ALLOW_ERROR_FRAMES: " << ConvertToParameterOnOff(iAllowErrorFrames) << "\n";
		std::cout << "\n";
	}
	else
		ShowStatus(stsResult);
}

#if defined(PCAN_ALLOW_ECHO_FRAMES)
void GetSetParameter::GetPCAN_ALLOW_ECHO_FRAMES()
{
	UINT32 iAllowEchoFrames;
	TPCANStatus stsResult = CAN_GetValue(PcanHandle, PCAN_ALLOW_ECHO_FRAMES, &iAllowEchoFrames, sizeof(UINT32));

	if (stsResult == PCAN_ERROR_OK)
	{
		std::cout << "-----------------------------------------------------------------------------------------\n";
		std::cout << "Get PCAN_ALLOW_ECHO_FRAMES: " << ConvertToParameterOnOff(iAllowEchoFrames) << "\n";
		std::cout << "\n";
	}
	else
		ShowStatus(stsResult);
}

void GetSetParameter::SetPCAN_ALLOW_ECHO_FRAMES(bool value)
{
	UINT32 iAllowEchoFrames;

	if (value)
		iAllowEchoFrames = PCAN_PARAMETER_ON;
	else
		iAllowEchoFrames = PCAN_PARAMETER_OFF;

	TPCANStatus stsResult = CAN_SetValue(PcanHandle, PCAN_ALLOW_ECHO_FRAMES, &iAllowEchoFrames, sizeof(UINT32));

	if (stsResult == PCAN_ERROR_OK)
	{
		std::cout << "-----------------------------------------------------------------------------------------\n";
		std::cout << "Set PCAN_ALLOW_ECHO_FRAMES: " << ConvertToParameterOnOff(iAllowEchoFrames) << "\n";
		std::cout << "\n";
	}
	else
		ShowStatus(stsResult);
}
#else
#pragma message("PCAN_ALLOW_ECHO_FRAMES requires PCANBasic version >=4.6, some functions are ignored...")
#endif

void GetSetParameter::GetPCAN_ACCEPTANCE_FILTER_11BIT()
{
	UINT64 iAcceptanceFilter11Bit;
	TPCANStatus stsResult = CAN_GetValue(PcanHandle, PCAN_ACCEPTANCE_FILTER_11BIT, &iAcceptanceFilter11Bit, sizeof(UINT64));

	if (stsResult == PCAN_ERROR_OK)
	{
		char result[MAX_PATH];
		std::cout << "-----------------------------------------------------------------------------------------\n";
		sprintf_s(result, sizeof(result), PRIX64, iAcceptanceFilter11Bit);
		std::cout << "Get PCAN_ACCEPTANCE_FILTER_11BIT: " << std::setfill('0') << std::setw(16) << std::hex << result << "h\n";
		std::cout << "\n";
	}
	else
		ShowStatus(stsResult);
}

void GetSetParameter::SetPCAN_ACCEPTANCE_FILTER_11BIT(UINT64 iacceptancefilter11bit)
{
	TPCANStatus stsResult = CAN_SetValue(PcanHandle, PCAN_ACCEPTANCE_FILTER_11BIT, &iacceptancefilter11bit, sizeof(UINT64));

	if (stsResult == PCAN_ERROR_OK)
	{
		char result[MAX_PATH];
		std::cout << "-----------------------------------------------------------------------------------------\n";
		sprintf_s(result, sizeof(result), PRIX64, iacceptancefilter11bit);
		std::cout << "Set PCAN_ACCEPTANCE_FILTER_11BIT: " << std::setfill('0') << std::setw(16) << std::hex << result << "h\n";
		std::cout << "\n";
	}
	else
		ShowStatus(stsResult);
}

void GetSetParameter::GetPCAN_ACCEPTANCE_FILTER_29BIT()
{
	UINT64 iAcceptanceFilter29Bit;
	TPCANStatus stsResult = CAN_GetValue(PcanHandle, PCAN_ACCEPTANCE_FILTER_29BIT, &iAcceptanceFilter29Bit, sizeof(UINT64));

	if (stsResult == PCAN_ERROR_OK)
	{
		char result[MAX_PATH];
		std::cout << "-----------------------------------------------------------------------------------------\n";
		sprintf_s(result, sizeof(result), PRIX64, iAcceptanceFilter29Bit);
		std::cout << "Get PCAN_ACCEPTANCE_FILTER_29BIT: " << std::setfill('0') << std::setw(16) << std::hex << result << "h\n";
		std::cout << "\n";
	}
	else
		ShowStatus(stsResult);
}

void GetSetParameter::SetPCAN_ACCEPTANCE_FILTER_29BIT(UINT64 iacceptancefilter29bit)
{
	TPCANStatus stsResult = CAN_SetValue(PcanHandle, PCAN_ACCEPTANCE_FILTER_29BIT, &iacceptancefilter29bit, sizeof(UINT64));

	if (stsResult == PCAN_ERROR_OK)
	{
		char result[MAX_PATH];
		std::cout << "-----------------------------------------------------------------------------------------\n";
		sprintf_s(result, sizeof(result), PRIX64, iacceptancefilter29bit);
		std::cout << "Set PCAN_ACCEPTANCE_FILTER_29BIT: " << std::setfill('0') << std::setw(16) << std::hex << result << "h\n";
		std::cout << "\n";
	}
	else
		ShowStatus(stsResult);
}

void GetSetParameter::GetPCAN_MESSAGE_FILTER()
{
	UINT32 iMessageFilter;
	TPCANStatus stsResult = CAN_GetValue(PcanHandle, PCAN_MESSAGE_FILTER, &iMessageFilter, sizeof(UINT32));

	if (stsResult == PCAN_ERROR_OK)
	{
		std::cout << "-----------------------------------------------------------------------------------------\n";
		std::cout << "Get PCAN_MESSAGE_FILTER: " << ConvertToFilterOpenCloseCustom(iMessageFilter) << "\n";
		std::cout << "\n";
	}
	else
		ShowStatus(stsResult);
}

void GetSetParameter::SetPCAN_MESSAGE_FILTER(UINT32 imessagefilter)
{
	TPCANStatus stsResult = CAN_SetValue(PcanHandle, PCAN_MESSAGE_FILTER, &imessagefilter, sizeof(UINT32));

	if (stsResult == PCAN_ERROR_OK)
	{
		std::cout << "-----------------------------------------------------------------------------------------\n";
		std::cout << "Set PCAN_MESSAGE_FILTER: " << ConvertToFilterOpenCloseCustom(imessagefilter) << "\n";
		std::cout << "\n";
	}
	else
		ShowStatus(stsResult);
}

void GetSetParameter::ShowConfigurationHelp()
{
	std::cout << "=========================================================================================\n";
	std::cout << "|                           PCAN-Basic GetSetParameter Example                           |\n";
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
	std::cout << "=========================================================================================\n";
	std::cout << "\n";
}

void GetSetParameter::ShowCurrentConfiguration()
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
	std::cout << "\n";
}

void GetSetParameter::ShowStatus(TPCANStatus status)
{
	std::cout << "=========================================================================================\n";
	char buffer[MAX_PATH];
	GetFormattedError(status, buffer);
	std::cout << buffer << "\n";
	std::cout << "=========================================================================================\n";
}

void GetSetParameter::FormatChannelName(TPCANHandle handle, LPSTR buffer, bool isFD)
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

void GetSetParameter::GetTPCANHandleName(TPCANHandle handle, LPSTR buffer)
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

void GetSetParameter::GetFormattedError(TPCANStatus error, LPSTR buffer)
{
	// Gets the text using the GetErrorText API function. If the function success, the translated error is returned.
	// If it fails, a text describing the current error is returned.
	if (CAN_GetErrorText(error, 0x09, buffer) != PCAN_ERROR_OK)
		sprintf_s(buffer, MAX_PATH, "An error occurred. Error-code's text (%Xh) couldn't be retrieved", error);
}

void GetSetParameter::ConvertBitrateToString(TPCANBaudrate bitrate, LPSTR buffer)
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

std::string GetSetParameter::ConvertDeviceTypeToString(BYTE devicetype)
{
	switch (devicetype)
	{
	case 0:
		return "PCAN_NONE";
	case 1:
		return "PCAN_PEAKCAN";
	case 2:
		return "PCAN_ISA";
	case 3:
		return "PCAN_DNG";
	case 4:
		return "PCAN_PCI";
	case 5:
		return "PCAN_USB";
	case 6:
		return "PCAN_PCC";
	case 7:
		return "PCAN_VIRTUAL";
	case 8:
		return "PCAN_LAN";
	default:
		return "";
	}
}

std::string GetSetParameter::ConvertToParameterOnOff(UINT32 value)
{
	switch (value)
	{
	case PCAN_PARAMETER_OFF:
		return "PCAN_PARAMETER_OFF";
	case PCAN_PARAMETER_ON:
		return "PCAN_PARAMETER_ON";
	default:
		return "Status unknown: " + std::to_string(value);
	}
}

std::string GetSetParameter::ConvertToChannelFeatures(UINT32 value)
{
	std::string sFeatures = "";
	if ((value & FEATURE_FD_CAPABLE) == FEATURE_FD_CAPABLE)
		sFeatures += "FEATURE_FD_CAPABLE";
	if ((value & FEATURE_DELAY_CAPABLE) == FEATURE_DELAY_CAPABLE)
		if (sFeatures != "")
			sFeatures += ", FEATURE_DELAY_CAPABLE";
		else
			sFeatures += "FEATURE_DELAY_CAPABLE";
	if ((value & FEATURE_IO_CAPABLE) == FEATURE_IO_CAPABLE)
		if (sFeatures != "")
			sFeatures += ", FEATURE_IO_CAPABLE";
		else
			sFeatures += "FEATURE_IO_CAPABLE";
	return sFeatures;
}

std::string GetSetParameter::ConvertToChannelCondition(UINT32 value)
{
	switch (value)
	{
	case PCAN_CHANNEL_UNAVAILABLE:
		return "PCAN_CHANNEL_UNAVAILABLE";
	case PCAN_CHANNEL_AVAILABLE:
		return "PCAN_CHANNEL_AVAILABLE";
	case PCAN_CHANNEL_OCCUPIED:
		return "PCAN_CHANNEL_OCCUPIED";
	case PCAN_CHANNEL_PCANVIEW:
		return "PCAN_CHANNEL_PCANVIEW";
	default:
		return "Status unknow: " + std::to_string(value);
	}
}

std::string GetSetParameter::ConvertToFilterOpenCloseCustom(UINT32 value)
{
	switch (value)
	{
	case PCAN_FILTER_CLOSE:
		return "PCAN_FILTER_CLOSE";
	case PCAN_FILTER_OPEN:
		return "PCAN_FILTER_OPEN";
	case PCAN_FILTER_CUSTOM:
		return "PCAN_FILTER_CUSTOM";
	default:
		return "Status unknown: " + std::to_string(value);
	}
}