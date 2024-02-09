/* SPDX-License-Identifier: LGPL-2.1-only */
/*
 * 02_GetSetParameter.h - PCANBasic Example: GetSetParameter
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
#include "linux_interop.h"
#include "PCANBasic.h"

class GetSetParameter
{
private:
	/// <summary>
	/// Sets the PCANHandle (Hardware Channel)
	/// </summary>
	const TPCANHandle PcanHandle = PCAN_USBBUS1;
	/// <summary>
	/// Sets the desired connection mode (CAN = false / CAN-FD = true)
	/// </summary>
	const bool IsFD = false;
	/// <summary>
	/// Sets the bitrate for normal CAN devices
	/// </summary>
	const TPCANBaudrate Bitrate = PCAN_BAUD_500K;
	/// <summary>
	/// Sets the bitrate for CAN FD devices.
	/// Example - Bitrate Nom: 1Mbit/s Data: 2Mbit/s:
	///   "f_clock_mhz=20, nom_brp=5, nom_tseg1=2, nom_tseg2=1, nom_sjw=1, data_brp=2, data_tseg1=3, data_tseg2=1, data_sjw=1"
	/// </summary>
	TPCANBitrateFD BitrateFD = const_cast<LPSTR>("f_clock_mhz=20, nom_brp=5, nom_tseg1=2, nom_tseg2=1, nom_sjw=1, data_brp=2, data_tseg1=3, data_tseg2=1, data_sjw=1");

public:
	// GetSetParameter constructor
	//
	GetSetParameter();

	// GetSetParameter destructor
	//
	~GetSetParameter();

private:
	/// <summary>
	/// Runs all commands for get or set parameters
	/// </summary>
	void RunSelectedCommands();

	/// <summary>
	/// Shows device identifier parameter
	/// </summary>
	void GetPCAN_DEVICE_ID();

	/// <summary>
	/// Sets device identifier parameter
	/// </summary>
	/// <param name="iDeviceID"></param>
	void SetPCAN_DEVICE_ID(UINT32 iDeviceID);

	/// <summary>
	/// Shows all information about attached channels
	/// </summary>
	void GetPCAN_ATTACHED_CHANNELS();

	/// <summary>
	/// Shows the status of selected PCAN-Channel
	/// </summary>
	void GetPCAN_CHANNEL_CONDITION();

	/// <summary>
	///  Shows the status from the status LED of the USB devices
	/// </summary>
	void GetPCAN_CHANNEL_IDENTIFYING();

	/// <summary>
	/// De/Activates the status LED of the USB devices
	/// </summary>
	/// <param name="value">True to turn on; False to turn off</param>
	void SetPCAN_CHANNEL_IDENTIFYING(bool value);

	/// <summary>
	/// Shows information about features
	/// </summary>
	void GetPCAN_CHANNEL_FEATURES();

	/// <summary>
	/// Shows the status from Bitrate-Adapting mode
	/// </summary>
	void GetPCAN_BITRATE_ADAPTING();

	/// <summary>
	/// De/Activates the Bitrate-Adapting mode
	/// </summary>
	/// <param name="value">True to turn on; False to turn off</param>
	void SetPCAN_BITRATE_ADAPTING(bool value);

	/// <summary>
	/// Shows the status from the reception of status frames
	/// </summary>
	void GetPCAN_ALLOW_STATUS_FRAMES();

	/// <summary>
	/// De/Activates the reception of status frames
	/// </summary>
	/// <param name="value">True to turn on; False to turn off</param>
	void SetPCAN_ALLOW_STATUS_FRAMES(bool value);

	/// <summary>
	/// Shows the status from the reception of RTR frames
	/// </summary>
	void GetPCAN_ALLOW_RTR_FRAMES();

	/// <summary>
	/// De/Activates the reception of RTR frames
	/// </summary>
	/// <param name="value">True to turn on; False to turn off</param>
	void SetPCAN_ALLOW_RTR_FRAMES(bool value);

	/// <summary>
	/// Shows the status from the reception of CAN error frames
	/// </summary>
	void GetPCAN_ALLOW_ERROR_FRAMES();

	/// <summary>
	/// De/Activates the reception of CAN error frames
	/// </summary>
	/// <param name="value">True to turn on; False to turn off</param>
	void SetPCAN_ALLOW_ERROR_FRAMES(bool value);

	/// <summary>
	/// Shows the status from the reception of Echo frames
	/// </summary>
	void GetPCAN_ALLOW_ECHO_FRAMES();

	/// <summary>
	/// De/Activates the reception of Echo frames
	/// </summary>
	/// <param name="value">True to turn on; False to turn off</param>
	void SetPCAN_ALLOW_ECHO_FRAMES(bool value);

	/// <summary>
	/// Shows the reception filter with a specific 11-bit acceptance code and mask
	/// </summary>
	void GetPCAN_ACCEPTANCE_FILTER_11BIT();

	/// <summary>
	/// Sets the reception filter with a specific 11-bit acceptance code and mask
	/// </summary>
	/// <param name="iacceptancefilter11bit">Acceptance code and mask</param>
	void SetPCAN_ACCEPTANCE_FILTER_11BIT(UINT64 iacceptancefilter11bit);

	/// <summary>
	/// Shows the reception filter with a specific 29-bit acceptance code and mask
	/// </summary>
	void GetPCAN_ACCEPTANCE_FILTER_29BIT();

	/// <summary>
	/// Sets the reception filter with a specific 29-bit acceptance code and mask
	/// </summary>
	/// <param name="iacceptancefilter29bit">Acceptance code and mask</param>
	void SetPCAN_ACCEPTANCE_FILTER_29BIT(UINT64 iacceptancefilter29bit);

	/// <summary>
	/// Shows the status of the reception filter
	/// </summary>
	void GetPCAN_MESSAGE_FILTER();

	/// <summary>
	/// De/Activates the reception filter
	/// </summary>
	/// <param name="imessagefilter">Configure reception filter</param>
	void SetPCAN_MESSAGE_FILTER(UINT32 imessagefilter);

	/// <summary>
	/// Shows/prints the configurable parameters for this sample and information about them
	/// </summary>
	void ShowConfigurationHelp();

	/// <summary>
	/// Shows/prints the configured paramters
	/// </summary>
	void ShowCurrentConfiguration();

	/// <summary>
	/// Shows formatted status
	/// </summary>
	/// <param name="status">Will be formatted</param>
	void ShowStatus(TPCANStatus status);

	/// <summary>
	/// Gets the formatted text for a PCAN-Basic channel handle
	/// </summary>
	/// <param name="handle">PCAN-Basic Handle to format</param>
	/// <parma name="buffer">A string buffer for the channel name</param>
	/// <param name="isFD">If the channel is FD capable</param>
	void FormatChannelName(TPCANHandle handle, LPSTR buffer, bool isFD);

	/// <summary>
	/// Gets name of a TPCANHandle
	/// </summary>
	/// <param name="handle">TPCANHandle to get name</param>
	/// <param name="buffer">A string buffer for the name of the TPCANHandle (size MAX_PATH)</param>
	void GetTPCANHandleName(TPCANHandle handle, LPSTR buffer);

	/// <summary>
	/// Help Function used to get an error as text
	/// </summary>
	/// <param name="error">Error code to be translated</param>
	/// <param name="buffer">A string buffer for the translated error (size MAX_PATH)</param>
	void GetFormattedError(TPCANStatus error, LPSTR buffer);

	/// <summary>
	/// Convert bitrate c_short value to readable string
	/// </summary>
	/// <param name="bitrate">Bitrate to be converted</param>
	/// <param name="buffer">A string buffer for the converted bitrate (size MAX_PATH)</param>
	void ConvertBitrateToString(TPCANBaudrate bitrate, LPSTR buffer);

	/// <summary>
	/// Convert BYTE value to readable string value
	/// </summary>
	/// <param name="devicetype"></param>
	/// <returns></returns>
	std::string ConvertDeviceTypeToString(BYTE devicetype);

	/// <summary>
	/// Convert uint value to readable string value
	/// </summary>
	/// <param name="value"></param>
	/// <returns></returns>
	std::string ConvertToParameterOnOff(UINT32 value);

	/// <summary>
	/// Convert uint value to readable string value
	/// </summary>
	/// <param name="value"></param>
	/// <returns></returns>
	std::string ConvertToChannelFeatures(UINT32 value);

	/// <summary>
	/// Convert uint value to readable string value
	/// </summary>
	/// <param name="value"></param>
	/// <returns></returns>
	std::string ConvertToChannelCondition(UINT32 value);

	/// <summary>
	/// Convert uint value to readable string value
	/// </summary>
	/// <param name="value"></param>
	/// <returns></returns>
	std::string ConvertToFilterOpenCloseCustom(UINT32 value);
};