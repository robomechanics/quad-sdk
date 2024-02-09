/* SPDX-License-Identifier: LGPL-2.1-only */
/*
 * 05_TimerRead.h - PCANBasic Example: TimerRead
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

class TimerRead
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
	/// <summary>
	/// Timerinterval (ms) for reading
	/// </summary>
	const int TimerInterval = 250;
	/// <summary>
	/// Shows if DLL was found
	/// </summary>
	bool m_DLLFound;
	/// <summary>
	/// Used for reading
	/// </summary>
	std::thread* m_hTimer;
	/// <summary>
	/// Shows if thread run
	/// </summary>
	bool m_TimerOn;

public:
	// TimerRead constructor
	//
	TimerRead();

	// TimerRead destructor
	//
	~TimerRead();

private:
	/// <summary>
	/// Thread function for reading messages
	/// </summary>
	void TimerThread();

	/// <summary>
	/// Function for reading PCAN-Basic messages
	/// </summary>
	void ReadMessages();

	/// <summary>
	/// Function for reading messages on CAN-FD devices
	/// </summary>
	/// <returns>A TPCANStatus error code</returns>
	TPCANStatus ReadMessageFD();

	/// <summary>
	/// Function for reading CAN messages on normal CAN devices
	/// </summary>
	/// <returns>A TPCANStatus error code</returns>
	TPCANStatus ReadMessage();

	/// <summary>
	/// Processes a received CAN message
	/// </summary>
	/// <param name="msg">The received PCAN-Basic CAN message</param>
	/// <param name="itsTimeStamp">Timestamp of the message as TPCANTimestamp structure</param>
	void ProcessMessageCan(TPCANMsg msg, TPCANTimestamp itsTimeStamp);

	/// <summary>
	/// Processes a received CAN-FD message
	/// </summary>
	/// <param name="msg">The received PCAN-Basic CAN-FD message</param>
	/// <param name="itsTimeStamp">Timestamp of the message as microseconds (ulong)</param>
	void ProcessMessageCanFD(TPCANMsgFD msg, TPCANTimestampFD itsTimeStamp);

	/// <summary>
	/// Checks for availability of the PCANBasic labrary
	/// </summary>
	/// <returns>If the library was found or not</returns>
	bool CheckForLibrary();

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
	/// Gets the string representation of the type of a CAN message
	/// </summary>
	/// <param name="msgType">Type of a CAN message</param>
	/// <returns>The type of the CAN message as string</returns>
	std::string GetMsgTypeString(TPCANMessageType msgType);

	/// <summary>
	/// Gets the string representation of the ID of a CAN message
	/// </summary>
	/// <param name="id">Id to be parsed</param>
	/// <param name="msgType">Type flags of the message the Id belong</param>
	/// <returns>Hexadecimal representation of the ID of a CAN message</returns>
	std::string GetIdString(UINT32 id, TPCANMessageType msgType);

	/// <summary>
	/// Gets the data length of a CAN message
	/// </summary>
	/// <param name="dlc">Data length code of a CAN message</param>
	/// <returns>Data length as integer represented by the given DLC code</returns>
	int GetLengthFromDLC(BYTE dlc);

	/// <summary>
	/// Gets the string representation of the timestamp of a CAN message, in milliseconds
	/// </summary>
	/// <param name="time">Timestamp in microseconds</param>
	/// <returns>String representing the timestamp in milliseconds</returns>
	std::string GetTimeString(TPCANTimestampFD time);

	/// <summary>
	/// Gets the data of a CAN message as a string
	/// </summary>
	/// <param name="data">Array of bytes containing the data to parse</param>
	/// <param name="msgType">Type flags of the message the data belong</param>
	/// <param name="dataLength">The amount of bytes to take into account wihtin the given data</param>
	/// <returns>A string with hexadecimal formatted data bytes of a CAN message</returns>
	std::string GetDataString(BYTE data[], TPCANMessageType msgType, int dataLength);
};
