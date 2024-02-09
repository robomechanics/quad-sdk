/* SPDX-License-Identifier: LGPL-2.1-only */
/*
 * 09_TraceFiles.h - PCANBasic Example: TraceFiles
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

class TraceFiles
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
	/// Sets if trace continue after reaching maximum size for the first file
	/// </summary>
	const bool TraceFileSingle = true;
	/// <summary>
	/// Set if date will be add to filename
	/// </summary>
	const bool TraceFileDate = true;
	/// <summary>
	/// Set if time will be add to filename
	/// </summary>
	const bool TraceFileTime = true;
	/// <summary>
	/// Set if existing tracefile overwrites when a new trace session is started
	/// </summary>
	const bool TraceFileOverwrite = false;
	/// <summary>
	/// Set if the column "Data Length" should be used instead of the column "Data Length Code"
	/// </summary>
	const bool TraceFileDataLength = false;
	/// <summary>
	/// Sets the size (megabyte) of an tracefile
	/// Example - 100 = 100 megabyte
	/// Range between 1 and 100
	/// </summary>
	const UINT32 TraceFileSize = 2;
	/// <summary>
	/// Sets a fully-qualified and valid path to an existing directory. In order to use the default path
	/// (calling process path) an empty string must be set.
	/// </summary>
	LPCSTR TracePath = "";
	/// <summary>
	/// Timerinterval (ms) for reading
	/// </summary>
	const int TimerInterval = 250;
	/// <summary>
	/// Thread for reading messages
	/// </summary>
	std::thread* m_ReadThread;
	/// <summary>
	/// Shows if thread run
	/// </summary>
	bool m_ThreadRun;

public:
	// TraceFiles constructor
	//
	TraceFiles();

	// TraceFiles destructor
	//
	~TraceFiles();

private:
	/// <summary>
	/// Thread function for reading messages
	/// </summary>
	void ThreadExecute();

	/// <summary>
	/// Reads PCAN-Basic messages
	/// </summary>
	void ReadMessages();

	/// <summary>
	/// Deactivates the tracing process
	/// </summary>
	void StopTrace();

	/// <summary>
	/// Configures the way how trace files are formatted
	/// </summary>
	/// <returns>Returns true if no error occurr</returns>
	bool ConfigureTrace();

	/// <summary>
	/// Activates the tracing process
	/// </summary>
	/// <returns>Returns true if no error occurr</returns>
	bool StartTrace();

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
	/// Convert bool value to readable string value
	/// </summary>
	/// <param name="value">Value to be converted</param>
	/// <returns>A text with the converted Value</returns>
	bool ConvertBoolToString(bool value);
};
