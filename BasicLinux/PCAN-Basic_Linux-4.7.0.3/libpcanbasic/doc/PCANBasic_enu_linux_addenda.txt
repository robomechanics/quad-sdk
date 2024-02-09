===============================================================================
PCANBasic_enu_linux_addenda.txt

PCAN-Basic Linux V4.6.1.0
Copyright (c) 2022 PEAK-System Technik GmbH Darmstadt, Germany
All rights reserved.
===============================================================================
	
API remarks
===========

PCANBasic Linux is built on top of libpcanfd library and therefore do not share the source code of its Windows version.
Some non-critical differences exists betwen the 2 versions and can be found listed below.

Linux developers looking for performance rather than interoperability should consider using libpcanfd API which is a minimal wrapper around PCAN Linux Driver.
Linux developers who would rather avoid installation of a third party library should consider the usage of socketCAN with the mainline linux drivers.

	
Differences with Windows Version
================================

Some features listed in the PCANBasic documentation (Windows version) are not available within Linux PCANBasic API. 

- Timestamps on Linux are the number of milliseconds that have elapsed since the epoch: the epoch is the point in time of 00:00 UTC, 1 January 1970.

- The following parameters will return PCAN_ERROR_ILLPARAMTYPE (0x4000, 16384) with the CAN_GetValue/CAN_SetValue functions: 
	* PCAN_5VOLTS_POWER
	* PCAN_CHANNEL_IDENTIFYING (CAN_GetValue only)
	* PCAN_IP_ADDRESS
	* PCAN_LAN_SERVICE_STATUS

- The following parameters have a slightly different behaviour/usage than the one described in the Windows documentation:
	* PCAN_CHANNEL_IDENTIFYING: with PCAN Linux Driver >=8.10, CAN_SetValue will be blocked while the LED blinks for 5 seconds (except with value PCAN_PARAMETER_OFF/0x00).
	* PCAN_RECEIVE_EVENT: CAN_SetValue will return PCAN_ERROR_ILLOPERATION as the feature is enabled by default.
	* PCAN_BUSSPEED_NOMINAL: required buffer size is 64 bits long.
	* PCAN_BUSSPEED_DATA: required buffer size is 64 bits long.
	* PCAN_ALLOW_ECHO_FRAMES: can return PCAN_ERROR_ILLPARAMTYPE if the device doesn't have the lastest available firmware.

- Trace files are ordered differently based on the device's capabilities
	* For devices that supports "message ECHO" feature: 
		* Trace data is updated on CAN_Read/CAN_ReadFD calls ONLY.
		* CAN frames are traced in chronological order of CAN reception.		
	* For other devices:
		* Trace data is updated on CAN_Read/CAN_ReadFD and CAN_Write/CAN_WriteFD calls.
		* CAN frames are NOT traced in chronological order (it will depends on CAN_Write/CAN_Read).
		* The timestamps for Tx frames correspond to the call to CAN_Write/CAN_WriteFD,
		  it is NOT the timestamp of the physical frame on the CAN bus
		  (a delay must be considered based on the size of the frame and the bus load at the time of the writing).
	
- Error codes' clarifications:
	* PCAN_ERROR_XMTFULL  (0x00001) PCAN driver Transmit buffer is full (check PCAN Linux driver's parameter: txqsize).
	* PCAN_ERROR_QXMTFULL (0x00080) CAN device Transmit buffer is full.  
	* PCAN_ERROR_QOVERRUN (0x00040) PCAN driver Receive buffer was read too late (check PCAN Linux driver's parameter: rxqsize).
	* PCAN_ERROR_OVERRUN  (0x00002) CAN device Receive buffer was read too late.

- Miscellaneous:
	* CAN_GetValue/CAN_SetValue used with an unavailable device will return PCAN_ERROR_ILLHANDLE instead of PCAN_ERROR_INITIALIZE.
	* When a parameter PCAN_ALLOW_xxx is modified, CAN_Read/CAN_ReadFD can return PCAN_ERROR_ILLDATA on Windows, this is not the case on Linux.

- Using events:
	* PCANBasic Linux uses File Descriptors instead of Windows Event objects.
	* Parameter PCAN_RECEIVE_EVENT is enabled by default. Use CAN_GetValue with that parameter to retrieve the file descriptor managing the receive queue.
	* Samples:
		* C: libpcanbasic/examples/console/c/src/pcaneventread.c
		* C++: libpcanbasic/examples/console/NativeC++/08_EventDrivenRead/08_EventDrivenRead.cpp

Notes specific to PCAN Linux Driver
===================================

- Since version 4.4.3, PCAN-Basic Linux overrides the parameter "deftsmode" from PCAN Linux Driver. 
  This is required to let the API compare Hardware and Software timestamps:
	* PCAN-Basic sets deftmods to PCANFD_OPT_HWTIMESTAMP_COOKED on successfully initialized channels.
	* If this value is not compatible with the selected hardware, deftmods is set to PCANFD_OPT_HWTIMESTAMP_OFF.