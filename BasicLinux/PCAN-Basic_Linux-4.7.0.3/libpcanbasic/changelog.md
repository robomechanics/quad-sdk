# Changelog
All notable changes to "libpcanbasic package" will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).

## [4.7.0] - 2023-03-29
### Added
- Added support of value TRACE_FILE_DATA_LENGTH with parameter PCAN_TRACE_CONFIGURE.
- Added .NET headers in "include" directory
### Changed
- Moved PCANBasic.h to "include" directory 
- Generated trace file includes a suffixed index only if TRACE_FILE_SEGMENTED is set.
- If TRACE_FILE_OVERWRITE is not set and a file already exists, tracing will fail (EOPNOTSUPP) and be disabled.
- in Makefile, commented LDFLAGS_CLEANUP_SO to prevent warnings due to an issue with Binutils <2.33 on ARM.
### Updated
- CAN_SetValue with PCAN_TRACE_LOCATION and PCAN_TRACE_STATUS better handles I/O error and returns corresponding PCAN-Basic status code.

## [4.6.2] - 2022-09-29
### Updated
- Fixed STARTTIME in trace file to use local time.
- Improved logs for fw update checks.
- Updated libpcanfd source code to v8.15.1.

## [4.6.1] - 2022-05-10
### Updated
- Prevented possible buffer overflows with PCAN_ALLOW_ECHO_FRAMES and PCAN_BUSOFF_AUTORESET.
- PCANBasic.h includes <stdint.h> if UINT64 is not defined
- CAN_Initialize and CAN_InitializeFD wait for bus to be active (up to 1ms) (PCAN_ERROR_CAUTION is returned if initialization succeeded but bus_state is not active).
- Reduced required buffer size for GetValue(PCAN_FIRMWARE_VERSION) (required size was >= 257)

## [4.6.0] - 2022-04-08
### Added
- Added support to parameter PCAN_ALLOW_ECHO_FRAMES and frame flag PCAN_MESSAGE_ECHO that allows the reception of self-sent messages.
- Added support to parameter PCAN_DEVICE_PART_NUMBER to get the part number of a PCAN hardware (a.k.a. IPEH number).
- Added support to fw and driver update checks.
### Changed
- Changed format ouput (hex) for pcaninfo fields: base, btr0btr1, devid, init_flags.
- pcanbasic_get_hw returns PCAN_ERROR_ILLHANDLE if hw group is unknown.
- pcaninfo changed to v1.3.1: minor improvements in output and log.

## [4.5.5] - 2022-02-07
### Updated
- Check that directory exists when setting PCAN_TRACE_LOCATION parameter.
- Added support to alternative TPCANHandles.

## [4.5.4] - 2022-01-18
### Added
- Added internal support to get the part number of a PCAN hardware (a.k.a. IPEH number).
- Added translations for error PCAN_ERROR_ILLMODE.
### Changed
- Changed required size for parameter PCAN_LOG_STATUS to uint8_t
- Parameter PCAN_LOG_TEXT automatically enables PCAN_LOG_STATUS
- Corrected a bug in pcanbasic_bus_state_to_condition (ERROR_ACTIVE flag was detected as ERROR_PASSIVE)
- If a status message is read the return value of CAN_Read/CAN_ReadFD is also the error code of the status frame.
- Makefile swapped command 'cp' in install to $(INSTALL) (defined as 'install')
- French text translations were revised/corrected
### Updated
- Corrected memory leak: close log file on exit
- Fixed issue when uninitializing/initializing channel with different bitrates within a few milliseconds.
- Fixed incorrect returned value when calling parameter channel condition on a specific use-case
- Fixed CAN_SetValue with parameters PCAN_TRACE_LOCATION and PCAN_LOG_LOCATION to allow a NULL buffer of size 0.
- Improved size checks for null-terminated get/set parameters
- Normalized status code for pcanbasic_get_value_xxx functions
- Updated some returned codes (in case of errors) for every entry points to match exactly PCANBasic Windows 4.6 behaviour.
- Added return values for functions in pcblog.h
- Updated PCANBasic_enu_linux_addenda.txt

## [4.5.3] - 2021-11-18
### Updated
- Fixed timestamp issue with CAN_Read.

## [4.5.2] - 2021-10-04
### Updated
- Fixed year in "start time" comments for .trc files.

## [4.5.1] - 2021-09-24
### Changed
- Fixed CAN_Uninitialize when Tx queue is not empty, waiting time was 50ms instead of 500ms.
- CAN_Uninitialize clears Tx queue before closing (avoiding any driver's extra waiting time)
- Fixed issue when calling CAN_Uninitialize(PCAN_NONEBUS) and continue using the API (involving parameter PCAN_AVAILABLE_CHANNELS).
- Fixed side-effects in CAN_SetValue on non-ARM systems (see previous ARM patch).
- Fixed syslog log_level
- library initialization now also initializes internal parameters 

## [4.5.0] - 2021-09-02
### Added
- Added support to new API entry point: CAN_LookUpChannel.
- Added automatic compilation of libpcanbasic 32 bit version
- Added mechanism to ignore status frame on reset (and CAN_Initialize, use definition KEEP_STATUS_FRAME_ON_RESET to disable)
- Added timestamps usage to examples
- Added support to future PCAN Linux 8.13 features (ioctl resets)
- Added missing functions to handle logging of LOG_FUNCTION_READ, LOG_FUNCTION_WRITE
### Changed
- Fixed $STARTTIME in trc file to match PEAK-TRACE spec.
- Replaced uses of deprecated 'gettimeofday' with 'clock_gettime'.
- Function pcanbasic_get_device now initializes a status parameter to get same error code as on Windows.
- Function pcanbasic_get_channel now have a TCANStatus param to improve and give information to caller.
- Factorized duplicate code with internal function pcanbasic_get_fd.
- Updated MakeFile to specify precisely exported functions (see def file).
- Fixed returned code (in case of erros) for every entry points to match exactly PCANBasic Windows behaviour.
- Given pathes are now internally converted and stored as absolute pathes (internal pathes are dynamically allocated).
- Enabling logging now checks and immediately opens the log file (same as Windows).
- CAN_Read/CAN_ReadFD and CAN_Write/CAN_WriteFD now check if the corresponding CAN_Initialize/CAN_InitializeFD was used (PCAN_ERROR_ILLOPERATION).
- CAN_Write/CAN_WriteFD now supports PCAN_ERROR_XMTFULL error code.
- Changed trace to support chronological ordering: libpcanfd msg-echoing is enabled by default (if supported, FD fw only).
- Fixed CAN_SetValue to return an error when the size of the buffer is invalid.
- Improved CAN_SetValue to better handle buffer with a size smaller than expected (ARM).
### Updated
- Header upgraded to match PCANBasic Windows v4.5.3.
- Fixed possible infinite loop in pcanbasic_parse_fd_init.

## [1.2.0] - 2020-12-16 [pcaninfo]
### Added 
- Outputs version of the installed PCAN-Basic API

## [1.1.0] - 2020-08-07 [pcaninfo]
### Changed
- Default behaviour now displays a short list of available devices.
### Added
- Option "all" (-a) outputs a detailed listing of PCAN devices.
- Option "list" (-l) outputs a short listing of PCAN devices with their PCAN-Basic handle and device id.
- Search filter now support PCAN-Basic handle name.
- A message is displayed if no device is detected.

## [4.4.3] - 2020-04-27
### Added
- Added support to clk_ref information from PCAN Linux driver 8.11+
### Changed
- CAN_SetValue for PCAN_DEVICE_ID and PCAN_5VOLTS_POWER now requires an initialized channel (as mentionned by documentation)
- Calls to deprecated function gettimeofday(..) was replaced with clock_gettime(clk_ref,...).
- Trace file now have a starttime set a few milliseconds prior initialisation.
- Trace file can display negative relative timestamp.
- Fixed minor typos in logs
- Compilation now supports distinct preprocessor's definitions of LOG_LEVEL and _DEBUG
- PCAN_PARAMETER_CHANNEL_VERSION now includes channel's type and PEAK copyright (to match string returned by PCANBasic Windows API)

## [4.4.2] - 2020-12-16
### Changed
- Fixed compilation error in -std=gnu90
- 'Initialize' functions will try to retrieve a more precise error when 'pcanfd_open' fails.
- API version string will end with "(debug)" if library is built in debug mode.
- Refactored logging in debug mode and fixed syslog logging.
- pcaninfo changed to v1.2.0: Outputs version of the installed PCAN-Basic API.

## [4.4.1] - 2020-08-14
### Updated
- Added checks to prevent possible buffer overflows when copying CAN/CANFD message's DATA.
- Added mutex in API entries to provide more robust thread-safe system.  

## [4.4.0] - 2020-08-07
### Changed
- Updated pcanbasic to match features from Windows PCAN-Basic 4.4
- Updated pcaninfo to display a short or detailed list of devices

## [4.3.4] - 2020-03-04
### Changed
- Fix pcanbasic/Makefile\_latest.mk so that shared object can be built under 
  Xenomai 3.1

## [4.3.3] - 2020-02-18
### Changed
- Sources files header now explicitely gives usage license (LGPL v2.1)

## [4.3.2] - 2019-10-08
### Added
- Python's PCAN-Basic example now supports Python3.

## [4.3.1.3] - 2019-04-12
### Added
- Added support to param PCAN_FIRMWARE_VERSION (0x29).
- Added support to sysfs params: clk_drift, dev_name, init_flags, mass_storage_mode, nom_sample_point, nom_tq, data_sample_point, data_tq, ts_fixed.
### Changed
- Fixed check on pcanfd_open's return value as RT system can return the value 0.

## [4.3.0.1] - 2019-02-04
### Added
- Added support to PCAN_IO_xxx parameters (syncing features with PCANBasic Windows v4.3).
### Changed
- Using libpcanfd 8.6.
- Cleared gcc's stringop-truncation warnings.
- Improved invalid values in pcanbasic_get_fd_dlc and pcanbasic_get_fd_len functions.
- Fixed cascading issue in pcanbasic_write_common (memcpy segfault) if DLC > 0x0F.
- Fixed debug/verbose log issue with pcaninfo.

## [4.2.3.7] - 2018-08-03
### Changed
- Fixed an issue with autoreset feature (message status PCANFD_TYPE_STATUS was wrongly analized).
- Fixed an issue with PCANFD bitrate string initialization: a space was required after each comma.
- Fixed compilation warning -Wformat-truncation with gcc 8.1 in pcaninfo.h.

## [4.2.2.6] - 2018-08-03
### Added
- Added missing support of device PCAN_USBX6.

## [4.2.1.5] - 2018-03-16
### Added
- Added a 50ms temporisation when closing a channel that still has Tx pending messages.

## [4.2.1.4] - 2018-03-06
### Changed
- Fixed PCANBasic status when receiving EAGAIN from libpcanfd (error is either QXMTFULL or QRCVEMPTY whether read or write function is called).

## [4.2.1.1] - 2018-03-03
### Added
- Added PCANBasic_enu_linux_addenda.txt that lists differences within documentation compared to Windows.
- Fixed CAN FD timestamp issue.

## [4.2.0.3] - 2017-11-03
### Changed
- Fixed alignments in generated trace file (timestamp and 11bit CAN ids).

## [4.2.0.2] - 2017-10-31
### Changed
- Fixed wrong month number in generated trace file.
- Fixed ill use of snprintf in trace file leading to trace only first byte of message data.

## [4.2.0.1] - 2017-10-09
### Added
- Added support of Windows PCANBasic 4.2 features (only with PCAN linux driver >8.5.x): 
	- Reception of Status Frames: PCAN_ALLOW_STATUS_FRAMES,
	- Reception of RTR Frames: PCAN_ALLOW_RTR_FRAMES,
	- Reception of Error Frames: PCAN_ALLOW_ERROR_FRAMES,
	- Interframe Transmit Delay: PCAN_INTERFRAME_DELAY,
	- Acceptance filters: PCAN_ACCEPTANCE_FILTER_11BIT and PCAN_ACCEPTANCE_FILTER_29BIT.
- Package now includes documentation (same as the Windows version, open .chm files with 'xchm' for instance).

## [4.1.2.5] - 2017-10-04
### Added
- Header now includes Windows PCANBasic 4.2 new features (not yet implemented, status will return 'illegal operation').
- Fixed PCAN_TRACE_xxx features (the feature was never launched when session is activated by user).

## [4.1.1.3] - 2017-02-17
### Changed
- Patched overflow timestamp issue.
- Corrected a few status error (bad convertions from errno to status).

## [4.1.0.2] - 2016-09-08
### Changed
- Fixed memory leaks issues within 'pcanjni' library.
- PCANBasic java example replaced with the sample in pcanjni folder.

## [4.1.0] - 2016-07-20
### Changed
- Changed version to match Windows PCAN-Basic API version (note: old API 2.0.4.6 still included to support PEAK linux driver prior to v8.x).

## [4.0.0.4] - 2016-05-20 + v2.0.4.6 legacy driver
### Changed
- CAN_GetErrorText now returns PCAN_ERROR_ILLPARAMVAL on unknown error status.
- v2.0.4.6: Fixed issue when retrieving/setting PCAN_DEVICE_NUMBER.
- Java 'peak.can' package updated to match Windows version.

## [4.0.0.3 + 2.0.4.5 legacy driver] - 2016-03-16
### Changed
- Fixed a null pointer exception when calling CAN_Unitialize(PCAN_NONEBUS) and continue using the API.

## [4.0.0.2 + 2.0.4.5 legacy driver] - 2016-03-01
### Changed
- v4.0.0.2: Upgraded pcaninfo to match changes in PCAN driver v8.0.17.

## [4.0.0.1 + 2.0.4.5 legacy driver] - 2016-01-29
### Changed
- v4.0.0.1: Upgraded to match changes in PCAN driver v8.0.7.

## [4.0.0 + 2.0.4.5 legacy driver] - 2016-01-18
### Changed
- v2.0.4.5: Fixed GetStatus function (require PCAN driver 7.16.0).
- v2.0.4.5: (known issue) USB channel's handles will be offseted if the minor.
  number of the first PCAN-USB device is not 0 (or n=32 in cat /proc/pcan).

## [4.0.0] - 2016-01-12
### Changed
- Upgraded PCANBasic Linux API to match Windows version 4.0.2.
- Requires PCAN driver v8.0.6.

## [2.0.4] - 2015-11-04
### Changed
- Fixed impossibility to write enhanced message with RTR flag.
- Fixed PCAN_CHANNEL_CONDITION request with PCAN-USB-PRO, PCAN-USB-FD, PCAN-USB-PRO-FD and PCAN-PCI-FD hardware.

## [2.0.3] - 2014-10-17
### Changed
- Change licences from GPL to LGPL.
- Fix issue with using CAN2 of PCAN-USB Pro.

## [2.0.2] - 2014-01-08
### Changed
- Change DWORD definition into pcanbasic_jni.c (same reason as above).

## [2.0.1] - 2013-11-12
### Changed
- Change all "c_ulong" types into "c_uint" to ensure 32-bits implementation whatever Linux arch data model is.

## [2.0.0] - 2013-11-08
### Changed
- Fix problem running in 32-bits applications with 64-bits Kernel in python and C++ examples.

## [1.0.0] - 2013-11-05
### Changed
- Include version number in PCANBAsic.h and remove TPCANMsg struc definition to use the one in <pcan.h>.
- Examples setup now an initial baudrate to 500K.
  
  
