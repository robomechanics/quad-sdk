# Changelog
All notable changes to "libpcanbasic_jni" will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).

## [4.7.0] - 2022-01-24
### Updated
- Updated copyright

## [4.6.1] - 2022-05-10
### Updated
- added checks on FindClass(...) calls, an exception is thrown when expected class is not found.

## [4.6.0] - 2022-03-25
### Updated
- API compatible with PCAN-Basic v4.6

## [4.5.1] - 2022-03-23
### Updated
- Fixed issue with captured SIGINT signal: signals are not used anymore.

## [4.5.0] - 2022-01-05
### Updated
- API forward compatible with PCAN-Basic Java 4.6

## [4.5.0] - 2021-08-26
### Updated
- Upgraded library to support new features from PCAN-Basic v4.5 (CAN_LookUpChannel)

## [4.4.1] - 2021-07-23
### Updated
- Fixed issue with TPCANTimestamp on ARM architecture: cast in setMillis changed from jint to jlong 
- Reminder: java TPCANTimestamp.millis is of type “long” to prevent overflow (instead of uint32_t as unsigned type doesn't exist)

## [4.4.0] - 2020-08-07
### Added
- Added support to Get/SetValue parameter where buffer's type is long (64 bit integer).
- Added support to Get/SetValue parameter where buffer's type is TPCANChannelInformation.
### Changed
- Minor changes in debugging functions

## [1.9.0.3] - 2018-03-03
### Changed
- Fixed CAN FD timestamp issue (value was truncated after some time).

# Changelog history from PCAN-Basic Linux Package 
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
  
  
