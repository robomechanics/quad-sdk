# Changelog
All notable changes to "pcaninfo" will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).

## [1.3.1] - 2022-03-21
### Changed
- Changed format ouput (hex) for pcaninfo fields: base, btr0btr1, devid, init_flags.
### Updated 
- When parsing a sysfs file fails, log displays file name and path.

## [1.3.0] - 2021-10-28
### Added 
- Option "all" displays adapter part number (if available).

## [1.2.0] - 2020-12-16
### Added 
- Outputs version of the installed PCAN-Basic API.

## [1.1.0] - 2020-08-07
### Changed
- Default behaviour now displays a short list of available devices.
### Added
- Option "all" (-a) outputs a detailed listing of PCAN devices.
- Option "list" (-l) outputs a short listing of PCAN devices with their PCAN-Basic handle and device id.
- Search filter now support PCAN-Basic handle name.
- A message is displayed if no device is detected.

## [1.0.x] - 2020-00-00
### Changed
- Sources files header now explicitely gives usage license (LGPL v2.1)

## [1.0.3] - 2019-06-17
### Added
- Filter based on device path.
### Changed 
- Makefile: Fixed CC Makefile variable to enable libpcanbasic cross-compilation
- Makefile: Remove useless -Wshadow from CFLAGS
- Makefile: Remove useless quotes around VERSION_FILE definition
- Makefile: merge of standalone pcanbasic and pcan package
