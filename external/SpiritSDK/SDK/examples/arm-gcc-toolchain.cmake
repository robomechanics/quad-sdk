
# Copyright (C) Ghost Robotics - All Rights Reserved
# Unauthorized copying of this file, via any medium is strictly prohibited
# Proprietary and confidential
# Written by Avik De <avik@ghostrobotics.io>
# Based on https://github.com/vpetrigo/arm-cmake-toolchains

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR ARM)

set(TOOLCHAIN_PREFIX arm-none-eabi-)

# Absolute path to arm-non-eabi-gcc compiler, which should be a user environment variable
set(BINUTILS_PATH $ENV{ARM_GCC_PATH}) 

if (${TOOLCAIN_PREFIX} MATCHES "arm-none-eabi-")
	# Without that flag CMake is not able to pass test compilation check
	set(CMAKE_EXE_LINKER_FLAGS_INIT "--specs=nosys.specs")
endif()

IF(${CMAKE_VERSION} VERSION_LESS 3.6.0)
	INCLUDE(CMakeForceCompiler)
	CMAKE_FORCE_C_COMPILER(${TOOLCHAIN_PREFIX}gcc GNU)
	CMAKE_FORCE_CXX_COMPILER(${TOOLCHAIN_PREFIX}g++ GNU)
ELSE()
	SET(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
	set(CMAKE_C_COMPILER ${TOOLCHAIN_PREFIX}gcc)
	set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}g++)
ENDIF()
set(CMAKE_ASM_COMPILER ${CMAKE_C_COMPILER})

set(CMAKE_OBJCOPY ${TOOLCHAIN_PREFIX}objcopy CACHE INTERNAL "objcopy tool")
set(CMAKE_SIZE_UTIL ${TOOLCHAIN_PREFIX}size CACHE INTERNAL "size tool")

set(CMAKE_FIND_ROOT_PATH ${BINUTILS_PATH})
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

SET(CMAKE_ASM_COMPILE_OBJECT "${CMAKE_ASM_COMPILER} <FLAGS> -c -o <OBJECT> <SOURCE>") 
