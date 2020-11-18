
# Copyright (C) Ghost Robotics - All Rights Reserved
# Unauthorized copying of this file, via any medium is strictly prohibited
# Proprietary and confidential
# Written by Avik De <avik@ghostrobotics.io> and Tom Jacobs <tom.jacobs@ghostrobotics.io>

# Description: Builds SDK example LimbControl for Spirit or Vision robots ("main.bin"), to be uploaded to the robot's mainboard.
# See LimbControl/README.md for use.

cmake_minimum_required(VERSION 3.10)

set(PLATFORM "mb" CACHE STRING "mb or gazebo")
set(ROBOT "NGR" CACHE STRING "robot") # NGR (Vision 60, "Next Generation Robot"), or SPIRIT

if (${PLATFORM} MATCHES "mb")
  set(TOOLCHAIN_PREFIX arm-none-eabi-)
  set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/arm-gcc-toolchain.cmake)
endif()

project(main)

# Set this
set(GHOST_SDK_DIR "${CMAKE_SOURCE_DIR}/..")
link_directories(${GHOST_SDK_DIR}/lib)
include_directories(${GHOST_SDK_DIR}/inc)

# Set a default build type if none was specified
set(default_build_type "Release")

# We need c99
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_C_FLAGS "-std=gnu99")
set(CMAKE_CXX_FLAGS "-std=gnu++11")
set(CMAKE_C_FLAGS_DEBUG "-Og -g")
set(CMAKE_C_FLAGS_RELEASE "-O1")
set(CMAKE_CXX_FLAGS_DEBUG "-Og -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O1")
MESSAGE( STATUS "CMAKE_CROSSCOMPILING " ${CMAKE_CROSSCOMPILING} )

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# Libraries
include_directories(
  ${GHOST_SDK_DIR}/thirdparty/nanopb
  ${GHOST_SDK_DIR}/thirdparty/eigen
  )

if (${PLATFORM} MATCHES "mb")
  # Compile flags
  set(MCU "-mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard")
  add_compile_definitions(USE_HAL_DRIVER STM32H743xx)
  set(BFLAGS "${MCU} ${OPT} -Wall -fdata-sections -ffunction-sections -fsingle-precision-constant -MMD -MP" CACHE INTERNAL "")
  set(CMAKE_C_FLAGS ${BFLAGS})
  set(CMAKE_CXX_FLAGS ${BFLAGS})
  if (CMAKE_BUILD_TYPE MATCHES "Debug")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -gdwarf-2")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -gdwarf-2")
  endif()

  set(CMAKE_EXE_LINKER_FLAGS "-specs=nano.specs  -u _printf_float -T${GHOST_SDK_DIR}/lib/STM32H743ZITx_FLASH.ld -Wl,-Map=mb80.map,--cref -Wl,--gc-sections" CACHE INTERNAL "")
  # HAL_LIB empty for others
  set(HAL_LIB -Wl,--whole-archive mb8xhal -Wl,--no-whole-archive)
  link_directories(${GHOST_SDK_DIR}/src/build ${GHOST_SDK_DIR}/src/build/mcu/mb8x) # testing latest
  
elseif (${PLATFORM} MATCHES "gazebo")
  file(GLOB SRC_PLATFORM "${CMAKE_SOURCE_DIR}/Gazebo/*.cpp")
  # use updated SDK in this dir for gazebo
  set(GHOST_SDK_DIR /home/kodlab/ghost_sdk)
  include_directories(${GHOST_SDK_DIR}/inc ${GHOST_SDK_DIR}/thirdparty/nanopb)
  link_directories(${GHOST_SDK_DIR}/lib)
  
  set(ROS_DIR /opt/ros/melodic)
  link_directories(${ROS_DIR}/lib)
  link_libraries(gazebo roscpp rosconsole roscpp_serialization rostime)
  include_directories(${ROS_DIR}/include /usr/include/eigen3)
  add_definitions(-DARCH_gazebo -DNDEBUG)
  
  find_package(Boost COMPONENTS system filesystem REQUIRED)
endif()

# Build LimbControl

# Modular function to add a SDK-using sketch to the build ---
function(add_sketch exec_base_name sketch_dir robot_name robot_version startup_delay)
  # gather files
  file(GLOB src_sketch "${sketch_dir}/*.cpp" "${sketch_dir}/*.c")
  set(exec_name "${exec_base_name}_${PLATFORM}_${robot_name}_${robot_version}")
  add_executable(${exec_name} ${src_sketch})
  # Defines used by the robot to set robot options using SDK function
  target_compile_definitions(${exec_name} PRIVATE -DROBOT_${robot_name} -DROBOT_VERSION=${robot_version} -DSTARTUP_DELAY=${startup_delay})

  if (${PLATFORM} MATCHES "mb")
    set(liblinkwa -Wl,--whole-archive mb8x -Wl,--no-whole-archive)
    target_link_libraries(${exec_name} PRIVATE ${libname} ${HAL_LIB})
    target_link_libraries(${exec_name} PRIVATE ${liblinkwa} -larm_cortexM4lf_math)

    add_custom_command(TARGET ${exec_name} POST_BUILD 
      COMMAND ${TOOLCHAIN_PREFIX}size ${exec_name}
      COMMAND ${TOOLCHAIN_PREFIX}objcopy -O ihex ${exec_name} ${exec_name}.bin
      COMMAND ${TOOLCHAIN_PREFIX}objcopy -O binary -S ${exec_name} ${exec_name}.bin
      COMMENT "Post-build"
    )
  elseif (${PLATFORM} MATCHES "gazebo")
    target_link_libraries(${exec_name} ${Boost_FILESYSTEM_LIBRARY})
    target_link_libraries(${exec_name} ${Boost_SYSTEM_LIBRARY})
    target_link_libraries(${exec_name} Threads::Threads)
    target_link_libraries(${exec_name} libgazebo.a)
  endif()
endfunction()

function(add_sketch_all exec_base_name sketch_dir)
  add_sketch(${exec_base_name} ${sketch_dir} VISION 0x37 0)
  add_sketch(${exec_base_name} ${sketch_dir} VISION 0x40 4000)
  add_sketch(${exec_base_name} ${sketch_dir} SPIRIT 0x12 7000)
endfunction()

add_sketch_all(FirstStep "${CMAKE_SOURCE_DIR}/FirstStep")
add_sketch_all(LimbControl "${CMAKE_SOURCE_DIR}/LimbControl")
add_sketch_all(read-write-mainboard "${CMAKE_SOURCE_DIR}/read-write-mainboard")
add_sketch_all(MAVLinkSpeedTest "${CMAKE_SOURCE_DIR}/MAVLinkSpeedTest")
#add_sketch_all(SpinMotor "${CMAKE_SOURCE_DIR}/SpinMotor")
