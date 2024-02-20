cmake_minimum_required(VERSION 3.10)
project(moteus_driver)

add_compile_options(-lmraa)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(PkgConfig REQUIRED)
pkg_check_modules(mraa QUIET mraa)

if(NOT "${mraa_FOUND}")
  string(ASCII 27 Esc)
  message(WARNING "${Esc}[1;33m MRAA LIB NOT FOUND !!!${Esc}[m")
endif()

set(CAN_LIBS pthread rt pcanbasic)

include_directories(
  include
  (${PROJECT_SOURCE_DIR}/mraa_api)
  (${PROJECT_SOURCE_DIR}/mraa_api/mraa)
  ${catkin_INCLUDE_DIRS}
  ${mraa_INCLUDE_DIRS}
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

file(GLOB MOTEUS_SRC src/*.cpp)

add_library(moteus_driver ${MOTEUS_SRC})
target_link_libraries(moteus_driver ${CAN_LIBS} ${catkin_LIBRARIES} mraa stdc++)
