cmake_minimum_required(VERSION 3.5)
project(moteus_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CAN_LIBS pthread rt pcanbasic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

file(GLOB MOTEUS_SRC src/*.cpp)

add_library(moteus_driver ${MOTEUS_SRC})
target_link_libraries(moteus_driver ${CAN_LIBS})
