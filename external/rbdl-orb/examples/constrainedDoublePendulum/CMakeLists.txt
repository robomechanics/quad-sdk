CMAKE_MINIMUM_REQUIRED ( VERSION 2.6 )

PROJECT( "constrainedDoublePendulumForwardDynamics" CXX C )
SET(CUSTOM_RBDL_PATH "" CACHE PATH "Path to specific RBDL Installation")

SET(TARGETS   
  constrainedDoublePendulumForwardDynamics
  CACHE STRING
  "Target to build" 
)

LIST( APPEND CMAKE_MODULE_PATH
	${PROJECT_SOURCE_DIR}/CMake
	${PROJECT_SOURCE_DIR}/CMake/Modules)

FIND_PACKAGE (RBDL COMPONENTS LUAMODEL REQUIRED)
FIND_PACKAGE (Lua51 REQUIRED)
FIND_PACKAGE (Eigen3 3.0.0 )


SET(BOOST_INC_DIR
  /usr/include
  CACHE STRING
  "Location of the BOOST header files")


INCLUDE_DIRECTORIES ( ${RBDL_INCLUDE_DIR} 
	                    ${EIGEN3_INCLUDE_DIR}
	                    ${LUA_INCLUDE_DIR}
	                    ${BOOST_INC_DIR} )

if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-long-long -pedantic -std=c++11 -fext-numeric-literals")
endif()


ADD_EXECUTABLE ( ${TARGETS} 
	src/${TARGETS}.cc
	src/csvtools.cc
	)

TARGET_LINK_LIBRARIES ( ${TARGETS}
	${RBDL_LIBRARY}
	${RBDL_LUAMODEL_LIBRARY}
	${LUA_LIBRARIES}
	)


message("CUSTOM_RBDL_PATH         :" ${CUSTOM_RBDL_PATH})
message("RBDL_INCLUDE_DIR         :" ${RBDL_INCLUDE_DIR})
message("RBDL_LIBRARY             :" ${RBDL_LIBRARY})
message("RBDL_LUAMODEL_LIBRARY    :" ${RBDL_LUAMODEL_LIBRARY})
message("LUA_LIBRARIES            :" ${LUA_LIBRARIES})
message("EIGEN3_INCLUDE_DIR       :" ${EIGEN3_INCLUDE_DIR})

