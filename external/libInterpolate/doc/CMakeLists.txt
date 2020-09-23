cmake_minimum_required( VERSION 2.8 )

# DOXYGEN
find_package(Doxygen)

# build the doxygen-based documentation
if(DOXYGEN_FOUND)

# look for doxygen config file
file( GLOB_RECURSE DOXYGEN_CONFIG
      RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
      "*.doxygen" )
    if( "${DOXYGEN_CONFIG}" STREQUAL "" )
      set( DOXYGEN_CONFIG "${PROJECT_NAME}.doxygen" )
      message(STATUS  "Did not find Doxygen configuration file.")
      message(STATUS  "  Creating template named ${DOXYGEN_CONFIG}.template")
      message(STATUS  "  Please rename the template file to ${DOXYGEN_CONFIG} and edit it for your needs.")
      message(WARNING "  ${DOXYGEN_CONFIG}.template will be OVERWRITTEN on the next run.")
      exec_program( ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}
                    ARGS -g ${DOXYGEN_CONFIG} )
    else()
      message(STATUS "Found Doxygen configuration file: ${DOXYGEN_CONFIG}")
    endif()


    add_custom_target(doc ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/${DOXYGEN_CONFIG}
                  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
                  COMMENT "Generating API documentation with Doxygen" VERBATIM)
endif()
