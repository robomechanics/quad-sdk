cmake_minimum_required( VERSION 3.1 )

OPTION( BUILD_UNIT_TESTS "Build unit tests for the library" ON )


if(BUILD_UNIT_TESTS)

set( binDir ${CMAKE_BINARY_DIR}/testBin )
set( buildDir ${CMAKE_BINARY_DIR} )

# Find all sources.
file( GLOB_RECURSE unitTest_SOURCES
      RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
      "./CatchTests/*.cpp" )
message(STATUS "Detected Catch-based Unit Test Sources:")
foreach(testSrc ${unitTest_SOURCES})
  message(STATUS "  ${testSrc}" )
endforeach()

set( testName ${PROJECT_NAME}_CatchTests )
add_executable(${testName} ${unitTest_SOURCES})
target_compile_definitions( ${testName} PRIVATE -DTESTING )
target_link_libraries(${testName} Interpolate)
target_include_directories( ${testName} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include" )
set_target_properties(${testName} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${binDir})
set_target_properties(${testName} PROPERTIES CXX_STANDARD 11)
if(MSVC)
  target_compile_options(${testName} PRIVATE /W4 /WX -wd4996)
else()
  target_compile_options(${testName} PRIVATE -Wall -Wextra -pedantic -Wno-unknown-pragmas -Wno-unused-parameter -Wno-gnu-zero-variadic-macro-arguments -Werror)
endif()

add_test(NAME ${testName} 
         WORKING_DIRECTORY ${buildDir}
         COMMAND ${binDir}/${testName} )

file(COPY data DESTINATION ${buildDir} )

endif()
