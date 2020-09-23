cmake_minimum_required( VERSION 3.1 )

find_package( Boost REQUIRED COMPONENTS program_options )

add_executable( interp-cli interp-cli.cpp )
target_link_libraries( interp-cli Interpolate Boost::program_options )
set_property( TARGET interp-cli PROPERTY CXX_STANDARD 11 )

add_subdirectory( .. libInterpolate )
configure_file( ../testing/CramTests/interp-cli.t ${CMAKE_CURRENT_BINARY_DIR}/interp-cli.t COPYONLY )
