#2a Give the project to build a name 
PROJECT (pendulumDynamics)

#2b. Specify a minimum version of CMake to use: this is important! When you
#        write your own CMakeLists.txt files insert you own version number 
#        of CMake here.

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

#2c. Assign the names of the programs/libraries that you wish to build
#    to a list called 'TARGETS'

SET(TARGETS  	
	pendulumForwardDynamics
	CACHE STRING
	"Target to build"	
)
LIST(REMOVE_DUPLICATES TARGETS)


#    2d. Add the folder 'CMake' & 'CMake/Modules' to be included: these contain
#        project-specific CMake files. If you open the folder 'CMake' you 
#        should find two files which have been written to find Eigen3 and 
#        RBDL with the following names.
#
#          FindEigen3.cmake  
#          FindRBDL.cmake

LIST( APPEND CMAKE_MODULE_PATH
  ${PROJECT_SOURCE_DIR}/CMake)



#    2e. Tell CMake what packages are needed to build this software and 
#        have it try to find these dependencies. The output of this process
#        is several variables in CMake that define where the libraries and
#        header files are located for each package. To see what these 
#        variables are called you need to look at the 'Find ... .cmake' file
#        for this package, or look for the package control (*.pc file) for this 
#        package. A bit more on this later.
#
#        -FIND_PACKAGE( )
#         This is a command that CMake provides which will search likely, and
#         recommended locations, for a specific library in a manner that works
#         on any operating system. When possible use these commands exclusively.
#
#        -SET(CUSTOM_RBDL_PATH "" CACHE PATH "Path to specific RBDL Installation")
#         The variable CUSTOM_RBDL_PATH is used in combination with 
#         CMake/FindRBDL.cmake to find a specific installation of RBDL

SET(CUSTOM_RBDL_PATH "" CACHE PATH "Path to specific RBDL Installation")

FIND_PACKAGE ( RBDL COMPONENTS LUAMODEL)
FIND_PACKAGE (Eigen3 3.0.0 REQUIRED)
FIND_PACKAGE (Lua51 REQUIRED)


#    2f. Tell CMake where the header files (these are folders that have header 
#        files). Here there is a mixture of methods used:
#
#        INCLUDE_DIRECTORIES ( 
#          ${RBDL_INCLUDE_DIR} 
#          ${EIGEN3_INCLUDE_DIR} 
#          ${LUA_INCLUDE_DIR}
#          ${BOOST_INC_DIR} 
#        )        
#
#        The variables RBDL_INCLUDE_DIR and EIGEN3_INCLUDE_DIR are variables
#        that are set by the CMake/FindRBDL.cmake and CMake/FindEigen3.cmake 
#        files. To know what variables you have to work with you need to open
#        these files and have a look at them.
#
#        The variable BOOST_INC_DIR is manually set, and hardcoded to /usr/local.
#        While this is quick and easy to do it is not recommended: the person
#        using this CMake file may have put the Boost installation somewhere
#        else. Further, this path is not cross-platform: Windows does not have
#        a /usr/local file. If this is running on your own machine it does not
#        matter a great deal what you do. If you want to contribute this to
#        a cross-platform code this would be a big problem.

SET(BOOST_INC_DIR
  /usr/include
  CACHE STRING
  "Location of the BOOST header files")

INCLUDE_DIRECTORIES ( 
	${RBDL_INCLUDE_DIR} 
	${EIGEN3_INCLUDE_DIR} 
	${LUA_INCLUDE_DIR}
  ${BOOST_INC_DIR} 
)


#    2g. Tell CMake what executable(s) you want to build (the Targets naturally)
#        and what files are needed to build the software
#
ADD_EXECUTABLE(${TARGETS} 
               src/${TARGETS}.cc 
               src/csvtools.h 
               src/csvtools.cc)	  
	
#    2h. Tell CMake what libraries are used at runtime. Here the pendulum makes
#        use of 4 libraries (*.so files on Ubuntu): the library of the pendulum,
#        the main rbdl library, the rbdl-lua library, and the lua libraries. 
#        If you open /orbGitHubCode/rbdl-orb-release-install/lib you can see
#        the various libraries that were made during the build process
#
#          librbdl_geometry.so        
#          librbdl_luamodel.so.2.6.0  
#          librbdl.so
#          librbdl_geometry.so.2.6.0  
#          librbdl_muscle.so          
#          librbdl.so.2.6.0
#          librbdl_luamodel.so        
#          librbdl_muscle.so.2.6.0    
#
#        If you look closely at the sizes and file types you should see that
#        some files are actual libraries (e.g. librbdl_geometry.so.2.6.0) while 
#        others are just links to libraries (e.g. librbdl_geometry.so) which 
#        take up much less memory.
#
#        As before the variables RBDL_LIBRARY, RBDL_LUAMODEL_LIBRARY, and 
#        LUA_LIBRARIES are all variables that have been set in the respective
#        'Find ... .cmake' files that exist for each of these packages

TARGET_LINK_LIBRARIES ( 
	${TARGETS}
	${RBDL_LIBRARY}
	${RBDL_LUAMODEL_LIBRARY}
	${LUA_LIBRARIES}	
	)


#    2i. Optional: Echo information back to the user. I have many versions of 
#        RBDL on my machine. To ensure that CMake is using the correct library
#        here the paths to the include files and libraries of RBDL are printed
#        to the screen.

message("CUSTOM_RBDL_PATH         :" ${CUSTOM_RBDL_PATH})
message("RBDL_INCLUDE_DIR         :" ${RBDL_INCLUDE_DIR})
message("RBDL_LIBRARY             :" ${RBDL_LIBRARY})
message("RBDL_LUAMODEL_LIBRARY    :" ${RBDL_LUAMODEL_LIBRARY})
message("LUA_LIBRARIES            :" ${LUA_LIBRARIES})
message("EIGEN3_INCLUDE_DIR       :" ${EIGEN3_INCLUDE_DIR})
