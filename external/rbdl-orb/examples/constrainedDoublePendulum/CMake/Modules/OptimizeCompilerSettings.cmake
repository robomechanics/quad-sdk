# File:   OptimizeCompilerSettings.cmake
# Author: Christian Hoffmann
# Date:   2007--2008
#
# This file is part of the MUSCOD/VPLAN suite, which is proprietary software of
#   Simulation and Optimization Workgroup
#   Interdisciplinary Center for Scientific Computing (IWR)
#   University of Heidelberg, Germany.
#
# Copyright (C) 2007--2008. All rights reserved.
#
####################################################################################################
#
# Modifies CMAKE's compiler-related variables to activate certain compiler-specific optimizations
#
####################################################################################################

IF( NOT _OPTIMIZECOMPILERSETTINGS_ )
SET( _OPTIMIZECOMPILERSETTINGS_ TRUE )

# Activate inter-module optimization and use pipes instead of temporary files
# See http://gcc.gnu.org/onlinedocs/gcc-4.1.2/gcc/Overall-Options.html#Overall-Options
# SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -combine -pipe" CACHE STRING "" FORCE )
# SET( CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -combine -pipe" CACHE STRING "" FORCE )
# SET( CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -combine -pipe" CACHE STRING "" FORCE )
# SET( CMAKE_CXX_FLAGS_MINSIZEREL "${CMAKE_CXX_FLAGS_MINSIZEREL} -combine -pipe" CACHE STRING "" FORCE )
# SET( CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -combine -pipe" CACHE STRING "" FORCE )
# SET( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -combine -pipe" CACHE STRING "" FORCE )
# SET( CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -combine -pipe" CACHE STRING "" FORCE )
# SET( CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -combine -pipe" CACHE STRING "" FORCE )
# SET( CMAKE_C_FLAGS_MINSIZEREL "${CMAKE_C_FLAGS_MINSIZEREL} -combine -pipe" CACHE STRING "" FORCE )
# SET( CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELWITHDEBINFO} -combine -pipe" CACHE STRING "" FORCE )

ENDIF()

