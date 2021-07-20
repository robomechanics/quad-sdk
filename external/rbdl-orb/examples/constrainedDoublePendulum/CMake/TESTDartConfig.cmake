# File     MUSCOD_TEST/CMake/DartConfig.txt
# Author   Christian Hoffmann
# Date     2007
#
# This file is part of the MUSCOD package. MUSCOD is proprietary software of
#   Simulation and Optimization Workgroup
#   Interdisciplinary Center for Scientific Computing (IWR)
#   University of Heidelberg, Germany.
#
####################################################################################################
#
# Configuration of SimOpts DART server for MUSCOD_TEST
# More infos about CMake/CTest: http://www.cmake.org
#
####################################################################################################

SET( NIGHTLY_START_TIME "21:00:00 UTC" )

SET( DROP_METHOD xmlrpc )
SET( DROP_SITE "http://liz.iwr.uni-heidelberg.de:8081" )
# SET( DROP_SITE_USER "")
# SET( DROP_SITE_PASSWORD "")
# SET( DROP_SITE_MODE "")
SET( DROP_LOCATION "MUSCOD-II" )
SET( COMPRESS_SUBMISSION ON )
# SET(TRIGGER_SITE "")
