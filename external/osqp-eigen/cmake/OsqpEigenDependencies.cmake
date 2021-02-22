# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.
#
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license. See the accompanying LICENSE file for details.

include(OsqpEigenFindOptionalDependencies)

#---------------------------------------------
## Required Dependencies
find_package(Eigen3 3.2.92 REQUIRED)
find_package(osqp REQUIRED)

#---------------------------------------------
## Optional Dependencies
find_package(Catch2 QUIET)
checkandset_optional_dependency(Catch2)

find_package(VALGRIND QUIET)
checkandset_optional_dependency(VALGRIND)
