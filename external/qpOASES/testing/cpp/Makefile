##
##	This file is part of qpOASES.
##
##	qpOASES -- An Implementation of the Online Active Set Strategy.
##	Copyright (C) 2007-2017 by Hans Joachim Ferreau, Andreas Potschka,
##	Christian Kirches et al. All rights reserved.
##
##	qpOASES is free software; you can redistribute it and/or
##	modify it under the terms of the GNU Lesser General Public
##	License as published by the Free Software Foundation; either
##	version 2.1 of the License, or (at your option) any later version.
##
##	qpOASES is distributed in the hope that it will be useful,
##	but WITHOUT ANY WARRANTY; without even the implied warranty of
##	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
##	See the GNU Lesser General Public License for more details.
##
##	You should have received a copy of the GNU Lesser General Public
##	License along with qpOASES; if not, write to the Free Software
##	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
##



##
##	Filename:  testing/cpp/Makefile
##	Author:    Hans Joachim Ferreau
##	Version:   3.2
##	Date:      2007-2017
##


include ../../make.mk

##
##	flags
##

IFLAGS      =  -I. \
               -I${IDIR}

QPOASES_TEST_EXES = \
	${BINDIR}/test_bench${EXE} \
	${BINDIR}/test_matrices${EXE} \
	${BINDIR}/test_matrices2${EXE} \
	${BINDIR}/test_matrices3${EXE} \
	${BINDIR}/test_indexlist${EXE} \
	${BINDIR}/test_example1${EXE} \
	${BINDIR}/test_example1a${EXE} \
	${BINDIR}/test_example1b${EXE} \
	${BINDIR}/test_example2${EXE} \
	${BINDIR}/test_example4${EXE} \
	${BINDIR}/test_example5${EXE} \
	${BINDIR}/test_example6${EXE} \
	${BINDIR}/test_example7${EXE} \
	${BINDIR}/test_exampleLP${EXE} \
	${BINDIR}/test_qrecipe${EXE} \
	${BINDIR}/test_qrecipeSchur${EXE} \
	${BINDIR}/test_smallSchur${EXE} \
	${BINDIR}/test_infeasible1${EXE} \
	${BINDIR}/test_hs268${EXE} \
	${BINDIR}/test_gradientShift${EXE} \
	${BINDIR}/test_runAllOqpExamples${EXE} \
	${BINDIR}/test_sebastien1${EXE} \
	${BINDIR}/test_vanBarelsUnboundedQP${EXE} \
	${BINDIR}/test_janick1${EXE} \
	${BINDIR}/test_janick2${EXE} \
	${BINDIR}/test_constraintProduct1${EXE} \
	${BINDIR}/test_constraintProduct2${EXE} \
	${BINDIR}/test_guessedWS1${EXE} \
	${BINDIR}/test_externalChol1${EXE} \
	${BINDIR}/test_identitySqproblem${EXE}


##
##	targets
##

all: ${QPOASES_TEST_EXES}

runTests: ${QPOASES_TEST_EXES}
	@cd .. && ./runUnitTests && ./checkForMemoryLeaks && cd cpp

${BINDIR}/%${EXE}: %.${OBJEXT} ${LINK_DEPENDS}
	@${ECHO} "Creating" $@
	@${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${QPOASES_LINK} ${LINK_LIBRARIES}

${BINDIR}/test_matrices2${EXE}: test_matrices2.${OBJEXT} test_qrecipe_data.hpp ${LINK_DEPENDS}
	@${ECHO} "Creating" $@
	@${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${QPOASES_LINK} ${LINK_LIBRARIES}

${BINDIR}/test_matrices3${EXE}: test_matrices3.${OBJEXT} test_qrecipe_data.hpp ${LINK_DEPENDS}
	@${ECHO} "Creating" $@
	@${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${QPOASES_LINK} ${LINK_LIBRARIES}

${BINDIR}/test_qrecipe${EXE}: test_qrecipe.${OBJEXT} test_qrecipe_data.hpp ${LINK_DEPENDS}
	@${ECHO} "Creating" $@
	@${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${QPOASES_LINK} ${LINK_LIBRARIES}

${BINDIR}/test_qrecipeSchur${EXE}: test_qrecipeSchur.${OBJEXT} test_qrecipe_data.hpp ${LINK_DEPENDS}
	@${ECHO} "Creating" $@
	@${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${QPOASES_LINK} ${LINK_LIBRARIES}


clean:
	@${ECHO} "Cleaning up (testing/cpp)"
	@${RM} -f *.${OBJEXT} ${QPOASES_TEST_EXES}

clobber: clean


${LINK_DEPENDS}:
	@cd ../..; ${MAKE} -s src

test_matrices2.${OBJEXT}: test_matrices2.cpp test_qrecipe_data.hpp
	@${ECHO} "Creating" $@
	@${CPP} ${DEF_TARGET} ${IFLAGS} ${CPPFLAGS} -c $<

test_matrices3.${OBJEXT}: test_matrices3.cpp test_qrecipe_data.hpp
	@${ECHO} "Creating" $@
	@${CPP} ${DEF_TARGET} ${IFLAGS} ${CPPFLAGS} -c $<

test_qrecipe.${OBJEXT}: test_qrecipe.cpp test_qrecipe_data.hpp
	@${ECHO} "Creating" $@
	@${CPP} ${DEF_TARGET} ${IFLAGS} ${CPPFLAGS} -c $<

test_qrecipeSchur.${OBJEXT}: test_qrecipeSchur.cpp test_qrecipe_data.hpp
	@${ECHO} "Creating" $@
	@${CPP} ${DEF_TARGET} ${IFLAGS} ${CPPFLAGS} -c $<

%.${OBJEXT}: %.cpp
	@${ECHO} "Creating" $@
	@${CPP} ${DEF_TARGET} ${IFLAGS} ${CPPFLAGS} -c $<


##
##	end of file
##
