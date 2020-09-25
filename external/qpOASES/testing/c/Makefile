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
##	Filename:  testing/c/Makefile
##	Author:    Hans Joachim Ferreau
##	Version:   3.2
##	Date:      2014-2017
##



include ../../make.mk

##
##	flags
##

IFLAGS      =  -I. \
               -I${IDIR} \
			   -I${TOP}/interfaces/c

QPOASES_TEST_EXES = \
	${BINDIR}/test_c_example1${EXE} \
	${BINDIR}/test_c_example1a${EXE} \
	${BINDIR}/test_c_example1b${EXE}


##
##	targets
##

all: ${QPOASES_TEST_EXES}

${BINDIR}/%${EXE}: %.${OBJEXT} ${LINK_DEPENDS_WRAPPER}
	@${ECHO} "Creating" $@
	@${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${QPOASES_LINK_WRAPPER} ${LINK_LIBRARIES_WRAPPER}


clean:
	@${ECHO} "Cleaning up (testing/cpp)"
	@${RM} -f *.${OBJEXT} ${QPOASES_TEST_EXES}

clobber: clean


${LINK_DEPENDS_WRAPPER}:
	@cd ../..; ${MAKE} -s c_wrapper

%.${OBJEXT}: %.c
	@${ECHO} "Creating" $@
	@${CC} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} $<


##
##	end of file
##
