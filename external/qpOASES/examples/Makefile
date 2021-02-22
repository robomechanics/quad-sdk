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
##	Filename:  examples/Makefile
##	Author:    Hans Joachim Ferreau
##	Version:   3.2
##	Date:      2007-2017
##

include ../make.mk

##
##	flags
##

IFLAGS      =  -I. \
               -I${IDIR}

QPOASES_EXES = \
	${BINDIR}/example1${EXE} \
	${BINDIR}/example1a${EXE} \
	${BINDIR}/example1b${EXE} \
	${BINDIR}/example2${EXE} \
	${BINDIR}/example3${EXE} \
	${BINDIR}/example3b${EXE} \
	${BINDIR}/example4${EXE} \
	${BINDIR}/example5${EXE} \
	${BINDIR}/exampleLP${EXE} \
	${BINDIR}/qrecipe${EXE} \
	${BINDIR}/qrecipeSchur${EXE}


##
##	targets
##

all: ${QPOASES_EXES}

${BINDIR}/%${EXE}: %.${OBJEXT} ${LINK_DEPENDS}
	@${ECHO} "Creating" $@
	@${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${QPOASES_LINK} ${LINK_LIBRARIES}

${BINDIR}/example4${EXE}: example4.${OBJEXT} example4CP.cpp ${LINK_DEPENDS}
	@${ECHO} "Creating" $@
	@${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${QPOASES_LINK} ${LINK_LIBRARIES}

${BINDIR}/qrecipe${EXE}: qrecipe.${OBJEXT} qrecipe_data.hpp ${LINK_DEPENDS}
	@${ECHO} "Creating" $@
	@${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${QPOASES_LINK} ${LINK_LIBRARIES}

${BINDIR}/qrecipeSchur${EXE}: qrecipeSchur.${OBJEXT} qrecipe_data.hpp ${LINK_DEPENDS}
	@${ECHO} "Creating" $@
	@${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${QPOASES_LINK} ${LINK_LIBRARIES}


clean:
	@${ECHO} "Cleaning up (examples)"
	@${RM} -f *.${OBJEXT} ${QPOASES_EXES}

clobber: clean


${LINK_DEPENDS}:
	@cd ..; ${MAKE} -s src

example4.${OBJEXT}: example4.cpp example4CP.cpp
	@${ECHO} "Creating" $@
	@${CPP} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} $<

qrecipe.${OBJEXT}: qrecipe.cpp qrecipe_data.hpp
	@${ECHO} "Creating" $@
	@${CPP} ${DEF_TARGET} ${IFLAGS} ${CPPFLAGS} -c $<

qrecipeSchur.${OBJEXT}: qrecipeSchur.cpp qrecipe_data.hpp
	@${ECHO} "Creating" $@
	@${CPP} ${DEF_TARGET} ${IFLAGS} ${CPPFLAGS} -c $<

%.${OBJEXT}: %.cpp
	@${ECHO} "Creating" $@
	@${CPP} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} $<


##
##	end of file
##
