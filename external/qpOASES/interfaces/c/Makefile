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
##	Filename:  interfaces/c/Makefile
##	Author:    Hans Joachim Ferreau
##	Version:   3.2
##	Date:      2014-2017
##


include ../../make.mk

##
##	flags
##

IFLAGS =	-I. \
		-I${IDIR} \
		-I${SRCDIR}

QPOASES_WRAPPER_OBJECTS = \
	${SRCDIR}/SQProblem.${OBJEXT} \
	${SRCDIR}/QProblem.${OBJEXT} \
	${SRCDIR}/QProblemB.${OBJEXT} \
	${SRCDIR}/Bounds.${OBJEXT} \
	${SRCDIR}/Constraints.${OBJEXT} \
	${SRCDIR}/SubjectTo.${OBJEXT} \
	${SRCDIR}/Indexlist.${OBJEXT} \
	${SRCDIR}/Flipper.${OBJEXT} \
	${SRCDIR}/Utils.${OBJEXT} \
	${SRCDIR}/Options.${OBJEXT} \
	${SRCDIR}/Matrices.${OBJEXT} \
	${SRCDIR}/MessageHandling.${OBJEXT} \
	qpOASES_wrapper.${OBJEXT}

QPOASES_DEPENDS = \
	${IDIR}/qpOASES.hpp \
	${IDIR}/qpOASES/LapackBlasReplacement.hpp \
	${IDIR}/qpOASES/SQProblem.hpp \
	${IDIR}/qpOASES/QProblem.hpp \
	${IDIR}/qpOASES/Flipper.hpp \
	${IDIR}/qpOASES/QProblemB.hpp \
	${IDIR}/qpOASES/Bounds.hpp \
	${IDIR}/qpOASES/Constraints.hpp \
	${IDIR}/qpOASES/SubjectTo.hpp \
	${IDIR}/qpOASES/Indexlist.hpp \
	${IDIR}/qpOASES/Utils.hpp \
	${IDIR}/qpOASES/Constants.hpp \
	${IDIR}/qpOASES/Types.hpp \
	${IDIR}/qpOASES/Options.hpp \
	${IDIR}/qpOASES/Matrices.hpp \
	${IDIR}/qpOASES/MessageHandling.hpp \
	qpOASES_wrapper.h

QPOASES_C_EXAMPLES = \
	${BINDIR}/c_example1${EXE} \
	${BINDIR}/c_example1a${EXE} \
	${BINDIR}/c_example1b${EXE}


##
##	targets
##

all: ${LINK_DEPENDS_WRAPPER} ${QPOASES_C_EXAMPLES}


${BINDIR}/libqpOASES_wrapper.${LIBEXT}: ${QPOASES_WRAPPER_OBJECTS}
	@${ECHO} "Creating" $@
	@${AR} r $@ qpOASES_wrapper.${OBJEXT}

${BINDIR}/libqpOASES_wrapper.${DLLEXT}: ${QPOASES_WRAPPER_OBJECTS}
	@${ECHO} "Creating" $@
	${CC} ${SHARED} ${DEF_TARGET} qpOASES_wrapper.${OBJEXT} ${LINK_LIBRARIES_WRAPPER}

clean:
	@${ECHO} "Cleaning up (interfaces/c)"
	@${RM} -f *.${OBJEXT} ${BINDIR}/*.${LIBEXT} ${BINDIR}/*.${DLLEXT} ${BINDIR}/c_example*${EXE}

clobber: clean


%.${OBJEXT}: %.cpp ${QPOASES_DEPENDS}
	@${ECHO} "Creating" $@
	@${CPP} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} -D__C_WRAPPER__ $<

c_example1.${OBJEXT}: c_example1.c ${LINK_DEPENDS_WRAPPER}
	@${ECHO} "Creating" $@
	@${CC} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} $<

c_example1a.${OBJEXT}: c_example1a.c ${LINK_DEPENDS_WRAPPER}
	@${ECHO} "Creating" $@
	@${CC} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} $<

c_example1b.${OBJEXT}: c_example1b.c ${LINK_DEPENDS_WRAPPER}
	@${ECHO} "Creating" $@
	@${CC} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} $<


${BINDIR}/%${EXE}: %.${OBJEXT} ${LINK_DEPENDS_WRAPPER}
	@${ECHO} "Creating" $@
	${CC} ${DEF_TARGET} ${IFLAGS} ${CPPFLAGS} $< ${QPOASES_LINK_WRAPPER} ${LINK_LIBRARIES_WRAPPER}


##
##	end of file
##
