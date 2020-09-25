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
##	Filename:  src/interfaces/matlab/Makefile
##	Author:    Christian Kirches, Hans Joachim Ferreau, Andreas Potschka
##	Version:   3.2
##	Date:      2007-2017
##

include ../../make.mk

##
##	flags
##

IDIR   = ../../include

IFLAGS      =  -I. \
               -I${IDIR} \
               -I${MATLAB_IDIR}
			   
LINK_MATLAB_LIBS = -L${MATLAB_LIBDIR} -lmex -lmat -lmx

EXTRAFLAGS = -largeArrayDims -O -D__cpluplus -D__MATLAB__ -fexceptions

QPOASES_OBJECT = qpOASES.${OBJEXT}
QPOASES_SEQUENCE_OBJECT = qpOASES_sequence.${OBJEXT}

##
##	targets
##

all: ${BINDIR}/qpOASES.${MEXEXT} ${BINDIR}/qpOASES_sequence.${MEXEXT} copyMs

${BINDIR}/qpOASES.${MEXEXT}: ${QPOASES_OBJECT} 
	@${ECHO} "Creating" $@
	@${CPP} -shared -o $@ $? ${QPOASES_LINK} ${LINK_LIBRARIES} ${LINK_MATLAB_LIBS}

${BINDIR}/qpOASES_sequence.${MEXEXT}: ${QPOASES_SEQUENCE_OBJECT} 
	@${ECHO} "Creating" $@
	@${CPP} -shared -o $@ $? ${QPOASES_LINK} ${LINK_LIBRARIES} ${LINK_MATLAB_LIBS}

copyMs:
	@${ECHO} "Copying M files ..."
	@${CP} qpOASES.m ${BINDIR}
	@${CP} qpOASES_options.m ${BINDIR}
	@${CP} qpOASES_sequence.m ${BINDIR}

#@${CP} qpOASES_sequenceVM.m ${BINDIR}
	
clean:
	@${RM} -f *.${OBJEXT} *.mex*

clobber: clean

%.${OBJEXT}: %.cpp 
	@${ECHO} "Creating" $@
	@${CPP} ${DEF_TARGET} -c ${IFLAGS} ${EXTRAFLAGS} ${CPPFLAGS} $<


##
##	end of file
##
