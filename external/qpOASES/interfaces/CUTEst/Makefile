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
##	Filename:  cutestDriver/Makefile
##	Author:    Dennis Janka
##	Version:   3.2
##	Date:      2015-2017
##

# include directories, relative
IDIR = ../../include
SRCDIR = ../../src
BINDIR = ../../bin
CUTESTLIBPATH = ${CUTEST}/objects/${MYARCH}/double

# don't change that (for now) ...
PROBLEMPATH = ./prob

# file extensions
CPP = g++
CPPFLAGS = -Wall -pedantic -Wshadow -O0 -finline-functions -fPIC -DLINUX -g
FF = gfortran
FFLAGS =

#link against CUTEst library, qpOASES library and compiled test problem
LINKOPTS = -L${CUTESTLIBPATH} -L${PROBLEMPATH} -Wl,-rpath=${PROBLEMPATH} \
		-L${BINDIR} -Wl,-rpath=${BINDIR} -lqpOASES \
		-lcutest -llapack -lm -lgfortran -lprob

IFLAGS      = -I${IDIR} \
	      -I${CUTEST}/include

QPOASES_EXES = \
	qpoasesCutest

##
##	targets
##

all: ${QPOASES_EXES}

qpoasesCutest: qpoasesCutest.o
	@echo "Creating" $@
	${CPP} -o $@ ${CPPFLAGS} $< ${LINKOPTS}

clean:
	rm -f *.o ${QPOASES_EXES}

clobber: clean

%.o: %.cpp
	@echo "Creating" $@
	@${CPP} -c ${IFLAGS} ${CPPFLAGS} $< -o $@

##
##	end of file
##
