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

## Example adapted from examples/example1.cpp.
## author of this file: Sebastian F. Walter

import numpy as np
from qpoases import PyQProblem as QProblem
from qpoases import PyPrintLevel as PrintLevel
from qpoases import PyOptions as Options
cimport numpy as np

def run():

    #Setup data of QP.

    cdef np.ndarray[np.double_t, ndim=2] H
    cdef np.ndarray[np.double_t, ndim=2] A
    cdef np.ndarray[np.double_t, ndim=1] g
    cdef np.ndarray[np.double_t, ndim=1] lb
    cdef np.ndarray[np.double_t, ndim=1] ub
    cdef np.ndarray[np.double_t, ndim=1] lbA
    cdef np.ndarray[np.double_t, ndim=1] ubA

    H   = np.array([1.0, 0.0, 0.0, 0.5 ]).reshape((2,2))
    A   = np.array([1.0, 1.0 ]).reshape((2,1))
    g   = np.array([1.5, 1.0 ])
    lb  = np.array([0.5, -2.0])
    ub  = np.array([5.0, 2.0 ])
    lbA = np.array([-1.0 ])
    ubA = np.array([2.0])

    # Setting up QProblem object.

    cdef example = QProblem(2, 1)
    cdef options = Options()
    options.printLevel = PrintLevel.NONE
    example.setOptions(options)

    # Solve first QP.

    cdef int nWSR = 10
    example.init(H, g, A, lb, ub, lbA, ubA, nWSR)

    # Solve subsequent QPs

    cdef int i,j
    for i in range(100000):
        for j in range(1, 100):
            g[0] = i%j
        example.hotstart(g, lb, ub, lbA, ubA, nWSR)

run()

