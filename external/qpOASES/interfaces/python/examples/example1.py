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
from qpoases import PyOptions as Options
from qpoases import PyPrintLevel as PrintLevel

#Setup data of first QP.

H   = np.array([1.0, 0.0, 0.0, 0.5 ]).reshape((2,2))
A   = np.array([1.0, 1.0 ]).reshape((2,1))
g   = np.array([1.5, 1.0 ])
lb  = np.array([0.5, -2.0])
ub  = np.array([5.0, 2.0 ])
lbA = np.array([-1.0 ])
ubA = np.array([2.0])


# Setup data of second QP.

g_new   = np.array([1.0, 1.5])
lb_new  = np.array([0.0, -1.0])
ub_new  = np.array([5.0, -0.5])
lbA_new = np.array([-2.0])
ubA_new = np.array([1.0])


# Setting up QProblem object.

example = QProblem(2, 1)
options = Options()
#options.printLevel = PrintLevel.NONE
example.setOptions(options)

# Solve first QP.
nWSR = np.array([10])
example.init(H, g, A, lb, ub, lbA, ubA, nWSR)

xOpt = np.zeros(2)
example.getPrimalSolution(xOpt)
print("\nxOpt = [ %e, %e ];  objVal = %e\n\n"%(xOpt[0],xOpt[1],example.getObjVal()))

# Solve second QP.
nWSR = np.array([10])
example.hotstart( g_new, lb_new, ub_new, lbA_new, ubA_new, nWSR)

# Get and print solution of second QP.

example.getPrimalSolution(xOpt)
print("\nxOpt = [ %e, %e ];  objVal = %e\n\n"%(xOpt[0],xOpt[1],example.getObjVal()))
example.printOptions()
