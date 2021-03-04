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

## Example adapted from examples/example2.cpp.
## author of this file: Sebastian F. Walter

import os
import sys
import numpy as np
from qpoases import PySQProblem as SQProblem
from qpoases import PySolutionAnalysis as SolutionAnalysis


#  Setup data of first QP.
H = np.array([ 1.0, 0.0, 0.0, 0.5 ]).reshape((2,2))
A = np.array([ 1.0, 1.0 ]).reshape((2,1))
g = np.array([ 1.5, 1.0 ])
lb = np.array([ 0.5, -2.0 ])
ub = np.array([ 5.0, 2.0 ])
lbA = np.array([ -1.0 ])
ubA = np.array([ 2.0 ])

#  Setup data of second QP.
H_new = np.array([ 1.0, 0.5, 0.5, 0.5 ]).reshape((2,2))
A_new = np.array([ 1.0, 5.0 ]).reshape((2,1))
g_new = np.array([ 1.0, 1.5 ])
lb_new = np.array([ 0.0, -1.0 ])
ub_new = np.array([ 5.0, -0.5 ])
lbA_new = np.array([ -2.0 ])
ubA_new = np.array([ 1.0 ])

#  Setting up SQProblem object and solution analyser.
example = SQProblem(2, 1)
analyser = SolutionAnalysis()

#  Solve first QP ...
nWSR = np.array([10])
example.init(H, g, A, lb, ub, lbA, ubA, nWSR)

#  ... and analyse it.
maxStat = np.zeros(1)
maxFeas = np.zeros(1)
maxCmpl = np.zeros(1)

analyser.getKktViolation(example, maxStat, maxFeas, maxCmpl)
print("maxStat: %e, maxFeas:%e, maxCmpl: %e\n"%(maxStat, maxFeas, maxCmpl))

#  Solve second QP ...
nWSR = np.array([10])
example.hotstart(H_new, g_new, A_new, lb_new, ub_new,
                 lbA_new, ubA_new, nWSR)

#  ... and analyse it.
analyser.getKktViolation(example, maxStat, maxFeas, maxCmpl)
print("maxStat: %e, maxFeas:%e, maxCmpl: %e\n"%(maxStat, maxFeas, maxCmpl))


#  ------------ VARIANCE-COVARIANCE EVALUATION --------------------

Var             = np.zeros(5*5)
Primal_Dual_Var = np.zeros(5*5)

Var.reshape((5,5))[0,0] = 1.
Var.reshape((5,5))[1,1] = 1.

#                  (  1   0   0   0   0   )
#                  (  0   1   0   0   0   )
#     Var     =    (  0   0   0   0   0   )
#                  (  0   0   0   0   0   )
#                  (  0   0   0   0   0   )


analyser.getVarianceCovariance(example, Var, Primal_Dual_Var)
print('Primal_Dual_Var=\n', Primal_Dual_Var.reshape((5,5)))
print("maxStat: %e, maxFeas:%e, maxCmpl: %e\n"%(maxStat, maxFeas, maxCmpl))
