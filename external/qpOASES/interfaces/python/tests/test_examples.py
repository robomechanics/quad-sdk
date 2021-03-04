"""
This file is part of qpOASES.

qpOASES -- An Implementation of the Online Active Set Strategy.
Copyright (C) 2007-2017 by Hans Joachim Ferreau et al. All rights reserved.

qpOASES is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

qpOASES is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with qpOASES; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

author Manuel Kudruss
version 3.2
date 2013-2017
"""

import os
import re
import numpy as np
from numpy.testing import *
from subprocess import Popen, PIPE, STDOUT

from qpoases import PyQProblem as QProblem
from qpoases import PyQProblemB as QProblemB
from qpoases import PySQProblem as SQProblem
from qpoases import PySolutionAnalysis as SolutionAnalysis
from qpoases import PyBooleanType as BooleanType
from qpoases import PySubjectToStatus as SubjectToStatus
from qpoases import PyOptions as Options
from qpoases import PyPrintLevel as PrintLevel

# get qpOASES path
qpoases_path = os.path.dirname(os.path.abspath(__file__))
qpoases_path = os.path.dirname(qpoases_path)
qpoases_path = os.path.dirname(qpoases_path)
qpoases_path = os.path.dirname(qpoases_path)

# set qpOASES binary path
bin_path = os.path.join(qpoases_path, "bin")

class TestExamples(TestCase):

    def test_example1(self):
        return 0
        # Example for qpOASES main function using the QProblem class.
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

        # Setting up QProblemB object.
        qp = QProblem(2, 1)
        options = Options()
        options.printLevel = PrintLevel.NONE
        qp.setOptions(options)

        # Solve first QP.
        nWSR = 10
        qp.init(H, g, A, lb, ub, lbA, ubA, nWSR)

        # Solve second QP.
        nWSR = 10
        qp.hotstart(g_new, lb_new, ub_new, lbA_new, ubA_new, nWSR)

        # Get and print solution of second QP.
        xOpt_actual = np.zeros(2)
        qp.getPrimalSolution(xOpt_actual)
        xOpt_actual = np.asarray(xOpt_actual, dtype=float)
        objVal_actual = qp.getObjVal()
        objVal_actual = np.asarray(objVal_actual, dtype=float)

        cmd = os.path.join(bin_path, "example1")
        p = Popen(cmd, shell=True, stdout=PIPE)
        stdout, stderr = p.communicate()
        stdout = str(stdout).replace('\\n', '\n')
        stdout = stdout.replace("'", '')
        print(stdout)

        # get c++ solution from std
        pattern = re.compile(r'xOpt\s*=\s*\[\s+(?P<xOpt>([0-9., e+-])*)\];')
        match = pattern.search(stdout)
        xOpt_expected = match.group('xOpt')
        xOpt_expected = xOpt_expected.split(",")
        xOpt_expected = np.asarray(xOpt_expected, dtype=float)

        pattern = re.compile(r'objVal = (?P<objVal>[0-9-+e.]*)')
        match = pattern.search(stdout)
        objVal_expected = match.group('objVal')
        objVal_expected = np.asarray(objVal_expected, dtype=float)

        print("xOpt_actual =", xOpt_actual)
        print("xOpt_expected =", xOpt_expected)
        print("objVal_actual = ", objVal_actual)
        print("objVal_expected = ", objVal_expected)

        assert_almost_equal(xOpt_actual, xOpt_expected, decimal=7)
        assert_almost_equal(objVal_actual, objVal_expected, decimal=7)

    def test_example1b(self):
        """Example for qpOASES main function using the QProblemB class."""
        # Setup data of first QP.
        H = np.array([1.0, 0.0, 0.0, 0.5]).reshape((2, 2))
        g = np.array([1.5, 1.0])
        lb = np.array([0.5, -2.0])
        ub = np.array([5.0, 2.0])

        # Setup data of second QP.

        g_new = np.array([1.0, 1.5])
        lb_new = np.array([0.0, -1.0])
        ub_new = np.array([5.0, -0.5])

        # Setting up QProblemB object.
        qp = QProblemB(2)

        options = Options()
        # options.enableFlippingBounds = BooleanType.FALSE
        options.initialStatusBounds = SubjectToStatus.INACTIVE
        options.numRefinementSteps = 1
        options.enableCholeskyRefactorisation = 1
        options.printLevel = PrintLevel.NONE
        qp.setOptions(options)

        # Solve first QP.
        nWSR = 10
        qp.init(H, g, lb, ub, nWSR)

        xOpt_actual = np.zeros(2)
        qp.getPrimalSolution(xOpt_actual)
        xOpt_actual = np.asarray(xOpt_actual, dtype=float)
        objVal_actual = qp.getObjVal()
        objVal_actual = np.asarray(objVal_actual, dtype=float)
        print 'xOpt_actual:', xOpt_actual
        print 'objVal_actual:', objVal_actual

        # Solve second QP.
        nWSR = 10
        qp.hotstart(g_new, lb_new, ub_new, nWSR)

        xOpt_actual = np.zeros(2)
        qp.getPrimalSolution(xOpt_actual)
        xOpt_actual = np.asarray(xOpt_actual, dtype=float)
        objVal_actual = qp.getObjVal()
        objVal_actual = np.asarray(objVal_actual, dtype=float)
        print 'xOpt_actual:', xOpt_actual
        print 'objVal_actual:', objVal_actual

        # Get and print solution of second QP.
        xOpt_actual = np.zeros(2)
        qp.getPrimalSolution(xOpt_actual)
        xOpt_actual = np.asarray(xOpt_actual, dtype=float)
        objVal_actual = qp.getObjVal()
        objVal_actual = np.asarray(objVal_actual, dtype=float)

        cmd = os.path.join(bin_path, "example1b")
        p = Popen(cmd, shell=True, stdout=PIPE)
        stdout, stderr = p.communicate()
        stdout = str(stdout).replace('\\n', '\n')
        stdout = stdout.replace("'", '')

        # get c++ solution from std
        pattern = re.compile(r'xOpt\s*=\s*\[\s+(?P<xOpt>([0-9., e+-])*)\];')
        match = pattern.findall(stdout)
        xOpt_expected = match[-1][0]
        xOpt_expected = xOpt_expected.split(",")
        xOpt_expected = np.asarray(xOpt_expected, dtype=float)

        pattern = re.compile(r'objVal = (?P<objVal>[0-9-+e.]*)')
        match = pattern.findall(stdout)
        print match
        objVal_expected = match[-1]
        objVal_expected = np.asarray(objVal_expected, dtype=float)

        print("xOpt_actual =", xOpt_actual)
        print("xOpt_expected =", xOpt_expected)
        print("objVal_actual = ", objVal_actual)
        print("objVal_expected = ", objVal_expected)

        assert_almost_equal(xOpt_actual, xOpt_expected, decimal=7)
        assert_almost_equal(objVal_actual, objVal_expected, decimal=7)


    def test_example2(self):
        # Example for qpOASES main function using the SQProblem class.
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
        qp = SQProblem(2, 1)
        options = Options()
        options.printLevel = PrintLevel.NONE
        qp.setOptions(options)

        analyser = SolutionAnalysis()

        # get c++ solution from std
        cmd = os.path.join(bin_path, "example2")
        p = Popen(cmd, shell=True, stdout=PIPE)
        stdout, stderr = p.communicate()
        stdout = str(stdout).replace('\\n', '\n')
        stdout = stdout.replace("'", '')
        print(stdout)

        #  Solve first QP ...
        nWSR = 10
        qp.init(H, g, A, lb, ub, lbA, ubA, nWSR)

        #  ... and analyse it.
        maxViol = np.zeros(1)
        maxStat = np.zeros(1)
        maxFeas = np.zeros(1)
        maxCmpl = np.zeros(1)
        maxViol[0] = analyser.getKktViolation(
            qp, maxStat, maxFeas, maxCmpl
        )
        print("maxViol: %e\n" % maxViol)
        actual = np.asarray(maxViol)

        pattern = re.compile(
            r'maxKktViolation: (?P<KktViolation>[0-9+-e.]*)'
        )

        match = pattern.findall(stdout)
        expected = np.asarray(match[0], dtype=float)

        assert_almost_equal(actual, expected, decimal=7)

        #  Solve second QP ...
        nWSR = 10
        qp.hotstart(
            H_new, g_new, A_new,
            lb_new, ub_new,
            lbA_new, ubA_new,
            nWSR
        )

        #  ... and analyse it.
        maxViol = np.zeros(1)
        maxStat = np.zeros(1)
        maxFeas = np.zeros(1)
        maxCmpl = np.zeros(1)
        maxViol[0] = analyser.getKktViolation(
            qp, maxStat, maxFeas, maxCmpl
        )
        print("maxViol: %e\n" % maxViol)
        actual = np.asarray(maxViol)

        expected = np.asarray(match[1], dtype=float)

        assert_almost_equal(actual, expected, decimal=7)

        # ------------ VARIANCE-COVARIANCE EVALUATION --------------------

        Var = np.zeros(5*5)
        Primal_Dual_Var = np.zeros(5*5)

        Var.reshape((5, 5))[0, 0] = 1.
        Var.reshape((5, 5))[1, 1] = 1.

        #                  (  1   0   0   0   0   )
        #                  (  0   1   0   0   0   )
        #     Var     =    (  0   0   0   0   0   )
        #                  (  0   0   0   0   0   )
        #                  (  0   0   0   0   0   )

        analyser.getVarianceCovariance(qp, Var, Primal_Dual_Var)
        print('Primal_Dual_Var=\n', Primal_Dual_Var.reshape((5, 5)))
        actual = Primal_Dual_Var.reshape((5, 5))

        pattern = re.compile(
            r'Primal_Dual_VAR = (?P<VAR>.*)',
            re.DOTALL
        )

        print(stdout)
        match = pattern.search(stdout)
        expected = match.group('VAR').strip().split("\n")
        expected = [x.strip().split() for x in expected]
        print(expected)
        expected = np.asarray(expected, dtype=float)

        assert_almost_equal(actual, expected, decimal=7)

    def test_example7(self):
        H   = np.array([
            0.8514828085899353, -0.15739890933036804, -0.081726007163524628, -0.530426025390625, 0.16773293912410736,
           -0.15739890933036804, 1.1552412509918213, 0.57780224084854126, -0.0072606131434440613, 0.010559185408055782,
           -0.081726007163524628, 0.57780224084854126, 0.28925251960754395, 5.324830453901086e-006, -3.0256599075073609e-006,
           -0.530426025390625, -0.0072606131434440613, 5.324830453901086e-006, 0.35609596967697144, -0.15124998986721039,
            0.16773293912410736, 0.010559185408055782, -3.0256599075073609e-006, -0.15124998986721039, 0.15129712224006653
            ], dtype=float
        ).reshape((5, 5))
        g   = np.array([0.30908384919166565, 0.99325823783874512, 0.49822014570236206, -0.26309865713119507, 0.024296050891280174], dtype=float).reshape((5,))
        A   = np.array([1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1], dtype=float).reshape((5, 5))
        lb  = np.array([-0.052359879016876221, -0.052359879016876221, -0.052359879016876221, -0.052359879016876221, -0.052359938621520996], dtype=float).reshape((5,))
        ub  = np.array([ 0.052359879016876221, 0.052359879016876221, 0.052359879016876221, 0, 0], dtype=float).reshape((5,))
        lbA = np.array([-0.052359879016876221, -0.052359879016876221, -0.052359879016876221, -0.052359879016876221, -0.052359938621520996], dtype=float).reshape((5,))
        ubA = np.array([0.052359879016876221, 0.052359879016876221, 0.052359879016876221, 0, 0], dtype=float).reshape((5,))

        # Setting up QProblem object.
        qp = QProblem(5, 5)
        options = Options()
        options.printLevel = PrintLevel.NONE
        qp.setOptions(options)

        # Solve first QP.
        nWSR = 100
        qp.init(H, g, A, lb, ub, lbA, ubA,  nWSR)

        result = np.zeros((5,))
        qp.getPrimalSolution(result)

        # TODO check against what?
        # Where can I find solution?

if __name__=="__main__":
    try:
        import nose
        nose.runmodule()

    except ImportError:
        sys.stderr.write("Please install nosestests for python unittesting.\n")

