/**
 * @file QPTest.cpp
 * @author Giulio Romualdi
 * @copyright Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2020
 */

// Catch2
#include <catch2/catch.hpp>

#include <OsqpEigen/OsqpEigen.h>

TEST_CASE("QPProblem")
{
    Eigen::Matrix2d H;
    H << 4, 1,
        1, 2;
    Eigen::SparseMatrix<double> H_s;
    H_s = H.sparseView();

    Eigen::Matrix<double,3,2> A;
    A << 1, 1,
        1, 0,
        0, 1;
    Eigen::SparseMatrix<double> A_s;
    A_s = A.sparseView();

    Eigen::Vector2d gradient;
    gradient << 1, 1;

    Eigen::Vector3d lowerBound;
    lowerBound << 1, 0, 0;

    Eigen::Vector3d upperBound;
    upperBound << 1, 0.7, 0.7;

    OsqpEigen::Solver solver;
    solver.settings()->setVerbosity(false);

    REQUIRE_FALSE(solver.data()->setHessianMatrix(H_s));
    solver.data()->setNumberOfVariables(2);

    solver.data()->setNumberOfConstraints(3);
    REQUIRE(solver.data()->setHessianMatrix(H_s));
    REQUIRE(solver.data()->setGradient(gradient));
    REQUIRE(solver.data()->setLinearConstraintsMatrix(A_s));
    REQUIRE(solver.data()->setLowerBound(lowerBound));
    REQUIRE(solver.data()->setUpperBound(upperBound));


    REQUIRE(solver.initSolver());

    REQUIRE(solver.solve());

    std::cerr << "The solution of the QP problem is" << std::endl;
    std::cerr << "[ " << solver.getSolution() << " ]"
              << std::endl;
}
