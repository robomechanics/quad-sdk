/**
 * @file UpdateMatricesTest.cpp
 * @author Giulio Romualdi
 * @copyright Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2020
 */

// Catch2
#include <catch2/catch.hpp>

// OsqpEigen
#include <OsqpEigen/OsqpEigen.h>

// colors
#define ANSI_TXT_GRN "\033[0;32m"
#define ANSI_TXT_MGT "\033[0;35m" //Magenta
#define ANSI_TXT_DFT "\033[0;0m" //Console default
#define GTEST_BOX "[     cout ] "
#define COUT_GTEST ANSI_TXT_GRN << GTEST_BOX //You could add the Default
#define COUT_GTEST_MGT COUT_GTEST << ANSI_TXT_MGT

Eigen::Matrix2d H;
Eigen::SparseMatrix<double> H_s;
Eigen::Matrix<double,3,2> A;
Eigen::SparseMatrix<double> A_s;
Eigen::Vector2d gradient;
Eigen::Vector3d lowerBound;
Eigen::Vector3d upperBound;

OsqpEigen::Solver solver;

TEST_CASE("QPProblem - FirstRun")
{
    // hessian matrix
    H << 4, 0,
        0, 2;
    H_s = H.sparseView();
    H_s.pruned(0.01);

    // linear constraint matrix
    A << 1, 1,
        1, 0,
        0, 1;
    A_s = A.sparseView();

    gradient << 1, 1;
    lowerBound << 1, 0, 0;
    upperBound << 1, 0.7, 0.7;

    solver.settings()->setVerbosity(false);

    solver.data()->setNumberOfVariables(2);
    solver.data()->setNumberOfConstraints(3);
    solver.settings()->setScaling(0);
    REQUIRE(solver.data()->setHessianMatrix(H_s));
    REQUIRE(solver.data()->setGradient(gradient));
    REQUIRE(solver.data()->setLinearConstraintsMatrix(A_s));
    REQUIRE(solver.data()->setLowerBound(lowerBound));
    REQUIRE(solver.data()->setUpperBound(upperBound));

    REQUIRE(solver.initSolver());
    REQUIRE(solver.solve());

    auto solution = solver.getSolution();
    std::cout << COUT_GTEST_MGT << "Solution [" << solution(0) << " "
              << solution(1) << "]"
              << ANSI_TXT_DFT << std::endl;
}

TEST_CASE("QPProblem - SparsityConstant")
{
    // update hessian matrix
    H << 4, 0,
        0, 2;
    H_s = H.sparseView();
    A << 2, 1,
        1, 0,
        0, 1;
    A_s = A.sparseView();

    REQUIRE(solver.updateHessianMatrix(H_s));
    REQUIRE(solver.updateLinearConstraintsMatrix(A_s));
    REQUIRE(solver.solve());

    auto solution = solver.getSolution();
    std::cout << COUT_GTEST_MGT << "Solution [" << solution(0) << " "
              << solution(1) << "]"
              << ANSI_TXT_DFT << std::endl;
};

TEST_CASE("QPProblem - SparsityChange")
{
    // update hessian matrix
    H << 1, 1,
        1, 2;
    H_s = H.sparseView();
    A << 1, 1,
        1, 0.4,
        0, 1;
    A_s = A.sparseView();

    REQUIRE(solver.updateHessianMatrix(H_s));
    REQUIRE(solver.updateLinearConstraintsMatrix(A_s));
    REQUIRE(solver.solve());

    auto solution = solver.getSolution();
    std::cout << COUT_GTEST_MGT << "Solution [" << solution(0) << " "
              << solution(1) << "]"
              << ANSI_TXT_DFT << std::endl;
};
