/**
 * @file MPCExample.cpp
 * @author Giulio Romualdi
 * @copyright Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */


// osqp-eigen
#include "OsqpEigen/OsqpEigen.h"

// eigen
#include <Eigen/Dense>

#include <iostream>


int main()
{
    // Cost function 1/2*x'*P*x + q'*x
    Eigen::Matrix<double,2,2> P_dense;
    P_dense << 1,0,
               0,1;

    // OSQP wants sparse matrices, use this function to convert P and A
    Eigen::SparseMatrix<double> P = P_dense.sparseView();

    Eigen::Matrix<double, 2, 1> q;
    q << 1,1;

    // Constraints l <= AX <= u
    Eigen::Matrix<double, 2, 1> l;
    l << -0.2,-0.2;
    Eigen::Matrix<double,2,2> A_dense;
    A_dense << 1,0,
               0,1;

    // OSQP wants sparse matrices, use this function to convert P and A
    Eigen::SparseMatrix<double> A = A_dense.sparseView();
    Eigen::Matrix<double, 2, 1> u;
    u << 0.4,0.6;

    // instantiate the solver
    OsqpEigen::Solver solver;

    // settings
    //solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);

    // set the initial data of the QP solver
    solver.data()->setNumberOfVariables(2);
    solver.data()->setNumberOfConstraints(2);
    solver.data()->setHessianMatrix(P);
    solver.data()->setGradient(q);
    solver.data()->setLinearConstraintsMatrix(A);
    solver.data()->setLowerBound(l);
    solver.data()->setUpperBound(u);

    // instantiate the solver
    if(!solver.initSolver()) return 1;

    Eigen::VectorXd QPSolution;

    // solve the QP problem
    if(!solver.solve()) return 1;

    // get the controller input
    QPSolution = solver.getSolution();
    
    std::cout << QPSolution << std::endl;

    return 0;
}
