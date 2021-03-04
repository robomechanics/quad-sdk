/**
 * @file Data.tpp
 * @author Giulio Romualdi
 * @copyright Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#include <iostream>

template<typename Derived>
bool OsqpEigen::Data::setHessianMatrix(const Eigen::SparseCompressedBase<Derived> &hessianMatrix)
{
    if(m_isHessianMatrixSet){
        std::cerr << "[OsqpEigen::Data::setHessianMatrix] The hessian matrix was already set. "
                  << "Please use clearHessianMatrix() method to deallocate memory."
                  << std::endl;
        return false;
    }

    if(!m_isNumberOfVariablesSet){
        std::cerr << "[OsqpEigen::Data::setHessianMatrix] Please set the number of variables before "
                  << "add the hessian matrix."
                  << std::endl;
        return false;
    }

    // check if the number of row and columns are equal to the number of the optimization variables
    if ((hessianMatrix.rows() != m_data->n) || (hessianMatrix.cols()!= m_data->n)){
        std::cerr << "[OsqpEigen::Data::setHessianMatrix] The Hessian matrix has to be a n x n size matrix."
                  << std::endl;
        return false;
    }

    //set the hessian matrix
    // osqp 0.6.0 required only the upper triangular part of the hessian matrix
    Derived hessianMatrixUpperTriangular = hessianMatrix.template triangularView<Eigen::Upper>();
    if(!OsqpEigen::SparseMatrixHelper::createOsqpSparseMatrix(hessianMatrixUpperTriangular, m_data->P)){
        std::cerr << "[OsqpEigen::Data::setHessianMatrix] Unable to instantiate the osqp sparse matrix."
                  << std::endl;
        return false;
    }
    m_isHessianMatrixSet = true;
    return true;
}

template<typename Derived>
bool OsqpEigen::Data::setLinearConstraintsMatrix(const Eigen::SparseCompressedBase<Derived> &linearConstraintsMatrix)
{
    if(m_isLinearConstraintsMatrixSet){
        std::cerr << "[OsqpEigen::Data::setLinearConstraintsMatrix] The linear constraint matrix was already set. "
                  << "Please use clearLinearConstraintsMatrix() method to deallocate memory."
                  << std::endl;
        return false;
    }

    if(!m_isNumberOfConstraintsSet){
        std::cerr << "[OsqpEigen::Data::setLinearConstraintsMatrix] Please set the number of constraints before add the constraint matrix."
                  << std::endl;
        return false;
    }

    if(!m_isNumberOfVariablesSet){
        std::cerr << "[OsqpEigen::Data::setLinearConstraintsMatrix] Please set the number of variables before add the constraint matrix."
                  << std::endl;
        return false;
    }

    if ((linearConstraintsMatrix.rows() != m_data->m) || (linearConstraintsMatrix.cols()!= m_data->n)){
        std::cerr << "[OsqpEigen::Data::setLinearConstraintsMatrix] The Linear constraints matrix has to be a m x n size matrix."
                  << std::endl;
        return false;
    }

    // set the hessian matrix
    if(!OsqpEigen::SparseMatrixHelper::createOsqpSparseMatrix(linearConstraintsMatrix,
                                                                m_data->A)){
        std::cerr << "[OsqpEigen::Data::setLinearConstraintsMatrix] osqp sparse matrix not created."
                  << std::endl;
        return false;
    }

    m_isLinearConstraintsMatrixSet = true;

    return true;
}
