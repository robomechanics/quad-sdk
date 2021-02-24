/**
 * @file Solver.tpp
 * @author Giulio Romualdi
 * @copyright Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#include <iostream>
#include <auxil.h>
#include <scaling.h>

template<typename Derived>
bool OsqpEigen::Solver::updateHessianMatrix(const Eigen::SparseCompressedBase<Derived> &hessianMatrix)
{
    if(!m_isSolverInitialized){
        std::cerr << "[OsqpEigen::Solver::updateHessianMatrix] The solver has not been initialized."
                  << std::endl;
        return false;
    }

    if(((c_int)hessianMatrix.rows() != m_workspace->data->n)||
       ((c_int)hessianMatrix.cols() != m_workspace->data->n)){
        std::cerr << "[OsqpEigen::Solver::updateHessianMatrix] The hessian matrix has to be a nxn matrix"
                  << std::endl;
        return false;
    }


    // evaluate the triplets from old and new hessian sparse matrices
    if(!OsqpEigen::SparseMatrixHelper::osqpSparseMatrixToTriplets(m_workspace->data->P,
                                                                    m_oldHessianTriplet)){
        std::cerr << "[OsqpEigen::Solver::updateHessianMatrix] Unable to evaluate triplets from the old hessian matrix."
                  << std::endl;
        return false;
    }
    if(!OsqpEigen::SparseMatrixHelper::eigenSparseMatrixToTriplets(hessianMatrix,
                                                                     m_newHessianTriplet)){
        std::cerr << "[OsqpEigen::Solver::updateHessianMatrix] Unable to evaluate triplets from the old hessian matrix."
                  << std::endl;
        return false;
    }

    selectUpperTriangularTriplets(m_newHessianTriplet, m_newUpperTriangularHessianTriplets);

    // try to update the hessian matrix without reinitialize the solver
    // according to the osqp library it can be done only if the sparsity pattern of the hessian
    // matrix does not change.

    if(evaluateNewValues(m_oldHessianTriplet,  m_newUpperTriangularHessianTriplets,
                         m_hessianNewIndices, m_hessianNewValues)){
        if (m_hessianNewValues.size() > 0) {
            if(osqp_update_P(m_workspace.get(), m_hessianNewValues.data(), m_hessianNewIndices.data(), m_hessianNewIndices.size()) != 0){
                std::cerr << "[OsqpEigen::Solver::updateHessianMatrix] Unable to update hessian matrix."
                          << std::endl;
                return false;
            }
        }
    }
    else{
        // the sparsity pattern has changed
        // the solver has to be setup again

        // get the primal and the dual variables

        if(!getPrimalVariable(m_primalVariables)){
            std::cerr << "[OsqpEigen::Solver::updateHessianMatrix] Unable to get the primal variable."
                      << std::endl;
            return false;
        }

        if(!getDualVariable(m_dualVariables)){
            std::cerr << "[OsqpEigen::Solver::updateHessianMatrix] Unable to get the dual variable."
                      << std::endl;
            return false;
        }

        // clear old hessian matrix
        m_data->clearHessianMatrix();

        // set new hessian matrix
        if(!m_data->setHessianMatrix(hessianMatrix)){
            std::cerr << "[OsqpEigen::Solver::updateHessianMatrix] Unable to update the hessian matrix in "
                      << "OptimizaroData object."
                      << std::endl;
            return false;
        }

        // clear the old solver
        clearSolver();

        // initialize a new solver
        if(!initSolver()){
            std::cerr << "[OsqpEigen::Solver::updateHessianMatrix] Unable to Initialize the solver."
                      << std::endl;
            return false;
        }

        // set the old primal and dual variables
        if(!setPrimalVariable(m_primalVariables)){
            std::cerr << "[OsqpEigen::Solver::updateHessianMatrix] Unable to set the primal variable."
                      << std::endl;
            return false;
        }

        if(!setDualVariable(m_dualVariables)){
            std::cerr << "[OsqpEigen::Solver::updateHessianMatrix] Unable to set the dual variable."
                      << std::endl;
            return false;
        }
    }
    return true;
}

template<typename Derived>
bool OsqpEigen::Solver::updateLinearConstraintsMatrix(const Eigen::SparseCompressedBase<Derived> &linearConstraintsMatrix)
{
    if(!m_isSolverInitialized){
        std::cerr << "[OsqpEigen::Solver::updateLinearConstraintsMatrix] The solver has not been initialized."
                  << std::endl;
        return false;
    }

    if(((c_int)linearConstraintsMatrix.rows() != m_workspace->data->m)||
       ((c_int)linearConstraintsMatrix.cols() != m_workspace->data->n)){
        std::cerr << "[OsqpEigen::Solver::updateLinearConstraintsMatrix] The constraints matrix has to be a mxn matrix"
                  << std::endl;
        return false;
    }

    // evaluate the triplets from old and new hessian sparse matrices

    if(!OsqpEigen::SparseMatrixHelper::osqpSparseMatrixToTriplets(m_workspace->data->A,
                                                                    m_oldLinearConstraintsTriplet)){
        std::cerr << "[OsqpEigen::Solver::updateLinearConstraintsMatrix] Unable to evaluate triplets from the old hessian matrix."
                  << std::endl;
        return false;
    }
    if(!OsqpEigen::SparseMatrixHelper::eigenSparseMatrixToTriplets(linearConstraintsMatrix,
                                                                     m_newLinearConstraintsTriplet)){
        std::cerr << "[OsqpEigen::Solver::updateLinearConstraintsMatrix] Unable to evaluate triplets from the old hessian matrix."
                  << std::endl;
        return false;
    }

    // try to update the linear constraints matrix without reinitialize the solver
    // according to the osqp library it can be done only if the sparsity pattern of the
    // matrix does not change.

    if(evaluateNewValues(m_oldLinearConstraintsTriplet, m_newLinearConstraintsTriplet,
                         m_constraintsNewIndices, m_constraintsNewValues)){
        if (m_constraintsNewValues.size() > 0) {
            if(osqp_update_A(m_workspace.get(), m_constraintsNewValues.data(), m_constraintsNewIndices.data(), m_constraintsNewIndices.size()) != 0){
                std::cerr << "[OsqpEigen::Solver::updateLinearConstraintsMatrix] Unable to update linear constraints matrix."
                          << std::endl;
                return false;
            }
        }
    }
    else{
        // the sparsity pattern has changed
        // the solver has to be setup again

        // get the primal and the dual variables

        if(!getPrimalVariable(m_primalVariables)){
            std::cerr << "[OsqpEigen::Solver::updateLinearConstraintsMatrix] Unable to get the primal variable."
                      << std::endl;
            return false;
        }

        if(!getDualVariable(m_dualVariables)){
            std::cerr << "[OsqpEigen::Solver::updateLinearConstraintsMatrix] Unable to get the dual variable."
                      << std::endl;
            return false;
        }

        // clear old linear constraints matrix
        m_data->clearLinearConstraintsMatrix();

        // set new linear constraints matrix
        if(!m_data->setLinearConstraintsMatrix(linearConstraintsMatrix)){
            std::cerr << "[OsqpEigen::Solver::updateLinearConstraintsMatrix] Unable to update the hessian matrix in "
                      << "Data object."
                      << std::endl;
            return false;
        }

        // clear the old solver
        clearSolver();

        if(!initSolver()){
            std::cerr << "[OsqpEigen::Solver::updateLinearConstraintsMatrix] Unable to Initialize the solver."
                      << std::endl;
            return false;
        }

        // set the old primal and dual variables
        if(!setPrimalVariable(m_primalVariables)){
            std::cerr << "[OsqpEigen::Solver::updateLinearConstraintsMatrix] Unable to set the primal variable."
                      << std::endl;
            return false;
        }

        if(!setDualVariable(m_dualVariables)){
            std::cerr << "[OsqpEigen::Solver::updateLinearConstraintsMatrix] Unable to set the dual variable."
                      << std::endl;
            return false;
        }
    }
    return true;
}

template<typename T, int n, int m>
bool OsqpEigen::Solver::setWarmStart(const Eigen::Matrix<T, n, 1> &primalVariable,
                                                  const Eigen::Matrix<T, m, 1> &dualVariable)
{
    if(primalVariable.rows() != m_workspace->data->n){
        std::cerr << "[OsqpEigen::Solver::setWarmStart] The size of the primal variable vector has to be equal to "
                  << " the number of variables."
                  << std::endl;
        return false;
    }

    if(dualVariable.rows() != m_workspace->data->m){
        std::cerr << "[OsqpEigen::Solver::setWarmStart] The size of the dual variable vector has to be equal to "
                  << " the number of constraints."
                  << std::endl;
        return false;
    }

    m_primalVariables = primalVariable.template cast <c_float>();
    m_dualVariables = dualVariable.template cast <c_float>();

    return (osqp_warm_start(m_workspace.get(), m_primalVariables.data(), m_dualVariables.data()) == 0);

}

template<typename T, int n>
bool OsqpEigen::Solver::setPrimalVariable(const Eigen::Matrix<T, n, 1> &primalVariable)
{
    if(primalVariable.rows() != m_workspace->data->n){
        std::cerr << "[OsqpEigen::Solver::setPrimalVariable] The size of the primal variable vector has to be equal to "
                  << " the number of variables."
                  << std::endl;
        return false;
    }

    m_primalVariables = primalVariable.template cast <c_float>();

    return (osqp_warm_start_x(m_workspace.get(), m_primalVariables.data()) == 0);
}


template<typename T, int m>
bool OsqpEigen::Solver::setDualVariable(const Eigen::Matrix<T, m, 1> &dualVariable)
{
    if(dualVariable.rows() != m_workspace->data->m){
        std::cerr << "[OsqpEigen::Solver::setDualVariable] The size of the dual variable vector has to be equal to "
                  << " the number of constraints."
                  << std::endl;
        return false;
    }

    m_dualVariables = dualVariable.template cast <c_float>();

    return (osqp_warm_start_y(m_workspace.get(), m_dualVariables.data()) == 0);
}

template<typename T, int n>
bool OsqpEigen::Solver::getPrimalVariable(Eigen::Matrix<T, n, 1> &primalVariable)
{
    if(n == Eigen::Dynamic){
        primalVariable.resize(m_workspace->data->n, 1);
    }
    else{
        if (n != m_workspace->data->n){
            std::cerr << "[OsqpEigen::Solver::getPrimalVariable] The size of the vector has to be equal to the number of "
                      << "variables. (You can use an eigen dynamic vector)"
                      << std::endl;
            return false;
        }
    }

    primalVariable = Eigen::Map<Eigen::Matrix<c_float, n, 1>>(m_workspace->x, m_workspace->data->n).template cast <T>();

    return true;
}

template<typename T, int m>
bool OsqpEigen::Solver::getDualVariable(Eigen::Matrix<T, m, 1> &dualVariable)
{
    if(m == Eigen::Dynamic){
        dualVariable.resize(m_workspace->data->m, 1);
    }
    else{
        if (m != m_workspace->data->m){
            std::cerr << "[OsqpEigen::Solver::getDualVariable] The size of the vector has to be equal to the number of "
                      << "constraints. (You can use an eigen dynamic vector)"
                      << std::endl;
            return false;
        }
    }

    dualVariable = Eigen::Map<Eigen::Matrix<c_float, m, 1>>(m_workspace->y, m_workspace->data->m).template cast <T>();

    return true;
}

template<typename T>
bool OsqpEigen::Solver::evaluateNewValues(const std::vector<Eigen::Triplet<T>> &oldMatrixTriplet,
                                                     const std::vector<Eigen::Triplet<T>> &newMatrixTriplet,
                                                     std::vector<c_int> &newIndices,
                                                     std::vector<c_float> &newValues) const
{
    //When updating the matrices for osqp, we need to provide the indeces to modify of the value vector. The following can work since, when extracting triplets from osqp sparse matrices, the order of the triplets follows the same order of the value vector.
    // check if the sparsity pattern is changed
    size_t valuesAdded = 0;
    if(newMatrixTriplet.size() == oldMatrixTriplet.size()){
        for(int i = 0; i < newMatrixTriplet.size(); i++){
            // check if the sparsity pattern is changed
            if((newMatrixTriplet[i].row() != oldMatrixTriplet[i].row()) ||
               (newMatrixTriplet[i].col() != oldMatrixTriplet[i].col()))
                return false;

            // check if an old value is changed
            if(newMatrixTriplet[i].value() != oldMatrixTriplet[i].value()){
                if (valuesAdded >= newValues.size()) {
                    newValues.push_back((c_float) newMatrixTriplet[i].value());
                    newIndices.push_back((c_int) i);
                    valuesAdded++;
                } else {
                    newValues[valuesAdded] = static_cast<c_float>(newMatrixTriplet[i].value());
                    newIndices[valuesAdded] = static_cast<c_int>(i);
                    valuesAdded++;
                }
            }
        }
        newValues.erase(newValues.begin() + valuesAdded, newValues.end());
        newIndices.erase(newIndices.begin() + valuesAdded, newIndices.end());
        return true;
    }
    return false;
}

template<typename T>
void OsqpEigen::Solver::selectUpperTriangularTriplets(const std::vector<Eigen::Triplet<T>> &fullMatrixTriplets,
                                                                 std::vector<Eigen::Triplet<T>> &upperTriangularMatrixTriplets) const {

    int upperTriangularTriplets = 0;
    for (int i = 0; i < fullMatrixTriplets.size(); ++i) {
        if (fullMatrixTriplets[i].row() <= fullMatrixTriplets[i].col()) {
            if (upperTriangularTriplets < upperTriangularMatrixTriplets.size()) {
                upperTriangularMatrixTriplets[upperTriangularTriplets] = fullMatrixTriplets[i];
            } else {
                upperTriangularMatrixTriplets.push_back(fullMatrixTriplets[i]);
            }
            upperTriangularTriplets++;
        }
    }

    upperTriangularMatrixTriplets.erase(upperTriangularMatrixTriplets.begin() + upperTriangularTriplets, upperTriangularMatrixTriplets.end());
}
