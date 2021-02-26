/**
 * @file SparseMatrixHelper.tpp
 * @author Giulio Romualdi
 * @copyright  Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

template<typename Derived>
bool OsqpEigen::SparseMatrixHelper::createOsqpSparseMatrix(const Eigen::SparseCompressedBase<Derived> &eigenSparseMatrix,
                                                             csc*& osqpSparseMatrix)

{
    Eigen::SparseMatrix<typename Derived::value_type, Eigen::ColMajor> colMajorCopy; //Copying into a new sparse matrix to be sure to use a CSC matrix
    colMajorCopy = eigenSparseMatrix; //This may perform merory allocation, but this is already the case for allocating the osqpSparseMatrix
    // get number of row, columns and nonZeros from Eigen SparseMatrix
    c_int rows = colMajorCopy.rows();
    c_int cols = colMajorCopy.cols();
    c_int numberOfNonZeroCoeff = colMajorCopy.nonZeros();

    // get innerr and outer index
    const int* outerIndexPtr = colMajorCopy.outerIndexPtr();
    const int* innerNonZerosPtr = colMajorCopy.innerNonZeroPtr();

    // instantiate csc matrix
    // MEMORY ALLOCATION!!
    if(osqpSparseMatrix != nullptr){
        std::cerr << "[OsqpEigen::SparseMatrixHelper::createOsqpSparseMatrix] osqpSparseMatrix pointer is not a null pointer! "
                  << std::endl;
        return false;
    }

    osqpSparseMatrix = csc_spalloc(rows, cols, numberOfNonZeroCoeff, 1, 0);

    int innerOsqpPosition = 0;
    for(int k = 0; k < cols; k++) {
        if (colMajorCopy.isCompressed()) {
            osqpSparseMatrix->p[k] = static_cast<c_int>(outerIndexPtr[k]);
        } else {
            if (k == 0) {
                osqpSparseMatrix->p[k] = 0;
            } else {
                osqpSparseMatrix->p[k] = osqpSparseMatrix->p[k-1] + innerNonZerosPtr[k-1];
            }
        }
        for (typename Eigen::SparseMatrix<typename Derived::value_type, Eigen::ColMajor>::InnerIterator it(colMajorCopy,k); it; ++it) {
            osqpSparseMatrix->i[innerOsqpPosition] = static_cast<c_int>(it.row());
            osqpSparseMatrix->x[innerOsqpPosition] = static_cast<c_float>(it.value());
            innerOsqpPosition++;
        }
    }
    osqpSparseMatrix->p[static_cast<int>(cols)] = static_cast<c_int>(innerOsqpPosition);

    assert(innerOsqpPosition == numberOfNonZeroCoeff);

    return true;
}

template<typename T>
bool OsqpEigen::SparseMatrixHelper::osqpSparseMatrixToTriplets(const csc* const & osqpSparseMatrix,
                                                               std::vector<Eigen::Triplet<T>> &tripletList)
{
    // if the matrix is not instantiate the triplets vector is empty
    if(osqpSparseMatrix == nullptr){
        std::cerr << "[OsqpEigen::SparseMatrixHelper::osqpSparseMatrixToTriplets] the osqpSparseMatrix is not initialized."
                  << std::endl;
        return false;
    }

    // get row and column data
    c_int* innerIndexPtr = osqpSparseMatrix->i;
    c_int* outerIndexPtr = osqpSparseMatrix->p;

    // get values data
    c_float* valuePtr = osqpSparseMatrix->x;
    c_int numberOfNonZeroCoeff =  osqpSparseMatrix->p[osqpSparseMatrix->n];

    // populate the tripletes vector
    int column=0;
    int row;
    c_float value;

    tripletList.resize(numberOfNonZeroCoeff);
    for(int i = 0; i<numberOfNonZeroCoeff; i++) {
        row = innerIndexPtr[i];
        value = valuePtr[i];

        while(i >= outerIndexPtr[column+1])
            column++;

        tripletList[i] = Eigen::Triplet<T>(row, column, static_cast<T>(value));
    }

    tripletList.erase(tripletList.begin() + numberOfNonZeroCoeff, tripletList.end());

    return true;
}

template<typename T>
bool OsqpEigen::SparseMatrixHelper::osqpSparseMatrixToEigenSparseMatrix(const csc* const & osqpSparseMatrix,
                                                                        Eigen::SparseMatrix<T> &eigenSparseMatrix)
{
    // if the matrix is not instantiate the eigen matrix is empty
    if(osqpSparseMatrix == nullptr) {
        std::cerr << "[OsqpEigen::SparseMatrixHelper::osqpSparseMatrixToEigenSparseMatrix] the osqpSparseMatrix is not initialized."
                  << std::endl;
        return false;
    }

    // get the number of rows and columns
    int rows = osqpSparseMatrix->m;
    int cols = osqpSparseMatrix->n;

    // get the triplets from the csc matrix
    std::vector<Eigen::Triplet<T>> tripletList;

    OsqpEigen::SparseMatrixHelper::osqpSparseMatrixToTriplets(osqpSparseMatrix, tripletList);

    // resize the eigen matrix
    eigenSparseMatrix.resize(rows, cols);

    // set the eigen matrix from triplets
    eigenSparseMatrix.setFromTriplets(tripletList.begin(), tripletList.end());
    return true;
}

template<typename Derived, typename T>
bool OsqpEigen::SparseMatrixHelper::eigenSparseMatrixToTriplets(const Eigen::SparseCompressedBase<Derived> &eigenSparseMatrix,
                                                                std::vector<Eigen::Triplet<T>> &tripletList)
{
    if(eigenSparseMatrix.nonZeros() == 0){
        std::cerr << "[OsqpEigen::SparseMatrixHelper::eigenSparseMatrixToTriplets] The eigenSparseMatrix is empty."
                  << std::endl;
        return false;
    }

    tripletList.resize(eigenSparseMatrix.nonZeros());
    // populate the triplet list
    int nonZero = 0;
    for (int k=0; k < eigenSparseMatrix.outerSize(); ++k){
        for (typename Eigen::SparseCompressedBase<Derived>::InnerIterator it(eigenSparseMatrix,k); it; ++it){
            tripletList[nonZero] = Eigen::Triplet<T>(it.row(), it.col(), static_cast<T>(it.value()));
            nonZero++;
        }
    }
    tripletList.erase(tripletList.begin() + eigenSparseMatrix.nonZeros(), tripletList.end());

    return true;
}
