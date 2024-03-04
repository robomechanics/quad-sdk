/* $Id: ClpCholeskyPardiso.hpp 1665 2011-01-04 17:55:54Z lou $ */
// Copyright (C) 2003, International Business Machines
// Corporation and others.  All Rights Reserved.
// This code is licensed under the terms of the Eclipse Public License (EPL).

#ifndef ClpCholeskyPardiso_H
#define ClpCholeskyPardiso_H
#ifdef PARDISO_BARRIER

#include "ClpCholeskyBase.hpp"
#include "mkl_pardiso.h"
#include "mkl_types.h"

class ClpMatrixBase;
class ClpCholeskyDense;

/** Pardiso class for Clp Cholesky factorization

*/
class ClpCholeskyPardiso : public ClpCholeskyBase {

public:
  /**@name Virtual methods that the derived classes provides  */
  //@{
  /** Orders rows and saves pointer to matrix.and model.
      Returns non-zero if not enough memory */
  virtual int order(ClpInterior *model);
  /** Does Symbolic factorization given permutation.
         This is called immediately after order.  If user provides this then
         user must provide factorize and solve.  Otherwise the default factorization is used
         returns non-zero if not enough memory */
  virtual int symbolic();
  /** Factorize - filling in rowsDropped and returning number dropped.
         If return code negative then out of memory */
  virtual int factorize(const double *diagonal, int *rowsDropped);
  /** Uses factorization to solve. */
  virtual void solve(double *region);
  //@}

  /**@name Constructors, destructor */
  //@{
  /** Constructor which has dense columns activated.
         Default is off. */
  ClpCholeskyPardiso(int denseThreshold = -1);
  /** Destructor  */
  virtual ~ClpCholeskyPardiso();
  // Copy
  ClpCholeskyPardiso(const ClpCholeskyPardiso &);
  // Assignment
  ClpCholeskyPardiso &operator=(const ClpCholeskyPardiso &);
  /// Clone
  virtual ClpCholeskyBase *clone() const;
  //@}

private:
  /**@name Data members */
  //@{
  int lastNumberDropped_;
  //MKL_INT iparm_[64];
  //@}
};

#endif
#endif

/* vi: softtabstop=2 shiftwidth=2 expandtab tabstop=2
*/
