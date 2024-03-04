/// (C) Copyright International Business Machines Corporation 2007
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// Authors :
// Pierre Bonami, International Business Machines Corporation
//
// Date : 10/06/2007

#ifndef BonQuadRow_H
#define BonQuadRow_H

#include "CoinPackedVector.hpp"
#include "BonTMatrix.hpp"
#include "BonQuadCut.hpp"

namespace Bonmin{

  /** Store column and row of the entry.*/
  typedef std::pair<int, int> matEntry;
  /** Store the number of times entry is used and its index in the matrix.*/
  typedef std::pair<int, int> matIdx;
#if HAS_HASH_MAP
  typedef std::has_map<matEntry, matIdx, std::hash< matEntry> > AdjustableMat;
#else
  typedef std::map<matEntry, matIdx> AdjustableMat;
#endif

/** Stores a quadratic row of the form l < c + ax + x^T Q x < u. 
    Does computation usefull for nlp-solver.
    It can only be initialized from a QuadCut.*/
class QuadRow {
 public:
 /** Default constructor.*/
 QuadRow();

 /** Copy constructor.*/
 QuadRow(const QuadRow & other);

 /** Assignment operator.*/
 QuadRow& operator=(const QuadRow& rhs);

 /** Constructor from a quadratic cut.*/
 QuadRow(const QuadCut &cut);

 /** Assignment form a quadrattic &cut.*/
 QuadRow& operator=(const QuadCut & rhs);

 /** Constructor from a linear cut.*/
 QuadRow(const OsiRowCut &cut);

 /** Assignment form a linear &cut.*/
 QuadRow& operator=(const OsiRowCut & rhs);

 /** Evaluate quadratic form.*/
 double eval_f(const double *x, bool new_x);

 /** Get number of non-zeroes in the gradiant.*/
 int nnz_grad();
 /** Get structure of gradiant */
  void gradiant_struct(const int nnz, int * indices, bool offset);
 /** Evaluate gradiant of quadratic form.*/
 void eval_grad(const int nnz, const double * x, bool new_x, double * values);

 /** number of non-zeroes in hessian. */
 int nnz_hessian(){
   return Q_.nnz_;}

 /** Says if the constraint is linear.*/
 bool isLinear(){
   return Q_.nnz_ == 0;}

 /** Return hessian value (i.e. Q_).*/
 void eval_hessian(double lambda, double * values);

 /** Add row to a bigger hessian.*/
  void add_to_hessian(AdjustableMat &H, bool offset); 

 /** Remove row from a bigger hessian.*/ 
  void remove_from_hessian(AdjustableMat &H);
/** Print quadratic constraint.*/
void print();

 private:
 /** Initialize once quadratic form is know.*/
 void initialize();

 /** Does internal work to evaluate gradiant of this in x.*/
 void internal_eval_grad(const double *x);

 /** lower bound.*/
 double lb_;
 /** upper bound.*/
 double ub_;
 /** Constant term.*/
 double c_;
 /** linear term in sparse storage.*/
 CoinPackedVector a_;
 /** Quadratic term.*/
 TMat Q_;


#if HAS_HASH_MAP
  typedef  std::has_map<int, std::pair<double, double >, std::hash<int> > gStore;
#else
  typedef std::map<int, std::pair<double, double> > gStore;
#endif

 gStore g_;
 /** To have fast access to gradiant entries for a_.*/
 std::vector<gStore::iterator> a_grad_idx_;
 /** To have fast access to gradient entries for rows Q_*/
 std::vector<gStore::iterator> Q_row_grad_idx_;
 /** To have fast access to gradient entries for cols Q_*/
 std::vector<gStore::iterator> Q_col_grad_idx_;
 /** To have fast access to entries in full hessian of Q_*/
 std::vector<AdjustableMat::iterator> Q_hessian_idx_;
 /** Flag indicating if gradiant has been evaluated.*/
 bool grad_evaled_;
};
}//End Bonmin namespace
#endif
