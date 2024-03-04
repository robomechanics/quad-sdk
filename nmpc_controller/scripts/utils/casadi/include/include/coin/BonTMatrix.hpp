// (C) Copyright International Business Machines Corporation 2007
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// Authors :
// Pierre Bonami, International Business Machines Corporation
//
// Date : 10/06/2007

#ifndef BonTMatrix_H
#define BonTMatrix_H

#include "CoinPackedMatrix.hpp"
#include "BonArraysHelpers.hpp"
#include <vector>
#include <list>
#include <algorithm> 
#include "BonQuadCut.hpp"

namespace Bonmin {

struct TMat{
  int * iRow_;
  int * jCol_;
  double * value_;
  int nnz_;
  int capacity_;


 /** Storage for non empty rows.
     first is row number and second is first element in row.*/
 typedef vector< std::pair< int, int> > RowS;

  /** Default constructor.*/
  TMat(): iRow_(NULL), jCol_(NULL), value_(NULL), nnz_(0),
               capacity_(0)
  {}


  void freeSpace(){
     delete [] iRow_;
     delete [] jCol_;
     delete [] value_;
  } 

  /** Copy constructor.*/
  TMat(const TMat &other);

  /** Construct from a CoinPackedMatrix*/
  TMat(const CoinPackedMatrix &M,  MatrixStorageType T);

  /** Assignment operator.*/
  TMat& operator=(const TMat &rhs);

  /** Assignment from a CoinPackedMatrix.*/
  TMat & operator=(const CoinPackedMatrix &M);

  void resize(int nnz){
    Bonmin::resizeAndCopyArray(iRow_, nnz_, nnz);
    Bonmin::resizeAndCopyArray(jCol_, nnz_, nnz);
    Bonmin::resizeAndCopyArray(value_, nnz_, nnz);
    nnz_ = nnz;
  }

  ~TMat();

 /** Get number of non empty rows.*/
 int numNonEmptyRows();

 /** Get the list of non empty row.*/
 const RowS & nonEmptyRows() const {
    return nonEmptyRows_;}

 /** Get number of non empty cols.*/
 int numNonEmptyCols();

 /** Get the list of non empty row.*/
 const RowS & nonEmptyCols() const {
    return nonEmptyCols_;}

 private:
 /** Structure for ordering matrix.*/
 struct TMatOrdering{
   TMat * M_;
   TMatOrdering(TMat *M):
     M_(M){}
 };

 /** Structure for ordering matrix by columns.*/ 
 struct ColumnOrder : public TMatOrdering {
   ColumnOrder(TMat *M):
     TMatOrdering(M){}

   bool operator()(const int& i, const int& j){
      if (M_->jCol_[i] < M_->jCol_[j])
        return true;
      if (M_->jCol_[i] == M_->jCol_[j] && M_->iRow_[i] < M_->iRow_[j])
        return true;
     return false;
   }
 };


 /** Structure for ordering matrix by columns.*/ 
 struct RowOrder : public TMatOrdering {
   RowOrder(TMat *M):
     TMatOrdering(M){}
   bool operator()(const int& i, const int& j){
      if (M_->iRow_[i]< M_->iRow_[j])
        return true;
      if (M_->iRow_[i] == M_->iRow_[j] && M_->jCol_[i] < M_->jCol_[j])
        return true;
     return false;
   }
 };
 public:
 /** Orders current matrix by columns. */
 const vector<int>& orderByColumns(){
    resizeOrdering(columnOrdering_, nnz_);
    std::sort(columnOrdering_.begin(), columnOrdering_.end(),ColumnOrder(this));
    return columnOrdering_;
 }
 /** Orders current matrix by rows.*/
 const vector<int>& orderByRows(){
    resizeOrdering(rowOrdering_, nnz_);
    std::sort(rowOrdering_.begin(), rowOrdering_.end(), RowOrder(this));
    return rowOrdering_;
 }

 /** Remove the duplicated entries.*/
 void removeDuplicates();

 /** Assuming that this is representing a quadratic form. Produce equivalent
     quadratic form with only upper triange stored.*/
 void makeQuadUpperDiag();

 void resizeOrdering(vector<int> &ordering, unsigned int newSize){
        size_t oldSize = ordering.size();
        ordering.resize(newSize);
        for(size_t i = oldSize ; i < newSize ; i++)
           ordering[i] = static_cast<int>(i);
   }

   /** Create the TMat from M.*/
   void create(const CoinPackedMatrix &M);
 
   vector<int> columnOrdering_;
 
   vector<int> rowOrdering_;

   void make_upper_triangular(const MatrixStorageType &T);

   void make_lower_to_be_upper();

   void make_full_upper_triangular();

   // Stores non empty rows for computing jacobian structure
   RowS nonEmptyRows_;

   // Stores non empty cols for computing jacobian structure
   RowS nonEmptyCols_;
 };

}//Ends Bonmin namespace

#endif

