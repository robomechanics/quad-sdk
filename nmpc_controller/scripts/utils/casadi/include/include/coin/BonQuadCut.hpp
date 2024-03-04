// (C) Copyright International Business Machines Corporation 2007
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// Authors :
// Pierre Bonami, International Business Machines Corporation
//
// Date : 10/06/2007

#ifndef BonQuadCut_H
#define BonQuadCut_H

#include "CoinPackedMatrix.hpp"
#include "OsiRowCut.hpp"
#include "OsiCuts.hpp"
#include "BonTypes.hpp"
#include <list>


namespace Bonmin {

  enum MatrixStorageType {
   Upper /** Stores only the upper triangle of a symetric Q.*/,
   Lower /** Stores the lower triangle of a symetric Q.*/,
   Full /** Stores the whole matrix of a non-symetric Q.*/};

class QuadCut : public OsiRowCut {
 public:

  /// Default constructor
  QuadCut();

  /// Copy constructor
  QuadCut(const QuadCut & other);

  /// Assignment operator
  QuadCut& operator=(const QuadCut & rhs);

  /// Virtual copy
  virtual OsiRowCut * clone() const;

  /// Destructor
  ~QuadCut(); 

  /// Print
  void print() const; 

  ///Return the matrix stored
  CoinPackedMatrix& Q(){
   return Q_;
  }

  ///Return the matrix stored
  const CoinPackedMatrix& Q() const{
   return Q_;
  }

  /// Acces storage type
  /// Acces storage type
  MatrixStorageType& type(){
    return type_;}

  const MatrixStorageType& type() const{
    return type_;}

  /// Acces the constant
  double & c(){return c_;}

  /// Acces the constant
  const double & c() const {return c_;}

  /// Compute cut violation
  double violated(const double * solution) const;

 private:
   /// Stores the constant part of the cut
   double c_;
   ///Stores quadratic part of cut
   CoinPackedMatrix Q_;
   ///Storage type
   MatrixStorageType type_;

   /** \name Arithmetic operators not implemented.*/
  //@{
    /// add <code>value</code> to every vector entry
    void operator+=(double value);

    /// subtract <code>value</code> from every vector entry
    void operator-=(double value);

    /// multiply every vector entry by <code>value</code>
    void operator*=(double value);

    /// divide every vector entry by <code>value</code>
    void operator/=(double value);
  //@}

};

/** Generalizes OsiCuts to handle quadratic cuts.*/
class Cuts : public OsiCuts {
 public:
  typedef vector<QuadCut *> QuadCutPtrStorage;
  /** Default constructor.*/
  Cuts();

  /** Copy constructor.*/
  Cuts(const Cuts& other);

  /** Assignment operator.*/
  Cuts& operator=(const Cuts & rhs);

 /** Destructor */
 ~Cuts();

 /** insert a quadratic cut into the collection. */
 inline void insert(const QuadCut& c);

 /** insert a quadratic cut into the collection (take control of the pointer and
     put a NULL on return).
     \warning c has to have been created with new (no malloc).
   */
  inline void insert(QuadCut* &c);

 /** insert a set of Cuts.*/
  inline void insert(const Cuts &cs);

 /** Number of quadratic cuts in the collection.*/
  inline int sizeQuadCuts() const;

 /** Total number of cuts in the collection. */
 inline int sizeCuts() const;

 /** Print all cuts in the collection.*/
 void printCuts() const;


 /** Access to a quadratic cut by pointer.*/
 inline QuadCut * quadCutPtr(int i);

 /** Access to a quadratic cut by const pointer.*/
 inline const QuadCut * quadCutPtr(int i) const;

 /** Access to a quadratic cut by reference.*/
 inline QuadCut& quadCut(int i);


 /** Access to a quadratic cut by reference.*/
 inline const QuadCut& quadCut(int i) const;

 /** Erase quadratic cut from the collection.*/
 inline void eraseQuadCut(int i);

 private:
   QuadCutPtrStorage quadCuts_;
};

void
Cuts::insert(const QuadCut &c){
  quadCuts_.push_back(new QuadCut(c));
}

void
Cuts::insert(QuadCut * &c){
  quadCuts_.push_back(c);
  c = NULL;
}

void 
Cuts::insert(const Cuts & cs){
  OsiCuts::insert(cs);
  for(unsigned int i = 0 ; i < cs.quadCuts_.size() ; i++){
    quadCuts_.push_back(new QuadCut(*cs.quadCuts_[i]));
  }
}

int 
Cuts::sizeQuadCuts() const {
  return static_cast<int>(quadCuts_.size());
}

int
Cuts::sizeCuts() const {
  return static_cast<int>(quadCuts_.size()) + OsiCuts::sizeCuts();
}

QuadCut *
Cuts::quadCutPtr(int i) {
  return quadCuts_[i];
}

const QuadCut *
Cuts::quadCutPtr(int i) const {
  return quadCuts_[i];
}

QuadCut &
Cuts::quadCut(int i) {
  return *quadCuts_[i];
}

const QuadCut &
Cuts::quadCut(int i) const {
  return *quadCuts_[i];
}

void
Cuts::eraseQuadCut(int i){
  delete quadCuts_[i];
  quadCuts_.erase(quadCuts_.begin() + i);
}
typedef std::list<QuadCut*> list_QuadCut; 

}// Ends Bonmin namespace
#endif


