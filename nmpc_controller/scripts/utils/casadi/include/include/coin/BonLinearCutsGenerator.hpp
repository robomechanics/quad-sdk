// (C) Copyright International Business Machines Corporation 2007
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// Authors :
// Pierre Bonami, International Business Machines Corporation
//
// Date : 10/06/2007

#ifndef BonLinearCutsGenerator_H
#define BonLinearCutsGenerator_H

#include "CglCutGenerator.hpp"
#include "CoinSmartPtr.hpp"
#include "BonOuterApprox.hpp"
#include "BonBonminSetup.hpp"
#include <list>

namespace Bonmin {
class LinearCutsGenerator : public CglCutGenerator {
   public:
    /** Type for cut generation method with its frequency and string identification. */
    struct CuttingMethod : public Coin::ReferencedObject 
    {
      int frequency;
      std::string id;
      CglCutGenerator * cgl;
      bool atSolution;
      bool normal;
      CuttingMethod():
          atSolution(false),
          normal(true)
      {}

      CuttingMethod(const CuttingMethod & other):
          frequency(other.frequency),
          id(other.id),
          cgl(other.cgl),
          atSolution(other.atSolution),
          normal(other.normal)
      {}
    };
   LinearCutsGenerator():
     CglCutGenerator(),
     methods_(){
   }


   LinearCutsGenerator(const LinearCutsGenerator & other):
    CglCutGenerator(other),
     methods_(other.methods_){
   }

   CglCutGenerator * clone() const {
     return new LinearCutsGenerator(*this);
   }

   virtual ~LinearCutsGenerator(){
   }

   bool needsOptimalBasis() { return false;}

   void initialize(BabSetupBase& s);

   void generateCuts(const OsiSolverInterface &solver, OsiCuts &cs,
		     const CglTreeInfo info = CglTreeInfo());

   private:
     std::list<Coin::SmartPtr<CuttingMethod> > methods_; 
};

}/* Ends Bonmin namespace.*/

#endif

