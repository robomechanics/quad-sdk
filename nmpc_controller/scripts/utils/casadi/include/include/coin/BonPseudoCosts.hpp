// (C) Copyright International Business Machines Corporation 2007
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// Authors :
// Pierre Bonami, International Business Machines Corporation
//
// Date : 04/12/2007

#ifndef BonPseudoCosts_H
#define BonPseudoCosts_H

#include "OsiChooseVariable.hpp"
namespace Bonmin
{

  class PseudoCosts: public OsiPseudoCosts
  {
  public:
    /** Default constructor.*/
    PseudoCosts();

    /** Copy constructor.*/
    PseudoCosts(const PseudoCosts & rhs);

    /** Assignment operator const version.*/
    PseudoCosts & operator=(const PseudoCosts&rhs);
#if 0
    /** Acces upTotalChange.*/
    inline double * upTotalChange()
    {
      return upTotalChange_;
    }

    /** Acces downTotalChange.*/
    inline double * downTotalChange()
    {
      return downTotalChange_;
    }

    /** Acces upNumber.*/
    inline int * upNumber()
    {
      return upNumber_;
    }

    /** Acces downNumber.*/
    inline int * downNumber()
    {
      return downNumber_;
    }

    /** Acces upTotalChange.*/
    inline const double * upTotalChange() const
    {
      return upTotalChange_;
    }

    /** Acces downTotalChange.*/
    inline const double * downTotalChange() const
    {
      return downTotalChange_;
    }

    /** Acces upNumber.*/
    inline const int * upNumber() const
    {
      return upNumber_;
    }

    /** Acces downNumber.*/
    inline const int * downNumber() const
    {
      return downNumber_;
    }

    /** Access number objects.*/
    inline int numberObjects() const
    {
      return numberObjects_;
    }
#endif
    /** Add a pseudo cost information.*/
    void addInfo(int way, double originalObj, double originalInfeas,
        double newObj, double newInfeas, int status);

  };

}/* End Bonmin namespace.*/

#endif
