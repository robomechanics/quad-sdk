// Authors: Matthew Saltzman and Ted Ralphs
// Copyright 2015, Matthew Saltzman and Ted Ralphs
// Licensed under the Eclipse Public License 1.0

#ifndef CoinRational_H
#define CoinRational_H

#include <cmath>

//Small class for rational numbers
class CoinRational {

public:
  long getDenominator() { return denominator_; }
  long getNumerator() { return numerator_; }

  CoinRational()
    : numerator_(0)
    , denominator_(1) {};

  CoinRational(long n, long d)
    : numerator_(n)
    , denominator_(d) {};

  CoinRational(double val, double maxdelta, long maxdnom)
  {
    if (!nearestRational_(val, maxdelta, maxdnom)) {
      numerator_ = 0;
      denominator_ = 1;
    }
  };

private:
  long numerator_;
  long denominator_;

  bool nearestRational_(double val, double maxdelta, long maxdnom);
};

#endif

/* vi: softtabstop=2 shiftwidth=2 expandtab tabstop=2
*/
