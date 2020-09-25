#include <spirit_utils/fast_interpolator.h>

FastInterpolator::FastInterpolator() {

}

double FastInterpolator::interp1(std::vector<double> X, std::vector<double> Y, double Xq) {
  // Handle out of bounds error
  if (Xq < X.front() || Xq > X.back())
  {
    throw std::runtime_error("Requested value of Xq (" + std::to_string(Xq)
           + ") outside of the range of X [" + std::to_string(X.front()) + ", " + std::to_string(X.back()) + "].");
  }

  std::vector<double>::iterator high;
  high = std::lower_bound(X.begin(), X.end(), Xq); // Confusingly named, finds first element in X >= Xq
  
  int hi_index = high - X.begin();
  int lo_index = hi_index - 1;

  double t = (Xq - X.at(lo_index))/(X.at(hi_index) - X.at(lo_index));

  // Linear interpolation
  double Yq = Y.at(lo_index) + t * (Y.at(hi_index) - Y.at(lo_index));

  return Yq;
}

std::vector<double> FastInterpolator::interp1(std::vector<double> X, std::vector<double> Y, std::vector<double> Xq) {

  std::vector<double> Yq(Xq.size());
  for (size_t i = 0; i < Xq.size(); ++i)
  {
    Yq.at(i) = interp1(X,Y,Xq.at(i));
  }
  return Yq;
}