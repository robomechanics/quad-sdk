#include <chrono>

#include <spirit_utils/interpolator.h>
#include <libInterpolate/Interpolate.hpp>

int main(int argc, char** argv)
{

  int N = 100; // Number of iterations

  int M = 10000; // Vector size
  std::vector<double> X(M);
  std::generate(X.begin(), X.end(), std::rand);
  std::sort(X.begin(), X.end());

  std::vector<double> Y(M);
  for (size_t i = 0; i < M; ++i)
  {
    Y.at(i) = 0.93274937 + 0.73343 * X.at(i) + 0.143746 * (X.at(i) * X.at(i)); // Random quadratic
  }

  double X_min = X.front();
  double X_max = X.back();
  std::vector<double> Xq(N);
  for (size_t i = 0; i < N; ++i)
  {
    double f = (double)rand()/RAND_MAX;
    Xq.at(i) = X_min + f*(X_max-X_min);
  }

  // Custom linear interpolation from spirit_utils
  Interpolator interpA;

  std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
  for (size_t i = 0; i < N; ++i)
  {
    interpA.interp1(X,Y,Xq.at(i));
  }
  std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1);
  std::cout << "Linear interp with spirit utils (vector size " << M << ", average single interp): " << time_span.count()/N*1e3 << " milliseconds." << std::endl;

  // Linear interpolation from libInterpolate
  _1D::LinearInterpolator<double> interpB;
  interpB.setData(X,Y);

  t1 = std::chrono::high_resolution_clock::now();
  for (size_t i = 0; i < N; ++i)
  {
    interpB(Xq.at(i));
  }
  t2 = std::chrono::high_resolution_clock::now();
  time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1);
  std::cout << "Linear interp with libInterpolate (vector size " << M << ", average single interp): " << time_span.count()/N*1e3 << " milliseconds." << std::endl;

  return 0;
}