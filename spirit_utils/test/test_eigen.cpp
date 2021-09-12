#include <ros/ros.h>
#include <gtest/gtest.h>

#include "spirit_utils/ros_utils.h"
Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

TEST(EigenTest, testMap) {

  const int N = 9;
  double data_c[N];
  for(int i = 0; i < N; ++i) {
    data_c[i] = (double) i;
  }

  // std::cout << Eigen::Map<Eigen::VectorXi>(array) << std::endl;
  
  Eigen::MatrixXd data_eigen;

  data_eigen = Eigen::Map<Eigen::Matrix<double,1,N>>(data_c);

  // std::cout << data_eigen.format(CleanFmt) << std::endl;
}