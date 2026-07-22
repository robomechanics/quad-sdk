#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "quad_utils/matrix_utils.hpp"
#include "quad_utils/math_utils.hpp"
#include "quad_utils/ros_utils.hpp"

namespace {
constexpr double kTol = 1e-9;
}

TEST(MathTest, testWrap) {
  int N = 201;
  double amplitude = 10;
  double period = 4 * M_PI;
  std::vector<double> data(N), t(N);
  for (int i = 0; i < data.size(); i++) {
    t[i] = i * period / N;
    data[i] = amplitude * sin(t[i]);
  }

  std::vector<double> data_wrapped = math_utils::wrapToPi(data);
  std::vector<double> data_unwrapped =
      math_utils::getUnwrappedVector(data_wrapped);

  double error = 0;
  for (int i = 0; i < data.size(); i++) {
    error += abs(data[i] - data_unwrapped[i]);
  }

  double tolerance = 1e-4;
  EXPECT_TRUE(error <= tolerance);
}

TEST(MathTest, InterpolationAndFilters) {
  EXPECT_DOUBLE_EQ(math_utils::lerp(2.0, 6.0, 0.25), 3.0);
  EXPECT_NEAR(math_utils::wrapTo2Pi(-M_PI / 2.0), 1.5 * M_PI, kTol);
  EXPECT_NEAR(math_utils::wrapToPi(3.0 * M_PI), -M_PI, kTol);

  const std::vector<double> times{0.0, 1.0, 2.0};
  const std::vector<std::vector<double>> values{{0.0, 10.0},
                                                {2.0, 12.0},
                                                {4.0, 14.0}};
  const auto interp = math_utils::interpMat(times, values, 0.5);
  ASSERT_EQ(interp.size(), 2u);
  EXPECT_NEAR(interp[0], 1.0, kTol);
  EXPECT_NEAR(interp[1], 11.0, kTol);
  EXPECT_THROW(math_utils::interpMat(times, values, -0.1), std::runtime_error);

  const std::vector<Eigen::Vector3d> vec_values{
      Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(2.0, 4.0, 6.0),
      Eigen::Vector3d(4.0, 8.0, 12.0)};
  EXPECT_TRUE(math_utils::interpVector3d(times, vec_values, 1.5)
                  .isApprox(Eigen::Vector3d(3.0, 6.0, 9.0)));

  const std::vector<std::vector<Eigen::Vector3d>> mat_vec_values{
      {Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(1.0, 1.0, 1.0)},
      {Eigen::Vector3d(2.0, 4.0, 6.0), Eigen::Vector3d(3.0, 5.0, 7.0)},
      {Eigen::Vector3d(4.0, 8.0, 12.0), Eigen::Vector3d(5.0, 9.0, 13.0)}};
  const auto interp_vecs =
      math_utils::interpMatVector3d(times, mat_vec_values, 0.5);
  ASSERT_EQ(interp_vecs.size(), 2u);
  EXPECT_TRUE(interp_vecs[0].isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
  EXPECT_TRUE(interp_vecs[1].isApprox(Eigen::Vector3d(2.0, 3.0, 4.0)));

  EXPECT_EQ(math_utils::interpInt(times, {2, 4, 6}, 1.25), 4);

  const auto filtered = math_utils::movingAverageFilter({1.0, 2.0, 100.0,
                                                         4.0, 5.0},
                                                        2);
  ASSERT_EQ(filtered.size(), 5u);
  EXPECT_NEAR(filtered[0], 1.0, kTol);
  EXPECT_NEAR(filtered[1], (1.0 + 2.0 + 100.0) / 3.0, kTol);
  EXPECT_NEAR(filtered[4], (4.0 + 5.0) / 2.0, kTol);

  const auto diff = math_utils::centralDiff({0.0, 1.0, 4.0, 9.0}, 1.0);
  ASSERT_EQ(diff.size(), 4u);
  EXPECT_NEAR(diff[0], 1.0, kTol);
  EXPECT_NEAR(diff[1], 2.0, kTol);
  EXPECT_NEAR(diff[2], 4.0, kTol);
  EXPECT_NEAR(diff[3], 5.0, kTol);
}

TEST(MathTest, MatrixUtilities) {
  Eigen::MatrixXd a(2, 2);
  a << 1.0, 2.0, 3.0, 4.0;
  Eigen::MatrixXd b(2, 1);
  b << 5.0, 6.0;

  Eigen::MatrixXd kron_expected(4, 2);
  kron_expected << 5.0, 10.0, 6.0, 12.0, 15.0, 20.0, 18.0, 24.0;
  EXPECT_TRUE(math::kron(a, b).isApprox(kron_expected));

  Eigen::MatrixXd block_expected = Eigen::MatrixXd::Zero(4, 3);
  block_expected.block(0, 0, 2, 2) = a;
  block_expected.block(2, 2, 2, 1) = b;
  EXPECT_TRUE(math::block_diag(a, b).isApprox(block_expected));

  Eigen::MatrixXd c(1, 2);
  c << 7.0, 8.0;
  Eigen::MatrixXd block3_expected = Eigen::MatrixXd::Zero(5, 5);
  block3_expected.block(0, 0, 2, 2) = a;
  block3_expected.block(2, 2, 2, 1) = b;
  block3_expected.block(4, 3, 1, 2) = c;
  EXPECT_TRUE(math::block_diag(a, b, c).isApprox(block3_expected));

  Eigen::MatrixXd reshaped = math::reshape(a, 4, 1);
  ASSERT_EQ(reshaped.rows(), 4);
  ASSERT_EQ(reshaped.cols(), 1);
  EXPECT_NEAR(reshaped(0, 0), 1.0, kTol);
  EXPECT_NEAR(reshaped(1, 0), 3.0, kTol);
  EXPECT_TRUE(math_utils::sdlsInv(Eigen::Matrix2d::Identity())
                  .isApprox(Eigen::Matrix2d::Identity()));
}

TEST(MathTest, InterpolationReturnsLastSampleAtEndpoint) {
  const std::vector<double> times{0.0, 1.0, 2.0};
  const std::vector<std::vector<double>> values{{0.0, 10.0},
                                                {2.0, 12.0},
                                                {4.0, 14.0}};
  EXPECT_EQ(math_utils::interpMat(times, values, 2.0), values.back());

  const std::vector<Eigen::Vector3d> vec_values{
      Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(2.0, 4.0, 6.0),
      Eigen::Vector3d(4.0, 8.0, 12.0)};
  EXPECT_TRUE(math_utils::interpVector3d(times, vec_values, 2.0)
                  .isApprox(vec_values.back()));

  const std::vector<std::vector<Eigen::Vector3d>> mat_vec_values{
      {Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(1.0, 1.0, 1.0)},
      {Eigen::Vector3d(2.0, 4.0, 6.0), Eigen::Vector3d(3.0, 5.0, 7.0)},
      {Eigen::Vector3d(4.0, 8.0, 12.0), Eigen::Vector3d(5.0, 9.0, 13.0)}};
  const auto interp_vecs =
      math_utils::interpMatVector3d(times, mat_vec_values, 2.0);
  ASSERT_EQ(interp_vecs.size(), mat_vec_values.back().size());
  EXPECT_TRUE(interp_vecs[0].isApprox(mat_vec_values.back()[0]));
  EXPECT_TRUE(interp_vecs[1].isApprox(mat_vec_values.back()[1]));

  EXPECT_EQ(math_utils::interpInt(times, {2, 4, 6}, 2.0), 6);
}
