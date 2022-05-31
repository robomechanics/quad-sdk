#include <quad_utils/math_utils.h>

namespace math_utils {

std::vector<double> interpMat(std::vector<double> input_vec,
                              std::vector<std::vector<double>> input_mat,
                              double query_point) {
  // Check bounds, throw an error if invalid since this shouldn't ever happen
  if ((query_point < input_vec.front()) || (query_point > input_vec.back())) {
    throw std::runtime_error("Tried to interp out of bounds");
  }

  // Declare variables for interpolating between, both for input and output data
  double t1, t2;
  std::vector<double> y1, y2, interp_data;

  // Find the correct values to interp between
  for (int i = 0; i < input_vec.size(); i++) {
    if (input_vec[i] <= query_point && query_point < input_vec[i + 1]) {
      t1 = input_vec[i];
      t2 = input_vec[i + 1];
      y1 = input_mat[i];
      y2 = input_mat[i + 1];
      break;
    }
  }

  // Apply linear interpolation for each element in the vector
  for (int i = 0; i < input_mat.front().size(); i++) {
    double result = y1[i] + (y2[i] - y1[i]) / (t2 - t1) * (query_point - t1);
    interp_data.push_back(result);
  }

  return interp_data;
}

Eigen::Vector3d interpVector3d(std::vector<double> input_vec,
                               std::vector<Eigen::Vector3d> input_mat,
                               double query_point) {
  // Check bounds, throw an error if invalid since this shouldn't ever happen
  if ((query_point < input_vec.front()) || (query_point > input_vec.back())) {
    throw std::runtime_error("Tried to interp out of bounds");
  }

  // Declare variables for interpolating between, both for input and output data
  double t1, t2;
  Eigen::Vector3d y1, y2, interp_data;

  // Find the correct values to interp between
  for (int i = 0; i < input_vec.size(); i++) {
    if (input_vec[i] <= query_point && query_point < input_vec[i + 1]) {
      t1 = input_vec[i];
      t2 = input_vec[i + 1];
      y1 = input_mat[i];
      y2 = input_mat[i + 1];
      break;
    }
  }

  // Apply linear interpolation for each element in the vector
  interp_data =
      y1.array() + (y2.array() - y1.array()) / (t2 - t1) * (query_point - t1);

  return interp_data;
}

std::vector<Eigen::Vector3d> interpMatVector3d(
    std::vector<double> input_vec,
    std::vector<std::vector<Eigen::Vector3d>> output_mat, double query_point) {
  // Check bounds, throw an error if invalid since this shouldn't ever happen
  if ((query_point < input_vec.front()) || (query_point > input_vec.back())) {
    throw std::runtime_error("Tried to interp out of bounds");
  }

  // Declare variables for interpolating between, both for input and output data
  double t1, t2;
  std::vector<Eigen::Vector3d> y1, y2, interp_data;

  // Find the correct values to interp between
  for (int i = 0; i < input_vec.size(); i++) {
    if (input_vec[i] <= query_point && query_point < input_vec[i + 1]) {
      t1 = input_vec[i];
      t2 = input_vec[i + 1];
      y1 = output_mat[i];
      y2 = output_mat[i + 1];
      break;
    }
  }

  // Apply linear interpolation for each element in the vector
  for (int i = 0; i < output_mat.front().size(); i++) {
    Eigen::Vector3d interp_eigen_vec;
    interp_eigen_vec = y1[i].array() + (y2[i].array() - y1[i].array()) /
                                           (t2 - t1) * (query_point - t1);
    interp_data.push_back(interp_eigen_vec);
  }

  return interp_data;
}

int interpInt(std::vector<double> input_vec, std::vector<int> output_vec,
              double query_point) {
  // Check bounds, throw an error if invalid since this shouldn't ever happen
  if ((query_point < input_vec.front()) || (query_point > input_vec.back())) {
    throw std::runtime_error("Tried to interp out of bounds");
  }

  // Declare variables for interpolating between, both for input and output data
  double t1, t2;
  Eigen::Vector3d y1, y2, interp_data;

  // Find the correct values to interp between
  int idx = 0;
  for (int i = 0; i < input_vec.size(); i++) {
    if (input_vec[i] <= query_point && query_point < input_vec[i + 1]) {
      return output_vec[i];
    }
  }

  throw std::runtime_error("Didn't find the query point, something happened");
}

std::vector<double> movingAverageFilter(std::vector<double> data,
                                        int window_size) {
  std::vector<double> filtered_data;
  int N = data.size();

  // Check to ensure window size is an odd integer, if not add one to make it so
  if ((window_size % 2) == 0) {
    window_size += 1;
    ROS_WARN_THROTTLE(
        0.5, "Filter window size is even, adding one to maintain symmetry");
  }

  // Make sure that the window size is acceptable
  if (window_size >= N) {
    ROS_WARN_THROTTLE(0.5, "Filter window size is bigger than data");
  }

  // Loop through the data
  for (int i = 0; i < N; i++) {
    // Initialize sum and count of data samples
    double sum = 0;
    double count = 0;

    // Shrink the window size if it would result in out of bounds data
    int current_window_size = std::min(window_size, 2 * i + 1);
    // int current_window_size = window_size;

    // Loop through the window, adding to the sum and averaging
    for (int j = 0; j < current_window_size; j++) {
      double index = i + (j - (current_window_size - 1) / 2);

      // Make sure data is in bounds
      if (index >= 0 && index < N) {
        sum += data[index];
        count += 1;
      }
    }

    filtered_data.push_back((float)sum / count);
  }

  return filtered_data;
}

std::vector<double> centralDiff(std::vector<double> data, double dt) {
  std::vector<double> data_diff;

  for (int i = 0; i < data.size(); i++) {
    // Compute lower and upper indices, with forward/backward difference at the
    // ends
    int lower_index = std::max(i - 1, 0);
    int upper_index = std::min(i + 1, (int)data.size() - 1);

    double estimate = (data[upper_index] - data[lower_index]) /
                      (dt * (upper_index - lower_index));
    data_diff.push_back(estimate);
  }

  return data_diff;
}

std::vector<double> unwrap(std::vector<double> data) {
  std::vector<double> data_unwrapped = data;
  for (int i = 1; i < data.size(); i++) {
    double diff = data[i] - data[i - 1];
    if (diff > M_PI) {
      for (int j = i; j < data.size(); j++) {
        data_unwrapped[j] = data_unwrapped[j] - 2 * M_PI;
      }
    } else if (diff < -M_PI) {
      for (int j = i; j < data.size(); j++) {
        data_unwrapped[j] = data_unwrapped[j] + 2 * M_PI;
      }
    }
  }

  return data_unwrapped;
}

Eigen::MatrixXd sdlsInv(const Eigen::MatrixXd &jacobian, double ratio) {
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);

  Eigen::VectorXd sig = svd.singularValues();
  Eigen::VectorXd sig_inv = sig;

  for (size_t i = 0; i < sig.size(); i++) {
    sig_inv(i) = std::min(1 / sig(i), 1 / 5e-2);
  }

  Eigen::MatrixXd jacobian_inv =
      svd.matrixV() * sig_inv.asDiagonal() * svd.matrixU().transpose();

  return jacobian_inv;
}

Eigen::MatrixXd sdlsInv(const Eigen::MatrixXd &jacobian) {
  return sdlsInv(jacobian, 1e-1);
}
}  // namespace math_utils
