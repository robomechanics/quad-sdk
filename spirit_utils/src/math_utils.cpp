#include <spirit_utils/math_utils.h>

std::vector<double> math_utils::interpMat(std::vector<double> input_vec, 
  std::vector<std::vector<double>> input_mat, double query_point) {

  // Check bounds, throw an error if invalid since this shouldn't ever happen
  if ((query_point < input_vec.front()) || (query_point > input_vec.back())){
    throw std::runtime_error("Tried to interp out of bounds");
  }

  // Declare variables for interpolating between, both for input and output data
  double t1, t2;
  std::vector<double> y1, y2, interp_data;

  // Find the correct values to interp between
  int idx=0;
  for(int i=0;i<input_vec.size();i++)
  {
      if(input_vec[i]<=query_point && query_point<input_vec[i+1])
      {
        t1 = input_vec[i];
        t2 = input_vec[i+1];
        y1 = input_mat[i];
        y2 = input_mat[i+1]; 
        break;
      }
  }

  // Apply linear interpolation for each element in the vector
  for (int i = 0; i<input_mat.front().size(); i++) {
    double result = y1[i] + (y2[i]-y1[i])/(t2-t1)*(query_point-t1);
    interp_data.push_back(result);
  }
  
  return interp_data;
}

Eigen::Vector3d math_utils::interpVector3d(std::vector<double> input_vec, std::vector<Eigen::Vector3d> input_mat, double query_point) {

  // Check bounds, throw an error if invalid since this shouldn't ever happen
  if ((query_point < input_vec.front()) || (query_point > input_vec.back())){
    throw std::runtime_error("Tried to interp out of bounds");
  }

  // Declare variables for interpolating between, both for input and output data
  double t1, t2;
  Eigen::Vector3d y1, y2, interp_data;

  // Find the correct values to interp between
  int idx=0;
  for(int i=0;i<input_vec.size();i++)
  {
      if(input_vec[i]<=query_point && query_point<input_vec[i+1])
      {
        t1 = input_vec[i];
        t2 = input_vec[i+1];
        y1 = input_mat[i];
        y2 = input_mat[i+1]; 
        break;
      }
  }

  // Apply linear interpolation for each element in the vector
  for (int i = 0; i<3; i++) {
    interp_data[i] = y1[i] + (y2[i]-y1[i])/(t2-t1)*(query_point-t1);
  }
  
  return interp_data;
}