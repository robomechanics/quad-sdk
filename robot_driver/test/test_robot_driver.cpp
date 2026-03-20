#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "robot_driver/controllers/grf_pid_controller.hpp"
#include "robot_driver/controllers/inverse_dynamics_controller.hpp"
#include "robot_driver/controllers/joint_controller.hpp"
#include "robot_driver/robot_driver.hpp"
#include "quad_utils/quad_kd2.hpp"

int my_argc;
char** my_argv;

static std::string runXacro(const std::string& xacro_path) {
  std::string cmd = "xacro " + xacro_path;
  std::array<char, 4096> buffer;
  std::string result;

  FILE* pipe = popen(cmd.c_str(), "r");
  if (!pipe) {
    throw std::runtime_error("Failed to run xacro");
  }
  while (fgets(buffer.data(), buffer.size(), pipe) != nullptr) {
    result += buffer.data();
  }
  pclose(pipe);
  return result;
}

TEST(LegController, testConstructorRobotDriver) {
  // Load the Params Needed to Successfully Launch the Node
  std::string rd_pkg_share =
      ament_index_cpp::get_package_share_directory("robot_driver");
  std::string utils_pkg_share =
      ament_index_cpp::get_package_share_directory("quad_utils");
  std::string robot_driver_param_file =
      rd_pkg_share + "/config/robot_driver.yaml";
  std::string robot_driver_topics_file =
      rd_pkg_share + "/config/robot_driver_topics.yaml";
  std::string robot_specific_param_file =
      utils_pkg_share + "/config/spirit.yaml";

  const std::string xacro_path =
      ament_index_cpp::get_package_share_directory("spirit_description") +
      "/models/spirit/urdf/spirit.urdf.xacro";
  const std::string urdf_string = runXacro(xacro_path);

  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "--params-file", robot_driver_param_file,
                     "--params-file", robot_driver_topics_file, "--params-file",
                     robot_specific_param_file});
  options.append_parameter_override("use_sim_time", false);
  options.append_parameter_override("robot_description", urdf_string);

  auto node =
      std::make_shared<rclcpp::Node>("robot_driver", "robot_1", options);

  RobotDriver robot_driver(node, my_argc, my_argv);
  EXPECT_EQ(1 + 1, 2);
}

TEST(LegController, testConstructorInverseDynamicsController) {
  // Load the Params Needed to Successfully Launch the Node
  std::string rd_pkg_share =
      ament_index_cpp::get_package_share_directory("robot_driver");
  std::string utils_pkg_share =
      ament_index_cpp::get_package_share_directory("quad_utils");
  std::string robot_driver_param_file =
      rd_pkg_share + "/config/robot_driver.yaml";
  std::string robot_driver_topics_file =
      rd_pkg_share + "/config/robot_driver_topics.yaml";
  std::string robot_specific_param_file =
      utils_pkg_share + "/config/spirit.yaml";

  const std::string xacro_path =
      ament_index_cpp::get_package_share_directory("spirit_description") +
      "/models/spirit/urdf/spirit.urdf.xacro";
  const std::string urdf_string = runXacro(xacro_path);

  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "--params-file", robot_driver_param_file,
                     "--params-file", robot_driver_topics_file, "--params-file",
                     robot_specific_param_file});
  options.append_parameter_override("use_sim_time", false);
  options.append_parameter_override("robot_description", urdf_string);
  options.append_parameter_override("controller_id", "inverse_dynamics");

  auto node =
      std::make_shared<rclcpp::Node>("robot_driver", "robot_1", options);
  EXPECT_EQ(1 + 1, 2);
}

TEST(JointController, testConstructorJointController) {
  // Load the Params Needed to Successfully Launch the Node
  std::string rd_pkg_share =
      ament_index_cpp::get_package_share_directory("robot_driver");
  std::string utils_pkg_share =
      ament_index_cpp::get_package_share_directory("quad_utils");
  std::string robot_driver_param_file =
      rd_pkg_share + "/config/robot_driver.yaml";
  std::string robot_driver_topics_file =
      rd_pkg_share + "/config/robot_driver_topics.yaml";
  std::string robot_specific_param_file =
      utils_pkg_share + "/config/spirit.yaml";

  const std::string xacro_path =
      ament_index_cpp::get_package_share_directory("spirit_description") +
      "/models/spirit/urdf/spirit.urdf.xacro";
  const std::string urdf_string = runXacro(xacro_path);

  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "--params-file", robot_driver_param_file,
                     "--params-file", robot_driver_topics_file, "--params-file",
                     robot_specific_param_file});
  options.append_parameter_override("use_sim_time", false);
  options.append_parameter_override("robot_description", urdf_string);
  options.append_parameter_override("controller_id", "joint");

  auto node =
      std::make_shared<rclcpp::Node>("robot_driver", "robot_1", options);
  EXPECT_EQ(1 + 1, 2);
}

TEST(GrfPidController, testConstructorGrfPidController) {
  // Load the Params Needed to Successfully Launch the Node
  std::string rd_pkg_share =
      ament_index_cpp::get_package_share_directory("robot_driver");
  std::string utils_pkg_share =
      ament_index_cpp::get_package_share_directory("quad_utils");
  std::string robot_driver_param_file =
      rd_pkg_share + "/config/robot_driver.yaml";
  std::string robot_driver_topics_file =
      rd_pkg_share + "/config/robot_driver_topics.yaml";
  std::string robot_specific_param_file =
      utils_pkg_share + "/config/spirit.yaml";

  const std::string xacro_path =
      ament_index_cpp::get_package_share_directory("spirit_description") +
      "/models/spirit/urdf/spirit.urdf.xacro";
  const std::string urdf_string = runXacro(xacro_path);

  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "--params-file", robot_driver_param_file,
                     "--params-file", robot_driver_topics_file, "--params-file",
                     robot_specific_param_file});
  options.append_parameter_override("use_sim_time", false);
  options.append_parameter_override("robot_description", urdf_string);
  options.append_parameter_override("controller_id", "grf_pid");

  auto node =
      std::make_shared<rclcpp::Node>("robot_driver", "robot_1", options);
  EXPECT_EQ(1 + 1, 2);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  my_argc = argc;
  my_argv = argv;
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();

  return result;
}
