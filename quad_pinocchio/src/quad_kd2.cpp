#include "quad_pinocchio/quad_kd2.hpp"

using namespace quad_pinocchio;

Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

QuadKD::QuadKD(rclcpp::Node::SharedPtr node) : node_(node) { initModel(""); }

QuadKD::QuadKD(rclcpp::Node::SharedPtr node, std::string ns) : node_(node)
{
  initModel("/" + ns + "/");
}

void QuadKD::initModel(std::string ns)
{
  std::string robot_ns_, robot_description_string;
  node_->declare_parameter<std::string>("namespace", "");
  node_->declare_parameter<std::string>("robot_description", "");
  node_->get_parameter("namespace", robot_ns_);

  if (!node_->get_parameter("robot_description", robot_description_string))
  {
    RCLCPP_FATAL(node_->get_logger(),
                 "Failed to load robot_description. Shutting down.");
    rclcpp::shutdown();
  }

  try
  {
    pinocchio::urdf::buildModelFromXML(robot_description_string, pinocchio::JointModelFreeFlyer(), model_);
    data_ = pinocchio::Data(model_);
    RCLCPP_INFO(node_->get_logger(),
                "Loaded Pinocchio model with %d joints and %d bodies.",
                model_.njoints, model_.nbodies);
  }
  catch (const std::exception &e)
  {
    RCLCPP_FATAL(node_->get_logger(), "Error loading model.");
    rclcpp::shutdown();
  }

  std::vector<std::string> hip_name_list = {"hip0", "hip1", "hip2", "hip3"};
  std::vector<std::string> upper_name_list = {"upper0", "upper1", "upper2",
                                              "upper3"};
  std::vector<std::string> lower_name_list = {"lower0", "lower1", "lower2",
                                              "lower3"};
  std::vector<std::string> toe_name_list = {"toe0", "toe1", "toe2", "toe3"};
  
  std::vector<std::string> abad_joint_list = {"8", "9", "10", "11"};
  std::vector<std::string> hip_joint_list = {"0", "2", "4", "6"};
  std::vector<std::string> knee_joint_list = {"1", "3", "5", "7"};
  
  // Get the Toe Frame ID's 
  toe_fids_.resize(4);
  for (size_t i = 0; i < toe_name_list.size(); ++i)
  {
    toe_fids_[i] = model_.getFrameId(toe_name_list[i]); // Grab Link Frame Id
  }

  leg_idx_list_.resize(4);
  std::iota(leg_idx_list_.begin(), leg_idx_list_.end(), 0);
  std::sort(leg_idx_list_.begin(), leg_idx_list_.end(), [&](int i, int j)
            { return toe_fids_[i] < toe_fids_[j]; });

  // Read leg geometry from URDF
  legbase_offsets_.resize(4);
  l0_vec_.resize(4);
  for (size_t i = 0; i < 4; i++)
  {
    pinocchio::JointIndex j_hip = model_.getJointId(hip_name_list.at(i));
    pinocchio::JointIndex j_upper = model_.getJointId(upper_name_list.at(i));
    pinocchio::JointIndex j_lower = model_.getJointId(lower_name_list.at(i));
    pinocchio::JointIndex j_toe = model_.getJointId(toe_name_list.at(i));

    legbase_offsets_[i] = model_.jointPlacements[j_hip].translation();

    l0_vec_[i] = model_.jointPlacements[j_upper].translation()(1);

    knee_offset_ = model_.jointPlacements[j_lower].translation();
    l1_ = knee_offset_.cwiseAbs().maxCoeff();

    foot_offset_ = model_.jointPlacements[j_toe].translation();
    l2_ = foot_offset_.cwiseAbs().maxCoeff();
  }

  // Abad offset from legbase
  abad_offset_ = {0, 0, 0};

  g_body_legbases_.resize(4);
  for (int leg_index = 0; leg_index < 4; leg_index++)
  {
    // Compute transforms
    g_body_legbases_[leg_index] =
        createAffineMatrix(legbase_offsets_[leg_index],
                           Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()));
  }

  joint_min_.resize(num_feet_);
  joint_max_.resize(num_feet_);

  // Set Limits for Each of the Joints from the URDF
  for (int leg_index = 0; leg_index < 4; leg_index++){
    // Grab the Upper and Lower Limits for Leg 0(8, 0, 1), Leg 1(9, 2, 3), Leg 2(10, 4, 5), Leg 3(11, 6, 7)
    // USing the Abad, Hip and Knee Joint Lists from the Loaded Model

    // Compile a Per Leg List of Joint Names
    std::vector<std::string> leg_joint_names = {
      abad_joint_list[leg_index], 
      hip_joint_list[leg_index],
      knee_joint_list[leg_index]
    };

    std::vector<double> lower_leg_limits;
    std::vector<double> upper_leg_limits;

    // For each Joint in the Leg, Grab its Upper and Lower Torque Limits
    for (const auto &name : leg_joint_names){
      pinocchio::JointIndex jid = model_.getJointId(name);
      Eigen::Index iq = static_cast<Eigen::Index>(model_.joints[jid].idx_q());

      lower_leg_limits.push_back(model_.lowerPositionLimit[iq]);
      upper_leg_limits.push_back(model_.upperPositionLimit[iq]);

    }

    joint_min_[leg_index] = lower_leg_limits;
    joint_max_[leg_index] = upper_leg_limits;
  }
 
}

Eigen::Matrix4d QuadKD::createAffineMatrix(Eigen::Vector3d trans,
                                           Eigen::Vector3d rpy) const
{
  Eigen::Transform<double, 3, Eigen::Affine> t;
  t = Eigen::Translation<double, 3>(trans);
  t.rotate(Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()));
  t.rotate(Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()));
  t.rotate(Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX()));

  return t.matrix();
}

Eigen::Matrix4d QuadKD::createAffineMatrix(Eigen::Vector3d trans,
                                           Eigen::AngleAxisd rot) const
{
  Eigen::Transform<double, 3, Eigen::Affine> t;
  t = Eigen::Translation<double, 3>(trans);
  t.rotate(rot);

  return t.matrix();
}

pinocchio::SE3 QuadKD::convertAffineToSE3(Eigen::Vector4d g_transform) const
{
  Eigen::Matrix3d rot = g_transform.topLeftCorner<3,3>();
  Eigen::Vector3d trans = g_transform.topRightCorner<3,1>();
  pinocchio::SE3 se3_transform(rot, trans);

  return se3_transform;
}

Eigen::Matrix4d QuadKD::convertSE3ToAffine(pinocchio::SE3 se3_transform) const
{
  Eigen::Matrix4d g_transform = se3_transform.toHomogeneousMatrix();

  return g_transform;
}

double QuadKD::getJointLowerLimit(int leg_index, int joint_index) const
{
  return joint_min_[leg_index][joint_index];
}

double QuadKD::getJointUpperLimit(int leg_index, int joint_index) const
{
  return joint_max_[leg_index][joint_index];
}

double QuadKD::getLinkLength(int leg_index, int link_index) const
{
  switch (link_index)
  {
  case 0:
    return l0_vec_[leg_index];
  case 1:
    return l1_;
  case 2:
    return l2_;
  default:
    throw std::runtime_error("Invalid link index");
  }
}



