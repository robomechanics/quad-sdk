// Include Files
#include "spirit_utils/foot_jacobians.h"

// Function Definitions
//
// CALC_FOOT_JACOBIAN0
//     FOOT_JACOBIAN0 = CALC_FOOT_JACOBIAN0(IN1,IN2)
// Arguments    : const double in1[18]
//                const double in2[6]
//                double foot_jacobian0[9]
// Return Type  : void
//

namespace spirit_utils
{
  void getFootJacobian(int leg_idx,
                       const Eigen::VectorXd &state, Eigen::MatrixXd &foot_jacobian)
  {
    assert(state.size() == 18);

    std::string robot_description_string;
    ros::param::get("trajectory/robot_description", robot_description_string);

    RigidBodyDynamics::Model *model = new RigidBodyDynamics::Model();

    if (!RigidBodyDynamics::Addons::URDFReadFromString(robot_description_string.c_str(), model, true))
    {
      std::cerr << "Error loading model " << std::endl;
      abort();
    }

    // RBDL state vector has the floating base state in the front and the joint state in the back
    // When reading from URDF, the order of the legs is 2301, which should be corrected by sorting the bodyID
    Eigen::VectorXd q(19);
    q.setZero();

    q.head(3) = state.segment(12, 3);

    tf2::Quaternion quat_tf;
    geometry_msgs::Quaternion quat_msg;
    quat_tf.setRPY(state(15), state(16), state(17));
    quat_msg = tf2::toMsg(quat_tf);
    q(3) = quat_msg.x;
    q(4) = quat_msg.y;
    q(5) = quat_msg.z;

    // RBDL uses quaternion for floating base direction, but w is placed at the end of the state vector
    q(18) = quat_msg.w;

    q.segment(6, 3) = state.segment(6, 3);
    q.segment(9, 3) = state.segment(9, 3);
    q.segment(12, 3) = state.segment(0, 3);
    q.segment(15, 3) = state.segment(3, 3);

    std::vector<std::string> body_name_list = {"toe0", "toe1", "toe2", "toe3"};

    unsigned int body_id = model->GetBodyId(body_name_list.at(leg_idx).c_str());

    foot_jacobian.setZero();

    Eigen::MatrixXd jac_block(3, 18);
    jac_block.setZero();
    RigidBodyDynamics::CalcPointJacobian(*model, q, body_id, Eigen::Vector3d::Zero(), jac_block);

    // RBDL uses Jacobian w.r.t. floating base angular velocity in body frame, which is multiplied by Jacobian to map it to Euler angle change rate here
    Eigen::MatrixXd ang_vel_jac(3, 3);
    ang_vel_jac << 1, 0, -sin(state(16)),
        0, cos(state(15)), cos(state(16)) * sin(state(15)),
        0, -sin(state(15)), cos(state(15)) * cos(state(16));
    jac_block.block(0, 3, 3, 3) = jac_block.block(0, 3, 3, 3) * ang_vel_jac;

    foot_jacobian.block(0, 0, 3, 3) = jac_block.block(0, 12, 3, 3);
    foot_jacobian.block(0, 3, 3, 3) = jac_block.block(0, 15, 3, 3);
    foot_jacobian.block(0, 6, 3, 3) = jac_block.block(0, 6, 3, 3);
    foot_jacobian.block(0, 9, 3, 3) = jac_block.block(0, 9, 3, 3);
    foot_jacobian.block(0, 12, 3, 6) = jac_block.block(0, 0, 3, 6);
  }

  void getJacobian(const Eigen::VectorXd &state, Eigen::MatrixXd &jacobian)
  {
    assert(state.size() == 18);

    std::string robot_description_string;
    ros::param::get("trajectory/robot_description", robot_description_string);

    RigidBodyDynamics::Model *model = new RigidBodyDynamics::Model();

    if (!RigidBodyDynamics::Addons::URDFReadFromString(robot_description_string.c_str(), model, true))
    {
      std::cerr << "Error loading model " << std::endl;
      abort();
    }

    // RBDL state vector has the floating base state in the front and the joint state in the back
    // When reading from URDF, the order of the legs is 2301, which should be corrected by sorting the bodyID
    Eigen::VectorXd q(19);
    q.setZero();

    q.head(3) = state.segment(12, 3);

    tf2::Quaternion quat_tf;
    geometry_msgs::Quaternion quat_msg;
    quat_tf.setRPY(state(15), state(16), state(17));
    quat_msg = tf2::toMsg(quat_tf);
    q(3) = quat_msg.x;
    q(4) = quat_msg.y;
    q(5) = quat_msg.z;

    // RBDL uses quaternion for floating base direction, but w is placed at the end of the state vector
    q(18) = quat_msg.w;

    q.segment(6, 3) = state.segment(6, 3);
    q.segment(9, 3) = state.segment(9, 3);
    q.segment(12, 3) = state.segment(0, 3);
    q.segment(15, 3) = state.segment(3, 3);

    std::vector<std::string> body_name_list = {"toe0", "toe1", "toe2", "toe3"};

    jacobian.setZero();

    for (size_t i = 0; i < body_name_list.size(); i++)
    {
      unsigned int body_id = model->GetBodyId(body_name_list.at(i).c_str());

      Eigen::MatrixXd jac_block(3, 18);
      jac_block.setZero();
      RigidBodyDynamics::CalcPointJacobian(*model, q, body_id, Eigen::Vector3d::Zero(), jac_block);

      // RBDL uses Jacobian w.r.t. floating base angular velocity in body frame, which is multiplied by Jacobian to map it to Euler angle change rate here
      Eigen::MatrixXd ang_vel_jac(3, 3);
      ang_vel_jac << 1, 0, -sin(state(16)),
          0, cos(state(15)), cos(state(16)) * sin(state(15)),
          0, -sin(state(15)), cos(state(15)) * cos(state(16));
      jac_block.block(0, 3, 3, 3) = jac_block.block(0, 3, 3, 3) * ang_vel_jac;

      jacobian.block(3 * i, 0, 3, 3) = jac_block.block(0, 12, 3, 3);
      jacobian.block(3 * i, 3, 3, 3) = jac_block.block(0, 15, 3, 3);
      jacobian.block(3 * i, 6, 3, 3) = jac_block.block(0, 6, 3, 3);
      jacobian.block(3 * i, 9, 3, 3) = jac_block.block(0, 9, 3, 3);
      jacobian.block(3 * i, 12, 3, 6) = jac_block.block(0, 0, 3, 6);
    }
  }
}