#include "quad_pinocchio/quad_kd2.hpp"

using namespace quad_pinocchio;

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("test_quad_kd_node");
    std::string robot_ns("robot_1");
    quad_pinocchio::QuadKD kd(node, robot_ns);
    Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

    RCLCPP_INFO(node->get_logger(), "Model joints:");
    for (pinocchio::JointIndex i = 0;
        i < static_cast<pinocchio::JointIndex>(kd.model().njoints);
        ++i)
    {
        RCLCPP_INFO(node->get_logger(),
                    "[%ld] %s (parent: %d)",
                    static_cast<long>(i),
                    kd.model().names[i].c_str(),
                    kd.model().parents[i]);
    }

    RCLCPP_INFO(node->get_logger(), "Testing forward kinematics...");

    Eigen::VectorXd q = Eigen::VectorXd::Zero(kd.model().nq);
    q.segment<4>(3) << 0, 0, 0, 1; 
    pinocchio::forwardKinematics(kd.model(), kd.data(), q);
    const Eigen::Vector3d p = kd.data().oMi[1].translation();
    RCLCPP_INFO(node->get_logger(), "Joint 1 translation: [%.4f, %.4f, %.4f]",
                p.x(), p.y(), p.z());

    RCLCPP_INFO(node->get_logger(), "root shortname: %s",
            kd.model().joints[1].shortname().c_str());
    
    RCLCPP_INFO(node->get_logger(), "nq=%d nv=%d", kd.model().nq, kd.model().nv);
    // Expect: nq=19, nv=18

    // Test Joint Limit Assignment
    // RCLCPP_INFO(node->get_logger(), "Joint Limits by Leg (abad, hip, knee):");
    // for (size_t leg_index = 0; leg_index < kd.joint_min_.size(); ++leg_index)
    // {
    //     const auto& lower = kd.joint_min_[leg_index];
    //     const auto& upper = kd.joint_max_[leg_index];

    //     RCLCPP_INFO(node->get_logger(),
    //                 "Leg %zu:\n  Lower: [%.3f, %.3f, %.3f]\n  Upper: [%.3f, %.3f, %.3f]",
    //                 leg_index,
    //                 lower[0], lower[1], lower[2],
    //                 upper[0], upper[1], upper[2]);
    // }
    

    // const Eigen::Vector3d p = kd.data().oMi[1].translation();
    // RCLCPP_INFO(node->get_logger(), "Joint 1 translation:\n%s",
    //             p.format(CleanFmt).c_str());

    rclcpp::shutdown();
    return 0;
}