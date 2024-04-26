#include "quad_utils/quad_kd.h"

using namespace quad_utils;

Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

QuadKD::QuadKD() { initModel(""); }

QuadKD::QuadKD(const std::string &ns) { initModel("/" + ns + "/"); }

void QuadKD::initModel(const std::string &ns) {
    std::string robot_description_string;

    if (!ros::param::get("robot_description", robot_description_string)) {
        std::cerr << "Error loading robot_description " << std::endl;
        abort();
    }

    model_ = new RigidBodyDynamics::Model();
    if (!RigidBodyDynamics::Addons::URDFReadFromString(
            robot_description_string.c_str(), model_, true)) {
        std::cerr << "Error loading model " << std::endl;
        abort();
    }

    body_name_list_ = {"toe0", "toe1", "toe2", "toe3"};

    body_id_list_.resize(4);
    for (size_t i = 0; i < body_name_list_.size(); i++) {
        body_id_list_.at(i) = model_->GetBodyId(body_name_list_.at(i).c_str());
    }

    leg_idx_list_.resize(4);
    std::iota(leg_idx_list_.begin(), leg_idx_list_.end(), 0);
    std::sort(leg_idx_list_.begin(), leg_idx_list_.end(), [&](int i, int j) {
        return body_id_list_.at(i) < body_id_list_.at(j);
    });

    // Read leg geometry from URDF
    legbase_offsets_.resize(4);
    l0_vec_.resize(4);
    std::vector<std::string> hip_name_list = {"hip0", "hip1", "hip2", "hip3"};
    std::vector<std::string> upper_name_list = {"upper0", "upper1", "upper2",
                                                "upper3"};
    std::vector<std::string> lower_name_list = {"lower0", "lower1", "lower2",
                                                "lower3"};
    std::vector<std::string> toe_name_list = {"toe0", "toe1", "toe2", "toe3"};
    RigidBodyDynamics::Math::SpatialTransform tform;
    for (size_t i = 0; i < 4; i++) {
        // From body COM to abad
        tform = model_->GetJointFrame(
            model_->GetBodyId(hip_name_list.at(i).c_str()));
        legbase_offsets_[i] = tform.r;

        // From abad to hip
        tform = model_->GetJointFrame(
            model_->GetBodyId(upper_name_list.at(i).c_str()));
        l0_vec_[i] = tform.r(1);

        // From hip to knee (we know they should be the same and the equation in
        // IK uses the magnitute of it)
        tform = model_->GetJointFrame(
            model_->GetBodyId(lower_name_list.at(i).c_str()));
        l1_ = tform.r.cwiseAbs().maxCoeff();
        knee_offset_ = tform.r;

        // From knee to toe (we know they should be the same and the equation in
        // IK uses the magnitute of it)
        tform = model_->GetJointFrame(
            model_->GetBodyId(toe_name_list.at(i).c_str()));
        l2_ = tform.r.cwiseAbs().maxCoeff();
        foot_offset_ = tform.r;
    }

    // Abad offset from legbase
    abad_offset_ = {0, 0, 0};

    g_body_legbases_.resize(4);
    for (int leg_index = 0; leg_index < 4; leg_index++) {
        // Compute transforms
        g_body_legbases_[leg_index] =
            createAffineMatrix(legbase_offsets_[leg_index],
                               Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()));
    }

    joint_min_.resize(num_feet_);
    joint_max_.resize(num_feet_);

    std::vector<double> joint_min_front = {-0.707, -M_PI * 0.5, 0};
    std::vector<double> joint_min_back = {-0.707, -M_PI, 0};
    std::vector<double> joint_max_front = {0.707, M_PI, M_PI};
    std::vector<double> joint_max_back = {0.707, M_PI * 0.5, M_PI};

    joint_min_ = {joint_min_front, joint_min_back, joint_min_front,
                  joint_min_back};
    joint_max_ = {joint_max_front, joint_max_back, joint_max_front,
                  joint_max_back};
}

Eigen::Matrix4d QuadKD::createAffineMatrix(const Eigen::Vector3d &trans,
                                           const Eigen::Vector3d &rpy) const {
    Eigen::Transform<double, 3, Eigen::Affine> t;
    t = Eigen::Translation<double, 3>(trans);
    t.rotate(Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()));
    t.rotate(Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()));
    t.rotate(Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX()));

    return t.matrix();
}

Eigen::Matrix4d QuadKD::createAffineMatrix(const Eigen::Vector3d &trans,
                                           const Eigen::AngleAxisd &rot) const {
    Eigen::Transform<double, 3, Eigen::Affine> t;
    t = Eigen::Translation<double, 3>(trans);
    t.rotate(rot);

    return t.matrix();
}

double QuadKD::getJointLowerLimit(int leg_index, int joint_index) const {
    return joint_min_[leg_index][joint_index];
}

double QuadKD::getJointUpperLimit(int leg_index, int joint_index) const {
    return joint_max_[leg_index][joint_index];
}

double QuadKD::getLinkLength(int leg_index, int link_index) const {
    switch (link_index) {
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

void QuadKD::transformBodyToWorld(const Eigen::Vector3d &body_pos,
                                  const Eigen::Vector3d &body_rpy,
                                  const Eigen::Matrix4d &transform_body,
                                  Eigen::Matrix4d &transform_world) const {
    // Compute transform from world to body frame
    Eigen::Matrix4d g_world_body = createAffineMatrix(body_pos, body_rpy);

    // Get the desired transform in the world frame
    transform_world = g_world_body * transform_body;
}

void QuadKD::transformWorldToBody(const Eigen::Vector3d &body_pos,
                                  const Eigen::Vector3d &body_rpy,
                                  const Eigen::Matrix4d &transform_world,
                                  Eigen::Matrix4d &transform_body) const {
    // Compute transform from world to body frame
    Eigen::Matrix4d g_world_body = createAffineMatrix(body_pos, body_rpy);

    // Compute the desired transform in the body frame
    transform_body = g_world_body.inverse() * transform_world;
}

void QuadKD::worldToLegbaseFKWorldFrame(
    int leg_index, const Eigen::Vector3d &body_pos,
    const Eigen::Vector3d &body_rpy, Eigen::Matrix4d &g_world_legbase) const {
    // Compute transforms
    Eigen::Matrix4d g_world_body = createAffineMatrix(body_pos, body_rpy);

    // Compute transform for leg base relative to the world frame
    g_world_legbase = g_world_body * g_body_legbases_[leg_index];
}

void QuadKD::worldToLegbaseFKWorldFrame(
    int leg_index, const Eigen::Vector3d &body_pos,
    const Eigen::Vector3d &body_rpy,
    Eigen::Vector3d &leg_base_pos_world) const {
    Eigen::Matrix4d g_world_legbase;
    worldToLegbaseFKWorldFrame(leg_index, body_pos, body_rpy, g_world_legbase);

    leg_base_pos_world = g_world_legbase.block<3, 1>(0, 3);
}

void QuadKD::worldToNominalHipFKWorldFrame(
    int leg_index, const Eigen::Vector3d &body_pos,
    const Eigen::Vector3d &body_rpy,
    Eigen::Vector3d &nominal_hip_pos_world) const {
    // Compute transforms
    Eigen::Matrix4d g_world_body = createAffineMatrix(body_pos, body_rpy);
    // Compute transform from body to legbase but offset by l0
    Eigen::Matrix4d g_body_nominal_hip = g_body_legbases_[leg_index];
    g_body_nominal_hip(1, 3) += 1.0 * l0_vec_[leg_index];

    // Compute transform for offset leg base relative to the world frame
    Eigen::Matrix4d g_world_nominal_hip = g_world_body * g_body_nominal_hip;

    nominal_hip_pos_world = g_world_nominal_hip.block<3, 1>(0, 3);
}

void QuadKD::bodyToFootFKBodyFrame(int leg_index,
                                   const Eigen::Vector3d &joint_state,
                                   Eigen::Matrix4d &g_body_foot) const {
    if (leg_index > (legbase_offsets_.size() - 1) || leg_index < 0) {
        throw std::runtime_error("Leg index is outside valid range");
    }

    // Define hip offset
    Eigen::Vector3d hip_offset = {0, l0_vec_[leg_index], 0};

    // Initialize transforms
    Eigen::Matrix4d g_legbase_abad;
    Eigen::Matrix4d g_abad_hip;
    Eigen::Matrix4d g_hip_knee;
    Eigen::Matrix4d g_knee_foot;

    g_legbase_abad = createAffineMatrix(
        abad_offset_,
        Eigen::AngleAxisd(joint_state[0], Eigen::Vector3d::UnitX()));

    g_abad_hip = createAffineMatrix(
        hip_offset,
        Eigen::AngleAxisd(joint_state[1], -Eigen::Vector3d::UnitY()));

    g_hip_knee = createAffineMatrix(
        knee_offset_,
        Eigen::AngleAxisd(joint_state[2], Eigen::Vector3d::UnitY()));

    g_knee_foot = createAffineMatrix(
        foot_offset_, Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()));

    // Get foot transform in world frame
    g_body_foot = g_body_legbases_[leg_index] * g_legbase_abad * g_abad_hip *
                  g_hip_knee * g_knee_foot;
}

void QuadKD::bodyToFootFKBodyFrame(int leg_index,
                                   const Eigen::Vector3d &joint_state,
                                   Eigen::Vector3d &foot_pos_body) const {
    Eigen::Matrix4d g_body_foot;
    QuadKD::bodyToFootFKBodyFrame(leg_index, joint_state, g_body_foot);

    // Extract cartesian position of foot
    foot_pos_body = g_body_foot.block<3, 1>(0, 3);
}

void QuadKD::worldToFootFKWorldFrame(int leg_index,
                                     const Eigen::Vector3d &body_pos,
                                     const Eigen::Vector3d &body_rpy,
                                     const Eigen::Vector3d &joint_state,
                                     Eigen::Matrix4d &g_world_foot) const {
    if (leg_index > (legbase_offsets_.size() - 1) || leg_index < 0) {
        throw std::runtime_error("Leg index is outside valid range");
    }

    // Define hip offset
    Eigen::Vector3d hip_offset = {0, l0_vec_[leg_index], 0};

    // Initialize transforms
    Eigen::Matrix4d g_body_legbase;
    Eigen::Matrix4d g_legbase_abad;
    Eigen::Matrix4d g_abad_hip;
    Eigen::Matrix4d g_hip_knee;
    Eigen::Matrix4d g_knee_foot;

    // Compute transforms
    Eigen::Matrix4d g_world_legbase;
    worldToLegbaseFKWorldFrame(leg_index, body_pos, body_rpy, g_world_legbase);

    g_legbase_abad = createAffineMatrix(
        abad_offset_,
        Eigen::AngleAxisd(joint_state[0], Eigen::Vector3d::UnitX()));

    g_abad_hip = createAffineMatrix(
        hip_offset,
        Eigen::AngleAxisd(joint_state[1], -Eigen::Vector3d::UnitY()));

    g_hip_knee = createAffineMatrix(
        knee_offset_,
        Eigen::AngleAxisd(joint_state[2], Eigen::Vector3d::UnitY()));

    g_knee_foot = createAffineMatrix(
        foot_offset_, Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()));

    // Get foot transform in world frame
    g_world_foot = g_world_legbase * g_legbase_abad * g_abad_hip * g_hip_knee *
                   g_knee_foot;
}

void QuadKD::worldToFootFKWorldFrame(int leg_index,
                                     const Eigen::Vector3d &body_pos,
                                     const Eigen::Vector3d &body_rpy,
                                     const Eigen::Vector3d &joint_state,
                                     Eigen::Vector3d &foot_pos_world) const {
    Eigen::Matrix4d g_world_foot;
    worldToFootFKWorldFrame(leg_index, body_pos, body_rpy, joint_state,
                            g_world_foot);

    // Extract cartesian position of foot
    foot_pos_world = g_world_foot.block<3, 1>(0, 3);
}

void QuadKD::worldToKneeFKWorldFrame(int leg_index,
                                     const Eigen::Vector3d &body_pos,
                                     const Eigen::Vector3d &body_rpy,
                                     const Eigen::Vector3d &joint_state,
                                     Eigen::Matrix4d &g_world_knee) const {
    if (leg_index > (legbase_offsets_.size() - 1) || leg_index < 0) {
        throw std::runtime_error("Leg index is outside valid range");
    }

    // Define hip offset
    Eigen::Vector3d hip_offset = {0, l0_vec_[leg_index], 0};

    // Initialize transforms
    Eigen::Matrix4d g_body_legbase;
    Eigen::Matrix4d g_legbase_abad;
    Eigen::Matrix4d g_abad_hip;
    Eigen::Matrix4d g_hip_knee;

    // Compute transforms
    Eigen::Matrix4d g_world_legbase;
    worldToLegbaseFKWorldFrame(leg_index, body_pos, body_rpy, g_world_legbase);

    g_legbase_abad = createAffineMatrix(
        abad_offset_,
        Eigen::AngleAxisd(joint_state[0], Eigen::Vector3d::UnitX()));

    g_abad_hip = createAffineMatrix(
        hip_offset,
        Eigen::AngleAxisd(joint_state[1], -Eigen::Vector3d::UnitY()));

    g_hip_knee = createAffineMatrix(
        knee_offset_,
        Eigen::AngleAxisd(joint_state[2], Eigen::Vector3d::UnitY()));

    // Get foot transform in world frame
    g_world_knee = g_world_legbase * g_legbase_abad * g_abad_hip * g_hip_knee;
}

void QuadKD::worldToKneeFKWorldFrame(int leg_index,
                                     const Eigen::Vector3d &body_pos,
                                     const Eigen::Vector3d &body_rpy,
                                     const Eigen::Vector3d &joint_state,
                                     Eigen::Vector3d &knee_pos_world) const {
    Eigen::Matrix4d g_world_knee;
    worldToKneeFKWorldFrame(leg_index, body_pos, body_rpy, joint_state,
                            g_world_knee);

    // Extract cartesian position of foot
    knee_pos_world = g_world_knee.block<3, 1>(0, 3);
}

bool QuadKD::worldToFootIKWorldFrame(int leg_index,
                                     const Eigen::Vector3d &body_pos,
                                     const Eigen::Vector3d &body_rpy,
                                     const Eigen::Vector3d &foot_pos_world,
                                     Eigen::Vector3d &joint_state) const {
    if (leg_index > (legbase_offsets_.size() - 1) || leg_index < 0) {
        throw std::runtime_error("Leg index is outside valid range");
    }

    // Calculate offsets
    Eigen::Vector3d legbase_offset = legbase_offsets_[leg_index];
    double l0 = l0_vec_[leg_index];

    // Initialize transforms
    Eigen::Matrix4d g_world_legbase;
    Eigen::Matrix4d g_world_foot;
    Eigen::Matrix4d g_legbase_foot;
    Eigen::Vector3d foot_pos_legbase;

    // Compute transforms
    worldToLegbaseFKWorldFrame(leg_index, body_pos, body_rpy, g_world_legbase);

    g_world_foot = createAffineMatrix(
        foot_pos_world, Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()));

    // Compute foot position relative to the leg base in cartesian coordinates
    g_legbase_foot = g_world_legbase.inverse() * g_world_foot;
    foot_pos_legbase = g_legbase_foot.block<3, 1>(0, 3);

    return legbaseToFootIKLegbaseFrame(leg_index, foot_pos_legbase,
                                       joint_state);
}

bool QuadKD::legbaseToFootIKLegbaseFrame(
    int leg_index, const Eigen::Vector3d &foot_pos_legbase,
    Eigen::Vector3d &joint_state) const {
    // Initialize exact bool
    bool is_exact = true;

    // Calculate offsets
    Eigen::Vector3d legbase_offset = legbase_offsets_[leg_index];
    double l0 = l0_vec_[leg_index];

    // Extract coordinates and declare joint variables
    double x = foot_pos_legbase[0];
    double y = foot_pos_legbase[1];
    double z = foot_pos_legbase[2];
    double q0;
    double q1;
    double q2;

    // Start IK, check foot pos is at least l0 away from leg base, clamp
    // otherwise
    double temp = l0 / sqrt(z * z + y * y);
    if (abs(temp) > 1) {
        ROS_DEBUG_THROTTLE(0.5,
                           "Foot too close, choosing closest alternative\n");
        is_exact = false;
        temp = std::max(std::min(temp, 1.0), -1.0);
    }

    // Compute both solutions of q0, use hip-above-knee if z<0 (preferred)
    // Store the inverted solution in case hip limits are exceeded
    if (z > 0) {
        q0 = -acos(temp) + atan2(z, y);
    } else {
        q0 = acos(temp) + atan2(z, y);
    }

    // Make sure abad is within joint limits, clamp otherwise
    if (q0 > joint_max_[leg_index][0] || q0 < joint_min_[leg_index][0]) {
        q0 = std::max(std::min(q0, joint_max_[leg_index][0]),
                      joint_min_[leg_index][0]);
        is_exact = false;
        ROS_DEBUG_THROTTLE(0.5, "Abad limits exceeded, clamping to %5.3f \n",
                           q0);
    }

    // Rotate to ab-ad fixed frame
    double z_body_frame = z;
    z = -sin(q0) * y + cos(q0) * z_body_frame;

    // Check reachibility for hip
    double acos_eps = 1.0;
    double temp2 = (l1_ * l1_ + x * x + z * z - l2_ * l2_) /
                   (2 * l1_ * sqrt(x * x + z * z));
    if (abs(temp2) > acos_eps) {
        ROS_DEBUG_THROTTLE(0.5,
                           "Foot location too far for hip, choosing closest"
                           " alternative \n");
        is_exact = false;
        temp2 = std::max(std::min(temp2, acos_eps), -acos_eps);
    }

    // Check reachibility for knee
    double temp3 = (l1_ * l1_ + l2_ * l2_ - x * x - z * z) / (2 * l1_ * l2_);

    if (temp3 > acos_eps || temp3 < -acos_eps) {
        ROS_DEBUG_THROTTLE(0.5,
                           "Foot location too far for knee, choosing closest"
                           " alternative \n");
        is_exact = false;

        temp3 = std::max(std::min(temp3, acos_eps), -acos_eps);
    }

    // Compute joint angles
    q1 = 0.5 * M_PI + atan2(x, -z) - acos(temp2);

    // Make sure hip is within joint limits
    if (q1 > joint_max_[leg_index][1] || q1 < joint_min_[leg_index][1]) {
        q1 = std::max(std::min(q1, joint_max_[leg_index][1]),
                      joint_min_[leg_index][1]);
        is_exact = false;
        ROS_DEBUG_THROTTLE(0.5, "Hip limits exceeded, clamping to %5.3f \n",
                           q1);
    }

    // Compute knee val to get closest toe position in the plane
    Eigen::Vector2d knee_pos, toe_pos, toe_offset;
    knee_pos << -l1_ * cos(q1), -l1_ * sin(q1);
    toe_pos << x, z;
    toe_offset = toe_pos - knee_pos;
    q2 = atan2(-toe_offset(1), toe_offset(0)) + q1;

    // Make sure knee is within joint limits
    if (q2 > joint_max_[leg_index][2] || q2 < joint_min_[leg_index][2]) {
        q2 = std::max(std::min(q2, joint_max_[leg_index][2]),
                      joint_min_[leg_index][2]);
        is_exact = false;
        ROS_DEBUG_THROTTLE(0.5, "Knee limit exceeded, clamping to %5.3f \n",
                           q2);
    }

    // q1 is undefined if q2=0, resolve this
    if (q2 == 0) {
        q1 = 0;
        ROS_DEBUG_THROTTLE(0.5,
                           "Hip value undefined (in singularity), setting to"
                           " %5.3f \n",
                           q1);
        is_exact = false;
    }

    if (z_body_frame - l0 * sin(q0) > 0) {
        ROS_DEBUG_THROTTLE(0.5,
                           "IK solution is in hip-inverted region! Beware!\n");
        is_exact = false;
    }

    joint_state = {q0, q1, q2};
    return is_exact;
}

void QuadKD::getJacobianGenCoord(const Eigen::VectorXd &state,
                                 Eigen::MatrixXd &jacobian) const {
    this->getJacobianBodyAngVel(state, jacobian);

    // RBDL uses Jacobian w.r.t. floating base angular velocity in body frame,
    // which is multiplied by Jacobian to map it to Euler angle change rate here
    for (size_t i = 0; i < 4; i++) {
        Eigen::MatrixXd transform_jac(3, 3);
        transform_jac << 1, 0, -sin(state(16)), 0, cos(state(15)),
            cos(state(16)) * sin(state(15)), 0, -sin(state(15)),
            cos(state(15)) * cos(state(16));
        jacobian.block(3 * i, 15, 3, 3) =
            jacobian.block(3 * i, 15, 3, 3) * transform_jac;
    }
}

void QuadKD::getJacobianBodyAngVel(const Eigen::VectorXd &state,
                                   Eigen::MatrixXd &jacobian) const {
    assert(state.size() == 18);

    // RBDL state vector has the floating base state in the front and the joint
    // state in the back When reading from URDF, the order of the legs is 2301,
    // which should be corrected by sorting the bodyID
    Eigen::VectorXd q(19);
    q.setZero();

    q.head(3) = state.segment(12, 3);

    tf2::Quaternion quat_tf;
    quat_tf.setRPY(state(15), state(16), state(17));
    q(3) = quat_tf.getX();
    q(4) = quat_tf.getY();
    q(5) = quat_tf.getZ();

    // RBDL uses quaternion for floating base direction, but w is placed at the
    // end of the state vector
    q(18) = quat_tf.getW();

    for (size_t i = 0; i < leg_idx_list_.size(); i++) {
        q.segment(6 + 3 * i, 3) = state.segment(3 * leg_idx_list_.at(i), 3);
    }

    jacobian.setZero();

    for (size_t i = 0; i < body_id_list_.size(); i++) {
        Eigen::MatrixXd jac_block(3, 18);
        jac_block.setZero();
        RigidBodyDynamics::CalcPointJacobian(*model_, q, body_id_list_.at(i),
                                             Eigen::Vector3d::Zero(),
                                             jac_block);

        for (size_t j = 0; j < 4; j++) {
            jacobian.block(3 * i, 3 * leg_idx_list_.at(j), 3, 3) =
                jac_block.block(0, 6 + 3 * j, 3, 3);
        }
        jacobian.block(3 * i, 12, 3, 6) = jac_block.block(0, 0, 3, 6);
    }
}

void QuadKD::getJacobianWorldAngVel(const Eigen::VectorXd &state,
                                    Eigen::MatrixXd &jacobian) const {
    this->getJacobianBodyAngVel(state, jacobian);

    // RBDL uses Jacobian w.r.t. floating base angular velocity in body frame,
    // which is multiplied by rotation matrix to map it to angular velocity in
    // world frame here
    for (size_t i = 0; i < 4; i++) {
        Eigen::Matrix3d rot;
        this->getRotationMatrix(state.segment(15, 3), rot);
        jacobian.block(3 * i, 15, 3, 3) = jacobian.block(3 * i, 15, 3, 3) * rot;
    }
}

void QuadKD::getRotationMatrix(const Eigen::VectorXd &rpy,
                               Eigen::Matrix3d &rot) const {
    rot = Eigen::AngleAxisd(rpy(2), Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(rpy(1), Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(rpy(0), Eigen::Vector3d::UnitX());
}

void QuadKD::computeInverseDynamics(const Eigen::VectorXd &state_pos,
                                    const Eigen::VectorXd &state_vel,
                                    const Eigen::VectorXd &foot_acc,
                                    const Eigen::VectorXd &grf,
                                    const std::vector<int> &contact_mode,
                                    Eigen::VectorXd &tau) const {
    // Convert q, q_dot into RBDL order
    Eigen::VectorXd q(19), q_dot(18);
    q.setZero();
    q_dot.setZero();

    q.head(3) = state_pos.segment(12, 3);
    q_dot.head(3) = state_vel.segment(12, 3);

    tf2::Quaternion quat_tf;
    quat_tf.setRPY(state_pos(15), state_pos(16), state_pos(17));
    q(3) = quat_tf.getX();
    q(4) = quat_tf.getY();
    q(5) = quat_tf.getZ();

    // RBDL uses quaternion for floating base direction, but w is placed at the
    // end of the state vector
    q(18) = quat_tf.getW();

    q_dot.segment(3, 3) = state_vel.segment(15, 3);

    for (size_t i = 0; i < leg_idx_list_.size(); i++) {
        q.segment(6 + 3 * i, 3) = state_pos.segment(3 * leg_idx_list_.at(i), 3);
        q_dot.segment(6 + 3 * i, 3) =
            state_vel.segment(3 * leg_idx_list_.at(i), 3);
    }

    // Compute jacobians
    Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(12, 18);
    jacobian.setZero();
    for (size_t i = 0; i < body_id_list_.size(); i++) {
        Eigen::MatrixXd jac_block(3, 18);
        jac_block.setZero();
        RigidBodyDynamics::CalcPointJacobian(*model_, q, body_id_list_.at(i),
                                             Eigen::Vector3d::Zero(),
                                             jac_block);
        jacobian.block(3 * i, 0, 3, 18) = jac_block;
    }

    // Compute the equivalent force in generalized coordinates
    Eigen::VectorXd tau_stance = -jacobian.transpose() * grf;

    // Compute EOM
    Eigen::MatrixXd M(18, 18);
    M.setZero();
    Eigen::VectorXd N(18);
    RigidBodyDynamics::CompositeRigidBodyAlgorithm(*model_, q, M);
    RigidBodyDynamics::NonlinearEffects(*model_, q, q_dot, N);

    // Compute J_dot*q_dot
    Eigen::VectorXd foot_acc_J_dot(12);
    for (size_t i = 0; i < 4; i++) {
        foot_acc_J_dot.segment(3 * i, 3) =
            RigidBodyDynamics::CalcPointAcceleration(
                *model_, q, q_dot, Eigen::VectorXd::Zero(18),
                body_id_list_.at(i), Eigen::Vector3d::Zero());
    }

    // Compute constraint Jacobian A and A_dot*q_dot
    int constraints_num =
        3 * std::count(contact_mode.begin(), contact_mode.end(), true);
    Eigen::MatrixXd A(constraints_num, 18);
    Eigen::VectorXd A_dotq_dot(constraints_num);
    int constraints_count = 0;
    for (size_t i = 0; i < 4; i++) {
        if (contact_mode.at(i)) {
            A.block(3 * constraints_count, 0, 3, 18) =
                jacobian.block(3 * i, 0, 3, 18);
            A_dotq_dot.segment(3 * constraints_count, 3) =
                foot_acc_J_dot.segment(3 * i, 3);
            constraints_count++;
        }
    }

    // Compute acceleration from J*q_ddot
    Eigen::VectorXd foot_acc_q_ddot = foot_acc - foot_acc_J_dot;

    // Compuate damped jacobian inverser
    Eigen::MatrixXd jacobian_inv =
        math_utils::sdlsInv(jacobian.block(0, 6, 12, 12));

    // In the EOM, we know M, N, tau_grf, and a = J_b*q_ddot_b + J_l*q_ddot_l,
    // we need to solve q_ddot_b and tau_swing
    Eigen::MatrixXd blk_mat =
        Eigen::MatrixXd::Zero(18 + constraints_num, 18 + constraints_num);
    blk_mat.block(0, 0, 6, 6) =
        -M.block(0, 0, 6, 6) +
        M.block(0, 6, 6, 12) * jacobian_inv * jacobian.block(0, 0, 12, 6);
    blk_mat.block(6, 0, 12, 6) =
        -M.block(6, 0, 12, 6) +
        M.block(6, 6, 12, 12) * jacobian_inv * jacobian.block(0, 0, 12, 6);
    for (size_t i = 0; i < 4; i++) {
        if (!contact_mode.at(leg_idx_list_.at(i))) {
            blk_mat.block(3 * i + 6, 3 * i + 6, 3, 3).diagonal().fill(1);
        }
    }
    blk_mat.block(0, 18, 18, constraints_num) = -A.transpose();
    blk_mat.block(18, 0, constraints_num, 6) =
        -A.leftCols(6) +
        A.rightCols(12) * jacobian_inv * jacobian.block(0, 0, 12, 6);

    // Perform inverse dynamics
    Eigen::VectorXd tau_swing(12), blk_sol(18 + constraints_num),
        blk_vec(18 + constraints_num);
    blk_vec.segment(0, 6) << N.segment(0, 6) + M.block(0, 6, 6, 12) *
                                                   jacobian_inv *
                                                   foot_acc_q_ddot;
    blk_vec.segment(6, 12) << N.segment(6, 12) +
                                  M.block(6, 6, 12, 12) * jacobian_inv *
                                      foot_acc_q_ddot -
                                  tau_stance.segment(6, 12);
    blk_vec.segment(18, constraints_num)
        << A_dotq_dot + A.leftCols(12) * jacobian_inv * foot_acc_q_ddot;
    blk_sol = blk_mat.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV)
                  .solve(blk_vec);
    tau_swing = blk_sol.segment(6, 12);

    // Convert the order back
    for (size_t i = 0; i < 4; i++) {
        if (contact_mode.at(leg_idx_list_.at(i))) {
            tau.segment(3 * leg_idx_list_.at(i), 3) =
                tau_stance.segment(6 + 3 * i, 3);
        } else {
            tau.segment(3 * leg_idx_list_.at(i), 3) =
                tau_swing.segment(3 * i, 3);
            // tau.segment(3 * leg_idx_list_.at(i), 3) =
            // Eigen::VectorXd::Zero(3);
        }
    }

    // Check inf or nan
    if (!(tau.array() == tau.array()).all() ||
        !((tau - tau).array() == (tau - tau).array()).all()) {
        tau.setZero();
    }
}

bool QuadKD::convertCentroidalToFullBody(const Eigen::VectorXd &body_state,
                                         const Eigen::VectorXd &foot_positions,
                                         const Eigen::VectorXd &foot_velocities,
                                         const Eigen::VectorXd &grfs,
                                         Eigen::VectorXd &joint_positions,
                                         Eigen::VectorXd &joint_velocities,
                                         Eigen::VectorXd &torques) {
    // Assume the conversion is exact unless a check below fails
    bool is_exact = true;

    // Extract kinematic quantities
    Eigen::Vector3d body_pos = body_state.segment<3>(0);
    Eigen::Vector3d body_rpy = body_state.segment<3>(3);

    auto t_start = std::chrono::steady_clock::now();
    // Perform IK for each leg
    for (int i = 0; i < num_feet_; i++) {
        Eigen::Vector3d leg_joint_state;
        Eigen::Vector3d foot_pos = foot_positions.segment<3>(3 * i);
        is_exact =
            is_exact && worldToFootIKWorldFrame(i, body_pos, body_rpy, foot_pos,
                                                leg_joint_state);
        joint_positions.segment<3>(3 * i) = leg_joint_state;
    }

    auto t_ik = std::chrono::steady_clock::now();

    // Load state positions
    Eigen::VectorXd state_positions(18), state_velocities(18);
    state_positions << joint_positions, body_pos, body_rpy;

    // Compute jacobian
    Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(12, 18);
    getJacobianBodyAngVel(state_positions, jacobian);

    auto t_jacob = std::chrono::steady_clock::now();

    // Compute joint velocities
    joint_velocities = jacobian.leftCols(12).colPivHouseholderQr().solve(
        foot_velocities - jacobian.rightCols(6) * body_state.tail(6));
    state_velocities << joint_velocities, body_state.tail(6);

    auto t_ik_vel = std::chrono::steady_clock::now();

    torques = -jacobian.leftCols(12).transpose() * grfs;

    // computeInverseDynamics(state_positions, state_velocities, foot_acc, grfs,
    // contact_mode, torques);

    auto t_id = std::chrono::steady_clock::now();

    std::chrono::duration<double> t_diff_ik =
        std::chrono::duration_cast<std::chrono::duration<double>>(t_ik -
                                                                  t_start);
    std::chrono::duration<double> t_diff_jacob =
        std::chrono::duration_cast<std::chrono::duration<double>>(t_jacob -
                                                                  t_ik);
    std::chrono::duration<double> t_diff_ik_vel =
        std::chrono::duration_cast<std::chrono::duration<double>>(t_ik_vel -
                                                                  t_jacob);
    std::chrono::duration<double> t_diff_id =
        std::chrono::duration_cast<std::chrono::duration<double>>(t_id -
                                                                  t_ik_vel);

    // std::cout << "t_diff_ik = " << t_diff_ik.count() << std::endl;
    // std::cout << "t_diff_jacob = " << t_diff_jacob.count() << std::endl;
    // std::cout << "t_diff_ik_vel = " << t_diff_ik_vel.count() << std::endl;
    // std::cout << "t_diff_id = " << t_diff_id.count() << std::endl;

    return is_exact;
}

bool QuadKD::applyMotorModel(const Eigen::VectorXd &torques,
                             Eigen::VectorXd &constrained_torques) {
    // Constrain torques to max values
    constrained_torques.resize(torques.size());
    constrained_torques = torques.cwiseMax(-tau_max_).cwiseMin(tau_max_);

    // Check if torques was modified
    return constrained_torques.isApprox(torques);
}

bool QuadKD::applyMotorModel(const Eigen::VectorXd &joint_torques,
                             const Eigen::VectorXd &joint_velocities,
                             Eigen::VectorXd &constrained_joint_torques) {
    // Constrain torques to max values
    Eigen::VectorXd constraint_violation(joint_torques.size());
    constrained_joint_torques.resize(joint_torques.size());
    constrained_joint_torques =
        joint_torques.cwiseMax(-tau_max_).cwiseMin(tau_max_);

    // Apply linear motor model
    Eigen::VectorXd emf = joint_velocities.cwiseProduct(mm_slope_);
    constrained_joint_torques =
        constrained_joint_torques.cwiseMax(-tau_max_ - emf)
            .cwiseMin(tau_max_ - emf);

    // Check if torques were modified
    return constrained_joint_torques.isApprox(joint_torques);
}

bool QuadKD::isValidFullState(const Eigen::VectorXd &body_state,
                              const Eigen::VectorXd &joint_state,
                              const Eigen::VectorXd &joint_torques,
                              const grid_map::GridMap &terrain,
                              Eigen::VectorXd &state_violation,
                              Eigen::VectorXd &control_violation) {
    // Check state constraints
    // Kinematics
    state_violation.setZero(num_feet_);
    for (int i = 0; i < num_feet_; i++) {
        Eigen::Vector3d knee_pos_world;
        worldToKneeFKWorldFrame(i, body_state.segment<3>(0),
                                body_state.segment<3>(3),
                                joint_state.segment<3>(3 * i), knee_pos_world);
        state_violation[i] = getGroundClearance(knee_pos_world, terrain);
    }
    bool state_valid = (state_violation.array() >= 0).all();

    // Check control constraints
    // Motor model
    Eigen::VectorXd constrained_joint_torques(12);
    bool control_valid = applyMotorModel(joint_torques, joint_state.tail(12),
                                         constrained_joint_torques);
    control_violation.setZero(joint_torques.size());
    control_violation = -(constrained_joint_torques - joint_torques).cwiseAbs();

    // Only valid if each subcheck is valid
    return (state_valid && control_valid);
}

bool QuadKD::isValidCentroidalState(
    const Eigen::VectorXd &body_state, const Eigen::VectorXd &foot_positions,
    const Eigen::VectorXd &foot_velocities, const Eigen::VectorXd &grfs,
    const grid_map::GridMap &terrain, Eigen::VectorXd &joint_positions,
    Eigen::VectorXd &joint_velocities, Eigen::VectorXd &joint_torques,
    Eigen::VectorXd &state_violation, Eigen::VectorXd &control_violation) {
    // Convert to full
    bool is_exact = convertCentroidalToFullBody(
        body_state, foot_positions, foot_velocities, grfs, joint_positions,
        joint_velocities, joint_torques);

    Eigen::VectorXd joint_state(24);
    joint_state << joint_positions, joint_velocities;
    bool is_valid =
        isValidFullState(body_state, joint_state, joint_torques, terrain,
                         state_violation, control_violation);

    return (is_exact && is_valid);
}
