#include "conflict_based_search/conflict_based_search.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace conflict_based_search {

ConflictBasedSearch::ConflictBasedSearch(rclcpp::Node::SharedPtr node)
    : node_(node) {

  robot_names_ = node_->declare_parameter<std::vector<std::string>>(
      "robot_names", std::vector<std::string>{"robot_1", "robot_2"});
  update_rate_ = node_->declare_parameter<double>("update_rate", 5.0);
  service_timeout_s_ =
      node_->declare_parameter<double>("service_timeout_s", 30.0);
  warm_start_ = node_->declare_parameter<bool>("warm_start", true);
  max_iterations_ = node_->declare_parameter<int>("max_iterations", 200);

  const double body_length =
      node_->declare_parameter<double>("body_length", 0.6);
  const double body_width = node_->declare_parameter<double>("body_width", 0.3);
  const double body_height =
      node_->declare_parameter<double>("body_height", 0.2);
  // Convert configured body dimensions to OBB half-extents.
  half_extents_ << 0.5 * body_length, 0.5 * body_width, 0.5 * body_height;

  for (const auto& robot : robot_names_) {
    robot_plan_pubs_[robot] =
        node_->create_publisher<quad_msgs::msg::RobotPlan>(
            "/" + robot + "/global_plan", 10);
  }
}

void ConflictBasedSearch::createServiceClients() {
  for (const auto& robot : robot_names_) {
    robot_clients_[robot] =
        node_->create_client<quad_msgs::srv::PlanWithConstraints>(
            "/" + robot + "/plan_with_constraints");
  }
}

bool ConflictBasedSearch::waitForServices(std::chrono::seconds timeout) {
  for (const auto& robot : robot_names_) {
    if (!robot_clients_[robot]->wait_for_service(timeout)) {
      RCLCPP_ERROR(node_->get_logger(),
                   "plan_with_constraints service for %s never came up",
                   robot.c_str());
      return false;
    }
  }
  return true;
}

bool ConflictBasedSearch::callPlanWithConstraints(
    const std::string& robot,
    const quad_msgs::msg::RobotPlanConstraints& constraints, bool warm_start,
    quad_msgs::msg::RobotPlan& out_plan, double& out_path_length) {
  auto request =
      std::make_shared<quad_msgs::srv::PlanWithConstraints::Request>();
  request->constraints = constraints;
  request->warm_start = warm_start;

  auto future = robot_clients_[robot]->async_send_request(request);

  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration<double>(service_timeout_s_);
  while (rclcpp::ok() && future.wait_for(std::chrono::milliseconds(0)) !=
                             std::future_status::ready) {
    if (std::chrono::steady_clock::now() > deadline) {
      RCLCPP_WARN(node_->get_logger(), "plan_with_constraints timed out for %s",
                  robot.c_str());
      return false;
    }
    rclcpp::spin_some(node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  if (!rclcpp::ok()) return false;

  auto response = future.get();
  if (!response->success) {
    RCLCPP_WARN(node_->get_logger(),
                "Planner for %s reported failure (path_length=%.2f)",
                robot.c_str(), response->path_length);
    return false;
  }
  out_plan = response->plan;
  out_path_length = response->path_length;
  return true;
}

bool ConflictBasedSearch::requestInitialPlans(CBSNode& node) {

  constexpr int kMaxAttempts = 5;
  constexpr auto kRetryDelay = std::chrono::milliseconds(500);

  for (const auto& robot : node.robot_names) {
    quad_msgs::msg::RobotPlanConstraints empty_constraints;
    empty_constraints.length = 2.0 * half_extents_[0];
    empty_constraints.width = 2.0 * half_extents_[1];
    empty_constraints.height = 2.0 * half_extents_[2];

    quad_msgs::msg::RobotPlan plan;
    double length = 0.0;
    bool ok = false;
    for (int attempt = 1; attempt <= kMaxAttempts && rclcpp::ok() && !ok;
         ++attempt) {
      ok = callPlanWithConstraints(robot, empty_constraints,
                                   /*warm_start=*/false, plan, length);
      if (!ok) {
        RCLCPP_WARN(node_->get_logger(),
                    "Initial plan for %s failed (attempt %d/%d), retrying",
                    robot.c_str(), attempt, kMaxAttempts);
        std::this_thread::sleep_for(kRetryDelay);
      }
    }
    if (!ok) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Initial plan for %s failed after %d attempts; aborting",
                   robot.c_str(), kMaxAttempts);
      return false;
    }
    node.robot_plan_map[robot] = plan;
    node.cost_map[robot] = length;
    // Seed constraint messages with the shared body dimensions.
    node.constraints[robot] = quad_msgs::msg::RobotPlanConstraints();
    node.constraints[robot].length = 2.0 * half_extents_[0];
    node.constraints[robot].width = 2.0 * half_extents_[1];
    node.constraints[robot].height = 2.0 * half_extents_[2];
  }
  node.updateCost();
  return true;
}

bool ConflictBasedSearch::requestReplan(CBSNode& node,
                                        const std::string& robot) {
  quad_msgs::msg::RobotPlan plan;
  double length = 0.0;
  if (!callPlanWithConstraints(robot, node.constraints[robot], warm_start_,
                               plan, length)) {
    return false;
  }
  node.robot_plan_map[robot] = plan;
  node.cost_map[robot] = length;
  node.updateCost();
  return true;
}

ConflictBasedSearch::BodyPose ConflictBasedSearch::poseFromState(
    const quad_msgs::msg::RobotState& state) {
  BodyPose p;
  p.pos =
      Eigen::Vector3d(state.body.pose.position.x, state.body.pose.position.y,
                      state.body.pose.position.z);
  // RobotPlan body orientation follows the standard ROS quaternion convention.
  const auto& q = state.body.pose.orientation;
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  p.yaw = std::atan2(siny_cosp, cosy_cosp);
  p.t = rclcpp::Time(state.header.stamp).seconds();
  return p;
}

ConflictBasedSearch::BodyPose ConflictBasedSearch::samplePoseAtTime(
    const quad_msgs::msg::RobotPlan& plan, double t) {
  if (plan.states.empty()) {
    BodyPose p;
    p.pos.setZero();
    p.yaw = 0.0;
    p.t = t;
    return p;
  }

  const double t_first =
      rclcpp::Time(plan.states.front().header.stamp).seconds();
  const double t_last = rclcpp::Time(plan.states.back().header.stamp).seconds();
  if (t <= t_first) return poseFromState(plan.states.front());
  if (t >= t_last) return poseFromState(plan.states.back());

  // Plans are short enough that linear search is acceptable here.
  for (size_t i = 0; i + 1 < plan.states.size(); ++i) {
    const double t_a = rclcpp::Time(plan.states[i].header.stamp).seconds();
    const double t_b = rclcpp::Time(plan.states[i + 1].header.stamp).seconds();
    if (t >= t_a && t <= t_b) {
      const double alpha = (t_b > t_a) ? (t - t_a) / (t_b - t_a) : 0.0;
      const BodyPose pa = poseFromState(plan.states[i]);
      const BodyPose pb = poseFromState(plan.states[i + 1]);
      BodyPose out;
      out.pos = pa.pos + alpha * (pb.pos - pa.pos);
      // Interpolate yaw on the unit circle to avoid wrap discontinuities.
      const Eigen::Vector2d v_a(std::cos(pa.yaw), std::sin(pa.yaw));
      const Eigen::Vector2d v_b(std::cos(pb.yaw), std::sin(pb.yaw));
      const Eigen::Vector2d v = v_a + alpha * (v_b - v_a);
      out.yaw = std::atan2(v.y(), v.x());
      out.t = t;
      return out;
    }
  }
  return poseFromState(plan.states.back());
}

bool ConflictBasedSearch::obbsOverlap(const BodyPose& a,
                                      const Eigen::Vector3d& half_a,
                                      const BodyPose& b,
                                      const Eigen::Vector3d& half_b) {
  // Early reject if the boxes do not overlap in height.
  if (std::abs(a.pos.z() - b.pos.z()) > half_a.z() + half_b.z()) {
    return false;
  }

  // Use planar SAT to reject boxes with a separating yaw-frame axis.
  const double cos_a = std::cos(a.yaw), sin_a = std::sin(a.yaw);
  const double cos_b = std::cos(b.yaw), sin_b = std::sin(b.yaw);

  const std::array<Eigen::Vector2d, 4> axes = {
      Eigen::Vector2d(cos_a, sin_a),
      Eigen::Vector2d(-sin_a, cos_a),
      Eigen::Vector2d(cos_b, sin_b),
      Eigen::Vector2d(-sin_b, cos_b),
  };

  auto corners = [](const BodyPose& p, const Eigen::Vector3d& he) {
    const double c = std::cos(p.yaw), s = std::sin(p.yaw);
    std::array<Eigen::Vector2d, 4> out;
    const double signs[4][2] = {{1, 1}, {1, -1}, {-1, -1}, {-1, 1}};
    for (int i = 0; i < 4; ++i) {
      const double lx = signs[i][0] * he.x();
      const double wy = signs[i][1] * he.y();
      out[i] = Eigen::Vector2d(p.pos.x() + c * lx - s * wy,
                               p.pos.y() + s * lx + c * wy);
    }
    return out;
  };

  const auto corners_a = corners(a, half_a);
  const auto corners_b = corners(b, half_b);
  for (const auto& axis : axes) {
    double min_a = std::numeric_limits<double>::infinity();
    double max_a = -std::numeric_limits<double>::infinity();
    double min_b = std::numeric_limits<double>::infinity();
    double max_b = -std::numeric_limits<double>::infinity();
    for (const auto& c : corners_a) {
      const double p = c.dot(axis);
      min_a = std::min(min_a, p);
      max_a = std::max(max_a, p);
    }
    for (const auto& c : corners_b) {
      const double p = c.dot(axis);
      min_b = std::min(min_b, p);
      max_b = std::max(max_b, p);
    }
    if (max_a < min_b || max_b < min_a) return false;
  }
  return true;
}

bool ConflictBasedSearch::findFirstConflict(const CBSNode& node,
                                            Conflict& out) const {
  // Choose the earliest conflict for deterministic CBS expansion order.
  bool found = false;
  Conflict best;
  double best_t_start = std::numeric_limits<double>::infinity();

  for (size_t i = 0; i < node.robot_names.size(); ++i) {
    for (size_t j = i + 1; j < node.robot_names.size(); ++j) {
      const std::string& ra = node.robot_names[i];
      const std::string& rb = node.robot_names[j];
      const auto& plan_a = node.robot_plan_map.at(ra);
      const auto& plan_b = node.robot_plan_map.at(rb);

      // Compare time-aligned OBBs, including midpoints for swept conflicts.
      int collision_run_start = -1;
      auto check_pair = [&](const BodyPose& pa, const BodyPose& pb, int idx) {
        const bool overlap = obbsOverlap(pa, half_extents_, pb, half_extents_);
        if (overlap) {
          if (collision_run_start < 0) {
            collision_run_start = idx;
          }
        } else if (collision_run_start >= 0) {
          // Run just ended at idx; record the conflict.
          if (pa.t < best_t_start || (!found && pa.t == best_t_start)) {
            best.robot_a = ra;
            best.robot_b = rb;
            best.t_start_idx = collision_run_start;
            best.t_end_idx = idx - 1;
            best_t_start = pa.t;
            found = true;
          }
          collision_run_start = -1;
        }
      };

      for (size_t k = 0; k < plan_a.states.size(); ++k) {
        const BodyPose pa = poseFromState(plan_a.states[k]);
        const BodyPose pb = samplePoseAtTime(plan_b, pa.t);
        check_pair(pa, pb, static_cast<int>(k));

        // Mid-segment check catches swept conflicts between samples.
        if (k + 1 < plan_a.states.size()) {
          const BodyPose pa_next = poseFromState(plan_a.states[k + 1]);
          BodyPose pa_mid;
          pa_mid.pos = 0.5 * (pa.pos + pa_next.pos);
          const Eigen::Vector2d v(std::cos(pa.yaw) + std::cos(pa_next.yaw),
                                  std::sin(pa.yaw) + std::sin(pa_next.yaw));
          pa_mid.yaw = std::atan2(v.y(), v.x());
          pa_mid.t = 0.5 * (pa.t + pa_next.t);
          const BodyPose pb_mid = samplePoseAtTime(plan_b, pa_mid.t);
          if (obbsOverlap(pa_mid, half_extents_, pb_mid, half_extents_) &&
              collision_run_start < 0) {
            // Attribute a midpoint-only collision to segment k -> k + 1.
            collision_run_start = static_cast<int>(k);
          }
        }
      }
      // Trailing run that runs to plan end.
      if (collision_run_start >= 0) {
        const double t_a =
            rclcpp::Time(plan_a.states.front().header.stamp).seconds();
        if (t_a < best_t_start || !found) {
          best.robot_a = ra;
          best.robot_b = rb;
          best.t_start_idx = collision_run_start;
          best.t_end_idx = static_cast<int>(plan_a.states.size()) - 1;
          best_t_start = t_a;
          found = true;
        }
      }
    }
  }
  if (found) out = best;
  return found;
}

quad_msgs::msg::RobotPlanConstraints
ConflictBasedSearch::buildConstraintFromConflict(
    const CBSNode& node, const Conflict& conflict,
    const std::string& robot_to_constrain) const {
  // Constrain this robot against the other robot's time-aligned poses.
  const std::string& other = (robot_to_constrain == conflict.robot_a)
                                 ? conflict.robot_b
                                 : conflict.robot_a;
  const auto& plan_a = node.robot_plan_map.at(conflict.robot_a);
  const auto& plan_other = node.robot_plan_map.at(other);

  // Start from inherited constraints so they accumulate down the CBS tree.
  quad_msgs::msg::RobotPlanConstraints out =
      node.constraints.at(robot_to_constrain);

  for (int i = conflict.t_start_idx; i <= conflict.t_end_idx; ++i) {
    if (i < 0 || i >= static_cast<int>(plan_a.states.size())) continue;
    const double t = rclcpp::Time(plan_a.states[i].header.stamp).seconds();
    const BodyPose other_pose = samplePoseAtTime(plan_other, t);
    out.pos_x.push_back(other_pose.pos.x());
    out.pos_y.push_back(other_pose.pos.y());
    out.pos_z.push_back(other_pose.pos.z());
    out.yaw.push_back(other_pose.yaw);
    out.t_start.push_back(t);
    out.t_end.push_back(t);
  }
  return out;
}

void ConflictBasedSearch::publishPlans(const CBSNode& node) {
  const rclcpp::Time new_ts = node_->now();
  for (const auto& robot : node.robot_names) {
    auto plan = node.robot_plan_map.at(robot);
    if (plan.states.empty()) continue;

    // Rebase state stamps to now while preserving inter-state intervals.
    const rclcpp::Time origin(plan.states.front().header.stamp);
    for (auto& state : plan.states) {
      const rclcpp::Duration offset = rclcpp::Time(state.header.stamp) - origin;
      state.header.stamp = new_ts + offset;
    }

    plan.header.stamp = new_ts;
    plan.global_plan_timestamp = new_ts;
    robot_plan_pubs_[robot]->publish(plan);
  }
}

void ConflictBasedSearch::run() {
  createServiceClients();
  if (!waitForServices(
          std::chrono::seconds(static_cast<int>(service_timeout_s_)))) {
    return;
  }

  std::priority_queue<std::shared_ptr<CBSNode>,
                      std::vector<std::shared_ptr<CBSNode>>, CBSNodeCompare>
      open;

  auto root = std::make_shared<CBSNode>();
  root->robot_names = robot_names_;
  if (!requestInitialPlans(*root)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to obtain initial plans");
    return;
  }
  open.push(root);

  // Keep per-expansion logs at DEBUG; INFO/WARN reports termination.
  const auto t_solve_start = std::chrono::steady_clock::now();
  int iters = 0;
  int total_replans = 0;

  while (!open.empty() && rclcpp::ok() && iters < max_iterations_) {
    ++iters;
    auto current = open.top();
    open.pop();

    Conflict conflict;
    if (!findFirstConflict(*current, conflict)) {
      const double solve_s =
          std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                        t_solve_start)
              .count();
      RCLCPP_INFO(node_->get_logger(),
                  "CBS converged | expansions=%d | replans=%d | "
                  "robots=%zu | final_cost=%.2f | solve=%.2fs",
                  iters, total_replans, current->robot_names.size(),
                  current->cost, solve_s);
      publishPlans(*current);
      return;
    }

    RCLCPP_DEBUG(
        node_->get_logger(),
        "Resolving conflict between %s and %s, idx [%d, %d] (cost=%.2f)",
        conflict.robot_a.c_str(), conflict.robot_b.c_str(),
        conflict.t_start_idx, conflict.t_end_idx, current->cost);

    // Branch once per involved robot and replan only that child robot.
    for (const std::string& replan_robot :
         {conflict.robot_a, conflict.robot_b}) {
      auto child = std::make_shared<CBSNode>(*current);
      child->constraints[replan_robot] =
          buildConstraintFromConflict(*current, conflict, replan_robot);
      if (requestReplan(*child, replan_robot)) {
        open.push(child);
        ++total_replans;
      }
    }
  }

  const double solve_s = std::chrono::duration<double>(
                             std::chrono::steady_clock::now() - t_solve_start)
                             .count();
  if (open.empty()) {
    RCLCPP_WARN(node_->get_logger(),
                "CBS gave up | expansions=%d | replans=%d | "
                "open list empty | solve=%.2fs",
                iters, total_replans, solve_s);
  } else {
    RCLCPP_WARN(node_->get_logger(),
                "CBS hit cap | expansions=%d | replans=%d | "
                "max_iterations=%d | publishing best-known | solve=%.2fs",
                iters, total_replans, max_iterations_, solve_s);
    publishPlans(*open.top());
  }
}

}  // namespace conflict_based_search
