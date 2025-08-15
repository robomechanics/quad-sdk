#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

// If you have your own state message, include it instead and adjust callback.
// #include <quad_msgs/msg/robot_state.hpp>

#include <random>
#include <chrono>
#include <string>
#include <cmath>

// GZ transport / msgs (Harmonic)
#include <gz/transport/Node.hh>
#include <gz/msgs/entity_wrench.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/wrench.pb.h>
#include <gz/msgs/vector3d.pb.h>
#include <gz/msgs/time.pb.h>

using namespace std::chrono_literals;

class ApplyForceNode : public rclcpp::Node
{
public:
  ApplyForceNode() : rclcpp::Node("apply_link_wrench")
  {
    // --- Parameters (bool flags select modes) ---
    world_name_         = declare_parameter<std::string>("world_name", "default");
    target_link_name_   = declare_parameter<std::string>("target_link_name", "robot_1::body"); // "model::link" or "link", depending on your model
    duration_sec_       = declare_parameter<double>("duration_sec", 0.5); // how long to apply (persistent topic)
    update_rate_hz_     = declare_parameter<double>("update_rate_hz", 50.0);

    // modes
    use_periodic_mode_  = declare_parameter<bool>("use_periodic_mode", true);
    use_distance_mode_  = declare_parameter<bool>("use_distance_mode", false); // mutually exclusive with periodic
    distance_thresh_    = declare_parameter<double>("distance_threshold", 1.0);

    // force/torque configuration
    use_random_force_   = declare_parameter<bool>("use_random_force", false);
    force_mag_fixed_    = declare_parameter<double>("force_magnitude", 10.0);      // used if not random
    force_mag_min_      = declare_parameter<double>("force_magnitude_min", 5.0);
    force_mag_max_      = declare_parameter<double>("force_magnitude_max", 15.0);

    // fixed direction (unit vector); if all zeros and random mode on, we pick a random direction
    fx_dir_ = declare_parameter<double>("force_dir_x", 1.0);
    fy_dir_ = declare_parameter<double>("force_dir_y", 0.0);
    fz_dir_ = declare_parameter<double>("force_dir_z", 0.0);

    // optional torque
    tx_ = declare_parameter<double>("torque_x", 0.0);
    ty_ = declare_parameter<double>("torque_y", 0.0);
    tz_ = declare_parameter<double>("torque_z", 0.0);

    period_sec_ = declare_parameter<double>("period_sec", 2.0);  // only used in periodic mode

    // nominal pose to measure displacement from (distance mode)
    nominal_x_ = declare_parameter<double>("nominal_x", 0.0);
    nominal_y_ = declare_parameter<double>("nominal_y", 0.0);
    nominal_z_ = declare_parameter<double>("nominal_z", 0.0);

    // --- Subscribers (replace with your state msg if needed) ---
    // Here we assume a pose topic; adapt to your /state/ground_truth.
    // For quad_msgs::RobotState, just store x,y,z from msg->body.pose.position.
    state_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      declare_parameter<std::string>("state_topic", "/robot_1/pose"),
      rclcpp::QoS(10),
      std::bind(&ApplyForceNode::stateCallback, this, std::placeholders::_1));

    // --- Timers ---
    auto dt = std::chrono::duration<double>(1.0 / std::max(1.0, update_rate_hz_));
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(dt),
                               std::bind(&ApplyForceNode::spinOnce, this));

    // Build GZ transport topics
    wrench_topic_persist_ = "/world/" + world_name_ + "/wrench/persistent";
    wrench_topic_clear_   = "/world/" + world_name_ + "/wrench/clear";

    RCLCPP_INFO(get_logger(), "ApplyLinkWrench using [%s] (start), clear [%s]",
                wrench_topic_persist_.c_str(), wrench_topic_clear_.c_str());

    // Seed RNG
    rng_.seed(std::random_device{}());
  }

private:
  // --- Robot state callback (store latest pose) ---
  void stateCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    last_x_ = msg->pose.position.x;
    last_y_ = msg->pose.position.y;
    last_z_ = msg->pose.position.z;
    have_pose_ = true;
  }

  // Euclidean distance
  static double dist3(double x1,double y1,double z1,double x2,double y2,double z2)
  {
    const double dx=x2-x1, dy=y2-y1, dz=z2-z1;
    return std::sqrt(dx*dx+dy*dy+dz*dz);
  }

  // Decide if we should apply force this tick
  bool shouldTrigger()
  {
    if (use_periodic_mode_) {
      const auto now = nowSteady();
      if (now - last_fire_time_ >= std::chrono::duration<double>(period_sec_)) {
        last_fire_time_ = now;
        return true;
      }
      return false;
    }
    if (use_distance_mode_ && have_pose_) {
      const double d = dist3(nominal_x_, nominal_y_, nominal_z_, last_x_, last_y_, last_z_);
      if (d >= distance_thresh_) {
        // update nominal like your ROS1 set_point increment
        nominal_x_ = last_x_;
        nominal_y_ = last_y_;
        nominal_z_ = last_z_;
        return true;
      }
      return false;
    }
    // default: do nothing unless a mode is enabled
    return false;
  }

  // Build and publish GZ EntityWrench (start persistent for duration, then clear)
  void applyForceOnce()
  {
    // Determine force vector
    double fx, fy, fz;
    if (use_random_force_) {
      // random magnitude, direction = provided unit dir or random if near-zero
      std::uniform_real_distribution<double> U(force_mag_min_, force_mag_max_);
      const double mag = U(rng_);
      double vx=fx_dir_, vy=fy_dir_, vz=fz_dir_;
      const double n = std::sqrt(vx*vx+vy*vy+vz*vz);
      if (n < 1e-6) {
        // random unit direction
        std::uniform_real_distribution<double> U2(-1.0, 1.0);
        vx = U2(rng_); vy = U2(rng_); vz = U2(rng_);
        const double n2 = std::sqrt(vx*vx+vy*vy+vz*vz);
        vx /= (n2>1e-9? n2:1.0); vy /= (n2>1e-9? n2:1.0); vz /= (n2>1e-9? n2:1.0);
      } else {
        vx/=n; vy/=n; vz/=n;
      }
      fx = mag*vx; fy = mag*vy; fz = mag*vz;
    } else {
      // fixed magnitude along provided direction
      double vx=fx_dir_, vy=fy_dir_, vz=fz_dir_;
      const double n = std::sqrt(vx*vx+vy*vy+vz*vz);
      if (n < 1e-6) { vx=1; vy=0; vz=0; } else { vx/=n; vy/=n; vz/=n; }
      fx = force_mag_fixed_*vx; fy = force_mag_fixed_*vy; fz = force_mag_fixed_*vz;
    }

    // Build EntityWrench
    gz::msgs::EntityWrench ew;
    auto *entity = ew.mutable_entity();
    entity->set_name(target_link_name_);              // Use name; Gazebo can resolve name to entity.
    entity->set_type(gz::msgs::Entity::LINK);         // target is a link

    auto *w = ew.mutable_wrench();
    w->mutable_force()->set_x(fx);
    w->mutable_force()->set_y(fy);
    w->mutable_force()->set_z(fz);
    w->mutable_torque()->set_x(tx_);
    w->mutable_torque()->set_y(ty_);
    w->mutable_torque()->set_z(tz_);

    // Apply persistently, then clear after duration
    bool ok = gz_node_.Request(wrench_topic_persist_, ew, /*timeout ms*/ 1000, nullptr);
    if (!ok) {
      RCLCPP_WARN(get_logger(), "Failed to send persistent wrench request on %s", wrench_topic_persist_.c_str());
      return;
    }
    // schedule a one-shot timer to clear
    auto self = shared_from_this();
    rclcpp::TimerBase::SharedPtr clear_timer = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(duration_sec_)),
      [this, self, id_name = target_link_name_]() {
        gz::msgs::Entity e;
        e.set_name(id_name);
        e.set_type(gz::msgs::Entity::LINK);
        bool ok2 = gz_node_.Request(wrench_topic_clear_, e, 1000, nullptr);
        if (!ok2) {
          RCLCPP_WARN(this->get_logger(), "Failed to clear wrench on %s", wrench_topic_clear_.c_str());
        }
      });
    one_shot_timers_.push_back(clear_timer);
  }

  void spinOnce()
  {
    if (shouldTrigger()) {
      updateMarkerLikeROS1(); // optional: keep for parity; not published here
      applyForceOnce();
    }
  }

  // (Optional parity placeholder – compute magnitude like your ROS1 code)
  void updateMarkerLikeROS1()
  {
    // You can add a Marker publisher here if you want RViz arrows like before.
  }

  // steady clock helper
  static std::chrono::steady_clock::time_point nowSteady() { return std::chrono::steady_clock::now(); }

private:
  // params
  std::string world_name_, target_link_name_;
  double duration_sec_{0.5};
  double update_rate_hz_{50.0};
  bool use_periodic_mode_{true};
  bool use_distance_mode_{false};
  double distance_thresh_{1.0};
  bool use_random_force_{false};
  double force_mag_fixed_{10.0}, force_mag_min_{5.0}, force_mag_max_{15.0};
  double fx_dir_{1.0}, fy_dir_{0.0}, fz_dir_{0.0};
  double tx_{0.0}, ty_{0.0}, tz_{0.0};
  double period_sec_{2.0};

  // nominal pose
  double nominal_x_{0.0}, nominal_y_{0.0}, nominal_z_{0.0};

  // last pose
  bool have_pose_{false};
  double last_x_{0.0}, last_y_{0.0}, last_z_{0.0};

  // ROS
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<rclcpp::TimerBase::SharedPtr> one_shot_timers_;

  // GZ
  gz::transport::Node gz_node_;
  std::string wrench_topic_persist_, wrench_topic_clear_;

  // timing + rng
  std::chrono::steady_clock::time_point last_fire_time_{std::chrono::steady_clock::now()};
  std::mt19937 rng_;
};
