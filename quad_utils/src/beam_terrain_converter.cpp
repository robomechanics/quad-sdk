#include "quad_utils/beam_terrain_converter.hpp"

#include <grid_map_ros/grid_map_ros.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace beam_terrain {

BeamTerrainConverter::BeamTerrainConverter(rclcpp::Node::SharedPtr node)
    : node_(node),
      grid_map_resolution_(kDefaultGridMapResolution),
      layer_name_(kDefaultLayerName),
      latch_grid_map_pub_(kDefaultLatchGridMapPub),
      verbose_(kDefaultVerbose),
      frame_id_(kDefaultFrameId),
      anchor_pose_topic_(kDefaultAnchorPoseTopic),
      beam_width_(kDefaultBeamWidth),
      beam_length_(kDefaultBeamLength),
      beam_back_(kDefaultBeamBack),
      platform_width_(kDefaultPlatformWidth),
      start_platform_len_(kDefaultStartPlatformLen),
      end_platform_len_(kDefaultEndPlatformLen),
      anchor_received_(false) {
  getParametersFromRos();
  advertiseTopics();

  // Publish a default beam immediately (origin, along +x, z=0) so a terrain map
  // always exists, then re-anchor to the robot on the first pose.
  generateBeam(0.0, 0.0, 0.0);
  RCLCPP_INFO(node_->get_logger(),
              "beam_terrain: default beam published (w=%.3f l=%.2f); will "
              "anchor to robot on first pose from '%s'",
              beam_width_, beam_length_, anchor_pose_topic_.c_str());

  anchor_pose_sub_ = node_->create_subscription<quad_msgs::msg::RobotState>(
      anchor_pose_topic_, rclcpp::QoS(10),
      std::bind(&BeamTerrainConverter::anchorPoseCallback, this,
                std::placeholders::_1));
}

void BeamTerrainConverter::getParametersFromRos() {
  node_->declare_parameter("grid_map_resolution", grid_map_resolution_);
  node_->get_parameter("grid_map_resolution", grid_map_resolution_);

  node_->declare_parameter("layer_name", layer_name_);
  node_->get_parameter("layer_name", layer_name_);

  node_->declare_parameter("latch_grid_map_pub", latch_grid_map_pub_);
  node_->get_parameter("latch_grid_map_pub", latch_grid_map_pub_);

  node_->declare_parameter("verbose", verbose_);
  node_->get_parameter("verbose", verbose_);

  node_->declare_parameter("frame_id", frame_id_);
  node_->get_parameter("frame_id", frame_id_);

  node_->declare_parameter("anchor_pose_topic", anchor_pose_topic_);
  node_->get_parameter("anchor_pose_topic", anchor_pose_topic_);

  node_->declare_parameter("beam_width", beam_width_);
  node_->get_parameter("beam_width", beam_width_);

  node_->declare_parameter("beam_length", beam_length_);
  node_->get_parameter("beam_length", beam_length_);

  node_->declare_parameter("beam_back", beam_back_);
  node_->get_parameter("beam_back", beam_back_);

  node_->declare_parameter("platform_width", platform_width_);
  node_->get_parameter("platform_width", platform_width_);

  node_->declare_parameter("start_platform_len", start_platform_len_);
  node_->get_parameter("start_platform_len", start_platform_len_);

  node_->declare_parameter("end_platform_len", end_platform_len_);
  node_->get_parameter("end_platform_len", end_platform_len_);
}

void BeamTerrainConverter::advertiseTopics() {
  rclcpp::QoS qos(1);
  if (latch_grid_map_pub_) {
    qos = qos.transient_local();
  }
  grid_map_pub_ =
      node_->create_publisher<grid_map_msgs::msg::GridMap>("terrain_map_raw",
                                                           qos);
}

void BeamTerrainConverter::anchorPoseCallback(
    const quad_msgs::msg::RobotState::SharedPtr msg) {
  if (anchor_received_) {
    return;
  }
  const auto& q = msg->body.pose.orientation;
  const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  // Center the beam on the FOOT-STANCE center (average foot x/y), not the body
  // pose -- this is immune to any mocap pivot offset, so the beam sits centered
  // under where the feet actually are. Foot-contact height = lowest foot z
  // (keeps the robot on the floor regardless of absolute mocap z). Fall back to
  // the body pose if no feet are present.
  double x = msg->body.pose.position.x;
  double y = msg->body.pose.position.y;
  double foot_z = msg->body.pose.position.z;
  if (!msg->feet.feet.empty()) {
    double sx = 0.0, sy = 0.0;
    foot_z = msg->feet.feet[0].position.z;
    for (const auto& f : msg->feet.feet) {
      sx += f.position.x;
      sy += f.position.y;
      foot_z = std::min(foot_z, f.position.z);
    }
    x = sx / static_cast<double>(msg->feet.feet.size());
    y = sy / static_cast<double>(msg->feet.feet.size());
  }
  anchor_received_ = true;

  // NOTE: yaw is only logged, NOT applied. Rotating the beam staircases the grid
  // edges (which corrupts foothold selection), so the beam is axis-aligned and
  // the robot must face along map X. Warn if the heading is far off.
  RCLCPP_INFO(node_->get_logger(),
              "beam_terrain: anchoring to robot x=%.3f y=%.3f foot_z=%.3f "
              "yaw=%.1f deg (w=%.3f). Beam is axis-aligned (map X) -- face the "
              "robot down map X.",
              x, y, foot_z, yaw * 180.0 / M_PI, beam_width_);
  generateBeam(x, y, foot_z - kToeRadius);

  anchor_pose_sub_.reset();  // one-shot
}

bool BeamTerrainConverter::generateBeam(double x, double y, double z_top) {
  // Axis-aligned beam along map X, lateral center at y, start at x.
  // Forward (map X) regions ahead of/behind the start point:
  //   [-beam_back_, start_platform_len_]               -> wide start platform
  //   [start_platform_len_, +beam_length_]             -> narrow beam
  //   [..., + end_platform_len_]                        -> wide end platform
  const double s0 = start_platform_len_;
  const double s1 = s0 + beam_length_;
  const double s2 = s1 + end_platform_len_;
  const double hw_beam = beam_width_ / 2.0;
  const double hw_plat = platform_width_ / 2.0;

  // Axis-aligned bounding box (no rotation): X in [x-beam_back, x+s2], Y spans
  // the widest part (the platforms).
  const double minx = x - beam_back_, maxx = x + s2;
  const double miny = y - hw_plat, maxy = y + hw_plat;
  const double margin = 0.25;

  grid_map::GridMap map;
  map.setFrameId(frame_id_);
  grid_map::Position center((minx + maxx) / 2.0, (miny + maxy) / 2.0);
  map.setGeometry(
      grid_map::Length((maxx - minx) + 2 * margin, (maxy - miny) + 2 * margin),
      grid_map_resolution_, center);

  map.add(layer_name_);
  const grid_map::Size sz = map.getSize();
  Eigen::MatrixXf x_data(sz(0), sz(1)), y_data(sz(0), sz(1));
  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    const grid_map::Index idx(*it);
    grid_map::Position p;
    map.getPosition(idx, p);
    const double fwd = p.x() - x;   // along map X
    const double lat = p.y() - y;   // lateral
    double half = -1.0;             // <0 => off-terrain
    if (fwd >= -beam_back_ && fwd <= s0) {
      half = hw_plat;               // start platform
    } else if (fwd > s0 && fwd <= s1) {
      half = hw_beam;               // narrow beam
    } else if (fwd > s1 && fwd <= s2) {
      half = hw_plat;               // end platform
    }
    const bool on = (half > 0.0 && std::fabs(lat) <= half);
    map.at(layer_name_, idx) = on
                                   ? static_cast<float>(z_top)
                                   : std::numeric_limits<float>::quiet_NaN();
    x_data(idx(0), idx(1)) = p.x();
    y_data(idx(0), idx(1)) = p.y();
  }
  map.add("x", x_data);
  map.add("y", y_data);

  map.setTimestamp(node_->now().nanoseconds());
  auto message_ptr = grid_map::GridMapRosConverter::toMessage(map);
  grid_map_pub_->publish(*message_ptr);
  if (verbose_) {
    RCLCPP_INFO(node_->get_logger(),
                "beam_terrain: published beam %.2f x %.2f m (%i x %i), top "
                "z=%.3f",
                map.getLength().x(), map.getLength().y(), sz(0), sz(1), z_top);
  }
  return true;
}

}  // namespace beam_terrain
