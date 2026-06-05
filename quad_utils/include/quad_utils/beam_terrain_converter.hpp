#ifndef BEAM_TERRAIN_CONVERTER_H
#define BEAM_TERRAIN_CONVERTER_H

#include <rclcpp/rclcpp.hpp>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <quad_msgs/msg/robot_state.hpp>

#include <memory>
#include <string>

namespace beam_terrain {

constexpr double kDefaultGridMapResolution = 0.01;
static const std::string kDefaultLayerName = "z";
constexpr bool kDefaultLatchGridMapPub = true;
constexpr bool kDefaultVerbose = true;
static const std::string kDefaultFrameId = "map";
static const std::string kDefaultAnchorPoseTopic = "/robot_1/state/ground_truth";
constexpr double kToeRadius = 0.02;  // m, matches local_planner toe_radius
// Beam layout (forward axis): wide START platform under the robot -> narrow beam
// -> wide END platform. The robot spawns on the start platform. Lengths matched
// to the b_beam world mesh: start platform x[-0.65,0.523], narrow beam
// x[0.523,2.872]=2.349 m, end platform x[2.872,4.173]=1.301 m, platforms 2.0 m.
constexpr double kDefaultBeamWidth = 0.11;          // m, narrow-beam full width
constexpr double kDefaultBeamLength = 2.349;        // m, narrow-beam length
constexpr double kDefaultPlatformWidth = 2.0;       // m, platform full width
constexpr double kDefaultStartPlatformLen = 0.523;  // m, start platform ahead of robot
constexpr double kDefaultEndPlatformLen = 1.301;    // m, end platform length
constexpr double kDefaultBeamBack = 0.65;           // m, start platform behind robot

// Publishes a virtual "balance beam" terrain (grid_map on terrain_map_raw) for
// a flat/imaginary beam. The beam is generated analytically (a rotated
// rectangle), so it is clean at any heading (no mesh rasterization) and its
// width is a parameter. It anchors to the robot's start pose (centered in x/y,
// aligned to heading, beam top at foot-contact height) so the robot starts on a
// straight beam regardless of where it is placed.
class BeamTerrainConverter {
 public:
  explicit BeamTerrainConverter(rclcpp::Node::SharedPtr node);

 private:
  void getParametersFromRos();
  void advertiseTopics();

  // Capture the robot's start pose (once) and regenerate the beam anchored to
  // it, then stop listening.
  void anchorPoseCallback(const quad_msgs::msg::RobotState::SharedPtr msg);

  // Generate + publish the beam, AXIS-ALIGNED along map X (no rotation, so the
  // grid edges are clean — never staircased). Centered laterally at y, starting
  // at x: a wide start platform (under the robot) -> narrow beam -> wide end
  // platform, top surface at z_top. Cells inside get z_top; outside get NaN.
  // The robot must face along map X (the beam axis) for the beam to be straight
  // ahead of it.
  bool generateBeam(double x, double y, double z_top);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_pub_;
  rclcpp::Subscription<quad_msgs::msg::RobotState>::SharedPtr anchor_pose_sub_;

  // Parameters
  double grid_map_resolution_;
  std::string layer_name_;
  bool latch_grid_map_pub_;
  bool verbose_;
  std::string frame_id_;
  std::string anchor_pose_topic_;
  double beam_width_;
  double beam_length_;
  double beam_back_;
  double platform_width_;
  double start_platform_len_;
  double end_platform_len_;

  bool anchor_received_;
};

}  // namespace beam_terrain

#endif  // BEAM_TERRAIN_CONVERTER_H
