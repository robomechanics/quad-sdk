#ifndef GAZEBO_SPIRIT_ESTIMATOR_PLUGIN
#define GAZEBO_SPIRIT_ESTIMATOR_PLUGIN

<<<<<<< HEAD
#include <quad_msgs/RobotState.h>
#include <quad_utils/math_utils.h>
#include <quad_utils/ros_utils.h>
=======
<<<<<<< HEAD
#include <quad_msgs/RobotState.h>
#include <quad_utils/math_utils.h>
#include <quad_utils/ros_utils.h>
#include <ros/ros.h>

#include <functional>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo {
class QuadEstimatorGroundTruth : public ModelPlugin {
  /// \brief Constructor.
 public:
  QuadEstimatorGroundTruth();
  /// \brief Destructor.
  ~QuadEstimatorGroundTruth();
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
  void OnUpdate();

 private:
  double update_rate_;
  common::Time last_time_;

  ros::Publisher ground_truth_state_pub_;
  ros::Publisher ground_truth_state_body_frame_pub_;
  physics::ModelPtr model_;
  event::ConnectionPtr updateConnection_;

  std::shared_ptr<quad_utils::QuadKD> quadKD_;
};
GZ_REGISTER_MODEL_PLUGIN(QuadEstimatorGroundTruth)
}  // namespace gazebo
=======
#include <functional>
>>>>>>> d5a072b3a89924f1b027bb8b8d27919519fafc18
#include <ros/ros.h>

#include <functional>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo {
class QuadEstimatorGroundTruth : public ModelPlugin {
  /// \brief Constructor.
 public:
  QuadEstimatorGroundTruth();
  /// \brief Destructor.
  ~QuadEstimatorGroundTruth();
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
  void OnUpdate();

 private:
  double update_rate_;
  common::Time last_time_;

  ros::Publisher ground_truth_state_pub_;
  ros::Publisher ground_truth_state_body_frame_pub_;
  physics::ModelPtr model_;
  event::ConnectionPtr updateConnection_;

<<<<<<< HEAD
  std::shared_ptr<quad_utils::QuadKD> quadKD_;
};
GZ_REGISTER_MODEL_PLUGIN(QuadEstimatorGroundTruth)
}  // namespace gazebo
=======
      ros::Publisher ground_truth_state_pub_;
      ros::Publisher ground_truth_state_body_frame_pub_;
      physics::ModelPtr model_;
      event::ConnectionPtr updateConnection_;
  };
  GZ_REGISTER_MODEL_PLUGIN(QuadEstimatorGroundTruth)
}
>>>>>>> Switch build system to catkin_tools, switch spirit* to quad*
>>>>>>> d5a072b3a89924f1b027bb8b8d27919519fafc18
#endif
