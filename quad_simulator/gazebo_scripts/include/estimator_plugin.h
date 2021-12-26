#ifndef GAZEBO_SPIRIT_ESTIMATOR_PLUGIN
#define GAZEBO_SPIRIT_ESTIMATOR_PLUGIN

#include <functional>
#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
 #include <gazebo/common/Plugin.hh>
 #include <gazebo/common/UpdateInfo.hh>
#include <ignition/math/Vector3.hh>
#include <quad_msgs/RobotState.h>
#include <quad_utils/math_utils.h>
#include <quad_utils/ros_utils.h>
#include <quad_utils/tail_type.h>

namespace gazebo
{
  class QuadEstimatorGroundTruth : public ModelPlugin
  {
    /// \brief Constructor.
    public:
      QuadEstimatorGroundTruth();
        /// \brief Destructor.
      ~QuadEstimatorGroundTruth();
      void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
      void OnUpdate();

      std::vector< std::string > joint_names_;
      
    private:
      double update_rate_;
      common::Time last_time_;

      ros::Publisher ground_truth_state_pub_;
      ros::Publisher ground_truth_state_body_frame_pub_;
      physics::ModelPtr model_;
      event::ConnectionPtr updateConnection_;
     
      int tail_type_;
  };
  GZ_REGISTER_MODEL_PLUGIN(QuadEstimatorGroundTruth)
}
#endif
