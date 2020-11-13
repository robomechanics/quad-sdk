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
#include <spirit_msgs/StateEstimate.h>

namespace gazebo
{
  class SpiritEstimatorGroundTruth : public ModelPlugin
  {
    /// \brief Constructor.
    public:
      SpiritEstimatorGroundTruth();
        /// \brief Destructor.
      ~SpiritEstimatorGroundTruth();
      void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
      void OnUpdate();

    private:
      double update_rate_;
      common::Time last_time_;

      ros::Publisher state_estimate_pub_;
      physics::ModelPtr model_;
      event::ConnectionPtr updateConnection_;
  };
  GZ_REGISTER_MODEL_PLUGIN(SpiritEstimatorGroundTruth)
}
#endif
