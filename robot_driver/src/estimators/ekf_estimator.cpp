#include "robot_driver/estimators/ekf_estimator.h"
#include <tf2/LinearMath/Quaternion.h>
#include <quad_utils/quad_kd.h>
#include <sensor_msgs/Imu.h>


// implementation of algorithm used by Shuo Yan on A1

EKFEstimator::EKFEstimator() {

  eye3.setIdentity();
  C.setZero();

  for (int i=0; i<NUM_LEG; ++i) 
  {
      C.block<3,3>(i*3,0) = -eye3;  //-pos
      C.block<3,3>(i*3,6+i*3) = eye3;  //foot pos
      C.block<3,3>(NUM_LEG*3+i*3,3) = eye3;  // vel
      C(NUM_LEG*6+i,6+i*3+2) = 1;  // height z of foot ** Check Spirit Params
  }

  // Q R are fixed
  Q.setIdentity();
  Q.block<3, 3>(0, 0) = PROCESS_NOISE_PIMU * eye3; // position transition
  Q.block<3, 3>(3, 3) = PROCESS_NOISE_VIMU * eye3; // velocity transition

  // Populating Q
  for (int i = 0; i < NUM_LEG; ++i)
  {
      Q.block<3, 3>(6 + i * 3, 6 + i * 3) = PROCESS_NOISE_PFOOT * eye3; // foot position transition
  }

  R.setIdentity();
  for (int i = 0; i < NUM_LEG; ++i)
  {
      R.block<3, 3>(i * 3, i * 3) = SENSOR_NOISE_PIMU_REL_FOOT * eye3;                             // fk estimation
      R.block<3, 3>(NUM_LEG * 3 + i * 3, NUM_LEG * 3 + i * 3) = SENSOR_NOISE_VIMU_REL_FOOT * eye3; // vel estimation
      R(NUM_LEG * 6 + i, NUM_LEG * 6 + i) = SENSOR_NOISE_ZFOOT;                                    // height z estimation
  }


  // set A to identity
  A.setIdentity();

  // set B to zero
  B.setZero();

  assume_flat_ground = true;  // This boolean might be an issue when finding uneven terrain. Keep an eye out

  

}

// check for status of the flat ground assumption
// EKFEstimator::EKFEstimator(bool assume_flat_ground_) : EKFEstimator()
// {

//     // constructor
//     assume_flat_ground = assume_flat_ground_;
//     // change R according to this flag, if we do not assume the robot moves on flat ground,
//     // then we cannot infer height z using this way
//     if (assume_flat_ground == false)
//     {
//         for (int i = 0; i < NUM_LEG; ++i)
//         {
//             R(NUM_LEG * 6 + i, NUM_LEG * 6 + i) = 1e5; // height z estimation not reliable
//         }
//     }

// }

void EKFEstimator::init(ros::NodeHandle& nh) {
  nh_ = nh;
  std::cout << "EKF Estimator Initiated" << std::endl;

  // Load rosparams from parameter server
  //quad_utils::loadROSParam(nh_, "/state/joints", joint_state_);
  //quad_utils::loadROSParam(nh_, "/state/imu", last_imu_topic_);
  // quad_utils::loadROSParam(nh_, "/state/estimate", state_estimate_topic_);
  // quad_utils::loadROSParam(nh_, "/control/grfs", grf_topic_);
  // quad_utils::loadROSParam(nh_, "/contact_mode", contact_topic_);
  
  // h.param<double>("ekf_estimator/update_rate", update_rate_,200);
  //nh.param<double>("ekf_estimator/joint_state_max_time", joint_state_msg_time_diff_max_, 20);



  P.setIdentity();
  P = P * 3;

  // set initial value of x
  x.setZero();
  x.segment<3>(0) = Eigen::Vector3d(0, 0, 0.09); // Check on the initial z for spirit
  single_joint_state.setZero(); // Initiating the joint states as 0
  StateEstimator::readIMU(last_imu_msg_, fk_, wk_, qk_);
  tf2::Quaternion qek(qk_.x(), qk_.y(), qk_.z(), qk_.w());
  qek.normalize();
  tf2::Matrix3x3 m(qek);
  m.getRPY(rpy_ekf[0], rpy_ekf[1], rpy_ekf[2]);
  quadKD_->getRotationMatrix(rpy_ekf,rot_ekf_);

  for (int i = 0; i < NUM_LEG; ++i)
  {

    // 1. Find the foot positions on the body frame

    quadKD_->bodyToFootFKBodyFrame(i, single_joint_state, foot_pos);
 
    //2. Transform the different foot positions in BF to the shifted position by Rot*fk + x0
    
    x.segment<3>(6 + i * 3) = rot_ekf_* foot_pos + x.segment<3>(0); // Transforming above relative position by the Rot Matrix + initial position
    
  }


}

bool EKFEstimator::updateOnce(quad_msgs::RobotState& last_robot_state_msg) {
  
  // update A B using latest dt
  // Find out what dt value to pass
  double dt = 0.05;
  A.block<3, 3>(0, 3) = dt* eye3;
  B.block<3, 3>(3, 0) = dt * eye3;

  // control input u is Ra + ag
  //Eigen::Vector3d u = state.root_rot_mat * state.imu_acc + Eigen::Vector3d(0, 0, -9.81);
  
  // Getting rotation matrix
  StateEstimator::readIMU(last_imu_msg_, fk_, wk_, qk_);
  tf2::Quaternion qek(qk_.x(), qk_.y(), qk_.z(), qk_.w());
  qek.normalize();
  tf2::Matrix3x3 m(qek);
  m.getRPY(rpy_ekf[0], rpy_ekf[1], rpy_ekf[2]);
  quadKD_->getRotationMatrix(rpy_ekf,rot_ekf_);

  // Calculating u
  Eigen::Vector3d u = rot_ekf_*fk_ + Eigen::Vector3d(0,0,-9.81);

  //Setting contact estimation using standing modes and forces

  /*
  
  For the case of sim on flat terrain, we simply use the local planner generator schedule
  
  To Estimate contact based on forces or momentum observer here 
  
  
  */ 

  // update Q
  Q.block<3, 3>(0, 0) = PROCESS_NOISE_PIMU * dt / 20.0 * eye3;
  Q.block<3, 3>(3, 3) = PROCESS_NOISE_VIMU * dt * 9.8 / 20.0 * eye3;

  // Trying to create object to access contact schedule
  


  // process update
  xbar = A * x + B * u;
  Pbar = A * P * A.transpose() + Q;

  // measurement construction
  yhat = C * xbar;

  // actual measurement
  for (int i = 0; i < NUM_LEG; ++i)
  {
      quadKD_->bodyToFootFKBodyFrame(i, single_joint_state, foot_pos);
      //Eigen::Vector3d fk_pos = state.foot_pos_rel.block<3, 1>(0, i);

      // y.block<3, 1>(i * 3, 0) = rot_ekf_ * foot_pos; // fk estimation
      // Eigen::Vector3d leg_v = -state.foot_vel_rel.block<3, 1>(0, i) - Utils::skew(state.imu_ang_vel) * fk_pos;
      // y.block<3, 1>(NUM_LEG * 3 + i * 3, 0) =
      //     (1.0 - estimated_contacts[i]) * x.segment<3>(3) + estimated_contacts[i] * state.root_rot_mat * leg_v; // vel estimation

      // y(NUM_LEG * 6 + i) =
      //     (1.0 - estimated_contacts[i]) * (x(2) + fk_pos(2)) + estimated_contacts[i] * 0; // height z estimation
  
  
  }

  std::cout << "EKF Estimator Updated Once" << std::endl;
  return true;
}
