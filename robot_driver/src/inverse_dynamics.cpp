#include "robot_driver/inverse_dynamics.h"

InverseDynamicsController::InverseDynamicsController() {}

bool InverseDynamicsController::computeLegCommandArray(
  const quad_msgs::RobotState &robot_state_msg,
  quad_msgs::LegCommandArray &leg_command_array_msg,
  quad_msgs::GRFArray &grf_array_msg)
{
  
  if ((last_local_plan_msg_ == NULL) || 
    ((ros::Time::now() - last_local_plan_msg_->header.stamp).toSec() >= 0.1) ||
    (last_grf_sensor_msg_ == NULL)) {
    
    return false;
  } else {

    leg_command_array_msg.leg_commands.resize(num_feet_);

    // Define vectors for joint positions and velocities
    Eigen::VectorXd joint_positions(3*num_feet_), joint_velocities(3*num_feet_), body_state(12);
    quad_utils::vectorToEigen(robot_state_msg.joints.position, joint_positions);
    quad_utils::vectorToEigen(robot_state_msg.joints.velocity, joint_velocities);
    body_state = quad_utils::bodyStateMsgToEigen(robot_state_msg.body);

    // Define vectors for state positions and velocities 
    Eigen::VectorXd state_positions(3*num_feet_+6), state_velocities(3*num_feet_+6);
    state_positions << joint_positions, body_state.head(6);
    state_velocities << joint_velocities, body_state.tail(6);

    // Initialize variables for ff and fb
    quad_msgs::RobotState ref_state_msg;
    Eigen::VectorXd tau_array(3*num_feet_), tau_swing_leg_array(3*num_feet_);

    // Get reference state and grf from local plan or traj + grf messages
    ros::Time t_first_state = last_local_plan_msg_->states.front().header.stamp;
    double t_now = (ros::Time::now() - last_local_plan_msg_->state_timestamp).toSec(); // Use time of state - RECOMMENDED
    // double t_now = (ros::Time::now() - last_local_plan_time_).toSec(); // Use time of plan receipt
    // double t_now = (ros::Time::now() - t_first_state).toSec(); // Use time of first state in plan

    if ( (t_now < (last_local_plan_msg_->states.front().header.stamp - t_first_state).toSec()) || 
          (t_now > (last_local_plan_msg_->states.back().header.stamp - t_first_state).toSec()) ) {
      ROS_ERROR("ID node couldn't find the correct ref state!");
    }

    // Storage of the current plan index
    int current_idx;

    // Interpolate the local plan to get the reference state and ff GRF
    for (int i = 0; i < last_local_plan_msg_->states.size()-1; i++) {
      
      if ( (t_now >= (last_local_plan_msg_->states[i].header.stamp - t_first_state).toSec()) && 
          ( t_now <  (last_local_plan_msg_->states[i+1].header.stamp - t_first_state).toSec() )) {

        // Record the current plan index
        current_idx = i;

        double t_interp = (t_now - (last_local_plan_msg_->states[i].header.stamp-t_first_state).toSec())/
          (last_local_plan_msg_->states[i+1].header.stamp.toSec() - 
            last_local_plan_msg_->states[i].header.stamp.toSec());
        
        quad_utils::interpRobotState(last_local_plan_msg_->states[i],
          last_local_plan_msg_->states[i+1], t_interp, ref_state_msg);

        // quad_utils::interpGRFArray(last_local_plan_msg_->grfs[i],
        //   last_local_plan_msg_->grfs[i+1], t_interp, grf_array_msg);

        // ref_state_msg = last_local_plan_msg_->states[i];
        grf_array_msg = last_local_plan_msg_->grfs[i];

        break;
      }
    }

    // Declare plan and state data as Eigen vectors
    Eigen::VectorXd ref_body_state(12), grf_array(3 * num_feet_),
        ref_foot_positions(3 * num_feet_), ref_foot_velocities(3 * num_feet_), ref_foot_acceleration(3 * num_feet_),
        current_foot_positions(3 * num_feet_), current_foot_velocities(3 * num_feet_), current_foot_acceleration(3 * num_feet_),
        nominal_foot_positions(3 * num_feet_), nominal_foot_velocities(3 * num_feet_), nominal_foot_acceleration(3 * num_feet_);

    // Load current foot data
    quad_utils::multiFootStateMsgToEigen(
        robot_state_msg.feet, current_foot_positions, current_foot_velocities, current_foot_acceleration);

    // Load nominal foot data
    quad_utils::multiFootStateMsgToEigen(
        last_local_plan_msg_->states[current_idx].feet_nominal, nominal_foot_positions, nominal_foot_velocities, nominal_foot_acceleration);

    // Load plan and state data from messages
    ref_body_state = quad_utils::bodyStateMsgToEigen(ref_state_msg.body);
    quad_utils::multiFootStateMsgToEigen(
        ref_state_msg.feet, ref_foot_positions, ref_foot_velocities, ref_foot_acceleration);
    grf_array = quad_utils::grfArrayMsgToEigen(grf_array_msg);
    if (last_grf_array_.norm() >= 1e-3) {
      grf_array = grf_exp_filter_const_*grf_array.array() + 
        (1 - grf_exp_filter_const_)*last_grf_array_.array();
      quad_utils::eigenToGRFArrayMsg(grf_array, ref_state_msg.feet, grf_array_msg);
    }

    // Load contact mode
    std::vector<int> contact_mode(num_feet_);
    for (int i = 0; i < num_feet_; i++) {
      contact_mode[i] = ref_state_msg.feet.feet[i].contact;
    }

    // Contact sensing
    // Start from nominal contact schedule
    std::vector<int> adaptive_contact_mode = contact_mode;
  
    for (size_t i = 0; i < num_feet_; i++)
    {
      // Swing or contact to miss
      if (contact_mode.at(i) &&
          !last_contact_sensing_msg_.data.at(i) &&
          (current_foot_positions(3 * i + 2) - body_state(2)) < -0.295)
          // (current_foot_positions(3 * i + 2) - nominal_foot_positions(3 * i + 2)) < -0.1)
      {
        ROS_WARN_STREAM("Leg controller: swing or contact to miss leg: " << i);

        last_contact_sensing_msg_.data.at(i) = true;

        // Record the joint position and hold it there
        // for (size_t j = 0; j < 3; j++)
        // {
        //   joint_pos_miss_contact_(3 * i + j) = last_local_plan_msg_->states.at(current_idx).joints_nominal.position.at(3 * i + j);
        //   ref_state_msg.joints.position.at(3 * i + j) = joint_pos_miss_contact_(3 * i + j);
        // }

        adaptive_contact_mode.at(i) = false;
        grf_array.segment(3*i, 3) << 0, 0, 0;
      }

      // // Miss to contact
      // if (last_contact_sensing_msg_.data.at(i) &&
      //     contact_mode.at(i) &&
      //     last_grf_sensor_msg_->contact_states.at(i) &&
      //     last_grf_sensor_msg_->vectors.at(i).z >= 5)
      // {
      //   ROS_WARN_STREAM("Leg controller: miss to contact leg: " << i);

      //   last_contact_sensing_msg_.data.at(i) = false;
      // }

      // // Miss to swing
      // if (last_contact_sensing_msg_.data.at(i) &&
      //     !contact_mode.at(i))
      // {
      //   ROS_WARN_STREAM("Leg controller: miss to swing leg: " << i);

      //   last_contact_sensing_msg_.data.at(i) = false;
      // }

      // Miss to contact or swing
      if (last_contact_sensing_msg_.data.at(i) &&
          last_grf_sensor_msg_->contact_states.at(i) &&
          last_grf_sensor_msg_->vectors.at(i).z >= 5)
      {
        ROS_WARN_STREAM("Leg controller: miss to contact or swing leg: " << i);

        last_contact_sensing_msg_.data.at(i) = false;
      }

      // Keep miss
      if (last_contact_sensing_msg_.data.at(i) &&
          contact_mode.at(i))
      {
        // Hold the joint position as the record
        // for (size_t j = 0; j < 3; j++)
        // {
        //   ref_state_msg.joints.position.at(3 * i + j) = joint_pos_miss_contact_(3 * i + j);
        // }

        adaptive_contact_mode.at(i) = false;
        grf_array.segment(3*i, 3) << 0, 0, 0;
      }

      // Otherwise is keep contact, keep swing, swing to contact, or contact to swing
    }

    // Compute swing hold position
    Eigen::VectorXd refined_foot_positions = ref_foot_positions;
    for (size_t i = 0; i < 4; i++)
    {
      if (last_contact_sensing_msg_.data.at(i))
      {
        Eigen::Vector3d hip_pos, nominal_foot_shift;
        quadKD_->worldToNominalHipFKWorldFrame(i, body_state.segment(0, 3), body_state.segment(3, 3), hip_pos);
        nominal_foot_shift << 0, 0, -0.35;

        Eigen::Matrix3d rotation_matrix, body_rotation_matrix;
        double theta = 0.707;
        double yaw = body_state(5);
        body_rotation_matrix << cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1;
        body_rotation_matrix = body_rotation_matrix.transpose();
        rotation_matrix << 1, 0, 0, 0, cos(theta), -sin(theta), 0, sin(theta), cos(theta);
        rotation_matrix = rotation_matrix.transpose();

        refined_foot_positions.segment(3 * i, 3) = (body_rotation_matrix * rotation_matrix * nominal_foot_shift + hip_pos);
      }
    }

    // Compute foot state
    quad_msgs::RobotState refined_state_msg;
    quad_msgs::MultiFootState refined_foot_msg;
    refined_foot_msg.feet.resize(4);
    for (size_t i = 0; i < 4; i++)
    {
      quad_utils::eigenToFootStateMsg(refined_foot_positions.segment(3 * i, 3), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), refined_foot_msg.feet[i]);
    }
    refined_state_msg.body = quad_utils::eigenToBodyStateMsg(body_state);
    refined_state_msg.feet = refined_foot_msg;
    quad_utils::ikRobotState(*quadKD_, refined_state_msg);

    // Compute joint torques
    quadKD_->computeInverseDynamics(state_positions, state_velocities, ref_foot_acceleration, grf_array,
                                    contact_mode, tau_array);

    // Copy to the ref state messsage
    for (size_t i = 0; i < 4; i++)
    {
      // if (contact_mode.at(i))
      // {
      //   for (size_t j = 0; j < 3; j++)
      //   {
      //     ref_state_msg.joints.position.at(3 * i + j) = refined_state_msg.joints.position.at(3 * i + j);
      //     ref_state_msg.joints.velocity.at(3 * i + j) = refined_state_msg.joints.velocity.at(3 * i + j);
      //   }
      // }

      if (last_contact_sensing_msg_.data.at(i))
      {
        for (size_t j = 0; j < 3; j++)
        {
          ref_state_msg.joints.position.at(3 * i + j) = refined_state_msg.joints.position.at(3 * i + j);
          ref_state_msg.joints.velocity.at(3 * i + j) = 0;
          tau_array(3 * i + j) = 0;
        }
      }
    }

    for (int i = 0; i < num_feet_; ++i) {
      leg_command_array_msg.leg_commands.at(i).motor_commands.resize(3);
      for (int j = 0; j < 3; ++j) {

        int joint_idx = 3*i+j;

        leg_command_array_msg.leg_commands.at(i).motor_commands.at(j).pos_setpoint = 
          ref_state_msg.joints.position.at(joint_idx);
        leg_command_array_msg.leg_commands.at(i).motor_commands.at(j).vel_setpoint = 
          ref_state_msg.joints.velocity.at(joint_idx);
        leg_command_array_msg.leg_commands.at(i).motor_commands.at(j).torque_ff =
            tau_array(joint_idx);

        if (contact_mode.at(i))
        {
          if (adaptive_contact_mode.at(i))
          {
            if (last_grf_sensor_msg_->contact_states.at(i) &&
                last_grf_sensor_msg_->vectors.at(i).z >= 5)
            {
              leg_command_array_msg.leg_commands.at(i).motor_commands.at(j).kp = stance_kp_.at(j);
              leg_command_array_msg.leg_commands.at(i).motor_commands.at(j).kd = stance_kd_.at(j);
            }
            else
            {
              leg_command_array_msg.leg_commands.at(i).motor_commands.at(j).kp = extend_kp_.at(j);
              leg_command_array_msg.leg_commands.at(i).motor_commands.at(j).kd = extend_kd_.at(j);
            }
          }
          else
          {
            leg_command_array_msg.leg_commands.at(i).motor_commands.at(j).kp = retraction_kp_.at(j);
            leg_command_array_msg.leg_commands.at(i).motor_commands.at(j).kd = retraction_kd_.at(j);
          }
        }
        else
        {
          leg_command_array_msg.leg_commands.at(i).motor_commands.at(j).kp = swing_kp_.at(j);
          leg_command_array_msg.leg_commands.at(i).motor_commands.at(j).kd = swing_kd_.at(j);
        }
      }
    }

    last_grf_array_ = grf_array;
    return true;
  }
}
