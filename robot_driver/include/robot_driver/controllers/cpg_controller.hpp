#pragma once
#include "robot_driver/controllers/leg_controller.hpp"
#include "robot_driver/controllers/cpg_command_guard.hpp"
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <stdexcept>

// Contact-free Conga runtime command sink. Hardware is opt-in. The driver remains the sole joint-command publisher.
class CpgController : public LegController {
 public:
  CpgController(rclcpp::Node::SharedPtr node, const std::string& ns,
                std::shared_ptr<quad_utils::QuadKD2> kd)
      : LegController(node, ns, kd) {
    node_->get_parameter("is_hardware", hardware_);
    bool enabled=false;
    quad_utils::loadROSParamDefault(node_,"cpg.hardware_enabled",enabled,false);
    if (hardware_ && !enabled)
      throw std::runtime_error("Conga hardware requires explicit enable");
    fault_pub_=node_->create_publisher<std_msgs::msg::String>("cpg/fault",1);
    fault_sub_=node_->create_subscription<std_msgs::msg::String>("cpg/fault",1,
        [this](std_msgs::msg::String::SharedPtr msg){ fail(msg->data.c_str()); });
    sub_ = node_->create_subscription<quad_msgs::msg::LegCommandArray>(
        "cpg/target", 1, [this](quad_msgs::msg::LegCommandArray::SharedPtr msg) {
          if (fault_) return;
          if (msg->leg_commands.size()!=4) {fail("invalid leg count");return;}
          const double stamp=rclcpp::Time(msg->header.stamp).seconds();
          const double age=node_->now().seconds()-stamp;
          if (age<-.01 || age>.06 || (received_&&stamp<=last_stamp_)) {
            fail("stale, duplicate or regressing target timestamp");return;
          }
          for (const auto& leg:msg->leg_commands) {
            if (leg.motor_commands.size()!=3) {fail("invalid motor count");return;}
            for (int j=0;j<3;++j) {
              const auto& m=leg.motor_commands[j];
              if (!cpg_guard::targetRange(m.pos_setpoint,j)||m.kp!=25.||m.kd!=.5||
                  m.torque_ff!=0.||m.vel_setpoint!=0.) {fail("invalid target/gains");return;}
            }
          }
          latest_=*msg;received_=true;last_stamp_=stamp;
          last_wall_=std::chrono::steady_clock::now();
        });
  }
  bool computeLegCommandArray(const quad_msgs::msg::RobotState& state,
      quad_msgs::msg::LegCommandArray& output, quad_msgs::msg::GRFArray&) override {
    if (received_ && (node_->now().seconds()-last_stamp_>.06 ||
        (hardware_&&std::chrono::duration<double>(std::chrono::steady_clock::now()-last_wall_).count()>.06)))
      fail("target stream stopped");
    if (fault_) {
      // No abrupt position jump to standing while a foot may be raised. Damping
      // is a harness-test emergency response, NOT a guaranteed upright recovery.
      output.leg_commands.resize(4);
      for(auto& leg:output.leg_commands) {
        leg.motor_commands.resize(3);
        for(auto& m:leg.motor_commands) {m.pos_setpoint=0;m.vel_setpoint=0;m.kp=0;m.kd=.5;m.torque_ff=0;}
      }
      return true;
    }
    if (!received_) return false; // driver standing posture before first target
    output=latest_;
    if(state.joints.position.size()!=12||state.joints.velocity.size()!=12) {
      fail("missing joint state");return computeLegCommandArray(state,output,unused_);
    }
    for(int leg=0;leg<4;++leg) for(int j=0;j<3;++j) {
      auto& m=output.leg_commands[leg].motor_commands[j];int idx=3*leg+j;
      cpg_guard::Motor cmd{m.pos_setpoint,m.vel_setpoint,m.kp,m.kd,m.torque_ff};
      try { cpg_guard::limit(cmd,state.joints.position[idx],state.joints.velocity[idx]); }
      catch(const std::exception& e) { fail(e.what());return computeLegCommandArray(state,output,unused_); }
      m.kp=cmd.kp;m.kd=cmd.kd;m.torque_ff=cmd.ff;
    }
    return true;
  }
  void latchFault(const char* reason) { fail(reason); }
  bool faulted() const { return fault_; }
 private:
  void fail(const char* reason) {
    if(fault_) return;
    fault_=true;
    std_msgs::msg::String msg;msg.data=reason;fault_pub_->publish(msg);
    RCLCPP_ERROR(node_->get_logger(),"CPG fault latched: %s; damping only; restart required",reason);
  }
  bool hardware_=true,received_=false,fault_=false;
  double last_stamp_=0.;
  std::chrono::steady_clock::time_point last_wall_;
  quad_msgs::msg::GRFArray unused_;
  quad_msgs::msg::LegCommandArray latest_;
  rclcpp::Subscription<quad_msgs::msg::LegCommandArray>::SharedPtr sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr fault_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fault_pub_;
};
