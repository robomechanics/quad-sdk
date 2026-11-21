#include <gtest/gtest.h>
#include <robot_driver/controllers/cpg_controller.hpp>
#include <thread>

class CpgSink : public ::testing::Test {
 protected:
  void SetUp() override {
    if(!rclcpp::ok()) {int argc=0;char** argv=nullptr;rclcpp::init(argc,argv);}
    rclcpp::NodeOptions opts;opts.parameter_overrides({rclcpp::Parameter("is_hardware",false)});
    opts.automatically_declare_parameters_from_overrides(true);
    node=rclcpp::Node::make_shared("cpg_sink_contract",opts);
    sink=std::make_unique<CpgController>(node,"",nullptr);
    pub=node->create_publisher<quad_msgs::msg::LegCommandArray>("cpg/target",1);
    for(int i=0;i<100 && pub->get_subscription_count()==0;++i) {
      rclcpp::spin_some(node);std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    ASSERT_GT(pub->get_subscription_count(),0u);
    command.leg_commands.resize(4);
    for(auto& l:command.leg_commands) {
      l.motor_commands.resize(3);
      for(int j=0;j<3;++j) {
        auto& m=l.motor_commands[j];m.pos_setpoint=(j==0?0.:(j==1?.8:-1.5));m.kp=25.;m.kd=.5;
        state.joints.position.push_back(m.pos_setpoint);state.joints.velocity.push_back(0.);
      }
    }
    command.header.stamp=node->now();
  }
  void send() {
    pub->publish(command);
    for(int i=0;i<5;++i){rclcpp::spin_some(node);std::this_thread::sleep_for(std::chrono::milliseconds(1));}
  }
  void expectDamping() {
    ASSERT_TRUE(sink->computeLegCommandArray(state,output,grf));ASSERT_TRUE(sink->faulted());
    ASSERT_EQ(output.leg_commands.size(),4u);
    for(const auto& l:output.leg_commands) for(const auto& m:l.motor_commands) {
      EXPECT_EQ(m.kp,0.);EXPECT_EQ(m.kd,.5);EXPECT_EQ(m.torque_ff,0.);
    }
    // A later valid target must not clear a fault.
    command.header.stamp=node->now();send();EXPECT_TRUE(sink->faulted());
  }
  rclcpp::Node::SharedPtr node;std::unique_ptr<CpgController> sink;
  rclcpp::Publisher<quad_msgs::msg::LegCommandArray>::SharedPtr pub;
  quad_msgs::msg::LegCommandArray command,output;quad_msgs::msg::RobotState state;quad_msgs::msg::GRFArray grf;
};
TEST_F(CpgSink,StartupWaitAndValidBoundedTarget) {
  EXPECT_FALSE(sink->computeLegCommandArray(state,output,grf));
  send();ASSERT_TRUE(sink->computeLegCommandArray(state,output,grf));ASSERT_FALSE(sink->faulted());
  EXPECT_EQ(output.leg_commands[0].motor_commands[0].kp,25.);
  state.joints.position[0]=-.9;
  ASSERT_TRUE(sink->computeLegCommandArray(state,output,grf));
  const auto& m=output.leg_commands[0].motor_commands[0];EXPECT_LE(m.kp*.9,18.+1e-9);
}
TEST_F(CpgSink,DuplicateTimestampLatches){send();send();expectDamping();}
TEST_F(CpgSink,WrongGainsLatch){command.leg_commands[0].motor_commands[0].kp=26.;send();expectDamping();}
TEST_F(CpgSink,WrongLegCountLatches){command.leg_commands.resize(3);send();expectDamping();}
TEST_F(CpgSink,ImpossibleKneeTargetLatches){command.leg_commands[0].motor_commands[2].pos_setpoint=1.5;send();expectDamping();}
TEST_F(CpgSink,FutureTargetLatches){command.header.stamp=node->now()+rclcpp::Duration::from_seconds(1.);send();expectDamping();}
TEST_F(CpgSink,LostTargetsDampWithoutStandingJump){send();std::this_thread::sleep_for(std::chrono::milliseconds(70));expectDamping();}

// Constructor only: this class has ROS topics but no Unitree interface or DDS motor publisher.
TEST(CpgGate, HardwareRequiresExplicitEnableOnly) {
  if(!rclcpp::ok()){int argc=0;char** argv=nullptr;rclcpp::init(argc,argv);}
  for(bool enabled : {false, true}) {
    rclcpp::NodeOptions opts; opts.automatically_declare_parameters_from_overrides(true);
    opts.parameter_overrides({rclcpp::Parameter("is_hardware",true),
      rclcpp::Parameter("cpg.hardware_enabled",enabled)});
    auto node=rclcpp::Node::make_shared("conga_gate_constructor",opts);
    if(enabled) { EXPECT_NO_THROW(CpgController(node,"",nullptr)); }
    else { EXPECT_THROW(CpgController(node,"",nullptr),std::runtime_error); }
  }
}
