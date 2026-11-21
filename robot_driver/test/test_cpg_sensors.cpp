#include <gtest/gtest.h>
#include "robot_driver/hardware_interfaces/unitree_interface.hpp"
#include <thread>
class SensingInterface : public UnitreeInterface {
 public: using UnitreeInterface::lowStateHandler;
};
static unitree_go::msg::dds_::LowState_ sample(uint32_t tick){
 unitree_go::msg::dds_::LowState_ s;s.tick()=tick;
 for(int i=0;i<12;++i){s.motor_state()[i].q()=(i%3==2?-1.6f:i%3==1?.8f:0.f)+i*.01f;s.motor_state()[i].dq()=i*.2f;}
 for(int i=0;i<4;++i)s.foot_force()[i]=10+i;
 s.imu_state().quaternion()[0]=1.;s.imu_state().gyroscope()[1]=.125;
 return s;
}
static bool read(SensingInterface& iface,sensor_msgs::msg::JointState& j,sensor_msgs::msg::Imu& im){
 j.name.resize(12);j.position.resize(12);j.velocity.resize(12);j.effort.resize(12);Eigen::VectorXd v;
 return iface.recv(j,im,v);
}
TEST(CpgSensors, OneSampleMappingAndStamp){
 SensingInterface i;i.enableCpgGuard();auto s=sample(100);i.lowStateHandler(&s);
 sensor_msgs::msg::JointState j;sensor_msgs::msg::Imu im;ASSERT_TRUE(read(i,j,im));
 EXPECT_EQ(j.header.stamp,im.header.stamp);EXPECT_NE(j.header.stamp.sec,0);EXPECT_EQ(i.consumedTick(),100u);
 EXPECT_NEAR(j.position[0],.03,1e-6);EXPECT_NEAR(j.position[3],.09,1e-6);EXPECT_NEAR(j.position[6],0.,1e-6);
 EXPECT_EQ(i.getFootForcesRaw(),(std::array<int16_t,4>{11,13,10,12}));EXPECT_DOUBLE_EQ(im.orientation.w,1.);
 auto original=j.header.stamp;i.lowStateHandler(&s);ASSERT_TRUE(read(i,j,im));EXPECT_EQ(original,j.header.stamp);
 // A new callback arriving before ROS publication must not change consumed forces.
 auto next=sample(102);next.foot_force()[1]=90;i.lowStateHandler(&next);EXPECT_EQ(i.getFootForcesRaw()[0],11);
 ASSERT_TRUE(read(i,j,im));EXPECT_EQ(i.getFootForcesRaw()[0],90);
}
TEST(CpgSensors, RepeatedMcuPacketCannotRefreshAndFaultLatches){
 SensingInterface i;i.enableCpgGuard();auto s=sample(100);i.lowStateHandler(&s);
 sensor_msgs::msg::JointState j;sensor_msgs::msg::Imu im;ASSERT_TRUE(read(i,j,im));
 for(int n=0;n<6;++n){std::this_thread::sleep_for(std::chrono::milliseconds(10));i.lowStateHandler(&s);}
 EXPECT_FALSE(read(i,j,im));EXPECT_TRUE(i.cpgFaulted());s.tick()=200;i.lowStateHandler(&s);EXPECT_FALSE(read(i,j,im));
}
TEST(CpgSensors, McuWrapAllowedRegressionRejected){
 SensingInterface i;i.enableCpgGuard();auto s=sample(0xfffffffe);i.lowStateHandler(&s);
 sensor_msgs::msg::JointState j;sensor_msgs::msg::Imu im;ASSERT_TRUE(read(i,j,im));
 s.tick()=1;i.lowStateHandler(&s);EXPECT_TRUE(read(i,j,im));
 s.tick()=0;i.lowStateHandler(&s);EXPECT_FALSE(read(i,j,im));
}

TEST(CpgSensors, ImpossibleEncoderRejectedBeforeCommand){
 SensingInterface i;i.enableCpgGuard();auto s=sample(100);s.motor_state()[0].q()=100;
 i.lowStateHandler(&s);sensor_msgs::msg::JointState j;sensor_msgs::msg::Imu im;
 EXPECT_FALSE(read(i,j,im));EXPECT_TRUE(i.cpgFaulted());
}
