#include "robot_driver/hardware_interfaces/unitree_interface.hpp"

#include <cstdlib>
#include "robot_driver/controllers/cpg_command_guard.hpp"
#include <iostream>
#include <optional>
#include <unistd.h>

constexpr int UnitreeInterface::kLegMap[UnitreeInterface::kNumLegs]
                                       [UnitreeInterface::kJointsPerLeg];
constexpr int UnitreeInterface::kWheelMap[UnitreeInterface::kNumWheels];

UnitreeInterface::UnitreeInterface(const std::string& robot_name)
    : HardwareInterface(),
      robot_name_(robot_name),
      num_motors_(kNumJoints),
      has_wheels_(false) {
  if (robot_name_ == "go2w") {
    num_motors_ = kNumJoints + kNumWheels;
    has_wheels_ = true;
  } else if (robot_name_ != "go2") {
    std::cerr << "UnitreeInterface: unknown robot_name='" << robot_name_
              << "', falling back to Go2." << std::endl;
    robot_name_ = "go2";
  }
}

void UnitreeInterface::loadInterface(int /*argc*/, char** /*argv*/) {
  // ROBOT_MCU_IFACE is set by init_robot.sh based on which interface holds an
  // IP on the MCU subnet (192.168.123.0/24). Falls back to eth0 if unset.
  const char* env_iface = std::getenv("ROBOT_MCU_IFACE");
  std::string net_iface = (env_iface && *env_iface) ? env_iface : "eth0";
  std::cout << "UnitreeInterface: using network interface '" << net_iface
            << "' for MCU DDS domain." << std::endl;

  unitree::robot::ChannelFactory::Instance()->Init(0, net_iface);

  const auto subscribe_to_low_state = [this] {
    state_sub_.reset(new unitree::robot::ChannelSubscriber<
                     unitree_go::msg::dds_::LowState_>("rt/lowstate"));
    state_sub_->InitChannel(std::bind(&UnitreeInterface::lowStateHandler, this,
                                      std::placeholders::_1),
                            1);
  };
  if (cpg_guard_) {
    // Wait for DDS discovery and advancing telemetry while sport mode still owns
    // the robot. The normal command watchdog starts after handover.
    subscribe_to_low_state();
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    std::optional<uint32_t> first_tick;
    bool live_state = false;
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (cpg_fault_) break;
        if (state_received_) {
          if (!first_tick) first_tick = low_state_.tick();
          const auto age = std::chrono::steady_clock::now() - state_received_at_;
          live_state = low_state_.tick() != *first_tick && age <= std::chrono::milliseconds(50);
        }
      }
      if (live_state) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    if (!live_state) {
      throw std::runtime_error("Conga requires advancing Unitree low state before releasing sport mode");
    }
  }

  // Disable sport mode so low-level commands are not overridden.
  // Mirrors the retry loop in Unitree's go2_stand_example.
  unitree::robot::b2::MotionSwitcherClient msc;
  msc.SetTimeout(10.0f);
  msc.Init();

  auto queryMotionStatus = [&msc]() -> bool {
    std::string robotForm, motionName;
    int32_t ret = msc.CheckMode(robotForm, motionName);
    if (ret != 0) {
      std::cout << "CheckMode failed. Error code: " << ret << std::endl;
      return true;
    }
    if (motionName.empty()) {
      std::cout << "Motion control service deactivated." << std::endl;
      return false;
    }
    std::cout << "Active motion service: " << motionName << std::endl;
    return true;
  };

  while (queryMotionStatus()) {
    std::cout << "Releasing motion control service..." << std::endl;
    int32_t ret = msc.ReleaseMode();
    if (ret == 0) {
      std::cout << "ReleaseMode succeeded." << std::endl;
    } else {
      std::cout << "ReleaseMode failed. Error code: " << ret << std::endl;
    }
    sleep(5);
  }

  cmd_pub_.reset(new unitree::robot::ChannelPublisher<
                 unitree_go::msg::dds_::LowCmd_>("rt/lowcmd"));
  cmd_pub_->InitChannel();

  if (!cpg_guard_) subscribe_to_low_state();

  initLowCmd();
  if(cpg_guard_) {
    command_received_at_=std::chrono::steady_clock::now();
    watchdog_=std::thread(&UnitreeInterface::watchdogLoop,this);
  }
}

void UnitreeInterface::unloadInterface() {
  watchdog_stop_=true;
  if(watchdog_.joinable()) watchdog_.join();
  std::lock_guard<std::mutex> command_lock(command_mutex_);
  initLowCmd();
  low_cmd_.crc() = crc32Core(reinterpret_cast<uint32_t*>(&low_cmd_),
                             (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
  cmd_pub_->Write(low_cmd_);
}

void UnitreeInterface::initLowCmd() {
  low_cmd_.head()[0] = 0xFE;
  low_cmd_.head()[1] = 0xEF;
  low_cmd_.level_flag() = 0xFF;
  low_cmd_.gpio() = 0;

  for (int i = 0; i < 20; i++) {
    low_cmd_.motor_cmd()[i].mode() = 0x01;
    low_cmd_.motor_cmd()[i].q() = 2.146E+9f;
    low_cmd_.motor_cmd()[i].dq() = 16000.0f;
    low_cmd_.motor_cmd()[i].kp() = 0;
    low_cmd_.motor_cmd()[i].kd() = 0;
    low_cmd_.motor_cmd()[i].tau() = 0;
  }
}

void UnitreeInterface::lowStateHandler(const void* message) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  const auto& incoming=*(const unitree_go::msg::dds_::LowState_*)message;
  // Repeated DDS delivery does not refresh a frozen MCU sample. A wrapping
  // counter is allowed; source time is otherwise represented by host receipt.
  if(cpg_guard_ && state_received_) {
    const int32_t delta=static_cast<int32_t>(incoming.tick()-low_state_.tick());
    if(delta<0) cpg_fault_=true;
    if(delta<=0) return;
  }
  low_state_ = incoming;
  state_received_at_=std::chrono::steady_clock::now();
  state_stamp_=rclcpp::Clock(RCL_SYSTEM_TIME).now();
  // Reorder Unitree's foot_force [FR, FL, RR, RL] into quad-sdk leg order
  // [FL, RL, FR, RR] so downstream code uses a single convention.
  const auto& ff = low_state_.foot_force();
  for (int leg = 0; leg < kNumLegs; ++leg) {
    foot_force_quad_order_[leg] = ff[kFootForceMap[leg]];
  }
  state_received_ = true;
}

std::array<int16_t, UnitreeInterface::kNumLegs>
UnitreeInterface::getFootForcesRaw() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return consumed_forces_;
}

uint32_t UnitreeInterface::crc32Core(uint32_t* ptr, uint32_t len) {
  uint32_t xbit = 0;
  uint32_t data = 0;
  uint32_t CRC32 = 0xFFFFFFFF;
  const uint32_t dwPolynomial = 0x04c11db7;

  for (uint32_t i = 0; i < len; i++) {
    xbit = 1 << 31;
    data = ptr[i];
    for (uint32_t bits = 0; bits < 32; bits++) {
      if (CRC32 & 0x80000000) {
        CRC32 <<= 1;
        CRC32 ^= dwPolynomial;
      } else {
        CRC32 <<= 1;
      }
      if (data & xbit) {
        CRC32 ^= dwPolynomial;
      }
      xbit >>= 1;
    }
  }
  return CRC32;
}

bool UnitreeInterface::send(
    const quad_msgs::msg::LegCommandArray& leg_command_array_msg,
    const Eigen::VectorXd& user_tx_data) {
  std::array<double,12> measured_q{},measured_dq{};
  if(cpg_guard_) {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    if(!state_received_ || std::chrono::duration<double>(std::chrono::steady_clock::now()-state_received_at_).count()>.05)
      cpg_fault_=true;
    for(int i=0;i<12;++i) {
      measured_q[i]=low_state_.motor_state()[i].q();measured_dq[i]=low_state_.motor_state()[i].dq();
      if(!cpg_guard::measuredRange(measured_q[i],measured_dq[i],i%3)) cpg_fault_=true;
    }
    if(leg_command_array_msg.leg_commands.size()!=4) cpg_fault_=true;
    else for(const auto& leg:leg_command_array_msg.leg_commands) if(leg.motor_commands.size()!=3) cpg_fault_=true;
  }
  std::lock_guard<std::mutex> command_lock(command_mutex_);
  command_received_at_=std::chrono::steady_clock::now();
  if(cpg_guard_ && cpg_fault_) {writeDampingLocked();return false;}
  for (int leg = 0; leg < kNumLegs; ++leg) {
    const auto& leg_cmd = leg_command_array_msg.leg_commands.at(leg);

    // 12 leg motors.
    for (int j = 0; j < kJointsPerLeg; ++j) {
      int motor_idx = kLegMap[leg][j];
      auto mc = leg_cmd.motor_commands.at(j);
      if(cpg_guard_) {
        cpg_guard::Motor m{mc.pos_setpoint,mc.vel_setpoint,mc.kp,mc.kd,mc.torque_ff};
        try {
          if(mc.kp>60.||mc.kd>5.) throw std::invalid_argument("CPG excessive gains");
          cpg_guard::limit(m,measured_q[motor_idx],measured_dq[motor_idx]);
        } catch(const std::exception& e) {
          std::cerr<<"CPG interface fault: "<<e.what()<<std::endl;
          cpg_fault_=true;writeDampingLocked();return false;
        }
        mc.kp=m.kp;mc.kd=m.kd;mc.torque_ff=m.ff;
      }

      low_cmd_.motor_cmd()[motor_idx].mode() = 0x01;
      low_cmd_.motor_cmd()[motor_idx].q() = static_cast<float>(mc.pos_setpoint);
      low_cmd_.motor_cmd()[motor_idx].dq() =
          static_cast<float>(mc.vel_setpoint);
      low_cmd_.motor_cmd()[motor_idx].tau() = static_cast<float>(mc.torque_ff);
      low_cmd_.motor_cmd()[motor_idx].kp() = static_cast<float>(mc.kp);
      low_cmd_.motor_cmd()[motor_idx].kd() = static_cast<float>(mc.kd);
    }

    // 4 wheel motors (Go2-W only).
    if (has_wheels_) {
      int wheel_idx = kWheelMap[leg];

      const bool have_per_leg_wheel_cmd =
          (static_cast<int>(leg_cmd.motor_commands.size()) > kJointsPerLeg);
      const bool have_legacy_tx =
          (user_tx_data.size() == kWheelTxSize);

      float q = 0.0f;
      float vel = 0.0f;
      float kp = 0.0f;
      float kd = kDefaultWheelKd;
      float tau_ff = 0.0f;

      if (have_per_leg_wheel_cmd) {
        const auto& mc = leg_cmd.motor_commands.at(kJointsPerLeg);
        q = static_cast<float>(mc.pos_setpoint);
        vel = static_cast<float>(mc.vel_setpoint);
        kp = static_cast<float>(mc.kp);
        kd = static_cast<float>(mc.kd);
        tau_ff = static_cast<float>(mc.torque_ff);
      } else if (have_legacy_tx) {
        const int base = kTxRestartFlagOffset + leg * kWheelCmdFields;
        vel = static_cast<float>(user_tx_data[base + 0]);
        kd = static_cast<float>(user_tx_data[base + 1]);
        tau_ff = static_cast<float>(user_tx_data[base + 2]);
      }

      low_cmd_.motor_cmd()[wheel_idx].mode() = 0x01;
      low_cmd_.motor_cmd()[wheel_idx].q() = q;
      low_cmd_.motor_cmd()[wheel_idx].dq() = vel;
      low_cmd_.motor_cmd()[wheel_idx].kp() = kp;
      low_cmd_.motor_cmd()[wheel_idx].kd() = kd;
      low_cmd_.motor_cmd()[wheel_idx].tau() = tau_ff;
    }
  }

  // CRC must be computed before every publish.
  low_cmd_.crc() = crc32Core(reinterpret_cast<uint32_t*>(&low_cmd_),
                             (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
  cmd_pub_->Write(low_cmd_);
  return true;
}

bool UnitreeInterface::recv(sensor_msgs::msg::JointState& joint_state_msg,
                            sensor_msgs::msg::Imu& imu_msg,
                            Eigen::VectorXd& user_rx_data) {
  std::lock_guard<std::mutex> lock(state_mutex_);

  if (!state_received_) {
    std::cout << "Not receiving state from UnitreeInterface" << std::endl;
    return false;
  }

  if(cpg_guard_) {
    if(cpg_fault_ || std::chrono::duration<double>(std::chrono::steady_clock::now()-state_received_at_).count()>.05) {
      cpg_fault_=true;return false;
    }
  }
  if(cpg_guard_) for(int i=0;i<12;++i) {
    if(!cpg_guard::measuredRange(low_state_.motor_state()[i].q(),low_state_.motor_state()[i].dq(),i%3)) {
      cpg_fault_=true;return false;
    }
  }
  joint_state_msg.header.stamp=state_stamp_;
  imu_msg.header.stamp=state_stamp_;
  consumed_forces_=foot_force_quad_order_;
  consumed_tick_=low_state_.tick();
  // 12 leg joints, in quad-sdk leg-then-joint order.
  int idx = 0;
  for (int leg = 0; leg < kNumLegs; ++leg) {
    for (int j = 0; j < kJointsPerLeg; ++j) {
      int motor_idx = kLegMap[leg][j];
      joint_state_msg.name[idx] = joint_names_[idx];
      joint_state_msg.position[idx] = low_state_.motor_state()[motor_idx].q();
      joint_state_msg.velocity[idx] = low_state_.motor_state()[motor_idx].dq();
      joint_state_msg.effort[idx] =
          low_state_.motor_state()[motor_idx].tau_est();
      idx++;
    }
  }

  // 4 wheel joints, but only if caller has sized JointState for it.
  if (has_wheels_ &&
      static_cast<int>(joint_state_msg.name.size()) >=
          kNumJoints + kNumWheels) {
    for (int leg = 0; leg < kNumWheels; ++leg) {
      int motor_idx = kWheelMap[leg];
      joint_state_msg.name[idx] = wheel_joint_names_[leg];
      joint_state_msg.position[idx] = low_state_.motor_state()[motor_idx].q();
      joint_state_msg.velocity[idx] = low_state_.motor_state()[motor_idx].dq();
      joint_state_msg.effort[idx] =
          low_state_.motor_state()[motor_idx].tau_est();
      idx++;
    }
  }

  // Mirror wheel state into user_rx_data for downstream consumers that
  // bypass JointState (preserves Go2WInterface behavior).
  if (has_wheels_) {
    if (user_rx_data.size() != kWheelRxSize) {
      user_rx_data.resize(kWheelRxSize);
    }
    for (int leg = 0; leg < kNumWheels; ++leg) {
      int motor_idx = kWheelMap[leg];
      user_rx_data[leg * kWheelStateFields + 0] =
          low_state_.motor_state()[motor_idx].q();
      user_rx_data[leg * kWheelStateFields + 1] =
          low_state_.motor_state()[motor_idx].dq();
      user_rx_data[leg * kWheelStateFields + 2] =
          low_state_.motor_state()[motor_idx].tau_est();
    }
  }

  // Unitree SDK packs the quaternion as (w, x, y, z); ROS sensor_msgs::Imu
  // uses (x, y, z, w). Verified empirically: with the robot level on the
  // floor, the SDK[0] slot carries the scalar component (~1.0 modulo yaw).
  imu_msg.orientation.w = low_state_.imu_state().quaternion()[0];
  imu_msg.orientation.x = low_state_.imu_state().quaternion()[1];
  imu_msg.orientation.y = low_state_.imu_state().quaternion()[2];
  imu_msg.orientation.z = low_state_.imu_state().quaternion()[3];

  imu_msg.angular_velocity.x = low_state_.imu_state().gyroscope()[0];
  imu_msg.angular_velocity.y = low_state_.imu_state().gyroscope()[1];
  imu_msg.angular_velocity.z = low_state_.imu_state().gyroscope()[2];

  imu_msg.linear_acceleration.x = low_state_.imu_state().accelerometer()[0];
  imu_msg.linear_acceleration.y = low_state_.imu_state().accelerometer()[1];
  imu_msg.linear_acceleration.z = low_state_.imu_state().accelerometer()[2];

  return true;
}


void UnitreeInterface::writeDampingLocked() {
  initLowCmd();
  for(int i=0;i<12;++i) {
    auto& m=low_cmd_.motor_cmd()[i];m.q()=0.;m.dq()=0.;m.kp()=0.;m.kd()=.5;m.tau()=0.;
  }
  low_cmd_.crc()=crc32Core(reinterpret_cast<uint32_t*>(&low_cmd_),
      (sizeof(unitree_go::msg::dds_::LowCmd_)>>2)-1);
  cmd_pub_->Write(low_cmd_);
}

void UnitreeInterface::watchdogLoop() {
  // Best-effort watchdog in a separate thread. Process/OS/network failure still
  // requires the robot's independent protection and physical operator stop.
  while(!watchdog_stop_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    bool stale=false;
    {
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      stale=state_received_ && std::chrono::duration<double>(std::chrono::steady_clock::now()-state_received_at_).count()>.05;
    }
    std::lock_guard<std::mutex> command_lock(command_mutex_);
    if(stale || std::chrono::duration<double>(std::chrono::steady_clock::now()-command_received_at_).count()>.06)
      cpg_fault_=true;
    if(cpg_fault_) writeDampingLocked();
  }
}
