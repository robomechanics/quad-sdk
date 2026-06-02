#include <array>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class MujocoRecorder : public rclcpp::Node {
 public:
  MujocoRecorder() : rclcpp::Node("mujoco_recorder") {
    declare_parameter<std::string>("mjcf_path", "");
    declare_parameter<std::string>("namespace", "robot_1");
    declare_parameter<std::string>("output_path", "/tmp/mujoco_recording.mp4");
    declare_parameter<int>("width", 1280);
    declare_parameter<int>("height", 720);
    declare_parameter<int>("fps", 30);
    declare_parameter<double>("camera_distance", 3.0);
    declare_parameter<double>("camera_elevation", -20.0);
    declare_parameter<double>("camera_azimuth", 90.0);
    declare_parameter<bool>("camera_track_robot", true);
    declare_parameter<std::string>("odom_free_joint_name", "floating_base");
    declare_parameter<std::vector<std::string>>("joint_map_ros",
                                                std::vector<std::string>{});
    declare_parameter<std::vector<std::string>>("joint_map_mjc",
                                                std::vector<std::string>{});

    const auto mjcf_path = get_parameter("mjcf_path").as_string();
    namespace_ = get_parameter("namespace").as_string();
    output_path_ = get_parameter("output_path").as_string();
    width_ = static_cast<int>(get_parameter("width").as_int());
    height_ = static_cast<int>(get_parameter("height").as_int());
    fps_ = static_cast<int>(get_parameter("fps").as_int());
    track_ = get_parameter("camera_track_robot").as_bool();
    const auto free_name = get_parameter("odom_free_joint_name").as_string();
    const auto ros_names = get_parameter("joint_map_ros").as_string_array();
    const auto mjc_names = get_parameter("joint_map_mjc").as_string_array();

    if (mjcf_path.empty()) {
      throw std::runtime_error("`mjcf_path` parameter is empty");
    }
    if (ros_names.size() != mjc_names.size()) {
      throw std::runtime_error(
          "`joint_map_ros` and `joint_map_mjc` length mismatch");
    }
    // libx264 with yuv420p chroma subsampling refuses odd dimensions.
    if (width_ & 1) --width_;
    if (height_ & 1) --height_;

    char err[1024] = {0};
    model_ = mj_loadXML(mjcf_path.c_str(), nullptr, err, sizeof(err));
    if (!model_) {
      throw std::runtime_error("mj_loadXML failed: " + std::string(err));
    }
    data_ = mj_makeData(model_);

    model_->vis.global.offwidth = width_;
    model_->vis.global.offheight = height_;

    // Resolve qpos addresses once. The profile maps each ros2_control joint
    // name to the MJCF joint name; mj_name2id then gives qposadr so updates
    // land in the right slot every frame.
    for (size_t i = 0; i < ros_names.size(); ++i) {
      const int jid = mj_name2id(model_, mjOBJ_JOINT, mjc_names[i].c_str());
      if (jid < 0) {
        RCLCPP_WARN(get_logger(),
                    "MJCF joint '%s' not found; ros2_control name '%s' won't "
                    "be mirrored.",
                    mjc_names[i].c_str(), ros_names[i].c_str());
        continue;
      }
      joint_qposadr_[ros_names[i]] = model_->jnt_qposadr[jid];
    }
    const int free_id = mj_name2id(model_, mjOBJ_JOINT, free_name.c_str());
    free_qposadr_ = (free_id >= 0) ? model_->jnt_qposadr[free_id] : -1;
    if (free_qposadr_ < 0) {
      RCLCPP_WARN(get_logger(),
                  "Free joint '%s' not found; base pose won't be mirrored "
                  "(robot will appear pinned at origin).",
                  free_name.c_str());
    }

    // Hidden GLFW window — gives us an OpenGL context that mjr_makeContext
    // can attach to. Window stays unmapped so it never appears on screen.
    if (!glfwInit()) {
      throw std::runtime_error("glfwInit failed (no DISPLAY?)");
    }
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    window_ =
        glfwCreateWindow(width_, height_, "mujoco_recorder", nullptr, nullptr);
    if (!window_) {
      glfwTerminate();
      throw std::runtime_error("glfwCreateWindow failed");
    }
    glfwMakeContextCurrent(window_);

    mjv_defaultScene(&scene_);
    mjv_makeScene(model_, &scene_, 2000);
    mjv_defaultCamera(&cam_);
    cam_.distance = get_parameter("camera_distance").as_double();
    cam_.elevation = get_parameter("camera_elevation").as_double();
    cam_.azimuth = get_parameter("camera_azimuth").as_double();
    cam_.lookat[0] = 0.0;
    cam_.lookat[1] = 0.0;
    cam_.lookat[2] = 0.3;
    mjv_defaultOption(&opt_);
    mjv_defaultPerturb(&pert_);

    mjr_defaultContext(&con_);
    mjr_makeContext(model_, &con_, mjFONTSCALE_100);
    mjr_setBuffer(mjFB_OFFSCREEN, &con_);
    if (con_.currentBuffer != mjFB_OFFSCREEN) {
      RCLCPP_WARN(get_logger(),
                  "Offscreen framebuffer inactive — readback may be empty.");
    }

    rgb_buf_.resize(static_cast<size_t>(width_) * height_ * 3);
    flip_buf_.resize(rgb_buf_.size());

    // popen() is the simplest portable stdin pipe. ffmpeg writes the moov
    // atom (mp4 trailer) on EOF, so closing the pipe in ~MujocoRecorder()
    // produces a finalized file.
    char cmd[2048];
    std::snprintf(cmd, sizeof(cmd),
                  "ffmpeg -hide_banner -loglevel warning -y "
                  "-f rawvideo -pix_fmt rgb24 -s %dx%d -r %d -i - "
                  "-c:v libx264 -preset ultrafast -pix_fmt yuv420p '%s'",
                  width_, height_, fps_, output_path_.c_str());
    ffmpeg_ = popen(cmd, "w");
    if (!ffmpeg_) {
      throw std::runtime_error("popen(ffmpeg) failed — is ffmpeg installed?");
    }

    // joint_state_broadcaster + MujocoSystemInterface both publish RELIABLE.
    auto qos = rclcpp::QoS(10).reliable();
    js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/" + namespace_ + "/joint_states", qos,
        [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lk(mu_);
          for (size_t i = 0; i < msg->name.size() && i < msg->position.size();
               ++i) {
            latest_jpos_[msg->name[i]] = msg->position[i];
          }
        });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/" + namespace_ + "/odom", qos,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lk(mu_);
          // ROS quat is (x,y,z,w); MJCF qpos for a free joint is (w,x,y,z).
          latest_base_ = {
              msg->pose.pose.position.x,    msg->pose.pose.position.y,
              msg->pose.pose.position.z,    msg->pose.pose.orientation.w,
              msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
              msg->pose.pose.orientation.z,
          };
          have_base_ = true;
        });

    timer_ =
        create_wall_timer(std::chrono::milliseconds(std::max(1, 1000 / fps_)),
                          std::bind(&MujocoRecorder::tick, this));

    RCLCPP_INFO(get_logger(),
                "Recording to %s (%dx%d @ %d fps), namespace=%s, mjcf=%s",
                output_path_.c_str(), width_, height_, fps_, namespace_.c_str(),
                mjcf_path.c_str());
  }

  ~MujocoRecorder() override {
    // Closing the ffmpeg pipe sends EOF; ffmpeg writes the mp4 trailer and
    // exits. pclose() waits for it, so the file is finalized before we go.
    if (ffmpeg_) {
      pclose(ffmpeg_);
      ffmpeg_ = nullptr;
    }
    if (window_) {
      mjr_freeContext(&con_);
      mjv_freeScene(&scene_);
    }
    if (data_) mj_deleteData(data_);
    if (model_) mj_deleteModel(model_);
    if (window_) {
      glfwDestroyWindow(window_);
      glfwTerminate();
    }
    RCLCPP_INFO(get_logger(), "Recording done: %lu frames -> %s",
                static_cast<unsigned long>(frames_written_),
                output_path_.c_str());
  }

 private:
  void tick() {
    std::array<double, 7> base{};
    bool have_base = false;
    std::unordered_map<std::string, double> jpos;
    {
      std::lock_guard<std::mutex> lk(mu_);
      if (have_base_) {
        base = latest_base_;
        have_base = true;
      }
      jpos = latest_jpos_;
    }

    if (free_qposadr_ >= 0 && have_base) {
      double* q = data_->qpos + free_qposadr_;
      q[0] = base[0];
      q[1] = base[1];
      q[2] = base[2];
      q[3] = base[3];
      q[4] = base[4];
      q[5] = base[5];
      q[6] = base[6];
      if (track_) {
        cam_.lookat[0] = base[0];
        cam_.lookat[1] = base[1];
        cam_.lookat[2] = base[2];
      }
    }
    for (const auto& kv : jpos) {
      auto it = joint_qposadr_.find(kv.first);
      if (it != joint_qposadr_.end()) {
        data_->qpos[it->second] = kv.second;
      }
    }

    mj_forward(model_, data_);
    mjv_updateScene(model_, data_, &opt_, &pert_, &cam_, mjCAT_ALL, &scene_);

    mjrRect viewport{0, 0, width_, height_};
    mjr_render(viewport, &scene_, &con_);
    mjr_readPixels(rgb_buf_.data(), nullptr, viewport, &con_);

    // mjr_readPixels returns rows bottom-up; flip vertically so the mp4
    // isn't upside-down.
    const size_t row = static_cast<size_t>(width_) * 3;
    for (int y = 0; y < height_; ++y) {
      std::memcpy(&flip_buf_[y * row], &rgb_buf_[(height_ - 1 - y) * row], row);
    }

    if (ffmpeg_) {
      const size_t n =
          std::fwrite(flip_buf_.data(), 1, flip_buf_.size(), ffmpeg_);
      if (n != flip_buf_.size()) {
        RCLCPP_ERROR(get_logger(),
                     "ffmpeg pipe short write %zu/%zu — encoder may have died.",
                     n, flip_buf_.size());
      } else {
        ++frames_written_;
      }
    }
  }

  // Parameters / state
  std::string namespace_;
  std::string output_path_;
  int width_ = 1280, height_ = 720, fps_ = 30;
  bool track_ = true;

  // MuJoCo
  mjModel* model_ = nullptr;
  mjData* data_ = nullptr;
  int free_qposadr_ = -1;
  std::unordered_map<std::string, int> joint_qposadr_;
  mjvScene scene_{};
  mjvCamera cam_{};
  mjvOption opt_{};
  mjvPerturb pert_{};
  mjrContext con_{};

  // GLFW
  GLFWwindow* window_ = nullptr;

  // Pixel buffers
  std::vector<unsigned char> rgb_buf_, flip_buf_;

  // ffmpeg
  FILE* ffmpeg_ = nullptr;
  size_t frames_written_ = 0;

  // ROS
  std::mutex mu_;
  std::array<double, 7> latest_base_{};
  bool have_base_ = false;
  std::unordered_map<std::string, double> latest_jpos_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<MujocoRecorder>());
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("mujoco_recorder"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
