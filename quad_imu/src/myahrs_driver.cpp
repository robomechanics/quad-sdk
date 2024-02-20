//------------------------------------------------------------------------------
// Copyright (c) 2015, Yoonseok Pyo
// All rights reserved.

// License: BSD

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.

// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.

// * Neither the name of myahrs_driver nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//------------------------------------------------------------------------------
#include <myahrs_driver/myahrs_plus.hpp>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

//------------------------------------------------------------------------------
using namespace WithRobot;

//------------------------------------------------------------------------------
class MyAhrsDriverForROS : public iMyAhrsPlus
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  ros::Publisher imu_data_raw_pub_;
  ros::Publisher imu_data_pub_;
  ros::Publisher imu_mag_pub_;
  ros::Publisher imu_temperature_pub_;

  tf::TransformBroadcaster broadcaster_;

  Platform::Mutex lock_;
  SensorData sensor_data_;
  bool publish_tf;
  std::string parent_frame_id_;
  std::string frame_id_;
  double linear_acceleration_stddev_;
  double angular_velocity_stddev_;
  double magnetic_field_stddev_;
  double orientation_stddev_;

  void OnSensorData(int sensor_id, SensorData data)
  {
    LockGuard _l(lock_);
    sensor_data_ = data;
    publish_topic(sensor_id);
  }

  void OnAttributeChange(int sensor_id, std::string attribute_name, std::string value)
  {
    printf("OnAttributeChange(id %d, %s, %s)\n", sensor_id, attribute_name.c_str(), value.c_str());
  }

public:
  MyAhrsDriverForROS(std::string port="", int baud_rate=115200)
  : iMyAhrsPlus(port, baud_rate),
    nh_priv_("~")
  {
    // dependent on user device
    nh_priv_.setParam("port", port);
    nh_priv_.setParam("baud_rate", baud_rate);
    nh_priv_.param("publish_tf", publish_tf, false);
    // default frame id
    nh_priv_.param("frame_id", frame_id_, std::string("imu_link"));
    // for testing the tf
    nh_priv_.param("parent_frame_id", parent_frame_id_, std::string("base_link"));
    // defaults obtained experimentally from device
    nh_priv_.param("linear_acceleration_stddev", linear_acceleration_stddev_, 0.026831);
    nh_priv_.param("angular_velocity_stddev", angular_velocity_stddev_, 0.002428);
    nh_priv_.param("magnetic_field_stddev", magnetic_field_stddev_, 0.00000327486);
    nh_priv_.param("orientation_stddev", orientation_stddev_, 0.002143);
    // publisher for streaming
    imu_data_raw_pub_   = nh_.advertise<sensor_msgs::Imu>("imu/data_raw", 1);
    imu_data_pub_       = nh_.advertise<sensor_msgs::Imu>("imu/data", 1);
    imu_mag_pub_        = nh_.advertise<sensor_msgs::MagneticField>("imu/mag", 1);
    imu_temperature_pub_= nh_.advertise<std_msgs::Float64>("imu/temperature", 1);
  }

  ~MyAhrsDriverForROS()
  {}

  bool initialize()
  {
    bool ok = false;

    do
    {
      if(start() == false) break;
      //Euler angle(x, y, z axis)
      //IMU(linear_acceleration, angular_velocity, magnetic_field)
      if(cmd_binary_data_format("EULER, IMU") == false) break;
      // 100Hz
      if(cmd_divider("1") == false) break;
      // Binary and Continue mode
      if(cmd_mode("BC") == false) break;
      ok = true;
    } while(0);

    return ok;
  }

  inline void get_data(SensorData& data)
  {
    LockGuard _l(lock_);
    data = sensor_data_;
  }

  inline SensorData get_data()
  {
    LockGuard _l(lock_);
    return sensor_data_;
  }

  void publish_topic(int sensor_id)
  {
    sensor_msgs::Imu imu_data_raw_msg;
    sensor_msgs::Imu imu_data_msg;
    sensor_msgs::MagneticField imu_magnetic_msg;
    std_msgs::Float64 imu_temperature_msg;

    double linear_acceleration_cov = linear_acceleration_stddev_ * linear_acceleration_stddev_;
    double angular_velocity_cov    = angular_velocity_stddev_ * angular_velocity_stddev_;
    double magnetic_field_cov      = magnetic_field_stddev_ * magnetic_field_stddev_;
    double orientation_cov         = orientation_stddev_ * orientation_stddev_;

    imu_data_raw_msg.linear_acceleration_covariance[0] =
    imu_data_raw_msg.linear_acceleration_covariance[4] =
    imu_data_raw_msg.linear_acceleration_covariance[8] =
    imu_data_msg.linear_acceleration_covariance[0] =
    imu_data_msg.linear_acceleration_covariance[4] =
    imu_data_msg.linear_acceleration_covariance[8] = linear_acceleration_cov;

    imu_data_raw_msg.angular_velocity_covariance[0] =
    imu_data_raw_msg.angular_velocity_covariance[4] =
    imu_data_raw_msg.angular_velocity_covariance[8] =
    imu_data_msg.angular_velocity_covariance[0] =
    imu_data_msg.angular_velocity_covariance[4] =
    imu_data_msg.angular_velocity_covariance[8] = angular_velocity_cov;

    imu_data_msg.orientation_covariance[0] =
    imu_data_msg.orientation_covariance[4] =
    imu_data_msg.orientation_covariance[8] = orientation_cov;

    imu_magnetic_msg.magnetic_field_covariance[0] =
    imu_magnetic_msg.magnetic_field_covariance[4] =
    imu_magnetic_msg.magnetic_field_covariance[8] = magnetic_field_cov;

    static double convertor_g2a  = 9.80665;    // for linear_acceleration (g to m/s^2)
    static double convertor_d2r  = M_PI/180.0; // for angular_velocity (degree to radian)
    static double convertor_r2d  = 180.0/M_PI; // for easy understanding (radian to degree)
    static double convertor_ut2t = 1000000;    // for magnetic_field (uT to Tesla)
    static double convertor_c    = 1.0;        // for temperature (celsius)

    double roll, pitch, yaw;

    // original sensor data used the degree unit, convert to radian (see ROS REP103)
    // we used the ROS's axes orientation like x forward, y left and z up
    // so changed the y and z aixs of myAHRS+ board
    roll  =  sensor_data_.euler_angle.roll*convertor_d2r;
    pitch = -sensor_data_.euler_angle.pitch*convertor_d2r;
    yaw   = -sensor_data_.euler_angle.yaw*convertor_d2r;

    ImuData<float>& imu = sensor_data_.imu;

    tf::Quaternion orientation = tf::createQuaternionFromRPY(roll, pitch, yaw);

    ros::Time now = ros::Time::now();

    imu_data_raw_msg.header.stamp =
    imu_data_msg.header.stamp     =
    imu_magnetic_msg.header.stamp = now;

    imu_data_raw_msg.header.frame_id =
    imu_data_msg.header.frame_id     =
    imu_magnetic_msg.header.frame_id = frame_id_;

    // orientation
    imu_data_msg.orientation.x = orientation[0];
    imu_data_msg.orientation.y = orientation[1];
    imu_data_msg.orientation.z = orientation[2];
    imu_data_msg.orientation.w = orientation[3];

    // original data used the g unit, convert to m/s^2
    imu_data_raw_msg.linear_acceleration.x =
    imu_data_msg.linear_acceleration.x     =  imu.ax * convertor_g2a;
    imu_data_raw_msg.linear_acceleration.y =
    imu_data_msg.linear_acceleration.y     = -imu.ay * convertor_g2a;
    imu_data_raw_msg.linear_acceleration.z =
    imu_data_msg.linear_acceleration.z     = -imu.az * convertor_g2a;

    // original data used the degree/s unit, convert to radian/s
    imu_data_raw_msg.angular_velocity.x =
    imu_data_msg.angular_velocity.x     =  imu.gx * convertor_d2r;
    imu_data_raw_msg.angular_velocity.y =
    imu_data_msg.angular_velocity.y     = -imu.gy * convertor_d2r;
    imu_data_raw_msg.angular_velocity.z =
    imu_data_msg.angular_velocity.z     = -imu.gz * convertor_d2r;

    // original data used the uTesla unit, convert to Tesla
    imu_magnetic_msg.magnetic_field.x =  imu.mx / convertor_ut2t;
    imu_magnetic_msg.magnetic_field.y = -imu.my / convertor_ut2t;
    imu_magnetic_msg.magnetic_field.z = -imu.mz / convertor_ut2t;

    // original data used the celsius unit
    imu_temperature_msg.data = imu.temperature;

    // publish the IMU data
    imu_data_raw_pub_.publish(imu_data_raw_msg);
    imu_data_pub_.publish(imu_data_msg);
    imu_mag_pub_.publish(imu_magnetic_msg);
    imu_temperature_pub_.publish(imu_temperature_msg);

    // publish tf
    if(publish_tf)
    {
      broadcaster_.sendTransform(tf::StampedTransform(tf::Transform(tf::createQuaternionFromRPY(roll, pitch, yaw),
                                 tf::Vector3(0.0, 0.0, 0.0)),
                                 ros::Time::now(),
                                 frame_id_,
                                 parent_frame_id_));
    }
  }
};


//------------------------------------------------------------------------------
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "myahrs_driver");

  std::string port = std::string("/dev/ttyACM0");
  int baud_rate    = 115200;

  ros::param::get("~port", port);
  ros::param::get("~baud_rate", baud_rate);

  MyAhrsDriverForROS sensor(port, baud_rate);

  if(sensor.initialize() == false)
  {
    ROS_ERROR("%s\n", "Initialize() returns false, please check your devices.");
    return 0;
  }
  else
  {
    ROS_INFO("Initialization OK!\n");
  }

  ros::spin();

  return 0;
}

//------------------------------------------------------------------------------
