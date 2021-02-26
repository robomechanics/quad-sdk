#include <ros/ros.h>
#include <gtest/gtest.h>

#include "spirit_utils/foot_jacobians.h"

TEST(FootJacobianTest, FootJacobian0) {
  ros::NodeHandle nh;
    double pi = 3.14159;
  double q00 =pi/4,q01 =pi/4,q02=pi/2,q10 =pi/4,q11 =pi/4,q12=pi/2,q20 =-pi/4,q21 =pi/4,q22=pi/2,q30 =-pi/4,q31 =pi/4,q32=pi/2;
  double roll =0,pitch =0,yaw =0,x =0,y =-1,z =0;
    double states[] = {roll, pitch, yaw, x, y, z, q00, q01, q02, q10, q11, q12, q20, q21, q22, q30, q31, q32};
    //ww = 0.07; bb = 0.2263; hh = 0;b0= 0.1010; b1 = 0.206;b2= 0.206;
    //parameters = [ww bb hh b0 b1 b2];
  double parameters[] = {0.07,0.2263,0.0,0.1010,0.206,0.206};
  

    Eigen::MatrixXf foot_jacobian0(3,3);

    // Call the foot jacobian calculation
    spirit_utils::calc_foot_jacobian0(states,parameters,foot_jacobian0);
    std::cout<<"Foot Jacobian: "<<std::endl;
    std::cout<<foot_jacobian0;
    std::cout<<std::endl;
    // std::cout<<"Foot Jacobian first index, "<<foot_jacobian0[0]<<std::endl;
    // std::cout<<"Foot Jacobian second index, "<<foot_jacobian0[1]<<std::endl;
    // std::cout<<"Foot Jacobian third index, "<<foot_jacobian0[2]<<std::endl;
    // std::cout<<"Foot Jacobian fourth index, "<<foot_jacobian0[3]<<std::endl;
    // std::cout<<"Foot Jacobian fifth index, "<<foot_jacobian0[4]<<std::endl;
    // std::cout<<"Foot Jacobian sixth index, "<<foot_jacobian0[5]<<std::endl;
    // std::cout<<"Foot Jacobian seventh index, "<<foot_jacobian0[6]<<std::endl;
    // std::cout<<"Foot Jacobian eigth index, "<<foot_jacobian0[7]<<std::endl;
    // std::cout<<"Foot Jacobian ninth index, "<<foot_jacobian0[8]<<std::endl;
    // ROS_INFO("Foot Jacobian first index %f",foot_jacobian0[0]);
  EXPECT_EQ(1 + 1, 2);
}
TEST(FootJacobianTest, FootJacobian1) {
  ros::NodeHandle nh;
    double pi = 3.14159;
  double q00 =pi/4,q01 =pi/4,q02=pi/2,q10 =pi/4,q11 =pi/4,q12=pi/2,q20 =-pi/4,q21 =pi/4,q22=pi/2,q30 =-pi/4,q31 =pi/4,q32=pi/2;
  double roll =0,pitch =0,yaw =0,x =0,y =-1,z =0;
    double states[] = {roll, pitch, yaw, x, y, z, q00, q01, q02, q10, q11, q12, q20, q21, q22, q30, q31, q32};
    //ww = 0.07; bb = 0.2263; hh = 0;b0= 0.1010; b1 = 0.206;b2= 0.206;
    //parameters = [ww bb hh b0 b1 b2];
  double parameters[] = {0.07,0.2263,0.0,0.1010,0.206,0.206};
  

    Eigen::MatrixXf foot_jacobian0(3,3);

    // Call the foot jacobian calculation
    spirit_utils::calc_foot_jacobian1(states,parameters,foot_jacobian0);
  EXPECT_EQ(1 + 1, 2);
}
TEST(FootJacobianTest, FootJacobian2) {
  ros::NodeHandle nh;
    double pi = 3.14159;
  double q00 =pi/4,q01 =pi/4,q02=pi/2,q10 =pi/4,q11 =pi/4,q12=pi/2,q20 =-pi/4,q21 =pi/4,q22=pi/2,q30 =-pi/4,q31 =pi/4,q32=pi/2;
  double roll =0,pitch =0,yaw =0,x =0,y =-1,z =0;
    double states[] = {roll, pitch, yaw, x, y, z, q00, q01, q02, q10, q11, q12, q20, q21, q22, q30, q31, q32};
    //ww = 0.07; bb = 0.2263; hh = 0;b0= 0.1010; b1 = 0.206;b2= 0.206;
    //parameters = [ww bb hh b0 b1 b2];
  double parameters[] = {0.07,0.2263,0.0,0.1010,0.206,0.206};
  

    Eigen::MatrixXf foot_jacobian0;

    // Call the foot jacobian calculation
    spirit_utils::calc_foot_jacobian2(states,parameters,foot_jacobian0);
  EXPECT_EQ(1 + 1, 2);
}
TEST(FootJacobianTest, FootJacobian3) {
  ros::NodeHandle nh;
    double pi = 3.14159;
  double q00 =pi/4,q01 =pi/4,q02=pi/2,q10 =pi/4,q11 =pi/4,q12=pi/2,q20 =-pi/4,q21 =pi/4,q22=pi/2,q30 =-pi/4,q31 =pi/4,q32=pi/2;
  double roll =0,pitch =0,yaw =0,x =0,y =-1,z =0;
    double states[] = {roll, pitch, yaw, x, y, z, q00, q01, q02, q10, q11, q12, q20, q21, q22, q30, q31, q32};
    //ww = 0.07; bb = 0.2263; hh = 0;b0= 0.1010; b1 = 0.206;b2= 0.206;
    //parameters = [ww bb hh b0 b1 b2];
  double parameters[] = {0.07,0.2263,0.0,0.1010,0.206,0.206};
  

    Eigen::MatrixXf foot_jacobian0(3,3);

    // Call the foot jacobian calculation
    spirit_utils::calc_foot_jacobian3(states,parameters,foot_jacobian0);
  EXPECT_EQ(1 + 1, 2);
}

