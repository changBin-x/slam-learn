/*
 * @Date: 2020-11-29 11:07:24
 * @LastEditTime: 2020-11-30 14:12:05
 * @Author:  Chang_Bin
 * @LastEditors: Chang_Bin
 * @Email: bin_chang@qq.com
 * @Description:mybot仿真
 */
#ifndef MYBOT_FAKE_H_
#define MYBOT_FAKE_H_

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int32.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <turtlebot3_msgs/SensorState.h>

const double WHEEL_RADIUS = 0.033;  //单位m

const int LEFT = 0;
const int RIGHT = 1;

const double MAX_LINEAR_VELOCITY = 0.22;   // m/s
const double MAX_ANGULAR_VELOCITY = 2.84;  // rad/s
const double VELOCITY_STEP = 0.01;         // m/s
const double VELOCITY_LINEAR_X = 0.01;     // m/s

const double VELOCITY_ANGULAR_Z = 0.1;  // rad/s
const int SCALE_VELOCITY_LINEAR_X = 1;
const int SCALE_VELOCITY_ANGULAR_Z = 1;

const int TORQUE_ENABLE = 1;   // 使能电机力矩
const int TORQUE_DISABLE = 0;  // 关闭电机力矩

template <typename T>
inline double DEG2RAD(const T& x) {
  return x * 0.01745329252;  // x*PI/180
}

template <typename T>
inline double RAD2DEG(const T& x) {
  return x * 57.2957795131;  // x*180/PI
}

class MybotFake {
 public:
  MybotFake(/* args */);
  ~MybotFake();
  bool init();
  bool update();

 private:
  // ROS节点
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  // ROS时间
  ros::Time last_cmd_vel_time_;
  ros::Time prev_update_time_;

  // ROS话题发布者
  ros::Publisher joint_states_pub_;
  ros::Publisher odom_pub_;

  // ROS话题订阅者
  ros::Subscriber cmd_vel_sub_;

  sensor_msgs::JointState joint_states_;     //关节状态
  nav_msgs::Odometry odom_;                  //里程计信息
  tf::TransformBroadcaster tf_broadcaster_;  //坐标系转换

  double wheel_speed_cmd_[2];     //轮速
  double goal_linear_velocity_;   //目标线速度
  double goal_angular_velocity_;  //目标角速度
  double cmd_vel_timeout_;

  float odom_pose_[3];
  float odom_vel_[3];
  double pose_conv_[36];

  std::string joint_states_name_[2];

  double last_position_[2];
  double last_velocity_[2];

  double wheel_seperation_;  //轮距
  double turning_radius_;    //转动半径
  double robot_radius_;      //小车运动的旋转半径

  // Function prototypes
  void commandVelocityCallback(const geometry_msgs::TwistConstPtr cmd_vel_msg);
  bool updateOdometry(ros::Duration diff_time);
  void updateJoint(void);
  void updateTF(geometry_msgs::TransformStamped& odom_tf);
};

#endif  // MYBOT_FAKE_H_