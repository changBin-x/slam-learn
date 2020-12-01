/*
 * @Date: 2020-11-29 11:25:39
 * @LastEditTime: 2020-12-01 09:21:27
 * @Author:  Chang_Bin
 * @LastEditors: Chang_Bin
 * @Email: bin_chang@qq.com
 * @Description:mybot仿真
 */
#include "mybot_fake/mybot_fake.h"

MybotFake::MybotFake() : nh_priv_("~") {
  // 初始化
  bool init_result = init();
  ROS_INFO("mybot initialing success");
  ROS_ASSERT(init_result);
}

/**
 * @brief 初始化函数
 */
bool MybotFake::init() {
  //初始化ros参数
  ROS_INFO("mybot initialing...");
  std::string robot_model = nh_.param<std::string>("mybot_model", "");
  if (!robot_model.compare("burger")) {
    wheel_seperation_ = 0.160;
    turning_radius_ = 0.080;
    robot_radius_ = 0.105;
  } else if (!robot_model.compare("waffle")) {
    wheel_seperation_ = 0.287;
    turning_radius_ = 0.1435;
    robot_radius_ = 0.220;
  }

  nh_.param("wheel_left_joint_name", joint_states_name_[LEFT],
            std::string("wheel_left_joint"));
  nh_.param("wheel_right_joint_name", joint_states_name_[RIGHT],
            std::string("wheel_right_joint"));
  nh_.param("joint_states_frame", joint_states_.header.frame_id,
            std::string("base_footprint"));
  nh_.param("odom_frame", odom_.header.frame_id, std::string("odom"));
  nh_.param("base_frame", odom_.child_frame_id, std::string("base_footprint"));

  // initialize variables
  wheel_speed_cmd_[LEFT] = 0.0;
  wheel_speed_cmd_[RIGHT] = 0.0;
  goal_linear_velocity_ = 0.0;
  goal_angular_velocity_ = 0.0;
  cmd_vel_timeout_ = 1.0;
  last_position_[LEFT] = 0.0;
  last_position_[RIGHT] = 0.0;
  last_velocity_[LEFT] = 0.0;
  last_velocity_[RIGHT] = 0.0;

  double pcov[36] = {0.1, 0, 0,   0, 0,   0, 0, 0.1, 0, 0,   0, 0,
                     0,   0, 1e6, 0, 0,   0, 0, 0,   0, 1e6, 0, 0,
                     0,   0, 0,   0, 1e6, 0, 0, 0,   0, 0,   0, 0.2};
  memcpy(&(odom_.pose.covariance), pcov, sizeof(double) * 36);
  memcpy(&(odom_.twist.covariance), pcov, sizeof(double) * 36);

  odom_pose_[0] = 0.0;
  odom_pose_[1] = 0.0;
  odom_pose_[2] = 0.0;

  odom_vel_[0] = 0.0;
  odom_vel_[1] = 0.0;
  odom_vel_[2] = 0.0;

  joint_states_.name.push_back(joint_states_name_[LEFT]);
  joint_states_.name.push_back(joint_states_name_[RIGHT]);
  joint_states_.position.resize(2, 0.0);
  joint_states_.velocity.resize(2, 0.0);
  joint_states_.effort.resize(2, 0.0);

  // initialize publishers
  joint_states_pub_ =
      nh_.advertise<sensor_msgs::JointState>("joint_states", 100);
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 100);

  // initialize subscribers
  cmd_vel_sub_ =
      nh_.subscribe("cmd_vel", 100, &MybotFake::commandVelocityCallback, this);

  prev_update_time_ = ros::Time::now();

  return true;
};

/**
 * @brief 速度回调函数
 */
void MybotFake::commandVelocityCallback(
    const geometry_msgs::TwistConstPtr cmd_vel_msg) {
  last_cmd_vel_time_ = ros::Time::now();

  goal_linear_velocity_ = cmd_vel_msg->linear.x;
  goal_angular_velocity_ = cmd_vel_msg->angular.z;

  //车辆差速驱动
  //左轮线速度vL=车辆(基点)线速度V-角速度w×(轮距D/2)
  wheel_speed_cmd_[LEFT] =
      goal_linear_velocity_ - (goal_angular_velocity_ * wheel_seperation_ / 2);
  //右轮线速度vR=车辆(基点)线速度+角速度w×(轮距D/2)
  wheel_speed_cmd_[RIGHT] =
      goal_linear_velocity_ + (goal_angular_velocity_ * wheel_seperation_ / 2);
};

/**
 * @brief 更新里程计,
 * 速度航迹推算，速度积分累积误差较大，最终精度10%左右
 */
bool MybotFake::updateOdometry(ros::Duration diff_time) {
  double wheel_l, wheel_r;  //车轮的转动角度(rad)
  double delta_s, delta_theta;
  double v[2], w[2];

  wheel_l = wheel_r = 0.0;
  delta_s = delta_theta = 0.0;

  v[LEFT] = wheel_speed_cmd_[LEFT];
  w[LEFT] = v[LEFT] / WHEEL_RADIUS;  // 右轮转动速度wL=vL/r;
  v[RIGHT] = wheel_speed_cmd_[RIGHT];
  w[RIGHT] = v[RIGHT] / WHEEL_RADIUS;

  last_velocity_[LEFT] = w[LEFT];
  last_velocity_[RIGHT] = w[RIGHT];

  //小车轮子角度变化量△theta=w*△t
  wheel_l = w[LEFT] * diff_time.toSec();
  wheel_r = w[RIGHT] * diff_time.toSec();

  if (isnan(wheel_l)) wheel_l = 0.0;
  if (isnan(wheel_r)) wheel_r = 0.0;

  last_position_[LEFT] += wheel_l;
  last_position_[RIGHT] += wheel_r;

  //弧长sl=△thetaL*R
  delta_s = WHEEL_RADIUS * (wheel_l + wheel_r) / 2.0;  //小车行走距离变化量
  delta_theta = WHEEL_RADIUS * (wheel_r - wheel_l) / wheel_seperation_;

  //计算里程计姿态，速度航迹推算
  odom_pose_[0] += delta_s * cos(odom_pose_[2] + (delta_theta / 2.0));
  odom_pose_[1] += delta_s * sin(odom_pose_[2] + (delta_theta / 2.0));
  odom_pose_[2] += delta_theta;

  //计算里程计瞬时速度
  odom_vel_[0] = delta_s / diff_time.toSec();  //速度
  odom_vel_[1] = 0.0;
  odom_vel_[2] = delta_theta / diff_time.toSec();  //角速度

  odom_.pose.pose.position.x = odom_pose_[0];
  odom_.pose.pose.position.y = odom_pose_[1];
  odom_.pose.pose.position.z = 0.0;
  //由航向角计算出一个四元数msg
  odom_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(odom_pose_[2]);

  //更新里程计的twist
  odom_.twist.twist.linear.x = odom_vel_[0];
  odom_.twist.twist.angular.z = odom_vel_[2];

  return true;
};

/**
 * @brief 计算关节状态
 */
void MybotFake::updateJoint(void) {
  joint_states_.position[LEFT] = last_position_[LEFT];
  joint_states_.position[RIGHT] = last_position_[RIGHT];
  joint_states_.velocity[LEFT] = last_velocity_[LEFT];
  joint_states_.velocity[RIGHT] = last_velocity_[RIGHT];
};

/**
 * @brief 计算坐标变换
 */
void MybotFake::updateTF(geometry_msgs::TransformStamped& odom_tf) {
  odom_tf.header = odom_.header;
  odom_tf.child_frame_id = odom_.child_frame_id;
  odom_tf.transform.translation.x = odom_.pose.pose.position.x;
  odom_tf.transform.translation.y = odom_.pose.pose.position.y;
  odom_tf.transform.translation.z = odom_.pose.pose.position.z;
  odom_tf.transform.rotation = odom_.pose.pose.orientation;
};

/**
 * @brief 更新函数
 */
bool MybotFake::update() {
  ros::Time time_now = ros::Time::now();
  ros::Duration step_time = time_now - prev_update_time_;
  prev_update_time_ = time_now;

  // 超时置0
  if ((time_now - last_cmd_vel_time_).toSec() > cmd_vel_timeout_) {
    wheel_speed_cmd_[LEFT] = 0.0;
    wheel_speed_cmd_[RIGHT] = 0.0;
  }

  // odom
  updateOdometry(step_time);
  odom_.header.stamp = time_now;
  odom_pub_.publish(odom_);

  // joint_states
  updateJoint();
  joint_states_.header.stamp = time_now;
  joint_states_pub_.publish(joint_states_);

  // tf
  geometry_msgs::TransformStamped odom_tf;
  updateTF(odom_tf);
  tf_broadcaster_.sendTransform(odom_tf);

  return true;
};

MybotFake::~MybotFake(){};

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "mybot_fake_node");
  MybotFake botfake;

  ros::Rate loop_rate(30);

  while (ros::ok()) {
    botfake.update();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}