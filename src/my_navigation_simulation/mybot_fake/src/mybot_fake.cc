/*
 * @Date: 2020-11-29 11:25:39
 * @LastEditTime: 2020-11-29 17:16:43
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
  //线速度v=角速度×(直径)/2
  wheel_speed_cmd_[LEFT] =
      goal_linear_velocity_ - (goal_angular_velocity_ * wheel_seperation_ / 2);
  wheel_speed_cmd_[RIGHT] =
      goal_linear_velocity_ + (goal_angular_velocity_ * wheel_seperation_ / 2);
};

/**
 * @brief 更新里程计
 */
bool MybotFake::updateOdometry(ros::Duration diff_time) {
  double wheel_l, wheel_r;  //车轮的转动角度(rad)
  double delta_s, delta_theta;
  double v[2], w[2];

  wheel_l = wheel_r = 0.0;
  delta_s = delta_theta = 0.0;

  v[LEFT] = wheel_speed_cmd_[LEFT];
  w[LEFT] = v[LEFT] / WHEEL_RADIUS;  // w=v/r;
  v[RIGHT] = wheel_speed_cmd_[RIGHT];
  w[RIGHT] = v[RIGHT] / WHEEL_RADIUS;

  last_velocity_[LEFT] = w[LEFT];
  last_velocity_[RIGHT] = w[RIGHT];

  wheel_l = w[LEFT] * diff_time.toSec();
  wheel_r = w[RIGHT] * diff_time.toSec();

  if (isnan(wheel_l)) wheel_l = 0.0;
  if (isnan(wheel_r)) wheel_r = 0.0;

  last_position_[LEFT] += wheel_l;
  last_position_[RIGHT] += wheel_r;

  delta_s = WHEEL_RADIUS * (wheel_l + wheel_r) / 2.0;
};
void MybotFake::updateJoint(void){};
void MybotFake::updateTF(geometry_msgs::TransformStamped& odom_tf){};
bool MybotFake::update(){};
MybotFake::~MybotFake(){};

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "turtlebot3_fake_node");
  MybotFake botfake;

  ros::Rate loop_rate(30);

  while (ros::ok()) {
    botfake.update();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}