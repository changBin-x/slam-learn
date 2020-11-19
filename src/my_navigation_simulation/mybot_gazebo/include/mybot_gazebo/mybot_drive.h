/*
 * @Date: 2020-11-01 23:03:30
 * @LastEditTime: 2020-11-18 10:59:19
 * @Author:  Chang_Bin
 * @LastEditors: Chang_Bin
 * @Email: bin_chang@qq.com
 * @Description:模拟机器人的驱动,代码来源于turtleBot3
 */
#ifndef MYBOT_DRIVE_H_
#define MYBOT_DRIVE_H_

#include <iostream>

#include "gazebo/transport/TransportTypes.hh"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "pcl/PCLPointCloud2.h"
#include "pcl/conversions.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.h"
#include "ros/ros.h"
// #include "sensor_msgs/LaserScan.h"
#include "pcl/features/normal_3d.h"
#include "sensor_msgs/PointCloud2.h"
//#include "velodyne_gazebo_plugins/GazeboRosVelodyneLaser.h"

const double DEG2RAD = M_PI / 180.0;
const int LEFT = 1;
const int RIGHT = 2;
const double LINEAR_VELOCITY = 0.3;
const double ANGULAR_VELOCITY = 1.5;

// 用于路径规划
const int GET_BOT_DIRECTION = 0;
const int BOT_DRIVE_FORWARD = 1;
const int BOT_RIGHT_TURN = 2;
const int BOT_LEFT_TURN = 3;

class MyBotDrive {
 public:
  MyBotDrive();
  ~MyBotDrive();
  bool init();
  bool controlLoop(int orientation);

 private:
  // ROS节点处理
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;
  // ROS话题发布节点
  ros::Publisher cmd_vel_pub_;
  // ROS话题订阅节点
  //里程计订阅
  ros::Subscriber odom_sub_;
  ros::Subscriber velodyne_scan_sub_;

  // gazebo::GazeboRosVelodyneLaser gazeboRosVelodyneLaser;
  double escape_range_;
  double check_forward_dist_;
  double check_side_dist_;

  double bot_pose_;
  double prev_bot_pose_;

  void updatecommandVelocity(double linear, double angular);
  void velodyneMsgCallBack(const sensor_msgs::PointCloud2ConstPtr& msgPtr);
  void odomMsgCallBack(const nav_msgs::Odometry::ConstPtr& OdomMsg);
};

#endif  // MYBOT_DRIVE_H_
