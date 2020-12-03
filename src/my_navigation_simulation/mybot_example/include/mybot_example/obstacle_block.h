/*
 * @Date: 2020-12-03 13:48:30
 * @LastEditTime: 2020-12-03 18:08:36
 * @Author:  Chang_Bin
 * @LastEditors: Chang_Bin
 * @Email: bin_chang@qq.com
 * @Description:
 */
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/common.h>
#include <pcl/conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include "pcl_ros/transforms.h"
#include "ros/ros.h"

const double LINEAR_VELOCITY = 0.22;
// const double STOP_DISTANCE = 0.5;
const double SAFE_STOP_DISTANCE = 0.85;

class ObstacleBlock {
 private:
  ros::NodeHandle nh_;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher points_cloud_pub_;

  ros::Subscriber odom_sub_;
  ros::Subscriber velodyne_scan_sub_;
  double xMin, yMin;

  void updatecommandVelocity(double linear, double angular);
  void velodyneMsgCallBack(const sensor_msgs::PointCloud2ConstPtr& msgPtr);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_filtered(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& sourceCloud);

 public:
  ObstacleBlock(/* args */);
  ~ObstacleBlock();
  bool init();
  void obstacle();
};

bool ObstacleBlock::init() {
  //初始化速度話題名
  //获取全局节点名
  std::string cmd_vel_topicName = nh_.param<std::string>("cmd_vel_topic", "");

  xMin = 0.0;
  yMin = 0.0;
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topicName, 10);
  points_cloud_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>("/points_cloud2_topic", 10);

  velodyne_scan_sub_ = nh_.subscribe("/velodyne_points", 1,
                                     &ObstacleBlock::velodyneMsgCallBack, this);

  return true;
}

ObstacleBlock::ObstacleBlock() {
  bool ret = init();
  ROS_ASSERT(ret);
}

ObstacleBlock::~ObstacleBlock() {}

/**
 * @brief 点云滤波操作
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr ObstacleBlock::pcl_cloud_filtered(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& sourceCloud) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFilter(
      new pcl::PointCloud<pcl::PointXYZ>());

  //直通滤波
  // 0.266是baselink的宽，0.098和baselink的高有关
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(sourceCloud);  //设置输入点云
  pass.setFilterFieldName("x");  //设置过滤时所需要点云类型的Z字段
  pass.setFilterLimits(0.266, 3.0);  //设置在过滤字段的范围
  pass.filter(*cloudFilter);

  pass.setInputCloud(cloudFilter);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(-3.0, 3.0);  //设置在过滤字段的范围
  pass.filter(*cloudFilter);

  pass.setInputCloud(cloudFilter);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.098, 3);
  pass.filter(*cloudFilter);

  int32_t pointsSize = cloudFilter->points.size();

  return cloudFilter;
}

void ObstacleBlock::velodyneMsgCallBack(
    const sensor_msgs::PointCloud2ConstPtr& msgPtr) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>());

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFilter(
      new pcl::PointCloud<pcl::PointXYZ>());

  pcl::fromROSMsg(*msgPtr, *cloud);

  cloudFilter = pcl_cloud_filtered(cloud);

  // int32_t pointsSize = cloudFilter->points.size();

  pcl::PointXYZ minXYZ;  //用于存放三个轴的最小值
  pcl::PointXYZ maxXYZ;  //用于存放三个轴的最大值
  pcl::getMinMax3D(*cloudFilter, minXYZ, maxXYZ);
  xMin = minXYZ.x;
  yMin = abs(minXYZ.y);

  sensor_msgs::PointCloud2 filterCloudMsg;
  pcl::toROSMsg(*cloudFilter, filterCloudMsg);

  points_cloud_pub_.publish(filterCloudMsg);
}

void ObstacleBlock::updatecommandVelocity(double linear, double angular) {
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = linear;
  cmd_vel.angular.z = angular;
  cmd_vel_pub_.publish(cmd_vel);
}

void ObstacleBlock::obstacle() {
  bool bot_moving = true;

  if (xMin < SAFE_STOP_DISTANCE) {
    if (bot_moving) {
      updatecommandVelocity(0.0, 0.0);
      bot_moving = false;
      ROS_INFO("Bot Stop!");
    }
  } else {
    updatecommandVelocity(LINEAR_VELOCITY, 0.0);
    bot_moving = true;
    ROS_INFO("Distance of the block:[%.3f,%.3f]", xMin, yMin);
  }
}