/*
 * @Date: 2020-11-02 14:03:29
 * @LastEditTime: 2020-11-26 16:59:55
 * @Author:  Chang_Bin
 * @LastEditors: Chang_Bin
 * @Email: bin_chang@qq.com
 * @Description:实现小车的简单避障
 */
#include "mybot_gazebo/mybot_drive.h"
MyBotDrive::MyBotDrive() : nh_priv_("~") {
  //初始化gazebo ros bot节点
  ROS_INFO("mybot Simulation Node Init");
  bool ret = init();
  //初始化失败程序结束
  ROS_ASSERT(ret);
}

MyBotDrive::~MyBotDrive() {
  updatecommandVelocity(0.0, 0.0);
  ros::shutdown();
}

/**
 * @description: 初始化函數
 * @param:
 * @return:
 */
bool MyBotDrive::init() {
  //初始化速度話題名
  std::string cmd_vel_topicName =
      nh_.param<std::string>("cmd_vel_topicName", "");

  //初始化变量
  escape_range_ = 0.8;
  check_forward_dist_ = 0.7;
  check_side_dist_ = 0.6;

  //初始姿态
  bot_pose_ = 0.0;
  prev_bot_pose_ = 0.0;

  //初始化发布者
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topicName, 10);
  temp_points_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>("/temp_points", 10);
  //初始化订阅者
  velodyne_scan_sub_ = nh_.subscribe("/velodyne_points", 1,
                                     &MyBotDrive::velodyneMsgCallBack, this);

  odom_sub_ = nh_.subscribe("odom", 10, &MyBotDrive::odomMsgCallBack, this);

  return true;
}

void MyBotDrive::tempPointsCallBack(
    const sensor_msgs::PointCloud2ConstPtr &msgPtr) {}

void MyBotDrive::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &OdomMsg) {
  double siny =
      2.0 *
      (OdomMsg->pose.pose.orientation.w * OdomMsg->pose.pose.orientation.z +
       OdomMsg->pose.pose.orientation.x * OdomMsg->pose.pose.orientation.y);
  double cosy =
      1.0 -
      2.0 *
          (OdomMsg->pose.pose.orientation.x * OdomMsg->pose.pose.orientation.x +
           OdomMsg->pose.pose.orientation.y * OdomMsg->pose.pose.orientation.y);
  //姿态角中的Yaw,航向角
  bot_pose_ = atan2(siny, cosy);
  // ROS_INFO("Yaw:%f", bot_pose_);
}

/**
 * @brief 点云滤波操作
 * @param
 * @return
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr MyBotDrive::pcl_cloud_filtered(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &sourceCloud) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFilter(
      new pcl::PointCloud<pcl::PointXYZ>());

  //直通滤波
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(sourceCloud);  //设置输入点云
  pass.setFilterFieldName("x");  //设置过滤时所需要点云类型的Z字段
  pass.setFilterLimits(0.140, 3.0);  //设置在过滤字段的范围
  pass.filter(*cloudFilter);

  pass.setInputCloud(cloudFilter);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(-3.0, 3.0);  //设置在过滤字段的范围
  pass.filter(*cloudFilter);

  pass.setInputCloud(cloudFilter);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.143, 3);
  pass.filter(*cloudFilter);

  int32_t pointsSize = cloudFilter->points.size();

  return cloudFilter;
}

void MyBotDrive::velodyneMsgCallBack(
    const sensor_msgs::PointCloud2ConstPtr &msgPtr) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>());

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFilter(
      new pcl::PointCloud<pcl::PointXYZ>());

  pcl::fromROSMsg(*msgPtr, *cloud);

  cloudFilter = pcl_cloud_filtered(cloud);

  int32_t pointsSize = cloudFilter->points.size();

  double xMin = 5.0;
  double y_xMin = 0.0;

  for (size_t i = 0; i < pointsSize; i++) {
    if (cloudFilter->points[i].x < xMin) {
      xMin = cloudFilter->points[i].x;
      y_xMin = cloudFilter->points[i].y;
    }
  }
  // ROS_INFO("xMin:%.6f,yMin:%.6f", xMin, y_xMin);

  scan_velodyne_[0] = xMin;
  scan_velodyne_[1] = y_xMin;

  sensor_msgs::PointCloud2 filterCloudMsg;
  pcl::toROSMsg(*cloudFilter, filterCloudMsg);

  temp_points_pub_.publish(filterCloudMsg);
}
void MyBotDrive::updatecommandVelocity(double linear, double angular) {
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = linear;
  cmd_vel.angular.z = angular;
  cmd_vel_pub_.publish(cmd_vel);
}

/**
 * @description: 控制循环函数,自动避障
 * @param:
 * @return:
 */
bool MyBotDrive::controlLoop() {
  if (scan_velodyne_[0] <= escape_range_ && scan_velodyne_[1] < 0) {
    ROS_INFO("[x:%.6f,y:%.6f] Left", scan_velodyne_[0], scan_velodyne_[1]);
    updatecommandVelocity(LINEAR_VELOCITY, ANGULAR_VELOCITY);
  } else if (scan_velodyne_[0] <= escape_range_ && scan_velodyne_[1] > 0) {
    ROS_INFO("[x:%.6f,y:%.6f] Right", scan_velodyne_[0], scan_velodyne_[1]);
    updatecommandVelocity(LINEAR_VELOCITY, -ANGULAR_VELOCITY);
  } else {
    ROS_INFO("[x:%.6f,y:%.6f] Forward", scan_velodyne_[0], scan_velodyne_[1]);
    updatecommandVelocity(LINEAR_VELOCITY, 0.0);
  }

  return true;
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "mybot_drive");
  MyBotDrive botDrive;

  ros::Rate loop_rate(125);

  while (ros::ok()) {
    botDrive.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}