/*
 * @Date: 2020-11-02 14:03:29
 * @LastEditTime: 2020-11-22 12:25:46
 * @Author:  Chang_Bin
 * @LastEditors: Chang_Bin
 * @Email: bin_chang@qq.com
 * @Description:
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
  escape_range_ = 0.03;
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
  // velodyne_scan_sub_ =
  //     nh_.subscribe("/temp_points", 1, &MyBotDrive::tempPointsCallBack,
  //     this);
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

void MyBotDrive::velodyneMsgCallBack(
    const sensor_msgs::PointCloud2ConstPtr &msgPtr) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFilter(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msgPtr, *cloud);

  // 剪除点云

  pcl::ConditionAnd<pcl::PointXYZ>::Ptr rangeCloud(
      new pcl::ConditionAnd<pcl::PointXYZ>);
  //设置点云作用域为z,取大于0.08且小于0.8的位置，保留在点云中，其余进行移除
  pcl::FieldComparison<pcl::PointXYZ>::ConstPtr comparisonOpsGT(
      new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT,
                                              0.008));
  pcl::FieldComparison<pcl::PointXYZ>::ConstPtr ComparisonOpsLT(
      new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT,
                                              0.8));
  rangeCloud->addComparison(comparisonOpsGT);
  rangeCloud->addComparison(ComparisonOpsLT);
  pcl::ConditionalRemoval<pcl::PointXYZ> cloud_after_rmv;
  cloud_after_rmv.setCondition(rangeCloud);
  cloud_after_rmv.setInputCloud(cloud);
  cloud_after_rmv.setKeepOrganized(true);
  cloud_after_rmv.filter(*cloudFilter);

  sensor_msgs::PointCloud2 filterCloudMsg;
  pcl::toROSMsg(*cloudFilter, filterCloudMsg);

  int32_t pointsSize = cloudFilter->points.size();

  double xMin = 200.0;
  double yMin = 200.0;

  for (size_t i = 0; i < pointsSize; i++) {
    if (xMin > abs(cloudFilter->points[i].x)) {
      xMin = abs(cloudFilter->points[i].x);
    }
    if (yMin > abs(cloudFilter->points[i].y)) {
      yMin = abs(cloudFilter->points[i].y);
    }
  }
  ROS_INFO("[xMin:%f,yMin:%f]", xMin, yMin);
  sacn_velodyne_[0] = xMin;
  sacn_velodyne_[1] = yMin;

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
  static uint8_t bot_state_num = 0;

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