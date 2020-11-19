/*
 * @Date: 2020-11-02 14:03:29
 * @LastEditTime: 2020-11-10 21:45:50
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
  escape_range_ = 30.0 * DEG2RAD;  // pi/6
  check_forward_dist_ = 0.7;
  check_side_dist_ = 0.6;

  //初始姿态
  bot_pose_ = 0.0;
  prev_bot_pose_ = 0.0;

  //初始化发布者
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topicName, 10);
  //初始化订阅者
  velodyne_scan_sub_ = nh_.subscribe("/velodyne_points", 1,
                                     &MyBotDrive::velodyneMsgCallBack, this);
  odom_sub_ = nh_.subscribe("odom", 10, &MyBotDrive::odomMsgCallBack, this);

  return true;
}

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
  //姿态角中的Yaw
  bot_pose_ = atan2(siny, cosy);
}

void MyBotDrive::velodyneMsgCallBack(
    const sensor_msgs::PointCloud2ConstPtr &msgPtr) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msgPtr, *cloud);
  int32_t pointsSize = cloud->points.size();
  for (size_t i = 0; i < pointsSize; i++) {
    ROS_INFO("Cloud (%f,%f,%f)", cloud->points[i].x, cloud->points[i].y,
             cloud->points[i].z);
  }
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
bool MyBotDrive::controlLoop(int orientation) {
  static uint8_t bot_state_num = orientation;

  switch (bot_state_num) {
    case BOT_DRIVE_FORWARD:
      updatecommandVelocity(LINEAR_VELOCITY, 0.0);
      // ROS_INFO("Moving to Forward...");
      // bot_state_num = GET_BOT_DIRECTION;
      break;
    case BOT_RIGHT_TURN:
      updatecommandVelocity(0.0, -1 * ANGULAR_VELOCITY);
      // ROS_INFO("Turning Right...");
      break;
    case BOT_LEFT_TURN:
      // ROS_INFO("Turning left...");
      updatecommandVelocity(0.0, ANGULAR_VELOCITY);
      break;
    default:
      // ROS_INFO("Moving to Backward...");
      updatecommandVelocity(-LINEAR_VELOCITY, 0.0);
      break;
  }
  return true;
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "mybot_drive");
  MyBotDrive botDrive;

  ros::Rate loop_rate(125);

  while (ros::ok()) {
    botDrive.controlLoop(LINEAR_VELOCITY);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}