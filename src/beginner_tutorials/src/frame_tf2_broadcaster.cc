/*
 * @Date: 2020-12-08 11:09:41
 * @LastEditTime: 2020-12-08 16:36:22
 * @Author:  Chang_Bin
 * @LastEditors: Chang_Bin
 * @Email: bin_chang@qq.com
 * @Description:本教程教您如何向tf2添加额外的固定框架
 * 第一只乌龟添加一个新框架。 该框架将成为第二只乌龟的“胡萝卜”。
 */
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "my_tf2_broadcaster");
  ros::NodeHandle node;

  tf2_ros::TransformBroadcaster tfb;
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.frame_id = "turtle1";
  transformStamped.child_frame_id = "carrot1";
  //平移，位置

  transformStamped.transform.translation.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0, 0, 0);
  //旋转，姿态
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  ros::Rate rate(10.0);
  while (node.ok()) {
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.transform.translation.x =
        2.0 * sin(ros::Time::now().toSec());
    transformStamped.transform.translation.y =
        2.0 * cos(ros::Time::now().toSec());
    tfb.sendTransform(transformStamped);
    rate.sleep();
    printf("sending\n");
  }
  return 0;
}