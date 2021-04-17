/*
 * @Date: 2020-12-08 16:30:39
 * @LastEditTime: 2020-12-08 20:46:04
 * @Author:  Chang_Bin
 * @LastEditors: Chang_Bin
 * @Email: bin_chang@qq.com
 * @Description:我们需要创建一个节点来完成在ROS上广播base_laser → base_link
 * 转换的工作
 */
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);
  tf2_ros::TransformBroadcaster broadcaster;

  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.frame_id = "base_link";
  transformStamped.child_frame_id = "base_laser";

  tf2::Quaternion q(0, 0, 0, 1);  //旋转
  tf2::Vector3 v(0.1, 0.0, 0.2);  //平移
  //   tf2::Transform tf2Transform(tf2::Quaternion(0, 0, 0, 1),   //旋转
  //                               tf2::Vector3(0.1, 0.0, 0.2));  //平移

  transformStamped.transform.translation.x = 0.1;
  transformStamped.transform.translation.y = 0.0;
  transformStamped.transform.translation.z = 0.2;

  transformStamped.transform.rotation.x = 0.0;
  transformStamped.transform.rotation.y = 0.0;
  transformStamped.transform.rotation.z = 0.0;
  transformStamped.transform.rotation.w = 1.0;

  while (n.ok()) {
    transformStamped.header.stamp = ros::Time::now();
    broadcaster.sendTransform(transformStamped);
    r.sleep();
  }
}