/*
 * @Date: 2020-12-08 17:02:34
 * @LastEditTime: 2020-12-09 09:56:38
 * @Author:  Chang_Bin
 * @LastEditors: Chang_Bin
 * @Email: bin_chang@qq.com
 * @Description:编写一个节点，它将使用该变换来获取“base_laser”帧中的一个点，
 * 并将其转换为“base_link”帧中的一个点
 */
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

void transformPoint(tf2_ros::Buffer& tfBuffer) {
  // we'll create a point in the base_laser frame that we'd like to transform to
  // the base_link frame
  geometry_msgs::PointStamped laser_point;
  laser_point.header.frame_id = "base_laser";

  // we'll just use the most recent transform available for our simple example
  laser_point.header.stamp = ros::Time();

  // just an arbitrary point in space
  laser_point.point.x = 1.0;
  laser_point.point.y = 0.2;
  laser_point.point.z = 0.0;

  tf2_ros::TransformListener listener(tfBuffer);
  geometry_msgs::TransformStamped transformStamped;

  try {
    geometry_msgs::PointStamped base_point;
    transformStamped =
        tfBuffer.lookupTransform("base_link", "base_laser", ros::Time(0));

    // tfBuffer.transform(laser_point, base_point, "base_link");
    tf2::doTransform(laser_point, base_point, transformStamped);

    // ROS_INFO(
    //     "base_laser: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, "
    //     "%.2f) "
    //     "at time %.2f",
    //     laser_point.point.x, laser_point.point.y, laser_point.point.z,
    //     base_point.point.x, base_point.point.y, base_point.point.z,
    //     base_point.header.stamp.toSec());
    ROS_INFO(
        "base_laser: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, "
        "%.2f) "
        "at time %.2f",
        laser_point.point.x, laser_point.point.y, laser_point.point.z,
        transformStamped.transform.translation.x,
        transformStamped.transform.translation.y,
        transformStamped.transform.translation.z,
        transformStamped.header.stamp);
  } catch (tf2::TransformException& ex) {
    ROS_ERROR("%s", ex.what());
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_tf_listener");
  ros::NodeHandle n;

  tf2_ros::Buffer tfBuffer(ros::Duration(10));

  // we'll transform a point once every second
  ros::Timer timer = n.createTimer(
      ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(tfBuffer)));

  ros::spin();
}