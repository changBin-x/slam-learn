/*
 * @Date: 2020-12-07 17:18:53
 * @LastEditTime: 2020-12-08 17:56:07
 * @Author:  Chang_Bin
 * @LastEditors: Chang_Bin
 * @Email: bin_chang@qq.com
 * @Description:create a tf2 listener to start using tf2.
 */

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <turtlesim/Spawn.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "my_tf2_listener");

  ros::NodeHandle node;

  ros::service::waitForService("spawn");
  ros::ServiceClient spawner = node.serviceClient<turtlesim::Spawn>("spawn");
  turtlesim::Spawn turtle;
  turtle.request.x = 4;
  turtle.request.y = 2;
  turtle.request.theta = 0;
  turtle.request.name = "turtle2";
  spawner.call(turtle);

  ros::Publisher turtle_vel =
      node.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Rate rate(10.0);
  while (node.ok()) {
    geometry_msgs::TransformStamped transformStamped;
    try {
      transformStamped =
          tfBuffer.lookupTransform("turtle2", "turtle1", ros::Time(0));
    } catch (tf2::TransformException& ex) {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    geometry_msgs::Twist vel_msg;

    vel_msg.angular.z = 4.0 * atan2(transformStamped.transform.translation.y,
                                    transformStamped.transform.translation.x);
    vel_msg.linear.x =
        0.5 * sqrt(pow(transformStamped.transform.translation.x, 2) +
                   pow(transformStamped.transform.translation.y, 2));
    turtle_vel.publish(vel_msg);

    rate.sleep();
  }
  return 0;
};