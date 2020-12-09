/*
 * @Date: 2020-12-07 15:14:45
 * @LastEditTime: 2020-12-07 17:01:55
 * @Author:  Chang_Bin
 * @LastEditors: Chang_Bin
 * @Email: bin_chang@qq.com
 * @Description:在海龟移动时广播它们不断变化的坐标系。
 */
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
// tf2软件包提供了TransformBroadcaster的实现，以帮助简化发布转换的任务
#include <tf2_ros/transform_broadcaster.h>
#include <turtlesim/Pose.h>

std::string turtle_name;

void poseCallback(const turtlesim::PoseConstPtr &msg) {
  // 创建一个TransformBroadcaster对象，稍后将使用它通过媒介发送转换。
  static tf2_ros::TransformBroadcaster br;

  // 创建一个Transform对象，并为其提供适当的元数据
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "world";
  transformStamped.child_frame_id = turtle_name;
  transformStamped.transform.translation.x = msg->x;
  transformStamped.transform.translation.y = msg->y;
  transformStamped.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, msg->theta);
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  // 使用TransformBroadcaster发送转换，仅需要传递转换本身。
  //   sendTransform和StampedTransform的父级和子级顺序相反
  br.sendTransform(transformStamped);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "my_tf2_broadcaster");
  ros::NodeHandle private_node("~");

  if (!private_node.hasParam("turtle")) {
    if (argc != 2) {
      ROS_ERROR("need turtle name as argument");
      return -1;
    }
    turtle_name = argv[1];
  } else {
    private_node.getParam("turtle", turtle_name);
  }
  ros::NodeHandle node;
  ros::Subscriber sub =
      node.subscribe(turtle_name + "/pose", 10, &poseCallback);
  ros::spin();
  return 0;
}