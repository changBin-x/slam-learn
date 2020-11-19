/*
 * @Date: 2020-10-28 17:15:01
 * @LastEditTime: 2020-10-28 17:19:38
 * @Author:  Chang_Bin
 * @LastEditors: Chang_Bin
 * @Email: bin_chang@qq.com
 * @Description:ROS下消息订阅节点
 */
#include "ros/ros.h"
#include "std_msgs/String.h"

void chatterCallBack(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO("I heard:[%s]", msg->data.c_str());
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallBack);
  //   ros::spin()进入自循环，可以尽可能快的调用消息回调函数
  ros::spin();
  return 0;
}