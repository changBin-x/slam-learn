/*
 * @Date: 2020-10-29 10:40:42
 * @LastEditTime: 2020-11-02 22:30:48
 * @Author:  Chang_Bin
 * @LastEditors: Chang_Bin
 * @Email: bin_chang@qq.com
 * @Description:自定义消息类型订阅节点
 */
#include "beginner_tutorials/Person.h"
#include "ros/ros.h"

void personCallBack(const beginner_tutorials::Person::ConstPtr& msg) {
  ROS_INFO("Subscribe msg:[name:%s age:%d sex:%d]", msg->name.c_str(), msg->age,
           msg->sex);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "personSub");
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("personInfo", 100, personCallBack);

  ros::spin();
  return 0;
}