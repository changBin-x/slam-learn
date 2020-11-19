/*
 * @Date: 2020-10-28 22:13:44
 * @LastEditTime: 2020-10-28 22:21:36
 * @Author:  Chang_Bin
 * @LastEditors: Chang_Bin
 * @Email: bin_chang@qq.com
 * @Description:自定义消息类型
 */
#include "beginner_tutorials/Person.h"
#include "ros/ros.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "personPub");
  ros::NodeHandle node;
  ros::Publisher pub =
      node.advertise<beginner_tutorials::Person>("personInfo", 10);
  ros::Rate rate(10);

  while (ros::ok()) {
    beginner_tutorials::Person PersonMsg;
    PersonMsg.age = 24;
    PersonMsg.sex = beginner_tutorials::Person::man;
    PersonMsg.name = "Boy";

    pub.publish(PersonMsg);

    ROS_INFO("Publish PersonMsg is: name:%s,age:%d,sex:%d",
             PersonMsg.name.c_str(), PersonMsg.age, PersonMsg.sex);
    rate.sleep();
  }
  return 0;
}