/*
 * @Date: 2020-10-28 17:58:03
 * @LastEditTime: 2020-10-28 19:14:47
 * @Author:  Chang_Bin
 * @LastEditors: Chang_Bin
 * @Email: bin_chang@qq.com
 * @Description:ROS Service节点
 */
#include "beginner_tutorials/AddTwoInts.h"
#include "ros/ros.h"

bool add(beginner_tutorials::AddTwoInts::Request &req,
         beginner_tutorials::AddTwoInts::Response &res) {
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%1d,y=%1d", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response:[%1d]", (long int)res.sum);
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "addTwoIntsServer");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("addTwoInts", add);

  ROS_INFO("Ready to add two ints");
  ros::spin();
  return 0;
}