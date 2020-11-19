/*
 * @Date: 2020-10-28 19:15:30
 * @LastEditTime: 2020-10-28 19:20:07
 * @Author:  Chang_Bin
 * @LastEditors: Chang_Bin
 * @Email: bin_chang@qq.com
 * @Description:ros消息客户端节点
 */
#include <cstdlib>

#include "beginner_tutorials/AddTwoInts.h"
#include "ros/ros.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "addTwoIntsClient");
  if (argc != 3) {
    ROS_INFO("Usage:add two ints client X Y");
    return -1;
  }
  ros::NodeHandle n;
  ros::ServiceClient client =
      n.serviceClient<beginner_tutorials::AddTwoInts>("addTwoInts");
  beginner_tutorials::AddTwoInts srv;
  srv.request.a = atoll(argv[1]);
  srv.request.b = atoll(argv[2]);

  if (client.call(srv)) {
    ROS_INFO("Sum:%1d", (long int)srv.response.sum);
  } else {
    ROS_ERROR("Failed to call service addTwoInts");
    return -1;
  }
  return 0;
}