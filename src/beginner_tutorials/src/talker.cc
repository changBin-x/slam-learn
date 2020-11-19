/*
 * @Date: 2020-10-28 16:50:50
 * @LastEditTime: 2020-10-28 17:14:18
 * @Author:  Chang_Bin
 * @LastEditors: Chang_Bin
 * @Email: bin_chang@qq.com
 * @Description: 第一个ROS话题发布者节点
 */

#include <sstream>

#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char** argv) {
  /* ros::init()需要传入argc和argv参数才能起作用
   * 允许通过命令行对节点名称进行重映射
   * 节点名字必须唯一
   */
  ros::init(argc, argv, "talker");

  /* 为这个进程的节点创建一个句柄。
  第一个创建的 NodeHandle 会为节点进行初始化，最后
  一个销毁的会清理节点使用的所有资源。 */
  ros::NodeHandle n;
  //     告诉 master 我们将要在 chatter topic 上发布一个 std_msgs/String
  //     的消息。这样 master
  // 就会告诉所有订阅了 chatter topic
  // 的节点，将要有数据发布。第二个参数是发布序列的大
  // 小。在这样的情况下，如果我们发布的消息太快，缓冲区中的消息在大于 1000
  // 个的时候就 会开始丢弃先前发布的消息。

  ros::Publisher cahtterPub = n.advertise<std_msgs::String>("chatter", 1000);

  /* ros::Rate 对象可以允许你指定自循环的频率。它会追踪记录自上一次调用
  Rate::sleep()后时间的流逝，并休眠直到一个频率周期的时间 */
  ros::Rate loopRate(100);

  int count = 0;
  while (ros::ok()) {
    std_msgs::String msg;
    std::stringstream ss;
    ss << "Hello ROS " << count;

    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    cahtterPub.publish(msg);

    ros::spinOnce();

    loopRate.sleep();
    ++count;
  }
  return 0;
}