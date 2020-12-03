/*
 * @Date: 2020-12-01 12:56:23
 * @LastEditTime: 2020-12-03 16:50:49
 * @Author:  Chang_Bin
 * @LastEditors: Chang_Bin
 * @Email: bin_chang@qq.com
 * @Description:障碍物检测，遇到障碍物停止
 */

#include "obstacle_block.h"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "mybot_obstacle_node");
  ObstacleBlock obstacleblock;
  ros::Rate loop_rate(100);

  while (ros::ok()) {
    obstacleblock.obstacle();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
