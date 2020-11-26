<!--
 * @Date: 2020-11-02 13:32:05
 * @LastEditTime: 2020-11-26 17:01:30
 * @Author:  Chang_Bin
 * @LastEditors: Chang_Bin
 * @Email: bin_chang@qq.com
 * @Description: 
-->
# 1.如何建立自己的模型

采用turtleBot3底盘+velodyne HDL32E雷达

组装方式参考[turtlebot3_simulation](https://github.com/ROBOTIS-GIT/turtlebot3_simulations)+[velodyne_simulator](https://bitbucket.org/DataspeedInc/velodyne_simulator.git)

仿真场景自建

# 2.如何驱动小车

通过发布/cmd_vel话题，然后odom订阅，实时改变小车的速度和方向

# 3. 简单避障

1. 获取雷达数据
   1. 订阅/velodyne_points话题，获取sensor_msgs::PointCloud2ConstPtr数据
   2. 把sensor_msgs::PointCloud2ConstPtr转换为pcl::PointCloud<pcl::PointXYZ>
2. 剪除部分点云
3. 根据保留的点云的x，y数据判断小车的行驶方向

[代码](mybot_gazebo/src/mybot_drive.cc)

运行简单避障的例子:[roslaunch mybot_gazebo obstacle_avoidance.launch](mybot_gazebo/launch/obstacle_avoidance.launch)