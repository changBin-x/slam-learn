# mybot模型

# 1.仿真场景

自建或由[turtlebot3的仿真场景](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/master/turtlebot3_gazebo)改建
# 2.实现简单的避障

## 2.1 步骤

1. 获取雷达数据
   1. 订阅/velodyne_points话题，获取sensor_msgs::PointCloud2ConstPtr数据
   2. 把sensor_msgs::PointCloud2ConstPtr转换为pcl::PointCloud<pcl::PointXYZ>
2. 剪除部分点云
3. 根据保留的点云的x，y数据判断小车的行驶方向

<div align="center">
<img src=images/obstacle_avoidance.svg>
</div>

## 2.2 原理

* 首先剪除点云：小车底盘box的大小为[0.140 0.140 0.143],单位(cm)，运用PCL的直通滤波器保留x方向上[0.140, 3.0]范围内的点云，y方向上[-3.0, 3.0]范围内的点云，也即只保留小车前方的点云；
* 然后查找扫描到的x方向点云值的最小值xMin，记录xMin及对应的y_xMin；
* 当`xMin<escape_range_`(设定的避障阀值)时准备避障，也即是准备转弯；当`y_xMin<0`时，左转，当`y_xMin>0`时，右转；其余情况直行；

[代码](src/mybot_drive.cc)

运行简单避障的例子的命令:
roslaunch mybot_gazebo [obstacle_avoidance.launch](launch/obstacle_avoidance.launch)

[运行结果示例](images/obstacle_avoidance.gif)

<div align="center">
<img src="images/obstacle_avoidance.gif">
</div>

## 2.3 存在的问题

1. 只能实现简单的左右转避障；
2. 当`xMin<escape_range_`且小车-y和y方向上同时有障碍物时，转向会失灵，避障失败；