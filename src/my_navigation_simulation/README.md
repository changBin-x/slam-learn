<!--
 * @Date: 2020-11-02 13:32:05
 * @LastEditTime: 2020-11-29 17:41:13
 * @Author:  Chang_Bin
 * @LastEditors: Chang_Bin
 * @Email: bin_chang@qq.com
 * @Description: 
-->
# 1.如何建立自己的模型

采用turtleBot3底盘+velodyne HDL32E雷达

组装方式参考[turtlebot3_simulation](https://github.com/ROBOTIS-GIT/turtlebot3_simulations)+[velodyne_simulator](https://bitbucket.org/DataspeedInc/velodyne_simulator.git)

仿真场景自建

建立完成的模型见[mybot_description](mybot_description/),有两种小车模型，[一种](mybot_description/urdf/mybot_burger.urdf.xacro)底盘更高，[一种](mybot_description/urdf/mybot_waffle.urdf.xacro)底盘更宽但是更低且加装有velodyne+RGBD摄像机，仿真时常用第二种

# 2.如何驱动小车

通过发布/cmd_vel话题，然后odom订阅，实时改变小车的速度和方向

# 3. [简单避障实现](mybot_gazebo/)

## 3.1 步骤

1. 获取雷达数据
   1. 订阅/velodyne_points话题，获取sensor_msgs::PointCloud2ConstPtr数据
   2. 把sensor_msgs::PointCloud2ConstPtr转换为pcl::PointCloud<pcl::PointXYZ>
2. 剪除部分点云
3. 根据保留的点云的x，y数据判断小车的行驶方向

<div align="center">
<img src=images/obstacle_avoidance.svg>
</div>

## 3.2 原理

* 首先剪除点云：小车底盘box的大小为[0.140 0.140 0.143],单位(cm)，运用PCL的直通滤波器保留x方向上[0.140, 3.0]范围内的点云，y方向上[-3.0, 3.0]范围内的点云，也即只保留小车前方的点云；
* 然后查找扫描到的x方向点云值的最小值xMin，记录xMin及对应的y_xMin；
* 当`xMin<escape_range_`(设定的避障阀值)时准备避障，也即是准备转弯；当`y_xMin<0`时，左转，当`y_xMin>0`时，右转；其余情况直行；

[代码](mybot_gazebo/src/mybot_drive.cc)

运行简单避障的例子的命令:
roslaunch mybot_gazebo [obstacle_avoidance.launch](mybot_gazebo/launch/obstacle_avoidance.launch)

[运行结果示例](images/obstacle_avoidance.gif)

<div align="center">
<img src="images/obstacle_avoidance.gif">
</div>

## 3.3 存在的问题

1. 只能实现简单的左右转避障；
2. 当`xMin<escape_range_`且小车-y和y方向上同时有障碍物时，转向会失灵，避障失败；

# 4. 三种类型的sensor_msgs

# 4.1 sensor_msgs/[LaserScan Message](http://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html)

原始数据定义:

```python
# 从平面(二维)激光测距仪(激光雷达)进行单次扫描存储的消息
#
# 如果您拥有另一台行为不同的设备 (e.g. 声呐
# 数据), 请查找或创建不同的消息, 因为该消息
# 只适用于特定的激光数据类型

Header header            # 标题中的时间戳是雷达获得第一束激光的时间
                         # 
                         #
                         # 在每一个帧号为id的数据帧中, 角度是绕z轴正向测量的 
                         # (z轴向上，角度为逆时针)，
                         # 沿x轴正向角度为0——右手螺旋
                         
float32 angle_min        # 开始扫描的角度 [rad]
float32 angle_max        # 结束扫描的角度 [rad]
float32 angle_increment  # 测量之间的角度增量 [rad]

float32 time_increment   # 相邻两次测量之间的时间 [seconds] - 如果雷达在移动
                         #  时间被用于3d点之间的插值得到的位置
                         
float32 scan_time        # 相邻两次扫描的时间 [seconds]

float32 range_min        # 测量范围最小值 [m]
float32 range_max        # 测量范围最大值 [m]

float32[] ranges         # 测量值 [m] (注意: values < range_min 或者 > range_max 被丢弃)
float32[] intensities    # 强度数据 [单位视设备而定].  
                         # 如果设备未提供强度值, 该数组置空
```

## 4.2 sensor_msgs/[PointCloud Message](http://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud.html)

原始数据定义

```python
# 此消息包含3d点的集合, 加上可选的每个点的附加信息

# 传感器数据获取时间，坐标帧ID。
Header header

# 3d点数组。每个Point32应该解释为在给定头部帧中的3d点
geometry_msgs/Point32[] points

# 每个通道应具有相同数量的元素作为点阵列，
# 每个通道中的数据应与每个点一一对应。
# ChannelFloat32.msg中列出了常用的频道名称。
ChannelFloat32[] channels
```

## 4.3 sensor_msgs/[PointCloud2 Message](http://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html)

原始消息定义

```python
# 此消息包含N维点的集合,消息中可能包含其他信息，例如法线，强度等
# 点数据存储为二进制块，点数据形式为“字段”数组

# 点云数据可以组织为2d（类似于图像）或1d(无序的)
# 2d形式的点云可能由深度相机传感器，如单目相机或飞行时间相机

# 传感器数据采集时间，坐标帧ID(对于3d点而言)
Header header

# 点云的2D结构。如果点云无序，那么高为1，宽为点云的长度(1维数据)
# 一般使用tof或者结构光原理的深度相机获取的点云是有序点云
# 无序点云一般是激光雷达其他设备获取的
uint32 height
uint32 width

#在二进制数据块中描述通道及其布局
PointField[] fields

bool    is_bigendian # 这是大端的数据吗？
uint32  point_step   # 点的长度（以字节为单位）
uint32  row_step     # 行的长度（以字节为单位）
uint8[] data         # 实际点数据，大小为（row_step*height）
bool is_dense        # 如果没有无效点，则为True
```

# 5. ROS小车的rviz仿真包[mybot_fake](mybot_fake/README.md)

mybot的假节点包。有了这个软件包，不需要机器人就可以完成简单的测试。

可以在没有真正的机器人的情况下在rviz上使用这个包进行简单的测试。

## 5.1 主要知识点

* [两轮差速移动机器人运动分析、建模和控制](https://blog.csdn.net/iProphet/article/details/83661753?utm_medium=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-3.channel_param&depth_1-utm_source=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-3.channel_param)