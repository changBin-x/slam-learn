# 1.如何建立自己的模型

采用turtleBot3底盘+velodyne HDL32E雷达

组装方式参考[turtlebot3_simulation](https://github.com/ROBOTIS-GIT/turtlebot3_simulations)+[velodyne_simulator](https://bitbucket.org/DataspeedInc/velodyne_simulator.git)

仿真场景自建

建立完成的模型见[mybot_description](mybot_description/),有两种小车模型，[一种](mybot_description/urdf/mybot_burger.urdf.xacro)底盘更高，[一种](mybot_description/urdf/mybot_waffle.urdf.xacro)底盘更宽但是更低且加装有velodyne+RGBD摄像机，仿真时常用第二种

# 2.如何驱动小车

通过发布/cmd_vel话题，然后odom订阅，实时改变小车的速度和方向

# 3. [简单避障实现](mybot_gazebo/README.md)

[运行结果展示](mybot_gazebo/images/obstacle_avoidance.gif)

<div align="center">
<img src="mybot_gazebo/images/obstacle_avoidance.gif">
</div>

# 4. 三种类型的sensor_msgs

## 4.1 sensor_msgs/[LaserScan Message](http://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html)

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

 [两轮差速移动机器人运动分析、建模和控制](mybot_fake/README.md)

# 6. [TF2与URDF](mybot_fake/README.md)

学习TF消息和TF树