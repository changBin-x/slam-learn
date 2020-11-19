/*
 * @Date: 2020-11-15 13:10:10
 * @LastEditTime: 2020-11-18 15:23:04
 * @Author:  Chang_Bin
 * @LastEditors: Chang_Bin
 * @Email: bin_chang@qq.com
 * @Description:pcl库的用法
 */
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <vector>

#include "pcl/features/normal_3d.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"

ros::Publisher pub;

//点云回调函数
void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloudMsg) {
  pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2);
  // sensorMsg转pcl数据类型
  pcl_conversions::toPCL(*cloudMsg, *cloud);
  //宽
  //* 它可以为无组织的数据集指定云中的点总数（等于点中的元素数–参见下文）；
  int cloudWidth = cloud->width;
  int cloudHeight = cloud->height;
  ROS_INFO("cloudWidth:%d,cloudHeight:%d", cloudWidth, cloudHeight);

  //对于包含XYZ数据的云，点包含pcl :: PointXYZ元素的向量
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*cloudMsg, *cloudXYZ);
  // std::vector<pcl::PointXYZ> pointData = cloudXYZ->points;
}

int main(int argc, char** argv) {
  //初始化ROS
  ros::init(argc, argv, "example");
  ros::NodeHandle nh;

  //订阅点云输入
  ros::Subscriber sub = nh.subscribe("/velodyne_points", 1, cloud_callback);
  //发布处理后的ros点云
  //   pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);

  ros::spin();
}