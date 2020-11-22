/*
 * @Date: 2020-11-22 14:58:47
 * @LastEditTime: 2020-11-22 15:20:40
 * @Author:  Chang_Bin
 * @LastEditors: Chang_Bin
 * @Email: bin_chang@qq.com
 * @Description:体素网格下采样
 */

#include "pcl/filters/voxel_grid.h"

#include <iostream>

#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/visualization/cloud_viewer.h"
#include "pcl/visualization/pcl_visualizer.h"

/**
 * 显示点云数据
 */
void showCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
               const std::string& windowName) {
  //直接创造一个显示窗口
  pcl::visualization::CloudViewer viewer(windowName);
  viewer.showCloud(cloud);
  //按'q'退出点云展示界面

  while (!viewer.wasStopped()) {
  }
}

int main(int argc, char** argv) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(
      new pcl::PointCloud<pcl::PointXYZ>);
  //读取点云数据
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(
          "/home/chang/catkin_ws/src/my_pcl_tutorial/data/"
          "table_scene_lms400.pcd",
          *cloud) < 0) {
    PCL_ERROR("load pcdFile failed");
    return -1;
  }

  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
            << " data points (" << pcl::getFieldsList(*cloud) << ")."
            << std::endl;

  //创建一个滤波对象
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);
  //   然后，创建一个pcl::VoxelGrid过滤器，
  //   其叶子大小为1cm，传递输入数据，并计算输出并将其存储在cloud_filtered中。
  sor.setLeafSize(0.01f, 0.01f, 0.01f);
  sor.filter(*cloud_filtered);

  std::cerr << "PointCloud after filtering: "
            << cloud_filtered->width * cloud_filtered->height
            << " data points (" << pcl::getFieldsList(*cloud_filtered) << ")."
            << std::endl;
  //滤波结果保存
  pcl::io::savePCDFile<pcl::PointXYZ>(
      "/home/chang/catkin_ws/src/my_pcl_tutorial/data/"
      "table_scene_lms400_downsampled.pcd",
      *cloud_filtered);

  showCloud(cloud, "initial cloud");
  showCloud(cloud_filtered, "filtered cloud");

  return 0;
}