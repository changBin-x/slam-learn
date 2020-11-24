/*
 * @Date: 2020-11-22 15:45:50
 * @LastEditTime: 2020-11-22 16:38:47
 * @Author:  Chang_Bin
 * @LastEditors: Chang_Bin
 * @Email: bin_chang@qq.com
 * @Description:(统计离群值移除)过滤器
 */

#include <iostream>

#include "pcl/filters/statistical_outlier_removal.h"
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

  std::cerr << "PointCloud before filtering" << std::endl;
  std::cout << *cloud << std::endl;

  //创建一个滤波对象
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);
  //   每个点要分析的邻居数设置为50，标准差乘数设置为1
  //这意味着所有距离查询点的平均距离均大于1个标准差的所有点都将标记为
  //异常值并删除
  sor.setMeanK(50);
  sor.setStddevMulThresh(1.0);
  sor.filter(*cloud_filtered);

  std::cerr << "PointCloud after filtering" << std::endl;
  std::cout << *cloud_filtered << std::endl;

  //滤波结果保存
  pcl::io::savePCDFile<pcl::PointXYZ>(
      "/home/chang/catkin_ws/src/my_pcl_tutorial/data/"
      "table_scene_lms400_inliers.pcd",
      *cloud_filtered);

  showCloud(cloud, "initial cloud");
  showCloud(cloud_filtered, "filtered_inliers_cloud");
  return 0;
}