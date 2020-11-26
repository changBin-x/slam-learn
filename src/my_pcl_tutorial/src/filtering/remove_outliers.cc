/*
 * @Date: 2020-11-22 17:57:45
 * @LastEditTime: 2020-11-26 10:47:10
 * @Author:  Chang_Bin
 * @LastEditors: Chang_Bin
 * @Email: bin_chang@qq.com
 * @Description:使用条件或半径过滤器来删除离群值
 */
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>

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
  if (argc != 2) {
    std::cerr << "please specify command line arg '-r' or '-c'" << std::endl;
    exit(0);
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(
      new pcl::PointCloud<pcl::PointXYZ>);
  //读取点云数据
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(
          "/home/chang/catkin_ws/src/my_pcl_tutorial/data/"
          "2.pcd",
          *cloud) < 0) {
    PCL_ERROR("load pcdFile failed");
    return -1;
  }

  std::cerr << "PointCloud before filtering" << std::endl;
  std::cout << *cloud << std::endl;

  if (strcmp(argv[1], "-r") == 0) {
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    outrem.setInputCloud(cloud);
    outrem.setRadiusSearch(2);
    outrem.setMinNeighborsInRadius(5);
    outrem.setKeepOrganized(true);
    outrem.filter(*cloud_filtered);
    showCloud(cloud_filtered, "radius_cloud_filtered");
  } else if (strcmp(argv[1], "-c") == 0) {
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cloud(
        new pcl::ConditionAnd<pcl::PointXYZ>());
    range_cloud->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT,
                                                -1.4)));
    range_cloud->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT,
                                                0)));
    pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
    condrem.setCondition(range_cloud);
    condrem.setInputCloud(cloud);
    condrem.setKeepOrganized(true);
    condrem.filter(*cloud_filtered);
    showCloud(cloud_filtered, "condition_cloud_filtered");
  } else {
    std::cerr << "please specify command line arg '-r' or '-c'" << std::endl;
    exit(0);
  }

  std::cerr << "PointCloud after filtering" << std::endl;
  std::cout << *cloud_filtered << std::endl;

  showCloud(cloud, "Initial_cloud");

  return 0;
}