/*
 * @Date: 2020-11-22 16:45:29
 * @LastEditTime: 2020-11-22 17:01:58
 * @Author:  Chang_Bin
 * @LastEditors: Chang_Bin
 * @Email: bin_chang@qq.com
 * @Description:使用参数模型投影点
 */

#include "pcl/filters/project_inliers.h"

#include <iostream>

#include "pcl/ModelCoefficients.h"
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

  //创建一组平面系数，其中X = Y = 0，Z = 1
  //   我们填写ModelCoefficients值。 在这种情况下，我们使用一个平面模型，
  //   其中ax+ by + cz + d = 0，
  //   其中a = b = d = 0，c =1，或者换句话说，X-Y平面。
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  coefficients->values.resize(4);
  coefficients->values[0] = coefficients->values[1] = 0;
  coefficients->values[2] = 1.0;
  coefficients->values[3] = 0;

  //   我们创建ProjectInliers对象，
  //   并将上面定义的ModelCoefficients用作要投影到的模型
  pcl::ProjectInliers<pcl::PointXYZ> sor;
  //模型类型为一个平面
  sor.setModelType(pcl::SACMODEL_PLANE);
  sor.setInputCloud(cloud);
  sor.setModelCoefficients(coefficients);
  sor.filter(*cloud_filtered);

  std::cerr << "PointCloud after filtering" << std::endl;
  std::cout << *cloud_filtered << std::endl;
  //滤波结果保存
  pcl::io::savePCDFile<pcl::PointXYZ>(
      "/home/chang/catkin_ws/src/my_pcl_tutorial/data/"
      "table_scene_lms400_projectInliers.pcd",
      *cloud_filtered);

  showCloud(cloud, "initial cloud");
  showCloud(cloud_filtered, "projectInliers_cloud");
  return 0;
}