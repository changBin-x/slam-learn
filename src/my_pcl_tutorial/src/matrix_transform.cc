/*
 * @Date: 2020-11-16 16:00:35
 * @LastEditTime: 2020-11-18 15:42:24
 * @Author:  Chang_Bin
 * @LastEditors: Chang_Bin
 * @Email: bin_chang@qq.com
 * @Description:如何使用4x4矩阵变换点云。
 * 我们将旋转和平移应用于加载的点云，然后显示结果
 */

#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <ros/ros.h>

#include <iostream>

/**
 * 显示帮助的函数
 */
void showHelp(char* program_name) {
  std::cout << std::endl;
  std::cout << "Usage: " << program_name << " cloud_filename.[pcd|ply]"
            << std::endl;
  std::cout << "-h:  Show this help." << std::endl;
}

/**
 * 以下代码片段将为输入数据集中的所有点估计一组表面法线
 */
void estimateSurfaceNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  //创建正常的估计类，并将输入数据集传递给它
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(cloud);  //提供指向输入数据集的指针

  //创建一个空的kdtree表示形式，并将其传递给正常的估算对象。
  //它的内容将根据给定的输入数据集填充到对象内部（因为未提供其他搜索表面）。
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>());

  //输出的数据集
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(
      new pcl::PointCloud<pcl::Normal>);
  //使用半径3cm内的球体内的所有的紧邻点
  ne.setRadiusSearch(0.03);
  //计算特征
  ne.compute(*cloud_normals);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "matrix_transform");  // Show help
  ros::NodeHandle nh("~");

  if (pcl::console::find_switch(argc, argv, "-h") ||
      pcl::console::find_switch(argc, argv, "--help")) {
    showHelp(argv[0]);
    return 0;
  }
  //在参数中获取点云文件名| 适用于PCD和PLY文件
  std::vector<int> filenames;
  bool file_is_pcd = false;

  pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud(
      new pcl::PointCloud<pcl::PointXYZ>());

  if (pcl::io::loadPLYFile("/home/chang/catkin_ws/src/my_pcl_tutorial/data/"
                           "bunny/data/bun315.ply",
                           *sourceCloud) < 0) {
    ROS_ERROR("read ply file failed!");
    return -1;
  }
  //   ROS_INFO("source size=%d", sourceCloud->size());

  /* 提醒：变换矩阵的工作方式:

          |-------> 该列是平移
    | 1 0 0 x |  \
    | 0 1 0 y |   }-> The identity 3x3 matrix (no rotation) on the left
    | 0 0 1 z |  /
    | 0 0 0 1 |    -> We do not use this line (and it has to stay 0,0,0,1)

    METHOD #1: Using a Matrix4f
    This is the "manual" method, perfect to understand but error prone !
  */
  Eigen::Matrix4f transform1 = Eigen::Matrix4f::Identity();

  float theta = M_PI / 4;
  //绕x轴旋转
  transform1(0, 0) = std::cos(theta);
  transform1(0, 1) = -sin(theta);
  transform1(1, 0) = sin(theta);
  transform1(1, 1) = std::cos(theta);
  //沿x轴方向上的2.5m的平移
  transform1(0, 3) = 2.5;
  //打印转换
  std::cout << "Method #1: using a Matrix4f\n" << transform1 << std::endl;

  /*方式#2：使用Affine3f,这种方法更简单、更不容易出错*/
  Eigen::Affine3f transform2 = Eigen::Affine3f::Identity();
  //定义一个沿x轴方向的2.5m的平移
  transform2.translation() << 2.5, 0.0, 0.0;
  //绕z轴旋转theta角度
  transform2.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));
  //输出信息
  std::cout << "Method #2: using a Affine3f\n"
            << transform2.matrix() << std::endl;
  //执行转化
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  //把转换用于transform1和transform2都是一样的
  pcl::transformPointCloud(*sourceCloud, *transformedCloud, transform2);
  //可视化
  std::cout << "\nPoint cloud colors :  white  = original point cloud\n"
               "                        red  = transformed point cloud"
            << std::endl;
  pcl::visualization::PCLVisualizer viewer("Matrix transformation example");

  //为点云设定RGB颜色值
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
      sourceCloudColorHandler(sourceCloud, 255, 255, 255);
  //在viewer中加入点云数据并且传入颜色处理器
  viewer.addPointCloud(sourceCloud, sourceCloudColorHandler, "original_cloud");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
      transformedCloudColorHandler(sourceCloud, 230, 20, 20);
  viewer.addPointCloud(transformedCloud, transformedCloudColorHandler,
                       "transformed_cloud");
  viewer.addCoordinateSystem(1.0, "cloud", 0);
  viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);
  viewer.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
  viewer.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
  //按'q'退出点云展示界面
  while (!viewer.wasStopped()) {
    viewer.spinOnce();
  }
  return 0;
}