/*
 * @Date: 2020-11-18 15:27:25
 * @LastEditTime: 2020-11-19 11:11:35
 * @Author:  Chang_Bin
 * @LastEditors: Chang_Bin
 * @Email: bin_chang@qq.com
 * @Description:
 */
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <boost/thread.hpp>
#include <iostream>
#include <string>

/**
 * 加载并显示PCD文件
 * return:加载的PCD数据
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr loadPCDfile(const std::string &pcdfile) {
  //创建PCL指针
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>());

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcdfile, *cloud) == -1) {
    // 读入PCD格式的文件，如果文件不存在，返回-1
    PCL_ERROR("Couldn't read file test_pcd.pcd \n");
    exit(-1);
  }

  return cloud;
}

void showCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
               const std::string &windowName) {
  //直接创造一个显示窗口
  pcl::visualization::CloudViewer viewer(windowName);
  viewer.showCloud(cloud);
  //按'q'退出点云展示界面

  while (!viewer.wasStopped()) {
  }
}

void estimateNormals(pcl::PointCloud<pcl::PointXYZ>::ConstPtr sourceCloud) {
  //创建法线的对象
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  //创建法线估计的对象
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
  normalEstimation.setInputCloud(sourceCloud);  //输入点云数据

  //对每一个点都用半径为3cm的近邻搜索
  normalEstimation.setRadiusSearch(0.03);
  // KD_tree是一种数据结构便于管理点云及搜索点云，法线估计对象会使用这种结构来找到最近邻点
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(
      new pcl::search::KdTree<pcl::PointXYZ>);
  normalEstimation.setSearchMethod(kdtree);

  //计算法线
  normalEstimation.compute(*normals);
  // 可视化
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
      new pcl::visualization::PCLVisualizer("Normals"));
  viewer->addPointCloud<pcl::PointXYZ>(sourceCloud, "cloud");
  viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(sourceCloud, normals,
                                                           20, 0.03, "normals");

  while (!viewer->wasStopped()) {
    viewer->spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(1000000));
  }
  // showCloud(sourceCloud, "Normals");
}

/**
 * 使用积分图像为有组织的点云计算法线。
 * 积分图像是对有序点云的发现的估计的一种方法。
 * 该算法把点云作为一个深度图像，并创建一定的矩形区域来计算法线，考虑到相邻像素关系，而无需建立树形查询结构。
 * 因此，它是非常有效的。
 */
void NormalEstimationUsingIntegralImages(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr sourceCloud) {
  //法线对象
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  //创建法线估计的对象
  pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal>
      normalEstimation;

  normalEstimation.setInputCloud(sourceCloud);

  // 法线估计方法: COVARIANCE_MATRIX, AVERAGE_DEPTH_CHANGE, SIMPLE_3D_GRADIENT.
  normalEstimation.setNormalEstimationMethod(
      normalEstimation.AVERAGE_3D_GRADIENT);
  //设置深度变换阈值
  normalEstimation.setMaxDepthChangeFactor(0.02f);
  //设置计算法线的区域
  normalEstimation.setNormalSmoothingSize(10.0f);
  //计算
  normalEstimation.compute(*normals);
  // 可视化
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
      new pcl::visualization::PCLVisualizer("Normals"));
  viewer->addPointCloud<pcl::PointXYZ>(sourceCloud, "cloud");
  viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(sourceCloud, normals,
                                                           20, 0.03, "normals");

  while (!viewer->wasStopped()) {
    viewer->spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(1000000));
  }
}

int main(int argc, char **argv) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud = loadPCDfile(
      "/home/chang/catkin_ws/src/my_pcl_tutorial/data/"
      "table_scene_mug_stereo_textured.pcd");

  // estimateNormals(sourceCloud);
  NormalEstimationUsingIntegralImages(sourceCloud);
}