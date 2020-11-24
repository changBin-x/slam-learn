/*
 * @Date: 2020-11-22 17:05:50
 * @LastEditTime: 2020-11-22 17:49:40
 * @Author:  Chang_Bin
 * @LastEditors: Chang_Bin
 * @Email: bin_chang@qq.com
 * @Description:基于分段算法输出的索引从点云中提取点的子集
 */
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <iostream>

int main(int argc, char** argv) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_blob(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_blob(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(
      new pcl::PointCloud<pcl::PointXYZ>);

  //读取点云数据
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(
          "/home/chang/catkin_ws/src/my_pcl_tutorial/data/"
          "table_scene_lms400.pcd",
          *cloud_blob) < 0) {
    PCL_ERROR("load pcdFile failed");
    return -1;
  }
  std::cerr << "PointCloud before filtering Size:" << cloud_blob->size()
            << std::endl;

  //创建一个下采样滤波对象
  //   此处进行数据下采样的基本原理只是为了减少点数,加快处理速度
  //   意味着减少在细分循环中花费的时间。
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud_blob);
  //   然后，创建一个pcl::VoxelGrid过滤器，
  //   其叶子大小为1cm，传递输入数据，并计算输出并将其存储在cloud_filtered中。
  sor.setLeafSize(0.01f, 0.01f, 0.01f);
  sor.filter(*cloud_filtered_blob);
  std::cerr << "PointCloud after VoxelGrid filtering Size:"
            << cloud_filtered_blob->size() << std::endl;

  //创建模型系数表
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  //合群点
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  //创建分割对象
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  //使用系数优化分割对象
  seg.setOptimizeCoefficients(true);
  //分割的模型类型为平面
  seg.setModelType(pcl::SACMODEL_PLANE);
  //分割的方法类型为SAC_RANSAC
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(0.01);

  //创建滤波对象
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  int i = 0, nr_points = (int)cloud_filtered_blob->size();
  int points_size = nr_points;
  //   当原始云的30％仍然存在
  while (cloud_filtered_blob->size() > 0.3 * nr_points) {
    //   从剩余的云中分割出最大的平面分量
    seg.setInputCloud(cloud_filtered_blob);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0) {
      std::cerr << "无法估算给定数据集的平面模型" << std::endl;
      break;
    }
    //提取合群点
    // 代表实际索引
    // 为了处理多个模型，我们循环运行该过程，并在提取每个模型之后，返回以获取剩余点并进行迭代。
    //  从分割过程中获得内部值
    extract.setInputCloud(cloud_filtered_blob);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_p);
    std::cerr << "表示平面组件的PointCloud: "
              << cloud_p->width * cloud_p->height << " data points."
              << std::endl;

    extract.setNegative(true);
    extract.filter(*cloud_f);
    cloud_filtered_blob.swap(cloud_f);
    i++;
  }

  return 0;
}