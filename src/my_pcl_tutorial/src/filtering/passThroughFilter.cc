/*
 * @Date: 2020-11-22 13:17:32
 * @LastEditTime: 2020-11-22 14:41:29
 * @Author:  Chang_Bin
 * @LastEditors: Chang_Bin
 * @Email: bin_chang@qq.com
 * @Description:直通滤波
 */

#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>

#include <iostream>

int main(int argc, char** argv) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(
      new pcl::PointCloud<pcl::PointXYZ>);

  //填充点云数据
  cloud->width = 5;
  cloud->height = 1;
  cloud->points.resize(cloud->height * cloud->width);

  for (auto& point : *cloud) {
    point.x = 1024 * rand() / (RAND_MAX + 1.0f);
    point.y = 1024 * rand() / (RAND_MAX + 1.0f);
    point.z = 1024 * rand() / (RAND_MAX + 1.0f);
  }

  std::cerr << "Cloud Before filtering:" << std::endl;
  for (const auto& point : *cloud)
    std::cerr << "   " << point.x << "  " << point.y << "  " << point.z
              << std::endl;

  //创建一个滤波对象
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud);
  // 过滤器字段名称设置为z坐标，可接受的间隔值设置为（0.0; 1.0）。
  //只保留点云z轴范围在(0.0,1.0)范围内的点云
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0, 1.0);
  // pass.setFilterLimitsNegative(true);
  pass.filter(*cloud_filtered);

  std::cerr << "cloud after filtering" << std::endl;
  for (const auto& point : *cloud_filtered)
    std::cerr << "   " << point.x << "  " << point.y << "  " << point.z
              << std::endl;
  return 0;
}

/********************* result ******************
 * Cloud Before filtering:
 *  0.352222  -0.151883  -0.106395
 *  -0.397406  -0.473106  0.292602
 *  -0.731898  0.667105  0.441304
 * -0.734766  0.854581  -0.0361733
 * -0.4607  -0.277468  -0.916762
 * cloud after filtering
 * -0.397406  -0.473106  0.292602
 * -0.731898  0.667105  0.441304
 ***********************************************/

/**************** result uncommenting ************
 * ************** pass.setFilterLimitsNegative ******
Cloud Before filtering:
   0.352222  -0.151883  -0.106395
   -0.397406  -0.473106  0.292602
   -0.731898  0.667105  0.441304
   -0.734766  0.854581  -0.0361733
   -0.4607  -0.277468  -0.916762
cloud after filtering
   0.352222  -0.151883  -0.106395
   -0.734766  0.854581  -0.0361733
   -0.4607  -0.277468  -0.916762
*********************************************/