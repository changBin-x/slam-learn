/*
 * @Date: 2020-11-28 15:37:09
 * @LastEditTime: 2020-11-28 15:42:13
 * @Author:  Chang_Bin
 * @LastEditors: Chang_Bin
 * @Email: bin_chang@qq.com
 * @Description:使用KdTree进行搜索
 */

/******************************************
 * 理论准备:
 * k维树是计算机科学中使用的一种数据结构，用于组织k维空间中的一些点。
 * 它是一个二叉搜索树，上面有其他约束。
 * K-d树对于范围搜索和最近邻居搜索非常有用。
 * 我们通常只处理三维空间的点云，因此我们所有的k-d树都是三维空间的。
 *  k-d树的每个级别都使用垂直于相应轴的超平面沿特定维度拆分所有子级。
 *  在树的根部，所有子项都将根据第一维进行拆分
 * （即，如果第一维坐标小于根，则它将在左子树中；如果大于根，则显然会在
 *右边的子树）。
 * 树中向下的每个级别都在下一个维度上划分，其他所有元素都用尽后，将返回到第一个维度。
 * 构造k-d树的最有效方法是使用一种分区方法，例如快速排序所使用的分区方法，将中点放置在根上，
 * 所有具有较小一维值的事物都放置在根部，而左侧则是较大的。
 *  然后，在左右两个子树上都重复此过程，直到要分区的最后一棵树仅由一个元素组成。
 *********************************************/

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>

#include <ctime>
#include <iostream>
#include <vector>

int main(int argc, char** argv) {
  srand(time(NULL));

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  // 产生点云数据
  cloud->width = 1000;
  cloud->height = 1;
  cloud->points.resize(cloud->width * cloud->height);

  for (std::size_t i = 0; i < cloud->size(); ++i) {
    (*cloud)[i].x = 1024.0f * rand() / (RAND_MAX + 1.0f);
    (*cloud)[i].y = 1024.0f * rand() / (RAND_MAX + 1.0f);
    (*cloud)[i].z = 1024.0f * rand() / (RAND_MAX + 1.0f);
  }

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

  kdtree.setInputCloud(cloud);

  pcl::PointXYZ searchPoint;

  searchPoint.x = 1024.0f * rand() / (RAND_MAX + 1.0f);
  searchPoint.y = 1024.0f * rand() / (RAND_MAX + 1.0f);
  searchPoint.z = 1024.0f * rand() / (RAND_MAX + 1.0f);

  // K 近邻搜索

  int K = 10;

  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);

  std::cout << "K nearest neighbor search at (" << searchPoint.x << " "
            << searchPoint.y << " " << searchPoint.z << ") with K=" << K
            << std::endl;

  if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch,
                            pointNKNSquaredDistance) > 0) {
    for (std::size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
      std::cout << "    " << (*cloud)[pointIdxNKNSearch[i]].x << " "
                << (*cloud)[pointIdxNKNSearch[i]].y << " "
                << (*cloud)[pointIdxNKNSearch[i]].z
                << " (squared distance: " << pointNKNSquaredDistance[i] << ")"
                << std::endl;
  }

  // 半径近邻搜索

  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

  float radius = 256.0f * rand() / (RAND_MAX + 1.0f);

  std::cout << "Neighbors within radius search at (" << searchPoint.x << " "
            << searchPoint.y << " " << searchPoint.z
            << ") with radius=" << radius << std::endl;

  if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch,
                          pointRadiusSquaredDistance) > 0) {
    for (std::size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
      std::cout << "    " << (*cloud)[pointIdxRadiusSearch[i]].x << " "
                << (*cloud)[pointIdxRadiusSearch[i]].y << " "
                << (*cloud)[pointIdxRadiusSearch[i]].z
                << " (squared distance: " << pointRadiusSquaredDistance[i]
                << ")" << std::endl;
  }

  return 0;
}