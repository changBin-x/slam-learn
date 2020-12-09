/*
 * @Date: 2020-12-08 15:09:45
 * @LastEditTime: 2020-12-08 15:18:33
 * @Author:  Chang_Bin
 * @LastEditors: Chang_Bin
 * @Email: bin_chang@qq.com
 * @Description:在tf2_ros::MessageFilter中使用Stamped数据类型
 */
#include <geometry_msgs/PointStamped.h>
#include <message_filters/subscriber.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

class PoseDrawer {
 private:
  std::string target_frame_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf2_;
  ros::NodeHandle n_;
  message_filters::Subscriber<geometry_msgs::PointStamped> point_sub_;
  tf2_ros::MessageFilter<geometry_msgs::PointStamped> tf2_filter_;

 public:
  PoseDrawer()
      : tf2_(buffer_),
        target_frame_("turtle1"),
        tf2_filter_(point_sub_, buffer_, target_frame_, 10, 0) {
    point_sub_.subscribe(n_, "turtle_point_stamped", 10);
    tf2_filter_.registerCallback(
        boost::bind(&PoseDrawer::msgCallback, this, _1));
  };
  void msgCallback(const geometry_msgs::PointStampedConstPtr& point_ptr) {
    geometry_msgs::PointStamped point_out;
    try {
      buffer_.transform(*point_ptr, point_out, target_frame_);

      ROS_INFO(
          "point of turtle 3 in frame of turtle 1 Position(x:%f y:%f z:%f)\n",
          point_out.point.x, point_out.point.y, point_out.point.z);
    } catch (tf2::TransformException& ex) {
      ROS_WARN("Failure %s\n", ex.what());  // Print exception which was caught
    }
  }
};
int main(int argc, char** argv) {
  ros::init(argc, argv, "pose_drawer");  // Init ROS
  PoseDrawer pd;                         // Construct class
  ros::spin();                           // Run until interupted
  return 0;
};