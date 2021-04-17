/*
 * @Date: 2020-12-09 11:39:28
 * @LastEditTime: 2020-12-09 13:13:52
 * @Author:  Chang_Bin
 * @LastEditors: Chang_Bin
 * @Email: bin_chang@qq.com
 * @Description:发布状态
 */
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_ros/transform_broadcaster.h>

#include <string>

int main(int argc, char** argv) {
  ros::init(argc, argv, "state_publisher");
  ros::NodeHandle node;
  ros::Publisher joint_pub =
      node.advertise<sensor_msgs::JointState>("joint_states", 1);
  tf2_ros::TransformBroadcaster broadcaster;
  ros::Rate loop_rate(30);

  const double degree = M_PI / 180;

  //机器人状态
  double tilt = 0;  //倾斜
  double tinc = degree;
  double swivel = 0;    //旋转
  double angle = 0;     //角度
  double height = 0;    //高度
  double hinc = 0.005;  //中心

  //消息声明
  geometry_msgs::TransformStamped odom_trans;

  sensor_msgs::JointState joint_state;
  joint_state.name.resize(3);
  joint_state.name[0] = "swivel";
  joint_state.name[1] = "tilt";
  joint_state.name[2] = "periscope";
  joint_state.position.resize(3);

  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "axis";

  tf2::Quaternion q;
  while (node.ok()) {
    joint_state.header.stamp = ros::Time::now();
    joint_state.position[0] = swivel;
    joint_state.position[1] = tilt;
    joint_state.position[2] = height;

    //更新变换
    //在半径=2的圆圈内移动
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.transform.translation.x = cos(angle) * 2;
    odom_trans.transform.translation.y = sin(angle) * 2;
    odom_trans.transform.translation.z = .7;
    q.setRPY(0, 0, angle + M_PI / 2);
    odom_trans.transform.rotation.x = q.x();
    odom_trans.transform.rotation.y = q.y();
    odom_trans.transform.rotation.z = q.z();
    odom_trans.transform.rotation.w = q.w();

    joint_pub.publish(joint_state);
    broadcaster.sendTransform(odom_trans);

    //创建一个新的机器人状态
    tilt += tinc;
    if (tilt < -.5 || tilt > 0) tinc *= -1;
    height += hinc;
    if (height > .2 || height < 0) hinc *= -1;
    swivel += degree;
    angle += degree / 4;

    // This will adjust as needed per iteration
    loop_rate.sleep();
  }
  return 0;
}