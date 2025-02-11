//
// Created by Zhang Zhimeng on 22-5-30.
//

#ifndef FUNNY_LIDAR_SLAM_ROS_UTILITY_H
#define FUNNY_LIDAR_SLAM_ROS_UTILITY_H

#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "common/constant_variable.h"
#include "common/data_type.h"

namespace util {
template <typename T>
struct identity {
  typedef T type;
};

template <typename NodeT, typename T>
std::enable_if_t<std::is_pointer<NodeT>::value ||
                     std::is_same<NodeT, std::shared_ptr<rclcpp::Node>>::value,
                 void>
param(NodeT node, const std::string &param_name, T &param,
      const typename identity<T>::type &default_value) {
  node->declare_parameter(param_name, default_value);
  node->get_parameter(param_name, param);
  RCLCPP_INFO_STREAM(node->get_logger(), param_name << " = " << param);
}

template <typename NodeT, typename T>
std::enable_if_t<std::is_pointer<NodeT>::value ||
                     std::is_same<NodeT, std::shared_ptr<rclcpp::Node>>::value,
                 void>
param_vector(NodeT node, const std::string param_name, T &param,
             const typename identity<T>::type &default_value) {
  node->declare_parameter(param_name, default_value);
  node->get_parameter(param_name, param);
  for (int i = 0; i < param.size(); i++) {
    RCLCPP_INFO_STREAM(node->get_logger(), param_name << "[" << i << "]"
                                                      << " = " << param[i]);
  }
}

}  // namespace util

inline Vec3d RosVec3dToEigen(
    const geometry_msgs::msg::Vector3_<std::allocator<void>> &v) {
  return {v.x, v.y, v.z};
}

inline Vec3d RosPoint3dToEigen(
    const geometry_msgs::msg::Point_<std::allocator<void>> &p) {
  return {p.x, p.y, p.z};
}

inline Eigen::Quaterniond RosQuaternionToEigen(
    const geometry_msgs::msg::Quaternion_<std::allocator<void>> &q) {
  return {q.w, q.x, q.y, q.z};
}

inline uint64_t RosTimeToUs(
    const std_msgs::msg::Header_<std::allocator<void>> &header) {
  return header.stamp.sec * 1000000ul + header.stamp.nanosec / 1000ul;
}

inline void PublishRosCloud(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pub,
    const PCLPointCloudXYZI::Ptr &cloud) {
  if (pub->get_subscription_count() == 0) {
    return;
  }
  rclcpp::Clock clock;
  sensor_msgs::msg::PointCloud2 cloud_ros;
  pcl::toROSMsg(*cloud, cloud_ros);
  cloud_ros.header.frame_id = kRosMapFrameID;
  cloud_ros.header.stamp = clock.now();
  pub->publish(cloud_ros);
}

inline geometry_msgs::msg::TransformStamped eigen2Transform(
    const Eigen::Matrix3d &rot, const Eigen::Vector3d &pos,
    const std::string &frame_id, const std::string &child_frame_id,
    const double &timestamp) {
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = frame_id;
  transform.header.stamp = rclcpp::Time(timestamp);
  transform.child_frame_id = child_frame_id;
  transform.transform.translation.x = pos(0);
  transform.transform.translation.y = pos(1);
  transform.transform.translation.z = pos(2);
  Eigen::Quaterniond q = Eigen::Quaterniond(rot);
  transform.transform.rotation.w = q.w();
  transform.transform.rotation.x = q.x();
  transform.transform.rotation.y = q.y();
  transform.transform.rotation.z = q.z();
  return transform;
}

#endif  // FUNNY_LIDAR_SLAM_ROS_UTILITY_H
