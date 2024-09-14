//
// Created by Zhang Zhimeng on 22-5-30.
//

#ifndef FUNNY_LIDAR_SLAM_ROS_UTILITY_H
#define FUNNY_LIDAR_SLAM_ROS_UTILITY_H

#include "common/data_type.h"
#include "common/constant_variable.h"

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>

#include <vector>
#include <Eigen/Dense>
#include <pcl_conversions/pcl_conversions.h>

inline Vec3d RosVec3dToEigen(const geometry_msgs::Vector3_<std::allocator<void>> &v) {
    return {v.x, v.y, v.z};
}

inline Vec3d RosPoint3dToEigen(const geometry_msgs::Point_<std::allocator<void>> &p) {
    return {p.x, p.y, p.z};
}

inline Eigen::Quaterniond RosQuaternionToEigen(const geometry_msgs::Quaternion_<std::allocator<void>> &q) {
    return {q.w, q.x, q.y, q.z};
}

inline uint64_t RosTimeToUs(const std_msgs::Header_<std::allocator<void>> &header) {
    return header.stamp.sec * 1000000ul + header.stamp.nsec / 1000ul;
}

inline void PublishRosCloud(ros::Publisher &pub, const PCLPointCloudXYZI::Ptr &cloud) {
    if (pub.getNumSubscribers() == 0) {
        return;
    }

    sensor_msgs::PointCloud2 cloud_ros;
    pcl::toROSMsg(*cloud, cloud_ros);
    cloud_ros.header.frame_id = kRosMapFrameID;
    pub.publish(cloud_ros);
}

#endif //FUNNY_LIDAR_SLAM_ROS_UTILITY_H
