//
// Created by Zhang Zhimeng on 24-3-31.
//

#ifndef FUNNY_LIDAR_SLAM_FRAME_H
#define FUNNY_LIDAR_SLAM_FRAME_H

#include "common/data_type.h"
#include "lidar/pointcloud_cluster.h"

struct Frame {
    using Ptr = std::shared_ptr<Frame>;

    TimeStampUs timestamp_ = 0u;
    Mat4d delta_pose_ = Mat4d::Identity(); // T_{i, i+1}
    PointcloudClusterPtr cloud_cluster_ptr_ = nullptr;
};

#endif //FUNNY_LIDAR_SLAM_FRAME_H
