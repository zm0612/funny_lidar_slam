//
// Created by Zhang Zhimeng on 23-10-29.
//

#ifndef FUNNY_LIDAR_SLAM_SAVE_FILE_H
#define FUNNY_LIDAR_SLAM_SAVE_FILE_H

#include "common/data_type.h"

#include <fstream>

inline void SaveTumPose(std::ofstream &file, const Mat4d &pose, uint64_t timestamp) {
    Eigen::Quaterniond q(pose.block<3, 3>(0, 0));
    Eigen::Vector3d t(pose.block<3, 1>(0, 3));

    file << std::setprecision(15) << timestamp / 1.0e6 << " "
         << t.x() << " " << t.y() << " " << t.z() << " "
         << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
}

#endif //FUNNY_LIDAR_SLAM_SAVE_FILE_H
