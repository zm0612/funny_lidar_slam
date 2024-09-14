//
// Created by Zhang Zhimeng on 24-6-6.
//

#ifndef FUNNY_LIDAR_SLAM_COMPARE_FUNCTION_H
#define FUNNY_LIDAR_SLAM_COMPARE_FUNCTION_H

#include "data_type.h"

struct LessVec2i {
    bool operator()(const Eigen::Matrix<int, 2, 1> &v1,
                    const Eigen::Matrix<int, 2, 1> &v2) const {
        return v1[0] < v2[0] || (v1[0] == v2[0] && v1[1] < v2[1]);
    }
};

struct LessVec3i {
    bool operator()(const Vec3i &v1, const Vec3i &v2) const {
        return v1[0] < v2[0] || (v1[0] == v2[0] && v1[1] < v2[1]) ||
               (v1[0] == v2[0] && v1[1] == v2[1] && v1[2] < v2[2]);
    }
};

#endif //FUNNY_LIDAR_SLAM_COMPARE_FUNCTION_H
