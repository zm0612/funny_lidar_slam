#ifndef FUNNY_LIDAR_SLAM_VOXEL_GRID_MAP_HASH_FUNCTION_H
#define FUNNY_LIDAR_SLAM_VOXEL_GRID_MAP_HASH_FUNCTION_H

#include "common/data_type.h"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

struct SpatialHashFunction {
    inline size_t operator()(const Vec3i &v) const {
        return size_t(((v[0]) * 73856093) ^ ((v[1]) * 471943) ^ ((v[2]) * 83492791)) % 10000000;
    }
};

struct PlaneHashFunction {
    inline size_t operator()(const Vec2i &v) const {
        return size_t(((v[0] * 73856093) ^ (v[1] * 471943)) % 10000000);
    }
};

#endif
