//
// Created by Zhang Zhimeng on 24-6-2.
//

#ifndef FUNNY_LIDAR_SLAM_SPLIT_MAP_H
#define FUNNY_LIDAR_SLAM_SPLIT_MAP_H

#include "common/data_type.h"
#include "common/file_manager.h"
#include "common/pointcloud_utility.h"

#define TILE_MAP_NAME(index) (std::to_string((index).x()) + "_" \
                            + std::to_string((index).y()) + ".pcd")

class SplitMap {
public:
    explicit SplitMap(double grid_size = 100.0);

    void SetTileMapGridSize(double grid_size = 100.0);

    void Split(const PCLPointCloudXYZI &map_cloud) const;

private:
    struct LessGridIndex {
        bool operator()(const Vec2i &v1,
                        const Vec2i &v2) const {
            return v1[0] < v2[0] || (v1[0] == v2[0] && v1[1] < v2[1]);
        }
    };

private:
    double grid_size_;
    double grid_half_size_;
};

#endif //FUNNY_LIDAR_SLAM_SPLIT_MAP_H
