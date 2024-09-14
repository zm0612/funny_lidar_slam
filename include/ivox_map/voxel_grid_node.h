#ifndef FUNNY_LIDAR_SLAM_VOXEL_GRID_MAP_VOXEL_GRID_NODE_H
#define FUNNY_LIDAR_SLAM_VOXEL_GRID_MAP_VOXEL_GRID_NODE_H

#include "common/data_type.h"
#include "common/math_function.h"

#include <pcl/common/centroid.h>
#include <algorithm>
#include <cmath>
#include <list>
#include <vector>

class VoxelGridNode {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    struct DistPoint {
        double dist = 0;
        VoxelGridNode *node = nullptr;
        int idx = 0;

        DistPoint() = default;

        DistPoint(const double d, VoxelGridNode *n, const int i) : dist(d), node(n), idx(i) {}

        [[nodiscard]] PCLPointXYZI Get() const { return node->GetPoint(idx); }

        inline bool operator()(const DistPoint &p1, const DistPoint &p2) { return p1.dist < p2.dist; }

        inline bool operator<(const DistPoint &rhs) const { return dist < rhs.dist; }
    };

    VoxelGridNode() = default;

    void InsertPoint(const PCLPointXYZI &pt);

    [[nodiscard]] bool Empty() const;

    [[nodiscard]] std::size_t Size() const;

    [[nodiscard]] PCLPointXYZI GetPoint(std::size_t idx) const;

    std::size_t KNNPointByCondition(std::vector<DistPoint> &dist_points,
                                    const PCLPointXYZI &point, std::size_t K, float max_range);


private:
    std::vector<PCLPointXYZI> points_{};
};

#endif