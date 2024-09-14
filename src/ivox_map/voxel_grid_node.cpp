//
// Created by Zhang Zhimeng on 23-12-13.
//
#include "ivox_map/voxel_grid_node.h"
#include "common/pointcloud_utility.h"

void VoxelGridNode::InsertPoint(const PCLPointXYZI &pt) {
    points_.template emplace_back(pt);
}

bool VoxelGridNode::Empty() const {
    return points_.empty();
}

std::size_t VoxelGridNode::Size() const {
    return points_.size();
}

PCLPointXYZI VoxelGridNode::GetPoint(const std::size_t idx) const {
    return points_[idx];
}

std::size_t VoxelGridNode::KNNPointByCondition(std::vector<DistPoint> &dist_points, const PCLPointXYZI &point,
                                               std::size_t K, float max_range = 5.0) {
    std::size_t old_size = dist_points.size();
    for (const auto &pt: points_) {
        double d = DistanceSquared(pt, point);
        if (d < max_range * max_range) {
            dist_points.emplace_back(d, this, &pt - points_.data());
        }
    }

    if (old_size + K >= dist_points.size()) {
    } else {
        std::nth_element(dist_points.begin() + static_cast<int>(old_size),
                         dist_points.begin() + static_cast<int>(old_size) + K - 1,
                         dist_points.end());
        dist_points.resize(old_size + K);
    }

    return dist_points.size();
}