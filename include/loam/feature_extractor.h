//
// Created by Zhang Zhimeng on 22-4-26.
//

#ifndef FUNNY_LIDAR_SLAM_FEATURE_EXTRACTOR_H
#define FUNNY_LIDAR_SLAM_FEATURE_EXTRACTOR_H

#include "common/data_type.h"
#include "lidar/pointcloud_cluster.h"

#include <pcl/filters/voxel_grid.h>

namespace loam {

class FeatureExtractor {
public:
    explicit FeatureExtractor(float corner_thr, float planar_thr,
                              int lidar_horizontal_scan, int lidar_vertical_scan);

    ~FeatureExtractor();

    void ExtractFeatures(PointcloudCluster &pointcloud_cluster);

private:
    void ComputeRoughness(const PointcloudCluster &pointcloud_cluster);

    void SelectValidPoints(const PointcloudCluster &pointcloud_cluster);

    void SelectFeatures(PointcloudCluster &pointcloud_cluster);

private:
    struct PointFeature {
        float roughness_ = std::numeric_limits<float>::lowest();
        unsigned int index_ = 0u;
    };

    std::vector<PointFeature> point_features_;
    float corner_threshold_;
    float planar_threshold_;
    bool *is_valid_points_; // true: valid point |  false: invalid point
    bool *is_corners_; // true: corner point  false: not corner point

    int lidar_horizontal_scan_;
    int lidar_vertical_scan_;
};

}

#endif //FUNNY_LIDAR_SLAM_FEATURE_EXTRACTOR_H
