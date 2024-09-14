//
// Created by Zhang Zhimeng on 22-5-11.
//

#ifndef FUNNY_LIDAR_SLAM_POINTCLOUD_PROJECTOR_H
#define FUNNY_LIDAR_SLAM_POINTCLOUD_PROJECTOR_H

#include "common/data_type.h"
#include "lidar/lidar_model.h"
#include "lidar/pointcloud_cluster.h"
#include "lidar/lidar_distortion_corrector.h"

namespace loam {

class PointcloudProjector {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PointcloudProjector() = delete;

    explicit PointcloudProjector(std::shared_ptr<LidarDistortionCorrector> lidar_distortion_corrector_ptr,
                                 int lidar_horizontal_scan, int lidar_vertical_scan, float lidar_horizontal_resolution,
                                 float min_distance, float max_distance);

    void Project(PointcloudCluster &pointcloud_cluster);

private:
    std::shared_ptr<LidarDistortionCorrector> lidar_distortion_corrector_ptr_ = nullptr;

    int lidar_vertical_scan_{}; // number of scans
    int lidar_horizontal_scan_{}; // number of horizontal scans
    float lidar_horizontal_resolution_{}; // horizontal resolution

    float max_distance_, min_distance_;

    MatXf range_mat_;
    PCLPointCloudXYZI temp_pointcloud_;
};

}
#endif //FUNNY_LIDAR_SLAM_POINTCLOUD_PROJECTOR_H