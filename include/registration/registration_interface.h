//
// Created by Zhang Zhimeng on 23-12-14.
//

#ifndef FUNNY_LIDAR_SLAM_REGISTRATION_INTERFACE_H
#define FUNNY_LIDAR_SLAM_REGISTRATION_INTERFACE_H

#include "common/data_type.h"
#include "lidar/pointcloud_cluster.h"

class RegistrationInterface {
public:
    virtual bool Match(const PointcloudClusterPtr& source_cloud_cluster, Mat4d& T) = 0;

    virtual ~RegistrationInterface() = default;

    virtual void AddCloudToLocalMap(const std::initializer_list<PCLPointCloudXYZI>& cloud_list) = 0;

    [[nodiscard]] virtual float GetFitnessScore(float max_range) const = 0;
};

#endif //FUNNY_LIDAR_SLAM_REGISTRATION_INTERFACE_H
