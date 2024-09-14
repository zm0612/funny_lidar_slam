//
// Created by Zhang Zhimeng on 22-4-28.
//

#ifndef FUNNY_LIDAR_SLAM_POINTCLOUD_CLUSTER_H
#define FUNNY_LIDAR_SLAM_POINTCLOUD_CLUSTER_H

#include "common/data_type.h"
#include "common/sensor_data_type.h"
#include "lidar/lidar_point_type.h"
#include "lidar/lidar_model.h"

struct PointcloudCluster {
    pcl::PointCloud<PointXYZIRT> raw_cloud_; // lidar frame
    PCLPointCloudXYZI ordered_cloud_; //(undistorted) or (undistorted and ordered) imu frame
    PCLPointCloudXYZI corner_cloud_; // imu frame
    PCLPointCloudXYZI planar_cloud_; // imu frame

    std::vector<float> point_depth_vec_;
    std::vector<int> row_start_index_vec_;
    std::vector<int> row_end_index_vec_;
    std::vector<int> point_col_index_vec_;

    std::vector<IMUData> imu_data_;

    uint64_t timestamp_ = 0; //us

    PointcloudCluster() = default;

    PointcloudCluster(const PointcloudCluster& pointcloud_cluster) noexcept {
        raw_cloud_ = pointcloud_cluster.raw_cloud_;
        ordered_cloud_ = pointcloud_cluster.ordered_cloud_;
        corner_cloud_ = pointcloud_cluster.corner_cloud_;
        planar_cloud_ = pointcloud_cluster.planar_cloud_;

        point_col_index_vec_ = pointcloud_cluster.point_col_index_vec_;
        point_depth_vec_ = pointcloud_cluster.point_depth_vec_;
        row_end_index_vec_ = pointcloud_cluster.row_end_index_vec_;
        row_start_index_vec_ = pointcloud_cluster.row_start_index_vec_;

        timestamp_ = pointcloud_cluster.timestamp_;
    }

    PointcloudCluster& operator=(const PointcloudCluster& pointcloud_cluster) noexcept {
        if (this == &pointcloud_cluster) {
            return *this;
        }

        raw_cloud_ = pointcloud_cluster.raw_cloud_;
        ordered_cloud_ = pointcloud_cluster.ordered_cloud_;
        corner_cloud_ = pointcloud_cluster.corner_cloud_;
        planar_cloud_ = pointcloud_cluster.planar_cloud_;

        point_col_index_vec_ = pointcloud_cluster.point_col_index_vec_;
        point_depth_vec_ = pointcloud_cluster.point_depth_vec_;
        row_end_index_vec_ = pointcloud_cluster.row_end_index_vec_;
        row_start_index_vec_ = pointcloud_cluster.row_start_index_vec_;

        timestamp_ = pointcloud_cluster.timestamp_;

        return *this;
    }

    PointcloudCluster(PointcloudCluster&& pointcloud_cluster) noexcept {
        point_col_index_vec_ = std::move(pointcloud_cluster.point_col_index_vec_);
        point_depth_vec_ = std::move(pointcloud_cluster.point_depth_vec_);
        row_end_index_vec_ = std::move(pointcloud_cluster.row_end_index_vec_);
        row_start_index_vec_ = std::move(pointcloud_cluster.row_start_index_vec_);

        raw_cloud_ = std::move(pointcloud_cluster.raw_cloud_);
        ordered_cloud_ = std::move(pointcloud_cluster.ordered_cloud_);
        corner_cloud_ = std::move(pointcloud_cluster.corner_cloud_);
        planar_cloud_ = std::move(pointcloud_cluster.planar_cloud_);

        timestamp_ = pointcloud_cluster.timestamp_;
    }


    void Clear() {
        raw_cloud_.clear();
        ordered_cloud_.clear();
        planar_cloud_.clear();
        corner_cloud_.clear();
    }
};

typedef std::shared_ptr<PointcloudCluster> PointcloudClusterPtr;

#endif //FUNNY_LIDAR_SLAM_POINTCLOUD_CLUSTER_H
