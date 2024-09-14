//
// Created by Zhang Zhimeng on 24-2-6.
//

#ifndef FUNNY_LIDAR_SLAM_KEYFRAME_H
#define FUNNY_LIDAR_SLAM_KEYFRAME_H

#include "common/timer.h"
#include "common/data_type.h"
#include "lidar/pointcloud_cluster.h"

#include <pcl/io/pcd_io.h>

struct KeyFrame {
    using Ptr = std::shared_ptr<KeyFrame>;
    using ID = int;
    static constexpr ID kInvalidID = -1;

    ID id_ = kInvalidID;
    TimeStampUs timestamp_ = 0u;
    Mat4d pose_ = Mat4d::Identity();
    PointcloudClusterPtr cloud_cluster_ptr_ = nullptr;

    [[nodiscard]] PCLPointCloudXYZIRT::Ptr LoadRawCloud() const {
        CHECK_NE(id_, kInvalidID);
        PCLPointCloudXYZIRT::Ptr cloud(new PCLPointCloudXYZIRT);
        pcl::io::loadPCDFile(kRawCloudPath + std::to_string(id_) + ".pcd", *cloud);
        return cloud;
    }

    [[nodiscard]] PCLPointCloudXYZI::Ptr LoadOrderedCloud() const {
        CHECK_NE(id_, kInvalidID);
        PCLPointCloudXYZI::Ptr cloud(new PCLPointCloudXYZI);
        pcl::io::loadPCDFile(kOrderedCloudPath + std::to_string(id_) + ".pcd", *cloud);
        return cloud;
    }

    [[nodiscard]] PCLPointCloudXYZI::Ptr LoadPlanarCloud() const {
        CHECK_NE(id_, kInvalidID);
        PCLPointCloudXYZI::Ptr cloud(new PCLPointCloudXYZI);
        pcl::io::loadPCDFile(kPlanarCloudPath + std::to_string(id_) + ".pcd", *cloud);
        return cloud;
    }

    [[nodiscard]] PCLPointCloudXYZI::Ptr LoadCornerCloud() const {
        CHECK_NE(id_, kInvalidID);
        PCLPointCloudXYZI::Ptr cloud(new PCLPointCloudXYZI);
        pcl::io::loadPCDFile(kCornerCloudPath + std::to_string(id_) + ".pcd", *cloud);
        return cloud;
    }

    void SaveAllCloud() {
        Timer timer;
        if (!cloud_cluster_ptr_) {
            LOG(ERROR) << "Cloud cluster nullptr";
        }

        if (!cloud_cluster_ptr_->raw_cloud_.empty()) {
            pcl::io::savePCDFileBinary(kRawCloudPath + std::to_string(id_) + ".pcd",
                                       cloud_cluster_ptr_->raw_cloud_);
        }

        if (!cloud_cluster_ptr_->ordered_cloud_.empty()) {
            pcl::io::savePCDFileBinary(kOrderedCloudPath + std::to_string(id_) + ".pcd",
                                       cloud_cluster_ptr_->ordered_cloud_);
        }

        if (!cloud_cluster_ptr_->corner_cloud_.empty()) {
            pcl::io::savePCDFileBinary(kCornerCloudPath + std::to_string(id_) + ".pcd",
                                       cloud_cluster_ptr_->corner_cloud_);
        }

        if (!cloud_cluster_ptr_->planar_cloud_.empty()) {
            pcl::io::savePCDFileBinary(kPlanarCloudPath + std::to_string(id_) + ".pcd",
                                       cloud_cluster_ptr_->planar_cloud_);
        }

        /*
        std::ofstream pose_file(kPosePath + std::to_string(id_) + ".txt", std::ios::trunc);
        const Eigen::Quaterniond q(pose_.block<3, 3>(0, 0));
        const Vec3d t(pose_.block<3, 1>(0, 3));
        pose_file << timestamp_ << " " << std::setprecision(15) << t.x() << " " << t.y() << " "
                  << t.z() << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
        */

        DLOG(INFO) << "Save All KeyFrame Cloud use time(ms):" << timer.End();
    }

    static const std::string kKeyFramePath;
    static const std::string kRawCloudPath;
    static const std::string kPlanarCloudPath;
    static const std::string kCornerCloudPath;
    static const std::string kOrderedCloudPath;
    static const std::string kPosePath;
};

#endif //FUNNY_LIDAR_SLAM_KEYFRAME_H
