//
// Created by Zhang Zhimeng on 23-10-15.
//

#ifndef FUNNY_LIDAR_SLAM_SENSOR_DATA_TYPE_H
#define FUNNY_LIDAR_SLAM_SENSOR_DATA_TYPE_H

#include "common/data_type.h"

struct DataBase {
    TimeStampUs timestamp_ = 0u; // us
};

struct IMUData : DataBase {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Vec3d angular_velocity_ = Vec3d::Zero();
    Vec3d linear_acceleration_ = Vec3d::Zero();
    Eigen::Quaterniond orientation_ = Eigen::Quaterniond::Identity();
};

struct GPSData : DataBase {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    double longitude_{0.0};
    double latitude_{0.0};
    double altitude_{0.0};

    Vec3d local_xyz_ = Vec3d::Zero();
    Vec3d local_xyz_velocity_ = Vec3d::Zero();
    Mat3d local_orientation = Mat3d::Identity();
};

struct NavStateData : DataBase {
    using Ptr = std::shared_ptr<NavStateData>;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void SetPose(const Mat4d& pose) {
        R_ = pose.block<3, 3>(0, 0);
        P_ = pose.block<3, 1>(0, 3);
    }

    [[nodiscard]] Mat4d Pose() const {
        Mat4d T = Mat4d::Identity();
        T.block<3, 3>(0, 0) = R_;
        T.block<3, 1>(0, 3) = P_;
        return T;
    }

    Vec3d P_ = Vec3d::Zero();
    Vec3d V_ = Vec3d::Zero();
    Mat3d R_ = Mat3d::Identity();

    Vec3d bg_ = Vec3d::Zero(); // gyro bias
    Vec3d ba_ = Vec3d::Zero(); // accel bias

    Mat15d info_ = Mat15d::Identity();
};

#endif //FUNNY_LIDAR_SLAM_SENSOR_DATA_TYPE_H
