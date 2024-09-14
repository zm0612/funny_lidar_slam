//
// Created by Zhang Zhimeng on 24-5-12.
//

#ifndef FUNNY_LIDAR_SLAM_IMU_INTERPOLATOR_H
#define FUNNY_LIDAR_SLAM_IMU_INTERPOLATOR_H

#include "common/data_type.h"
#include "common/sensor_data_type.h"
#include "common/motion_interpolator.h"

inline IMUData IMUInterpolator(const IMUData &imu_l, const IMUData &imu_r, TimeStampUs t) {
    CHECK_GE(t, imu_l.timestamp_);
    CHECK_LE(t, imu_r.timestamp_);

    IMUData imu_data;
    imu_data.linear_acceleration_ = MotionInterpolator::InterpolateVectorLerp(
            imu_l.linear_acceleration_, imu_r.linear_acceleration_,
            imu_l.timestamp_, imu_r.timestamp_, t
    );

    imu_data.angular_velocity_ = MotionInterpolator::InterpolateVectorLerp(
            imu_l.angular_velocity_, imu_r.angular_velocity_,
            imu_l.timestamp_, imu_r.timestamp_, t
    );

    imu_data.orientation_ = MotionInterpolator::InterpolateQuaternionSlerp(
            imu_l.orientation_, imu_r.orientation_,
            imu_l.timestamp_, imu_r.timestamp_, t
    );

    imu_data.timestamp_ = t;

    return imu_data;
}

#endif //FUNNY_LIDAR_SLAM_IMU_INTERPOLATOR_H
