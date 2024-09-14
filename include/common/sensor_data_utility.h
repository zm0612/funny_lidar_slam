//
// Created by Zhang Zhimeng on 23-11-27.
//

#ifndef FUNNY_LIDAR_SLAM_SENSOR_DATA_UTILITY_H
#define FUNNY_LIDAR_SLAM_SENSOR_DATA_UTILITY_H

#include "common/sensor_data_type.h"

/*!
 * 通过标定外参对IMU数据进行变换
 * imu to lidar
 * @param imu_data
 * @param T
 */
inline void TransformImuData(IMUData &imu_data, const Mat4d &T) {
    imu_data.orientation_ = T.block<3, 3>(0, 0) * imu_data.orientation_;
    imu_data.linear_acceleration_ = T.block<3, 3>(0, 0) * imu_data.linear_acceleration_ + T.block<3, 1>(0, 3);
    imu_data.angular_velocity_ = T.block<3, 3>(0, 0) * imu_data.angular_velocity_ + T.block<3, 1>(0, 3);
}

#endif //FUNNY_LIDAR_SLAM_SENSOR_DATA_UTILITY_H
