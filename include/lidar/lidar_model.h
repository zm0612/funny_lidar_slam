//
// Created by Zhang Zhimeng on 22-4-27.
//

#ifndef FUNNY_LIDAR_SLAM_LIDAR_H
#define FUNNY_LIDAR_SLAM_LIDAR_H

#include "common/data_type.h"
#include "common/math_function.h"
#include "lidar/lidar_point_type.h"

#include <pcl/point_cloud.h>
#include <glog/logging.h>

#include <algorithm>
#include <iostream>
#include <cmath>
#include <mutex>

//#define USE_FAST_ATAN2

class LidarModel {
public:
    enum LidarSensorType {
        VELODYNE, OUSTER, LIVOX_AVIA, RoboSense, LeiShen, LIVOX_MID_360, None
    };
    LidarSensorType lidar_sensor_type_ = LidarSensorType::None;

    LidarModel() = delete;

    LidarModel(const LidarModel &) = delete;

    LidarModel(LidarModel &&) = delete;

    LidarModel &operator=(const LidarModel &) = delete;

    ~LidarModel() {
        delete instance_;
    }

    static LidarModel *Instance(const std::string &lidar_type = "");

    /*!
     * Compute ring(row) index
     * @param x
     * @param y
     * @param z
     * @return ring(row) index
     */
    [[nodiscard]] inline int RowIndex(const float &x, const float &y, const float &z) const {
        float xy = std::sqrt(x * x + y * y);

#ifndef USE_FAST_ATAN2
        const int row = static_cast<int>(std::round((std::atan2(z, xy) + lower_angle_) / v_res_));
#else
        const int row = static_cast<int>(std::round((FastAtan2(z, xy) + lower_angle_) / v_res_));
#endif

        return row;
    }

    /*!
     * 计算点对应的列 (-pi对应的列索引是0, 0度对应的索引是水平采样数量的一半)
     * @param x
     * @param y
     * @return column index
     */
    [[nodiscard]] inline int ColIndex(const float &x, const float &y) const {

#ifndef USE_FAST_ATAN2
        int col = static_cast<int>(std::round(std::atan2(y, x) / h_res_)) + horizon_scan_num_ / 2;
#else
        int col = static_cast<int>(std::round(FastAtan2(y, x) / h_res_)) + horizon_scan_num_ / 2;
#endif

        if (col >= horizon_scan_num_) {
            col -= horizon_scan_num_;
        }

        return col;
    }

private:
    explicit LidarModel(const std::string &lidar_type = "");

public:
    int vertical_scan_num_{std::numeric_limits<int>::max()}; // number of scans
    int horizon_scan_num_{std::numeric_limits<int>::max()}; // number of horizontal scans
    float h_res_{std::numeric_limits<float>::max()}; // horizontal resolution, radian
    float v_res_{std::numeric_limits<float>::max()}; // vertical resolution, radian
    float lower_angle_{std::numeric_limits<float>::max()}; // lidar minimum detection angle(abs), radian

private:
    static LidarModel *instance_;
    static std::string lidar_type_;
};

#endif //FUNNY_LIDAR_SLAM_LIDAR_H