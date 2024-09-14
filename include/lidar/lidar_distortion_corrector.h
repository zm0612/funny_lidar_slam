//
// Created by Zhang Zhimeng on 22-8-31.
//

#ifndef FUNNY_LIDAR_SLAM_LIDAR_DISTORTION_CORRECTOR_H
#define FUNNY_LIDAR_SLAM_LIDAR_DISTORTION_CORRECTOR_H

#include "common/data_type.h"
#include "common/data_searcher.h"

class LidarDistortionCorrector {
public:
    explicit LidarDistortionCorrector(const Mat4d &T_lidar_to_imu);

    void SetDataSearcher(std::shared_ptr<DataSearcher<IMUData>> imu_data_searcher);

    /*!
     * Set reference data
     * @param ref_time us
     * @param imu_data_searcher
     * @return
     */
    bool SetRefTime(uint64_t ref_time);

    /*!
     *
     * @param x
     * @param y
     * @param z
     * @param x_corrected
     * @param y_corrected
     * @param z_corrected
     * @param relative_time s
     * @return
     */
    bool ProcessPoint(float x, float y, float z, float &x_corrected,
                      float &y_corrected, float &z_corrected, float relative_time);

private:
    uint64_t ref_time_ = 0u;

    Vec3d reference_t_;
    Eigen::Quaterniond q_ref_inv_;
    Mat4d T_lidar_to_imu_ = Mat4d::Identity();

    std::shared_ptr<DataSearcher<IMUData>> imu_data_searcher_ = nullptr;
};

#endif //FUNNY_LIDAR_SLAM_LIDAR_DISTORTION_CORRECTOR_H
