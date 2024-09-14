//
// Created by Zhang Zhimeng on 23-10-14.
//

#ifndef FUNNY_LIDAR_SLAM_PRE_INTEGRATION_H
#define FUNNY_LIDAR_SLAM_PRE_INTEGRATION_H

#include "common/data_type.h"
#include "common/math_function.h"
#include "common/sensor_data_type.h"

#include <glog/logging.h>

#include <deque>
#include <mutex>

class PreIntegration {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    struct ConfigPara {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Vec3d init_gyro_bias_ = Vec3d::Zero(); // init gyro bias
        Vec3d init_acc_bias_ = Vec3d::Zero(); // init accelerometer bias
        Vec3d gravity_ = Vec3d::Zero();
        Vec3d gyro_noise_std_ = Vec3d::Zero();
        Vec3d acc_noise_std_ = Vec3d::Zero();
        Vec3d integration_noise_cov_ = Vec3d::Zero();
    };

    explicit PreIntegration(const ConfigPara &config_para);

    /*!
     * preform pre-integration
     * @param curr_imu_data
     */
    void Integrate(const IMUData &curr_imu_data);

    void IntegrateDataSegment(const std::vector<IMUData> &curr_imu_data);

    /*!
     * calculate predicted navigation information
     * based on pre-integration increments
     * @param last_nav_state
     * @return
     */
    NavStateData Predict(const NavStateData &last_nav_state);

    void Reset();

    [[nodiscard]] const Mat9d &Covariance() const;

    [[nodiscard]] const Mat3d &DeltaR() const;

    [[nodiscard]] const Vec3d &DeltaV() const;

    [[nodiscard]] const Vec3d &DeltaP() const;

    [[nodiscard]] const Mat3d &Get_dR_dbg() const;

    [[nodiscard]] const Mat3d &Get_dP_dbg() const;

    [[nodiscard]] const Mat3d &Get_dP_dba() const;

    [[nodiscard]] const Mat3d &Get_dV_dbg() const;

    [[nodiscard]] const Mat3d &Get_dV_dba() const;

    [[nodiscard]] const Vec3d &GetAccBias() const;

    [[nodiscard]] const Vec3d &GetGyroBias() const;

    [[nodiscard]] const Vec3d &GetGravity() const;

    [[nodiscard]] const double &GetTotalIntegrationTime() const;

    [[nodiscard]] const TimeStampUs &GetStartIntegrationTimestamp() const;

    void SetGravity(const Vec3d &gravity);

    void SetGyroAccBias(const Vec3d &gyro_bias, const Vec3d &acc_bias);

    void SetGyroAccNoiseStd(const Vec3d &gyro_std, const Vec3d &acc_std);

private:
    Vec3d gyro_bias_ = Vec3d::Zero(); // Gyro bias
    Vec3d acc_bias_ = Vec3d::Zero(); // Accelerometer bias
    Mat6d gyro_acc_noise_cov_ = Mat6d::Zero(); // [0, 2] - gyro, [3, 5] - acc
    Mat3d integration_noise_cov_ = Mat3d::Zero(); // covariance due to speed integration

    Vec3d gravity_ = Vec3d::Zero(); // Earth's gravitational acceleration

    Mat3d delta_R_ = Mat3d::Identity();
    Vec3d delta_V_ = Vec3d::Zero();
    Vec3d delta_P_ = Vec3d::Zero();
    Mat9d cov_ = Mat9d::Zero();

    Mat3d dR_dbg_ = Mat3d::Zero();
    Mat3d dV_dbg_ = Mat3d::Zero();
    Mat3d dV_dba_ = Mat3d::Zero();
    Mat3d dP_dbg_ = Mat3d::Zero();
    Mat3d dP_dba_ = Mat3d::Zero();

    double total_time_{0.0}; // Total integration time of pre-integration
    uint64_t start_timestamp_{0u};

    IMUData last_imu_data_{};
    bool has_first_data_{false};
};

#endif //FUNNY_LIDAR_SLAM_PRE_INTEGRATION_H
