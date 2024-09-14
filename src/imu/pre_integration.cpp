//
// Created by Zhang Zhimeng on 23-10-14.
//

#include "imu/pre_integration.h"

PreIntegration::PreIntegration(const PreIntegration::ConfigPara &config_para) {
    gyro_bias_ = config_para.init_gyro_bias_;
    acc_bias_ = config_para.init_acc_bias_;
    integration_noise_cov_.diagonal() = config_para.integration_noise_cov_;
    gyro_acc_noise_cov_.diagonal().head(3) = config_para.gyro_noise_std_.array().pow(2.0);
    gyro_acc_noise_cov_.diagonal().tail(3) = config_para.acc_noise_std_.array().pow(2.0);
    gravity_ = config_para.gravity_;
}

void PreIntegration::IntegrateDataSegment(const std::vector<IMUData> &imu_data_segment) {
    CHECK_GE(imu_data_segment.size(), 2) << "Pre-integration requires at least two data";

    for (const auto &imu_data: imu_data_segment) {
        Integrate(imu_data);
    }
}

void PreIntegration::Integrate(const IMUData &curr_imu_data) {
    if (!has_first_data_) {
        last_imu_data_ = curr_imu_data;
        has_first_data_ = true;
        start_timestamp_ = curr_imu_data.timestamp_;
        return;
    }

    const auto delta_t = static_cast<double>(curr_imu_data.timestamp_ - last_imu_data_.timestamp_) / 1.0e6; // unit: s

    // eliminate bias
    const Vec3d acc_0_unbias = last_imu_data_.linear_acceleration_ - acc_bias_;
    const Vec3d gyro_0_unbias = last_imu_data_.angular_velocity_ - gyro_bias_;

    // eliminate bias
    const Vec3d acc_1_unbias = curr_imu_data.linear_acceleration_ - acc_bias_;
    const Vec3d gyro_1_unbias = curr_imu_data.angular_velocity_ - gyro_bias_;

    // use average to integrate
    const Vec3d acc_unbias = (acc_1_unbias + acc_0_unbias) * 0.5;
    const Vec3d gyro_unbias = (gyro_1_unbias + gyro_0_unbias) * 0.5;

    // compute delta rotation
    const Mat3d R_step = SO3Exp(gyro_unbias * delta_t);

    const Eigen::Matrix3d acc_hat = SO3Hat(acc_unbias);

    // A and B are noise transfer matrix
    // A:
    //  row 0-2 is rotation
    //  row 3-5 is velocity
    //  row 6-9 is position
    // B:
    //  row 0-2 is rotation
    //  row 3-5 is velocity
    //  row 6-9 is position
    Eigen::Matrix<double, 9, 9> A = Eigen::Matrix<double, 9, 9>::Identity(); // noise recurrence matrix
    Eigen::Matrix<double, 9, 6> B = Eigen::Matrix<double, 9, 6>::Zero(); // noise recurrence matrix

    // formula (13)
    A.block<3, 3>(0, 0) = R_step.transpose();
    A.block<3, 3>(3, 0) = -delta_R_ * acc_hat * delta_t;
    A.block<3, 3>(6, 0) = -0.5 * delta_R_ * acc_hat * delta_t * delta_t;
    A.block<3, 3>(6, 3) = delta_t * Eigen::Matrix3d::Identity();

    Mat3d Jr = So3JacobianRight(gyro_unbias * delta_t);

    // formula (13)
    B.block<3, 3>(0, 0) = Jr * delta_t;
    B.block<3, 3>(3, 3) = delta_R_ * delta_t;
    B.block<3, 3>(6, 3) = 0.5 * delta_R_ * delta_t * delta_t;

    dP_dbg_ = dP_dbg_ + dV_dbg_ * delta_t - 0.5 * delta_R_ * acc_hat * dR_dbg_ * delta_t * delta_t; // formula (21)
    dP_dba_ = dP_dba_ + dV_dba_ * delta_t - 0.5 * delta_R_ * delta_t * delta_t; // formula (22)
    dV_dbg_ = dV_dbg_ - delta_R_ * acc_hat * dR_dbg_ * delta_t; // formula (19)
    dV_dba_ = dV_dba_ - delta_R_ * delta_t; // formula (20)
    dR_dbg_ = R_step.transpose() * dR_dbg_ - Jr * delta_t; // formula (18)

    // 注意更新顺序是P, V, R. 如果顺序反了，将会带来较大的误差
    delta_P_ = delta_P_ + delta_V_ * delta_t + 0.5 * delta_R_ * acc_unbias * delta_t * delta_t; // formula (3)
    delta_V_ = delta_V_ + delta_R_ * acc_unbias * delta_t; // formula (2)
    delta_R_ = delta_R_ * R_step; // formula (1)

    // formula (14)
    cov_ = A * cov_ * A.transpose() + B * (gyro_acc_noise_cov_ / delta_t) * B.transpose();

    Mat3d i_cov = Mat3d::Identity() * integration_noise_cov_ * delta_t;
    cov_.block<3, 3>(6, 6) += i_cov;

    last_imu_data_ = curr_imu_data;

    total_time_ += delta_t;
}

NavStateData PreIntegration::Predict(const NavStateData &last_nav_state) {
    const Vec3d P_last = last_nav_state.P_;
    const Vec3d V_last = last_nav_state.V_;
    const Mat3d R_last = last_nav_state.R_;

    NavStateData pre_nav_state;

    pre_nav_state.P_ = R_last * delta_P_ + P_last + V_last * total_time_ + 0.5 * gravity_ * total_time_ * total_time_;
    pre_nav_state.V_ = R_last * delta_V_ + V_last + gravity_ * total_time_;
    pre_nav_state.R_ = R_last * delta_R_;

    return pre_nav_state;
}

void PreIntegration::Reset() {
    delta_R_ = Mat3d::Identity();
    delta_V_ = Vec3d::Zero();
    delta_P_ = Vec3d::Zero();
    cov_ = Mat9d::Zero();

    dR_dbg_ = Mat3d::Zero();
    dV_dbg_ = Mat3d::Zero();
    dV_dba_ = Mat3d::Zero();
    dP_dbg_ = Mat3d::Zero();
    dP_dba_ = Mat3d::Zero();

    total_time_ = 0.0;
    start_timestamp_ = 0u;

    has_first_data_ = false;
}

const Mat9d &PreIntegration::Covariance() const {
    return cov_;
}

const Vec3d &PreIntegration::DeltaP() const {
    return delta_P_;
}

const Mat3d &PreIntegration::DeltaR() const {
    return delta_R_;
}

const Vec3d &PreIntegration::DeltaV() const {
    return delta_V_;
}

const Mat3d &PreIntegration::Get_dR_dbg() const {
    return dR_dbg_;
}

const Mat3d &PreIntegration::Get_dP_dbg() const {
    return dP_dbg_;
}

const Mat3d &PreIntegration::Get_dP_dba() const {
    return dP_dba_;
}

const Mat3d &PreIntegration::Get_dV_dba() const {
    return dV_dba_;
}

const Mat3d &PreIntegration::Get_dV_dbg() const {
    return dV_dbg_;
}

const Vec3d &PreIntegration::GetAccBias() const {
    return acc_bias_;
}

const Vec3d &PreIntegration::GetGyroBias() const {
    return gyro_bias_;
}

const double &PreIntegration::GetTotalIntegrationTime() const {
    return total_time_;
}

const uint64_t &PreIntegration::GetStartIntegrationTimestamp() const {
    return start_timestamp_;
}

const Vec3d &PreIntegration::GetGravity() const {
    return gravity_;
}

void PreIntegration::SetGravity(const Vec3d &gravity) {
    gravity_ = gravity;
}

void PreIntegration::SetGyroAccBias(const Vec3d &gyro_bias, const Vec3d &acc_bias) {
    gyro_bias_ = gyro_bias;
    acc_bias_ = acc_bias;
}

void PreIntegration::SetGyroAccNoiseStd(const Vec3d &gyro_std, const Vec3d &acc_std) {
    gyro_acc_noise_cov_.diagonal().head(3) = gyro_std.array().pow(2.0);
    gyro_acc_noise_cov_.diagonal().tail(3) = acc_std.array().pow(2.0);
}
