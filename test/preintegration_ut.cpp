//
// Created by Zhang Zhimeng on 23-10-17.
//

#include "imu/pre_integration.h"

#include <gtest/gtest.h>

TEST(PreIntegrationTest, BiasCorrectionJacobian) {
    double imu_delta_time = 0.01;
    double pi_100 = M_PI / 100.0;
    Vec3d angular_velocity(pi_100, pi_100 * 3.0, pi_100 * 2.0);
    Vec3d acceleration(0.05, 0.09, 0.01);
    Vec3d gravity(0.0, 0.0, -9.8);

    double gyro_sigma = 0.5 * kDegree2Radian / 60.0;
    double accel_sigma = 0.1 / 60.0;

    PreIntegration::ConfigPara config_para;
    config_para.gravity_ = gravity;
    config_para.gyro_noise_std_ << gyro_sigma, gyro_sigma, gyro_sigma;
    config_para.acc_noise_std_ << accel_sigma, accel_sigma, accel_sigma;
    config_para.init_acc_bias_ << 0.0, 0.0, 0.0;
    config_para.init_gyro_bias_ << 0.0, 0.0, 0.0;

    PreIntegration pre_integration(config_para);

    for (unsigned int i = 0; i <= 101u; ++i) {
        IMUData imu_data;
        imu_data.angular_velocity_ = {pi_100, pi_100 * 3, pi_100 * 2};
        imu_data.linear_acceleration_ = {0.05, 0.09, 0.01};
        imu_data.timestamp_ = static_cast<uint64_t>((i * imu_delta_time) * 1.0e6);

        pre_integration.Integrate(imu_data);
    }

    Mat3d dR_dbg_true;
    dR_dbg_true << -1.0078, -0.0325178, 0.0476759,
            0.0315017, -1.00915, -0.017021,
            -0.0483534, 0.0149887, -1.00831;
    const Mat3d &dR_dbg = pre_integration.Get_dR_dbg();
    EXPECT_TRUE(dR_dbg.isApprox(dR_dbg_true, 1.0e-3));

    Mat3d dP_dba_true;
    dP_dba_true << -0.509505, 0.0104964, -0.0160171,
            -0.010748, -0.50984, 0.00505947,
            0.0158494, -0.00556269, -0.509631;
    const Mat3d &dP_dba = pre_integration.Get_dP_dba();
    EXPECT_TRUE(dP_dba.isApprox(dP_dba_true, 1.0e-3));

    Mat3d dP_dbg_true;
    dP_dbg_true << -0.000389658, -0.00141794, 0.0154636,
            0.00173211, -8.89504e-05, -0.00800937,
            -0.0153229, 0.00828978, -0.000429069;
    const Mat3d &dP_dbg = pre_integration.Get_dP_dbg();
    EXPECT_TRUE(dP_dbg.isApprox(dP_dbg_true, 1.0e-3));

    Mat3d dV_dba_true;
    dV_dba_true << -1.00783, 0.0311926, -0.0478735,
            -0.0321938, -1.00917, 0.0148455,
            0.0472061, -0.0168477, -1.00833;
    const Mat3d &dV_dba = pre_integration.Get_dV_dba();
    EXPECT_TRUE(dV_dba.isApprox(dV_dba_true, 1.0e-3));

    Mat3d dV_dbg_true;
    dV_dbg_true << -0.00155298, -0.00396709, 0.0463807,
            0.00521088, -0.000349525, -0.0234532,
            -0.0458268, 0.0245691, -0.00170903;
    const Mat3d &dV_dbg = pre_integration.Get_dV_dbg();
    EXPECT_TRUE(dV_dbg.isApprox(dV_dbg_true, 1.0e-3));

    Mat9d cov_true;
    cov_true
            << 2.136552508e-08, 5.272344755e-16, 3.514515723e-16, 9.832204912e-11, -1.411053378e-10, 9.42229404e-10, 3.284769798e-11, -4.847497497e-11, 3.138622159e-10,
            5.272344755e-16, 2.136552648e-08, 1.054455785e-15, 6.053876664e-11, 2.317724315e-11, -5.588727736e-10, 2.126788336e-11, 7.924201899e-12, -1.900603586e-10,
            3.514515723e-16, 1.054455785e-15, 2.13655256e-08, -9.788012618e-10, 4.878899472e-10, 1.084557875e-10, -3.263289837e-10, 1.662832065e-10, 3.626305183e-11,
            9.832204912e-11, 6.053876664e-11, -9.788012618e-10, 2.80561634e-06, -3.082951384e-11, -3.005102462e-12, 1.416828348e-06, -1.180303206e-11, -1.156453233e-12,
            -1.411053378e-10, 2.317724315e-11, 4.878899472e-10, -3.082951384e-11, 2.805571915e-06, -5.867706837e-12, -1.159484795e-11, 1.416811843e-06, -2.210844833e-12,
            9.42229404e-10, -5.588727736e-10, 1.084557875e-10, -3.005102462e-12, -5.867706837e-12, 2.805631556e-06, -1.167211764e-12, -2.271471233e-12, 1.41683419e-06,
            3.284769798e-11, 2.126788336e-11, -3.263289837e-10, 1.416828348e-06, -1.159484795e-11, -1.167211764e-12, 9.539681453e-07, -4.73368856e-12, -4.787299343e-13,
            -4.847497497e-11, 7.924201899e-12, 1.662832065e-10, -1.180303206e-11, 1.416811843e-06, -2.271471233e-12, -4.73368856e-12, 9.539616032e-07, -9.125261858e-13,
            3.138622159e-10, -1.900603586e-10, 3.626305183e-11, -1.156453233e-12, -2.210844833e-12, 1.41683419e-06, -4.787299343e-13, -9.125261858e-13, 9.539705362e-07;

    Mat9d cov = pre_integration.Covariance();

    EXPECT_TRUE(cov.isApprox(cov_true, 1.0e-9));
}