//
// Created by Zhang Zhimeng on 22-10-2.
//
#include "lidar/lidar_distortion_corrector.h"
#include "common/timer.h"

#include <gtest/gtest.h>

TEST(TestLidarDistortionCorrector, TestNormalInput) {
    Eigen::AngleAxisd yaw_0(1 / 180.0 * M_PI, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yaw_1(3 / 180.0 * M_PI, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yaw_2(5 / 180.0 * M_PI, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yaw_3(7 / 180.0 * M_PI, Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond q_yaw_0(yaw_0);
    Eigen::Quaterniond q_yaw_1(yaw_1);
    Eigen::Quaterniond q_yaw_2(yaw_2);
    Eigen::Quaterniond q_yaw_3(yaw_3);

    IMUData imu_data_0;
    imu_data_0.orientation_ = q_yaw_0;
    imu_data_0.timestamp_ = 1u;
    IMUData imu_data_1;
    imu_data_1.orientation_ = q_yaw_1;
    imu_data_1.timestamp_ = 3u;
    IMUData imu_data_2;
    imu_data_2.orientation_ = q_yaw_2;
    imu_data_2.timestamp_ = 5u;
    IMUData imu_data_3;
    imu_data_3.orientation_ = q_yaw_3;
    imu_data_3.timestamp_ = 7u;

    std::shared_ptr<DataSearcher<IMUData>> imu_data_searcher_ptr = std::make_shared<DataSearcher<IMUData>>(100);
    imu_data_searcher_ptr->CacheData(imu_data_0);
    imu_data_searcher_ptr->CacheData(imu_data_1);
    imu_data_searcher_ptr->CacheData(imu_data_2);
    imu_data_searcher_ptr->CacheData(imu_data_3);

    LidarDistortionCorrector lidar_distortion_corrector(Eigen::Matrix4d::Identity());
    lidar_distortion_corrector.SetDataSearcher(imu_data_searcher_ptr);
    lidar_distortion_corrector.SetRefTime(2);

    Vec3f pt(1, 2, 3), pt_corrected, pt_real;

    lidar_distortion_corrector.ProcessPoint(pt.x(), pt.y(), pt.z(),
                                            pt_corrected.x(), pt_corrected.y(), pt_corrected.z(), 4.000001 * 1e-6);

    Eigen::AngleAxisd yaw_ref(2 / 180.0 * M_PI, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yaw_curr(6 / 180.0 * M_PI, Eigen::Vector3d::UnitZ());
    pt_real = (Eigen::Quaterniond(yaw_ref).inverse() * Eigen::Quaterniond(yaw_curr) * pt.cast<double>()).cast<float>();

    EXPECT_FLOAT_EQ(pt_real.x(), pt_corrected.x());
    EXPECT_FLOAT_EQ(pt_real.y(), pt_corrected.y());
    EXPECT_FLOAT_EQ(pt_real.z(), pt_corrected.z());
}
