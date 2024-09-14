//
// Created by Zhang Zhimeng on 22-5-14.
//

#include "lidar/lidar_model.h"

#include <gtest/gtest.h>

TEST(TestLidarModel, TestNormalInput) {
    LidarModel *lidar_16_model = LidarModel::Instance("LeiShen_16");

    // (-10, 0) = 0
    EXPECT_EQ(lidar_16_model->ColIndex(-10.0f, 0.0f), 0);

    // (-10, 0.029671) = 1999
    EXPECT_EQ(lidar_16_model->ColIndex(-10.0f, 0.029671f), 1999);

    // (-10, 0.0331614) = 1998
    EXPECT_EQ(lidar_16_model->ColIndex(-10.0f, 0.0331614f), 1999);

    // (0, 10)
    EXPECT_EQ(lidar_16_model->ColIndex(0.0f, 10.f), 1500);

    // (10, 0.029671) = 1001
    EXPECT_EQ(lidar_16_model->ColIndex(10.0f, 0.029671f), 1001);

    // (10, -0.029671) = 999
    EXPECT_EQ(lidar_16_model->ColIndex(10.0f, -0.029671f), 999);

    // (-10, -0.029671) = 1
    EXPECT_EQ(lidar_16_model->ColIndex(-10.0f, -0.029671f), 1);

    // (-10, -0.0331614) = 1
    EXPECT_EQ(lidar_16_model->ColIndex(-10.0f, -0.0331614f), 1);

    /// Test LidarModel's function of RowIndex()
    // (10, 10, -3) = 2
    EXPECT_EQ(lidar_16_model->RowIndex(10.0f, 10.0f, -3.0f), 2);

    // (10, 10, -3.5) = 1
    EXPECT_EQ(lidar_16_model->RowIndex(10.0f, 10.0f, -3.5f), 1);

    // (10, 10, 1) = 10
    EXPECT_EQ(lidar_16_model->RowIndex(10.0f, 10.0f, 1.0f), 10);

    // (10, 10, 3.5) = 14
    EXPECT_EQ(lidar_16_model->RowIndex(10.0f, 10.0f, 3.5f), 14);

    // (10, 10, 3.6) = 15
    EXPECT_EQ(lidar_16_model->RowIndex(10.0f, 10.0f, 3.6f), 15);
}