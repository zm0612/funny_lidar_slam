//
// Created by Zhang Zhimeng on 22-8-7.
//
#include "common/data_searcher.h"
#include "common/timer.h"

#include <gtest/gtest.h>

TEST(TestDataSeacher, TestNomalInput) {
    constexpr size_t data_size = 1000;
    DataSearcher<IMUData> data_searcher(data_size);
    for (unsigned int i = 100; i <= data_size; ++i) {
        IMUData imu_data;
        imu_data.angular_velocity_ = Vec3d::Ones() * i;
        imu_data.linear_acceleration_ = Vec3d::Ones() * i;
        imu_data.orientation_.z() = i;

        imu_data.timestamp_ = i;

        data_searcher.CacheData(imu_data);
    }

    // exact search
    IMUData imu_data;
    data_searcher.SearchNearestData(100, imu_data);

    EXPECT_EQ(data_searcher.SearchNearestData(100, imu_data), true);
    EXPECT_EQ(imu_data.timestamp_, 100);

    EXPECT_EQ(data_searcher.SearchNearestData(99, imu_data), false);

    EXPECT_EQ(data_searcher.SearchNearestData(1001, imu_data), true);
    EXPECT_EQ(imu_data.timestamp_, 1000);

    // exact search two data.
    IMUData imu_data_l, imu_data_r;
    EXPECT_EQ(data_searcher.SearchNearestTwoData(100, imu_data_l, imu_data_r), true);
    EXPECT_EQ(imu_data_l.timestamp_, 100);
    EXPECT_EQ(imu_data_r.timestamp_, 101);

    EXPECT_EQ(data_searcher.SearchNearestTwoData(330, imu_data_l, imu_data_r), true);
    EXPECT_EQ(imu_data_l.timestamp_, 330);
    EXPECT_EQ(imu_data_r.timestamp_, 331);

    EXPECT_EQ(data_searcher.SearchNearestTwoData(1000, imu_data_l, imu_data_r), true);
    EXPECT_EQ(imu_data_l.timestamp_, 999);
    EXPECT_EQ(imu_data_r.timestamp_, 1000);

    EXPECT_EQ(data_searcher.SearchNearestTwoData(99, imu_data_l, imu_data_r), false);

    EXPECT_EQ(data_searcher.SearchNearestTwoData(1001, imu_data_l, imu_data_r), false);
}