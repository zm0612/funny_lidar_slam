//
// Created by Zhang Zhimeng on 23-11-6.
//
#include <gtest/gtest.h>

#define private public
#define protected public

#include "imu/imu_data_searcher.h"

TEST(TestIMUDataSearcher, HandleOutLeftBoundaryInput) {
    IMUDataSearcher imu_data_searcher(1000);

    for (unsigned int i = 10; i < 100u; ++i) {
        IMUData imu_data;
        imu_data.timestamp_ = i;
        imu_data.linear_acceleration_ = Vec3d::Identity() * i;
        imu_data.angular_velocity_ = Vec3d::Identity() * i;

        imu_data_searcher.CacheData(imu_data);
    }

    std::vector<IMUData> imu_data_segment;

    uint64_t timestamp_first;
    uint64_t timestamp_second;

    timestamp_first = 0u;
    timestamp_second = 15u;
    imu_data_segment = imu_data_searcher.GetDataSegment(timestamp_first, timestamp_second);
    EXPECT_TRUE(imu_data_segment.empty());
}

TEST(TestIMUDataSearcher, HandleOutRightBoundaryInput) {
    IMUDataSearcher imu_data_searcher(1000);

    for (unsigned int i = 10; i < 100u; ++i) {
        IMUData imu_data;
        imu_data.timestamp_ = i;
        imu_data.linear_acceleration_ = Vec3d::Identity() * i;
        imu_data.angular_velocity_ = Vec3d::Identity() * i;

        imu_data_searcher.CacheData(imu_data);
    }

    std::vector<IMUData> imu_data_segment;

    uint64_t timestamp_first;
    uint64_t timestamp_second;

    timestamp_first = 95u;
    timestamp_second = 105u;
    imu_data_segment = imu_data_searcher.GetDataSegment(timestamp_first, timestamp_second);
    EXPECT_TRUE(imu_data_segment.empty());
}

TEST(TestIMUDataSearcher, HandleNormalInput) {
    IMUDataSearcher imu_data_searcher(1000);

    for (unsigned int i = 10; i < 100u; ++i) {
        IMUData imu_data;
        imu_data.timestamp_ = i;
        imu_data.linear_acceleration_ = Vec3d::Identity() * i;
        imu_data.angular_velocity_ = Vec3d::Identity() * i;

        imu_data_searcher.CacheData(imu_data);
    }

    std::vector<IMUData> imu_data_segment;

    uint64_t timestamp_first;
    uint64_t timestamp_second;

    timestamp_first = 12u;
    timestamp_second = 18u;
    imu_data_segment = imu_data_searcher.GetDataSegment(timestamp_first, timestamp_second);
    EXPECT_EQ(imu_data_segment.size(), 7);
}

TEST(TestIMUDataSearcher, HandleRemainingData) {
    IMUDataSearcher imu_data_searcher(1000);

    for (unsigned int i = 10; i < 20; ++i) {
        IMUData imu_data;
        imu_data.timestamp_ = i;
        imu_data.linear_acceleration_ = Vec3d::Identity() * i;
        imu_data.angular_velocity_ = Vec3d::Identity() * i;

        imu_data_searcher.CacheData(imu_data);
    }

    std::vector<IMUData> imu_data_segment;

    uint64_t timestamp_first;
    uint64_t timestamp_second;

    timestamp_first = 12u;
    timestamp_second = 18u;
    imu_data_segment = imu_data_searcher.GetDataSegment(timestamp_first, timestamp_second);
    EXPECT_EQ(imu_data_segment.size(), 7u);
}

TEST(TestIMUDataSearcher, HandleContinueInput) {
    IMUDataSearcher imu_data_searcher(1000);

    for (unsigned int i = 10u; i < 20u; ++i) {
        IMUData imu_data;
        imu_data.timestamp_ = i;
        imu_data.linear_acceleration_ = Vec3d::Identity() * i;
        imu_data.angular_velocity_ = Vec3d::Identity() * i;

        imu_data_searcher.CacheData(imu_data);
    }

    std::vector<IMUData> imu_data_segment;

    uint64_t timestamp_first;
    uint64_t timestamp_second;

    timestamp_first = 12u;
    timestamp_second = 14u;
    imu_data_segment = imu_data_searcher.GetDataSegment(timestamp_first, timestamp_second);
    EXPECT_EQ(imu_data_segment.size(), 3);

    timestamp_first = 14u;
    timestamp_second = 18u;
    imu_data_segment = imu_data_searcher.GetDataSegment(timestamp_first, timestamp_second);
    EXPECT_EQ(imu_data_segment.size(), 5);
}

TEST(TestIMUDataSearcher, HandleSpcialData) {
    IMUDataSearcher imu_data_searcher(1000);

    for (unsigned int i = 10; i < 20; ++i) {
        IMUData imu_data;
        imu_data.timestamp_ = i;
        imu_data.linear_acceleration_ = Vec3d::Identity() * i;
        imu_data.angular_velocity_ = Vec3d::Identity() * i;

        imu_data_searcher.CacheData(imu_data);
    }

    std::vector<IMUData> imu_data_segment;

    uint64_t timestamp_first;
    uint64_t timestamp_second;

    timestamp_first = 12u;
    timestamp_second = 14u;
    imu_data_segment = imu_data_searcher.GetDataSegment(timestamp_first, timestamp_second);

    EXPECT_EQ(imu_data_segment.at(0).timestamp_, 12);
    EXPECT_EQ(imu_data_segment.at(1).timestamp_, 13);
    EXPECT_EQ(imu_data_segment.at(2).timestamp_, 14);
}
