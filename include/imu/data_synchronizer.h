//
// Created by Zhang Zhimeng on 23-11-5.
//

#ifndef FUNNY_LIDAR_SLAM_DATA_SYNCHRONIZER_H
#define FUNNY_LIDAR_SLAM_DATA_SYNCHRONIZER_H

#include <deque>
#include <mutex>
#include <optional>

#include "common/data_type.h"
#include "common/sensor_data_type.h"

class DataSynchronizer {
 public:
  DataSynchronizer() = default;

  void CacheData(IMUData imu_data);

  std::vector<IMUData> GetDataSegment(TimeStampUs timestamp_left,
                                      TimeStampUs timestamp_right);

  std::optional<IMUData> OldestData();

  std::optional<IMUData> LatestData();

 private:
  std::mutex mutex_;
  std::deque<IMUData> imu_data_deque_;
};

#endif  // FUNNY_LIDAR_SLAM_DATA_SYNCHRONIZER_H
