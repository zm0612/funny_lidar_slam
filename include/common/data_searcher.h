//
// Created by Zhang Zhimeng on 22-8-6.
//

#ifndef FUNNY_LIDAR_SLAM_DATA_SEARCHER_H
#define FUNNY_LIDAR_SLAM_DATA_SEARCHER_H

#include <glog/logging.h>

#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <type_traits>

#include "common/constant_variable.h"
#include "common/data_type.h"
#include "common/motion_interpolator.h"
#include "common/sensor_data_type.h"

/*!
 * Used to search for nearest data by timestamp
 * note: thread safety
 */
template <typename DataType>
class DataSearcher {
 public:
  static_assert(std::is_base_of<DataBase, DataType>::value,
                "The DataType must inherit from DataBase");

 public:
  explicit DataSearcher(size_t data_size) : data_size_(data_size) {
    CHECK_NE(data_size_, IntNaN);
  };

  explicit DataSearcher(const std::vector<DataType> &data_vector) {
    data_size_ = data_vector.size();
    for (const auto data : data_vector) {
      CacheData(data);
    }
  }

  ~DataSearcher() = default;

  bool CacheData(DataType data) {
    std::lock_guard<std::mutex> lg(mutex_deque_);

    if (!data_deque_.empty() &&
        data.timestamp_ < data_deque_.back().timestamp_) {
      return false;
    }

    data_deque_.emplace_back(std::move(data));

    if (data_deque_.size() > data_size_) {
      data_deque_.pop_front();
    }

    return true;
  }

  /*!
   * 搜索最近的一个数据
   * 注意：(1) 如果timestamp_target比队列中最老的数据还要老，则返回false
   *      (2)
   * 如果timestamp_target比队列中最新的数据还要新，则返回队列中最新的数据
   * @param timestamp_target
   * @param data
   * @return
   */
  bool SearchNearestData(TimeStampUs timestamp_target, DataType &data) {
    std::lock_guard<std::mutex> lg(mutex_deque_);

    if (data_deque_.empty()) {
      return false;
    }

    auto rit = data_deque_.rbegin();
    while (timestamp_target < rit->timestamp_) {
      if (rit == --data_deque_.rend()) {
        return false;
      }

      ++rit;
    }

    if (timestamp_target - rit->timestamp_ <
        (rit - 1)->timestamp_ - timestamp_target) {
      data = *(rit);
    } else {
      data = *(rit - 1);
    }

    return true;
  }

  /*!
   * 搜索两个最近的数据
   * 注意：(1)
   * 只有队列中数据满足存在把timestamp_target夹在中间时，才会返回true，并返回左右两个数据
   * @param timestamp_target
   * @param data_l
   * @param data_r
   * @return
   */
  bool SearchNearestTwoData(uint64_t timestamp_target, DataType &data_l,
                            DataType &data_r) {
    std::lock_guard<std::mutex> lg(mutex_deque_);

    if (data_deque_.empty()) {
      DLOG(WARNING) << "Data deque is empty";
      return false;
    }

    if (data_deque_.front().timestamp_ > timestamp_target ||
        data_deque_.back().timestamp_ < timestamp_target) {
      return false;
    }

    if (data_deque_.begin()->timestamp_ == timestamp_target) {
      data_l = *data_deque_.begin();
      data_r = *(data_deque_.begin() + 1);
      return true;
    }

    if (data_deque_.rbegin()->timestamp_ == timestamp_target) {
      data_r = *data_deque_.rbegin();
      data_l = *(data_deque_.rbegin() + 1);
      return true;
    }

    auto rit = data_deque_.rbegin();
    while (timestamp_target < rit->timestamp_) {
      ++rit;
    }

    data_l = *(rit);
    data_r = *(rit - 1);

    return true;
  }

  bool IsInBuffer(uint64_t timestamp_target) {
    std::lock_guard<std::mutex> lg(mutex_deque_);
    if (data_deque_.empty()) {
      return false;
    }

    if (data_deque_.front().timestamp_ > timestamp_target ||
        data_deque_.back().timestamp_ < timestamp_target) {
      return false;
    }

    return true;
  }

  std::optional<DataType> LatestData() {
    std::lock_guard<std::mutex> lg(mutex_deque_);
    if (data_deque_.empty()) {
      return {};
    } else {
      return data_deque_.back();
    }
  }

  std::optional<DataType> OldestData() {
    std::lock_guard<std::mutex> lg(mutex_deque_);
    if (data_deque_.empty()) {
      return {};
    } else {
      return data_deque_.front();
    }
  }

 protected:
  std::deque<DataType> data_deque_{};
  size_t data_size_ = 0u;
  std::mutex mutex_deque_{};
};

#endif  // FUNNY_LIDAR_SLAM_DATA_SEARCHER_H
