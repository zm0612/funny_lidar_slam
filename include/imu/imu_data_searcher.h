//
// Created by Zhang Zhimeng on 24-5-4.
//

#ifndef FUNNY_LIDAR_SLAM_IMU_DATA_SEARCHER_H
#define FUNNY_LIDAR_SLAM_IMU_DATA_SEARCHER_H

#include "common/data_searcher.h"

class IMUDataSearcher final : public DataSearcher<IMUData> {
public:
    explicit IMUDataSearcher(size_t data_size) : DataSearcher<IMUData>(data_size) {}

    ~IMUDataSearcher() = default;

    std::vector<IMUData> GetDataSegment(TimeStampUs timestamp_left,
                                        TimeStampUs timestamp_right) {
        std::lock_guard<std::mutex> lg(mutex_deque_);

        // Abnormal left and right timestamps
        if (timestamp_left >= timestamp_right) {
            return {};
        }

        // Make sure the search time is in the queue
        if (data_deque_.front().timestamp_ > timestamp_left ||
            data_deque_.back().timestamp_ < timestamp_right) {
            return {};
        }

        IMUData left_data, right_data;

        // find left boundary and left imu data
        auto left_boundary_iter = data_deque_.rend();
        if (data_deque_.begin()->timestamp_ == timestamp_left) {
            left_boundary_iter = data_deque_.rend() - 1;
            left_data = *(data_deque_.rend() - 1);
        }

        if (left_boundary_iter == data_deque_.rend()) {
            auto rit = data_deque_.rbegin();
            while (timestamp_left < rit->timestamp_) {
                ++rit;
            }

            left_boundary_iter = rit;

            IMUData data_l, data_r;
            data_l = *(rit);
            data_r = *(rit - 1);

            left_data.linear_acceleration_ = MotionInterpolator::InterpolateVectorLerp(
                    data_l.linear_acceleration_, data_r.linear_acceleration_,
                    data_l.timestamp_, data_r.timestamp_, timestamp_left
            );

            left_data.angular_velocity_ = MotionInterpolator::InterpolateVectorLerp(
                    data_l.angular_velocity_, data_r.angular_velocity_,
                    data_l.timestamp_, data_r.timestamp_, timestamp_left
            );

            left_data.orientation_ = MotionInterpolator::InterpolateQuaternionSlerp(
                    data_l.orientation_, data_r.orientation_,
                    data_l.timestamp_, data_r.timestamp_, timestamp_left
            );

            left_data.timestamp_ = timestamp_left;
        }

        // find right boundary and right imu data
        auto right_boundary_iter = data_deque_.rend();
        if (data_deque_.rbegin()->timestamp_ == timestamp_right) {
            right_boundary_iter = data_deque_.rbegin();
            right_data = *data_deque_.rbegin();
        }

        if (right_boundary_iter == data_deque_.rend()) {
            auto rit = data_deque_.rbegin();
            while (timestamp_right < rit->timestamp_) {
                ++rit;
            }

            right_boundary_iter = rit;

            IMUData data_l, data_r;
            data_l = *(rit);
            data_r = *(rit - 1);

            right_data.linear_acceleration_ = MotionInterpolator::InterpolateVectorLerp(
                    data_l.linear_acceleration_, data_r.linear_acceleration_,
                    data_l.timestamp_, data_r.timestamp_, timestamp_right
            );

            right_data.angular_velocity_ = MotionInterpolator::InterpolateVectorLerp(
                    data_l.angular_velocity_, data_r.angular_velocity_,
                    data_l.timestamp_, data_r.timestamp_, timestamp_right
            );

            right_data.orientation_ = MotionInterpolator::InterpolateQuaternionSlerp(
                    data_l.orientation_, data_r.orientation_,
                    data_l.timestamp_, data_r.timestamp_, timestamp_right
            );

            right_data.timestamp_ = timestamp_right;
        }

        std::vector<IMUData> data_segment;
        data_segment.push_back(left_data);
        while (--left_boundary_iter != right_boundary_iter) {
            data_segment.push_back(*left_boundary_iter);
        }
        data_segment.push_back(right_data);

        return data_segment;
    }
};

#endif //FUNNY_LIDAR_SLAM_IMU_DATA_SEARCHER_H
