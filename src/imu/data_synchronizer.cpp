//
// Created by Zhang Zhimeng on 23-11-5.
//

#include "imu/data_synchronizer.h"
#include "common/motion_interpolator.h"

void DataSynchronizer::CacheData(IMUData imu_data) {
    std::lock_guard<std::mutex> lg(mutex_);
    imu_data_deque_.emplace_back(std::move(imu_data));
}

std::vector<IMUData> DataSynchronizer::GetDataSegment(TimeStampUs timestamp_left,
                                                      TimeStampUs timestamp_right) {
    if (timestamp_left >= timestamp_right) {
        return {};
    }

    std::lock_guard<std::mutex> lg(mutex_);

    if (imu_data_deque_.front().timestamp_ > timestamp_left ||
        imu_data_deque_.back().timestamp_ < timestamp_right) {
        return {};
    }

    std::vector<IMUData> data_segment;

    while (!imu_data_deque_.empty()) {

        if (imu_data_deque_.size() < 2) {
            return {};
        }

        auto &first_data = imu_data_deque_.at(0);
        auto &second_data = imu_data_deque_.at(1);

        if (first_data.timestamp_ == timestamp_left) {
            data_segment.emplace_back(imu_data_deque_.front());
            imu_data_deque_.pop_front();
            break;
        }

        if (first_data.timestamp_ < timestamp_left &&
            second_data.timestamp_ > timestamp_left) {

            IMUData imu_data;
            imu_data.linear_acceleration_ = MotionInterpolator::InterpolateVectorLerp(first_data.linear_acceleration_,
                                                                                      second_data.linear_acceleration_,
                                                                                      first_data.timestamp_,
                                                                                      second_data.timestamp_,
                                                                                      timestamp_left);

            imu_data.angular_velocity_ = MotionInterpolator::InterpolateVectorLerp(first_data.angular_velocity_,
                                                                                   second_data.angular_velocity_,
                                                                                   first_data.timestamp_,
                                                                                   second_data.timestamp_,
                                                                                   timestamp_left);

            imu_data.orientation_ = MotionInterpolator::InterpolateQuaternionSlerp(first_data.orientation_,
                                                                                   second_data.orientation_,
                                                                                   first_data.timestamp_,
                                                                                   second_data.timestamp_,
                                                                                   timestamp_left);

            imu_data.timestamp_ = timestamp_left;

            data_segment.emplace_back(std::move(imu_data));
            imu_data_deque_.pop_front();
            break;
        }
        imu_data_deque_.pop_front();
    }

    while (!imu_data_deque_.empty()) {

        if (imu_data_deque_.size() < 2) {
            return {};
        }

        auto &first_data = imu_data_deque_.at(0);
        auto &second_data = imu_data_deque_.at(1);

        if (first_data.timestamp_ == timestamp_right) {
            data_segment.emplace_back(std::move(imu_data_deque_.front()));
            break;
        }

        if (first_data.timestamp_ < timestamp_right &&
            second_data.timestamp_ > timestamp_right) {

            IMUData imu_data;
            imu_data.linear_acceleration_ = MotionInterpolator::InterpolateVectorLerp(first_data.linear_acceleration_,
                                                                                      second_data.linear_acceleration_,
                                                                                      first_data.timestamp_,
                                                                                      second_data.timestamp_,
                                                                                      timestamp_right);

            imu_data.angular_velocity_ = MotionInterpolator::InterpolateVectorLerp(first_data.angular_velocity_,
                                                                                   second_data.angular_velocity_,
                                                                                   first_data.timestamp_,
                                                                                   second_data.timestamp_,
                                                                                   timestamp_right);

            imu_data.orientation_ = MotionInterpolator::InterpolateQuaternionSlerp(first_data.orientation_,
                                                                                   second_data.orientation_,
                                                                                   first_data.timestamp_,
                                                                                   second_data.timestamp_,
                                                                                   timestamp_right);

            imu_data.timestamp_ = timestamp_right;

            data_segment.emplace_back(std::move(imu_data));
            break;
        }

        data_segment.emplace_back(imu_data_deque_.front());
        imu_data_deque_.pop_front();
    }

    return data_segment;
}

std::optional<IMUData> DataSynchronizer::OldestData() {
    std::lock_guard<std::mutex> lg(mutex_);
    if (imu_data_deque_.empty()) {
        return {};
    } else {
        return imu_data_deque_.front();
    }
}

std::optional<IMUData> DataSynchronizer::LatestData() {
    std::lock_guard<std::mutex> lg(mutex_);
    if (imu_data_deque_.empty()) {
        return {};
    } else {
        return imu_data_deque_.back();
    }
}