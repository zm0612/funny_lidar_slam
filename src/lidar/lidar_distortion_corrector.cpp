//
// Created by Zhang Zhimeng on 22-8-31.
//
#include <utility>
#include <iomanip>

#include "lidar/lidar_distortion_corrector.h"
#include "common/motion_interpolator.h"

LidarDistortionCorrector::LidarDistortionCorrector(const Mat4d &T_lidar_to_imu) {
    T_lidar_to_imu_ = T_lidar_to_imu;
}

void LidarDistortionCorrector::SetDataSearcher(std::shared_ptr<DataSearcher<IMUData>> imu_data_searcher) {
    imu_data_searcher_ = std::move(imu_data_searcher);
}

bool LidarDistortionCorrector::SetRefTime(uint64_t ref_time) {
    CHECK_NOTNULL(imu_data_searcher_);

    ref_time_ = ref_time;

    IMUData imu_data_l, imu_data_r;
    if (!imu_data_searcher_->SearchNearestTwoData(ref_time_, imu_data_l, imu_data_r)) {
        LOG(WARNING) << "Don't have valid imu data to set reference";
        return false;
    }
    double ratio = static_cast<double>(ref_time_ - imu_data_l.timestamp_) /
                   static_cast<double>(imu_data_r.timestamp_ - imu_data_l.timestamp_);
    q_ref_inv_ = MotionInterpolator::InterpolateQuaternionLerp(imu_data_l.orientation_,
                                                               imu_data_r.orientation_,
                                                               ratio).inverse();
    return true;
    /// TODO: 增加平移
}

bool LidarDistortionCorrector::ProcessPoint(float x, float y, float z, float &x_corrected,
                                            float &y_corrected, float &z_corrected, float relative_time) {
    Vec3d pt_lidar(x, y, z);
    Eigen::Quaterniond q_curr;
    IMUData imu_data_l, imu_data_r;
    const auto t = static_cast<uint64_t>(static_cast<int64_t>(ref_time_) +
                                         static_cast<int64_t>(relative_time * 1.0e6));

    if (imu_data_searcher_->SearchNearestTwoData(t, imu_data_l, imu_data_r)) {
        double ratio = static_cast<double>(t - imu_data_l.timestamp_) /
                       static_cast<double>(imu_data_r.timestamp_ - imu_data_l.timestamp_);
        q_curr = MotionInterpolator::InterpolateQuaternionLerp(imu_data_l.orientation_, imu_data_r.orientation_, ratio);
    } else {
        LOG(ERROR) << "Search current point orientation error. Normally this error should not occur. "
                      "Please check the timestamp of the lidar point!";
        return false;
    }

    // transform cloud from lidar to imu
    Vec3d pt_imu = T_lidar_to_imu_.block<3, 3>(0, 0) * pt_lidar + T_lidar_to_imu_.block<3, 1>(0, 3);

    Vec3d pt_corrected = q_ref_inv_ * q_curr * pt_imu;
    x_corrected = static_cast<float>(pt_corrected.x());
    y_corrected = static_cast<float>(pt_corrected.y());
    z_corrected = static_cast<float>(pt_corrected.z());

    return true;
}