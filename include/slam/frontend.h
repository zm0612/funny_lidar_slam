//
// Created by Zhang Zhimeng on 22-10-30.
//

#ifndef FUNNY_LIDAR_SLAM_FRONTEND_H
#define FUNNY_LIDAR_SLAM_FRONTEND_H

#include "slam/system.h"
#include "imu/pre_integration.h"
#include "lidar/lidar_distortion_corrector.h"
#include "registration/loam_point_to_plane_kdtree.h"

#include <Eigen/Dense>

class FrontEnd {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    FrontEnd() = delete;

    explicit FrontEnd(System* system_ptr);

    void Run();

private:
    Mat4d InitOdometer();

    void InitMatcher();

    void InitPreIntegration();

    void CacheFrameToSystem() const;

    void SetLidarPoseInformation();

    void Optimize(NavStateData& curr_nav_state, const Mat4d& lidar_pose);

    [[nodiscard]] NavStateData IntegrateImuMeasures(const NavStateData& last_nav_state) const;

private:
    // function
    std::shared_ptr<PreIntegration> pre_integration_ptr_ = nullptr;
    std::shared_ptr<RegistrationInterface> matcher_ = nullptr;

    PointcloudClusterPtr curr_cloud_cluster_ptr_ = nullptr;

    // navigation state and bias
    NavStateData last_nav_state_;

    // information
    Mat6d lidar_pose_info_ = Mat6d::Identity();

    Vec3d gravity_vector_ = Vec3d::Zero();

    Mat4d delta_pose_ = Mat4d::Identity();
    Mat4d last_pose_ = Mat4d::Identity();

    TimeStampUs curr_time_us_ = 0u;

    System* system_ptr_ = nullptr;

    bool has_init_ = false;
};

#endif //FUNNY_LIDAR_SLAM_FRONTEND_H
