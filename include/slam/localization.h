//
// Created by Zhang Zhimeng on 24-6-4.
//

#ifndef FUNNY_LIDAR_SLAM_LOCALIZATION_H
#define FUNNY_LIDAR_SLAM_LOCALIZATION_H

#include "slam/system.h"
#include "slam/config_parameters.h"
#include "common/data_type.h"
#include "common/compare_function.h"
#include "imu/pre_integration.h"
#include "registration/registration_interface.h"
#include "registration/icp_optimized.h"

#include <unordered_set>

class Localization {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit Localization(System* system_ptr);

    PCLPointCloudXYZI GetLocalCloudMap();

    PCLPointCloudXYZI GetGlobalCloudMap();

    PCLPointCloudXYZI GetCurrentLidarCloudMap();

    [[nodiscard]] bool NeedUpdateLocalMapVisualization() const;

    void SetInitPose(const Mat4d& pose);

    void Run();

private:
    bool Init();

    void InitPreintegration();

    void InitMatcher();

    void SetLidarPoseInformation();

    void CacheResultToSystem(const NavStateData& state) const;

    /*!
     * @brief Use to display local map cloud
     */
    void UpdateLocalMapCloud(const PCLPointCloudXYZI& local_map_cloud);

    void UpdateCurrentLidarCloud(const PCLPointCloudXYZI& lidar_cloud);

    /*!
     * @brief Centered on pose, load nearby maps
     * @param pose
     * @return local map cloud
     */
    PCLPointCloudXYZI::Ptr LoadLocalMap(const Mat4d& pose);

    void LoadTileMapIndices();

    static PCLPointCloudXYZI::Ptr LoadGlobalMap(const std::string& path);

    static PCLPointCloudXYZI::Ptr LoadTileMap(const Vec2i& tile_map_index);

    /*!
     * @brief Centered on index, load nearby tile map indices
     * @param index
     * @param step_size
     * @return
     */
    static std::vector<Vec2i> GenerateTileMapIndices(const Vec2i& index, int step_size);

    [[nodiscard]] NavStateData IntegrateImuMeasures(const NavStateData& last_nav_state) const;

    void Optimize(NavStateData& curr_nav_state, const Mat4d& lidar_pose);

private:
    std::shared_ptr<PreIntegration> pre_integration_ptr_ = nullptr;
    std::shared_ptr<RegistrationInterface> matcher_ = nullptr;
    PointcloudClusterPtr curr_cloud_cluster_ptr_ = nullptr;

    System* system_ptr_ = nullptr;

    std::map<Vec2i, PCLPointCloudXYZI::Ptr, LessVec2i> local_map_data_;
    std::set<Vec2i, LessVec2i> map_data_indices_;

    // navigation state and bias
    NavStateData last_nav_state_;

    Mat4d delta_pose_ = Mat4d::Identity();
    Mat4d curr_pose_ = Mat4d::Identity();
    Mat4d last_pose_ = Mat4d::Identity();

    // information
    Mat6d lidar_pose_info_ = Mat6d::Identity();

    TimeStampUs curr_time_us_ = 0u;

    std::atomic_bool has_init_{false};
    std::atomic_bool need_update_local_map_visualization_{false};

    std::mutex mutex_lidar_cloud_;
    std::mutex mutex_local_map_cloud_;
    std::mutex mutex_global_map_cloud_;
    PCLPointCloudXYZI lidar_cloud_;
    PCLPointCloudXYZI local_map_cloud_;
    PCLPointCloudXYZI::Ptr global_map_cloud_ptr_{nullptr};

    bool use_tile_map_{false}; // Not recommended
    bool need_update_local_map_{};
    double tile_map_grid_size_;
    double tile_map_grid_size_half_;

    std::vector<double> local_map_edge_;

    std::optional<Mat4d> init_pose_optional_;
    std::mutex mutex_init_pose_optional_;
};

#endif //FUNNY_LIDAR_SLAM_LOCALIZATION_H
