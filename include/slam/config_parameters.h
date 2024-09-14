//
// Created by Zhang Zhimeng on 23-11-26.
//

#ifndef FUNNY_LIDAR_SLAM_CONFIG_PARAMETERS_H
#define FUNNY_LIDAR_SLAM_CONFIG_PARAMETERS_H

#include "common/data_type.h"

#include <memory>

class ConfigParameters {
public:
    // delete constructor
    ConfigParameters(ConfigParameters&&) = delete;

    ConfigParameters(const ConfigParameters&) = delete;

    ConfigParameters& operator=(const ConfigParameters&) = delete;

    // instance
    static ConfigParameters& Instance();

private:
    ConfigParameters() = default;

    static std::unique_ptr<ConfigParameters> instance_;

public:
    // imu sensor parameters
    Vec3d gravity_vector_{}; // Obtained by standing still for a period of time
    double gravity_norm{};
    double imu_init_acc_bias_{};
    double imu_init_gyro_bias_{};
    double imu_acc_noise_std_{};
    double imu_acc_rw_noise_std_{};
    double imu_gyro_noise_std_{};
    double imu_gyro_rw_noise_std_{};
    bool imu_has_orientation_{};
    int imu_data_searcher_buffer_size_{};

    // frontend parameters
    int frontend_fusion_opti_iters_{}; // optimization iterations
    std::string fusion_method_{};

    // system parameter
    double system_keyframe_delta_dist_{};
    double system_keyframe_delta_rotation_{};
    bool system_enable_loopclosure_{};
    bool system_enable_visualize_global_map_{};
    float system_global_map_visualization_resolution_{};

    // loopclosure
    int lc_skip_near_loopclosure_threshold_{};
    int lc_skip_near_keyframe_threshold_{};
    int lc_candidate_local_map_left_range_{};
    int lc_candidate_local_map_right_range_{};
    int lc_loopclosure_local_map_left_range_{};
    float lc_registration_converge_threshold_{};
    double lc_near_neighbor_distance_threshold_{};

    // lidar sensor parameters
    std::string lidar_sensor_type_{};
    int lidar_scan_{};
    int lidar_horizon_scan_{};
    int lidar_point_jump_span_{};
    double lidar_vertical_resolution_{};
    double lidar_lower_angle_{};
    double lidar_point_time_scale_{};
    double lidar_rotation_noise_std_{};
    double lidar_position_noise_std_{};
    float lidar_use_min_dist_{};
    float lidar_use_max_dist_{};

    // loam feature parameters
    std::string registration_and_searcher_mode_{};
    float loam_feature_corner_thres_{};
    float loam_feature_planar_thres_{};
    float loam_feature_planar_voxel_filter_size_{};
    float loam_feature_corner_voxel_filter_size_{};

    // registration parameters
    float registration_local_corner_filter_size_{};
    float registration_local_planar_filter_size_{};
    float registration_local_map_cloud_filter_size_{};
    float registration_source_cloud_filter_size_{};
    float registration_line_ratio_thres_{};
    float registration_point_search_thres_{};
    float registration_point_to_planar_thres_{};
    float registration_position_converge_thres_{};
    float registration_rotation_converge_thres_{};
    double registration_keyframe_delta_dist_{};
    double registration_keyframe_delta_rotation_{};
    double registration_ndt_voxel_size_{};
    double registration_ndt_outlier_threshold_{};
    int registration_ndt_min_points_in_voxel_{};
    int registration_ndt_max_points_in_voxel_{};
    int registration_ndt_min_effective_pts_{};
    int registration_ndt_capacity_{};
    int registration_local_map_size_{};
    int registration_local_planar_map_size_{};
    int registration_local_corner_map_size_{};
    int registration_opti_iter_num_{};

    // split map
    double tile_map_grid_size_{};

    // calibration parameters
    Mat4d calibration_lidar_to_imu_{};

    // sensor topic name
    std::string imu_topic_{};
    std::string lidar_topic_{};
};

#endif //FUNNY_LIDAR_SLAM_CONFIG_PARAMETERS_H
