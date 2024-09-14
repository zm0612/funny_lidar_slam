//
// Created by Zhang Zhimeng on 24-2-7.
//

#ifndef FUNNY_LIDAR_SLAM_LOOP_CLOSURE_H
#define FUNNY_LIDAR_SLAM_LOOP_CLOSURE_H

#include "common/data_type.h"
#include "slam/system.h"
#include "slam/config_parameters.h"

#include <pcl/filters/voxel_grid.h>

class LoopClosure {
public:
    using LoopClosureKeyFrameId = KeyFrame::ID;
    using LoopClosureCandidateKeyFrameId = KeyFrame::ID;
    using LoopClosurePair = std::pair<LoopClosureKeyFrameId, LoopClosureCandidateKeyFrameId>;
    using KeyFrameIdDistancePair = std::pair<KeyFrame::ID, double>;
    using CandidateKeyFramesResult = std::vector<KeyFrameIdDistancePair>;
    using PointCloudType = PCLPointCloudXYZI;
    using PointType = PCLPointXYZI;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit LoopClosure(System* system_ptr);

    void Run();

private:
    [[nodiscard]] KeyFrame::ID CheckCandidateKeyFrames(const CandidateKeyFramesResult& candidate_keyframes_result,
                                                       KeyFrame::ID keyframe_index_span_thre) const;

    [[nodiscard]] CandidateKeyFramesResult GetNearKeyFramesResult(double dist_thre) const;

    /*!
     * Get sub map
     * @param keyframe_id
     * @param left_range
     * @param right_range
     * @param use_local_pose whether to use global coordinate. If false, use keyframe_id as reference frame
     * @return
     */
    [[nodiscard]] PointCloudType::Ptr GetSubMap(KeyFrame::ID keyframe_id, KeyFrame::ID left_range,
                                                KeyFrame::ID right_range, bool use_local_pose) const;

    LoopClosurePair DetectByDistance();

    LoopClosurePair DetectByFeature();

    [[nodiscard]] bool HasNewKeyFrame() const;

    [[nodiscard]] KeyFrame::ID GetLatestKeyFrameID() const;

    [[nodiscard]] Mat4d GetKeyFramePoseByID(KeyFrame::ID id) const;

    static float Match(const PointCloudType::Ptr& source_cloud,
                       const PointCloudType::Ptr& target_cloud,
                       Mat4d& pose);

    void SetLoopClosureResult(const LoopClosurePair& loop_closure_pair, const Mat4d& match_pose) const;

private:
    System* system_ptr_ = nullptr;

    KeyFrame::ID last_detection_id_ = KeyFrame::kInvalidID;
    KeyFrame::ID last_loopclosure_id_ = KeyFrame::kInvalidID;
    KeyFrame::ID skip_near_loopclosure_threshold_{100};
    KeyFrame::ID curr_keyframe_id_ = KeyFrame::kInvalidID;
    KeyFrame::ID skip_near_keyframe_threshold_{100};

    std::string loopclosure_method_{};
    double near_neighbor_distance_threshold_{5.0};
    float registration_converge_threshold_{};
    int candidate_local_map_left_range_ = 20;
    int candidate_local_map_right_range_ = 20;
    int loopclosure_local_map_left_range_ = 20;
};

#endif //FUNNY_LIDAR_SLAM_LOOP_CLOSURE_H
