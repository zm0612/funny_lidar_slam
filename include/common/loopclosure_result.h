//
// Created by Zhang Zhimeng on 24-3-3.
//

#ifndef FUNNY_LIDAR_SLAM_LOOPCLOSURE_RESULT_H
#define FUNNY_LIDAR_SLAM_LOOPCLOSURE_RESULT_H

#include "data_type.h"
#include "keyframe.h"

struct LoopClosureResult {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    KeyFrame::ID loopclosure_keyframe_id_ = KeyFrame::kInvalidID;
    KeyFrame::ID candidate_keyframe_id_ = KeyFrame::kInvalidID;
    Mat4d match_pose_ = Mat4d::Identity(); // loopclosure to candidate
};

#endif //FUNNY_LIDAR_SLAM_LOOPCLOSURE_RESULT_H
