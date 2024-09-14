//
// Created by Zhang Zhimeng on 24-3-26.
//
#include "keyframe.h"

const std::string KeyFrame::kKeyFramePath = PROJECT_SOURCE_DIR + std::string("/data/keyframes");
const std::string KeyFrame::kPlanarCloudPath = kKeyFramePath + std::string("/planar_cloud_");
const std::string KeyFrame::kCornerCloudPath = kKeyFramePath + std::string("/corner_cloud_");
const std::string KeyFrame::kOrderedCloudPath = kKeyFramePath + std::string("/ordered_cloud_");
const std::string KeyFrame::kRawCloudPath = kKeyFramePath + std::string("/raw_cloud_");
const std::string KeyFrame::kPosePath = kKeyFramePath + std::string("/pose_");
