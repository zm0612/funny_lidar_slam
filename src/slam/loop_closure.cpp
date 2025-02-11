//
// Created by Zhang Zhimeng on 24-2-7.
//
#include "slam/loop_closure.h"

#include <pcl/io/pcd_io.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>

#include <execution>

#include "common/constant_variable.h"
#include "common/loopclosure_result.h"
#include "common/pointcloud_utility.h"
#include "common/timer.h"

LoopClosure::LoopClosure(System* system_ptr) : system_ptr_(system_ptr) {
  loopclosure_method_ = kLoopClosureByDistance;

  near_neighbor_distance_threshold_ =
      ConfigParameters::Instance().lc_near_neighbor_distance_threshold_;
  skip_near_keyframe_threshold_ =
      ConfigParameters::Instance().lc_skip_near_keyframe_threshold_;
  skip_near_loopclosure_threshold_ =
      ConfigParameters::Instance().lc_skip_near_loopclosure_threshold_;
  candidate_local_map_left_range_ =
      ConfigParameters::Instance().lc_candidate_local_map_left_range_;
  candidate_local_map_right_range_ =
      ConfigParameters::Instance().lc_candidate_local_map_right_range_;
  loopclosure_local_map_left_range_ =
      ConfigParameters::Instance().lc_loopclosure_local_map_left_range_;
  registration_converge_threshold_ =
      ConfigParameters::Instance().lc_registration_converge_threshold_;

  CHECK_NE(near_neighbor_distance_threshold_, DoubleNaN);
  CHECK_NE(registration_converge_threshold_, FloatNaN);
  CHECK_NE(skip_near_keyframe_threshold_, IntNaN);
  CHECK_NE(skip_near_loopclosure_threshold_, IntNaN);
  CHECK_NE(candidate_local_map_left_range_, IntNaN);
  CHECK_NE(candidate_local_map_right_range_, IntNaN);
  CHECK_NE(loopclosure_local_map_left_range_, IntNaN);
}

void LoopClosure::Run() {
  LOG(INFO) << "\033[1;32m----> LoopClosure Started.\033[0m";
  rclcpp::Rate rate(1);

  while (rclcpp::ok()) {
    Timer timer;

    // ros::spinOnce();
    rate.sleep();

    curr_keyframe_id_ = GetLatestKeyFrameID();
    const bool has_new_keyframe = HasNewKeyFrame();

    // If a successful loopclosure detection has just been performed, skip.
    const bool has_near_loopclosure =
        (curr_keyframe_id_ - last_loopclosure_id_) <
        skip_near_loopclosure_threshold_;

    if (!has_new_keyframe || has_near_loopclosure) {
      continue;
    }

    LoopClosurePair loop_closure_pair;

    if (loopclosure_method_ == kLoopClosureByDistance) {
      loop_closure_pair = DetectByDistance();
    } else if (loopclosure_method_ == kLoopClosureByFeature) {
      LOG(FATAL) << "LoopClosureByFeature will be supported soon";
      loop_closure_pair = DetectByFeature();
    } else {
      LOG(ERROR) << "Loopclosure detection method setting error. Only support: "
                 << kLoopClosureByDistance << " " << kLoopClosureByFeature;
      LOG(ERROR) << "You will not be able to use the loopclosure detection";
    }

    if (loop_closure_pair.second == KeyFrame::kInvalidID) {
      continue;
    }

    const auto candidate_local_map =
        GetSubMap(loop_closure_pair.second, candidate_local_map_left_range_,
                  candidate_local_map_right_range_, false);

    const auto loopclosure_local_map = GetSubMap(
        loop_closure_pair.first, loopclosure_local_map_left_range_, 0, true);

    CHECK(!loopclosure_local_map->empty());
    CHECK(!candidate_local_map->empty());

    Mat4d match_pose = GetKeyFramePoseByID(
        loop_closure_pair.first);  // loopclosure keyframe pose in world.
    const auto fitness_score =
        Match(loopclosure_local_map, candidate_local_map, match_pose);

#ifndef NDEBUG
    pcl::io::savePCDFileBinary(kDataPath + "loopclosure_target.pcd",
                               *candidate_local_map);
    pcl::io::savePCDFileBinary(kDataPath + "loopclosure_source.pcd",
                               *loopclosure_local_map);

    auto source_cloud_transformed =
        TransformPointCloud(*loopclosure_local_map, match_pose);
    pcl::io::savePCDFileBinary(kDataPath + "loopclosure_source_transformed.pcd",
                               source_cloud_transformed);
#endif

    const bool is_success =
        fitness_score < registration_converge_threshold_ ? true : false;

    const Mat4d pose_candidate_keyframe =
        GetKeyFramePoseByID(loop_closure_pair.second);
    const Mat4d delta_pose = pose_candidate_keyframe.inverse() * match_pose;

    if (is_success) {
      SetLoopClosureResult(loop_closure_pair, delta_pose);
      last_loopclosure_id_ = curr_keyframe_id_;
    } else {
      last_detection_id_ = curr_keyframe_id_;
      LOG(WARNING) << "LoopClosure match failed, fitness score: "
                   << fitness_score;
    }

    DLOG(INFO) << "LoopClosure use time(ms): " << timer.End();
  }
}

LoopClosure::LoopClosurePair LoopClosure::DetectByDistance() {
  const auto candidate_keyframes_id =
      GetNearKeyFramesResult(near_neighbor_distance_threshold_);
  const auto possible_loopclosure_id = CheckCandidateKeyFrames(
      candidate_keyframes_id, skip_near_keyframe_threshold_);
  return std::make_pair(curr_keyframe_id_, possible_loopclosure_id);
}

LoopClosure::LoopClosurePair LoopClosure::DetectByFeature() { return {}; }

KeyFrame::ID LoopClosure::GetLatestKeyFrameID() const {
  std::lock_guard<std::mutex> lg(system_ptr_->mutex_keyframes_);

  if (system_ptr_->keyframes_.empty()) {
    return KeyFrame::kInvalidID;
  } else {
    return system_ptr_->keyframes_.back()->id_;
  }
}

bool LoopClosure::HasNewKeyFrame() const {
  return curr_keyframe_id_ > last_detection_id_;
}

LoopClosure::CandidateKeyFramesResult LoopClosure::GetNearKeyFramesResult(
    double dist_thre) const {
  std::lock_guard<std::mutex> lg(system_ptr_->mutex_keyframes_);

  CandidateKeyFramesResult candidate_keyframe_result;
  const Vec3d& curr_keyframe_position =
      system_ptr_->keyframes_[curr_keyframe_id_]->pose_.block<3, 1>(0, 3);
  for (const auto& keyframe : system_ptr_->keyframes_) {
    const Vec3d& keyframe_position = keyframe->pose_.block<3, 1>(0, 3);
    const double dist = (curr_keyframe_position - keyframe_position).norm();

    if (dist < dist_thre) {
      std::pair<KeyFrame::ID, double> id_dist;
      id_dist.first = keyframe->id_;
      id_dist.second = dist;
      candidate_keyframe_result.emplace_back(std::move(id_dist));
    }
  }

  std::sort(std::execution::par, candidate_keyframe_result.begin(),
            candidate_keyframe_result.end(),
            [](const KeyFrameIdDistancePair& left,
               const KeyFrameIdDistancePair& right) -> bool {
              return left.second < right.second;
            });

  return candidate_keyframe_result;
}

KeyFrame::ID LoopClosure::CheckCandidateKeyFrames(
    const CandidateKeyFramesResult& candidate_keyframes_result,
    KeyFrame::ID keyframe_index_span_thre) const {
  if (candidate_keyframes_result.empty()) {
    return KeyFrame::kInvalidID;
  }

  for (const auto& keyframe_result : candidate_keyframes_result) {
    const KeyFrame::ID& candidate_keyframe_id = keyframe_result.first;
    if (curr_keyframe_id_ - candidate_keyframe_id > keyframe_index_span_thre) {
      return candidate_keyframe_id;
    }
  }

  return KeyFrame::kInvalidID;
}

LoopClosure::PointCloudType::Ptr LoopClosure::GetSubMap(
    const KeyFrame::ID keyframe_id, const KeyFrame::ID left_range,
    const KeyFrame::ID right_range, const bool use_local_pose) const {
  std::vector<PointCloudType::Ptr> local_map;
  std::vector<Mat4d> poses;

  Mat4d ref_pose;

  // Get the point clouds of the previous and next frames of the candidate
  // keyframe
  {
    std::lock_guard<std::mutex> lg(system_ptr_->mutex_keyframes_);

    ref_pose = system_ptr_->keyframes_[keyframe_id]->pose_;

    for (int i = -left_range; i <= right_range; ++i) {
      KeyFrame::ID keyframe_id_temp = keyframe_id + i;

      if (keyframe_id_temp < 0 ||
          keyframe_id_temp >=
              static_cast<KeyFrame::ID>(system_ptr_->keyframes_.size())) {
        continue;
      }

      const auto cloud =
          system_ptr_->keyframes_[keyframe_id_temp]->LoadOrderedCloud();
      CHECK_NOTNULL(cloud);
      const Mat4d& cloud_pose =
          system_ptr_->keyframes_[keyframe_id_temp]->pose_;
      local_map.push_back(cloud);
      poses.push_back(cloud_pose);
    }
  }

  if (use_local_pose) {
    Mat4d ref_pose_inv = ref_pose.inverse();
    for (auto& pose : poses) {
      pose = ref_pose_inv * pose;
    }
  }

  // Downsample every local keyframe cloud
  for (auto& cloud : local_map) {
    cloud = VoxelGridCloud(cloud, 0.2);
  }

  // Merge local map cloud
  PointCloudType::Ptr local_map_downsample(new PointCloudType);
  for (unsigned int i = 0; i < local_map.size(); ++i) {
    const auto& cloud = local_map[i];
    PointCloudType::Ptr transformed_cloud =
        TransformPointCloud(cloud, poses[i]);
    *local_map_downsample += *transformed_cloud;
  }

  return local_map_downsample;
}

float LoopClosure::Match(const PointCloudType::Ptr& source_cloud,
                         const PointCloudType::Ptr& target_cloud, Mat4d& pose) {
  PointCloudType::Ptr output_cloud(new PointCloudType);

  static const std::vector<float> resolution{10.0, 5.0, 3.0, 2.0};

  pcl::NormalDistributionsTransform<PointType, PointType> ndt;
  ndt.setStepSize(0.5);
  ndt.setEuclideanFitnessEpsilon(1e-3);
  ndt.setMaximumIterations(30);

  for (const auto r : resolution) {
    ndt.setResolution(r);
    const auto source_cloud_rough = VoxelGridCloud(source_cloud, r * 0.2f);
    const auto target_cloud_rough = VoxelGridCloud(target_cloud, r * 0.2f);
    ndt.setInputSource(source_cloud_rough);
    ndt.setInputTarget(target_cloud_rough);
    ndt.align(*output_cloud, pose.cast<float>());
    pose = ndt.getFinalTransformation().cast<double>();
  }

  const auto source_cloud_rough = VoxelGridCloud(source_cloud, 0.5f);
  const auto target_cloud_rough = VoxelGridCloud(target_cloud, 0.4f);
  pcl::GeneralizedIterativeClosestPoint<PointType, PointType> gicp;
  gicp.setMaximumIterations(30);
  gicp.setMaxCorrespondenceDistance(2.0);
  gicp.setInputTarget(target_cloud_rough);
  gicp.setInputSource(source_cloud_rough);
  gicp.setEuclideanFitnessEpsilon(1e-3);
  gicp.align(*output_cloud, pose.cast<float>());
  pose = gicp.getFinalTransformation().cast<double>();

  return static_cast<float>(gicp.getFitnessScore());
}

void LoopClosure::SetLoopClosureResult(const LoopClosurePair& loop_closure_pair,
                                       const Mat4d& match_pose) const {
  LoopClosureResult loop_closure_result;
  loop_closure_result.loopclosure_keyframe_id_ = loop_closure_pair.first;
  loop_closure_result.candidate_keyframe_id_ = loop_closure_pair.second;
  loop_closure_result.match_pose_ = match_pose;

  {
    std::lock_guard<std::mutex> lg(system_ptr_->mutex_loopclosure_results_);
    system_ptr_->loopclosure_result_deque_.push_back(
        std::move(loop_closure_result));
  }
}

Mat4d LoopClosure::GetKeyFramePoseByID(KeyFrame::ID id) const {
  std::lock_guard<std::mutex> lg(system_ptr_->mutex_keyframes_);
  return system_ptr_->keyframes_[id]->pose_;
}
