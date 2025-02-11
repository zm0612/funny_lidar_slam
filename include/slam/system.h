//
// Created by Zhang Zhimeng on 22-10-28.
//

#ifndef FUNNY_LIDAR_SLAM_SYSTEM_H
#define FUNNY_LIDAR_SLAM_SYSTEM_H

#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <condition_variable>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <thread>

#include "common/frame.h"
#include "common/keyframe.h"
#include "common/loopclosure_result.h"
#include "funny_lidar_slam/srv/save_map.hpp"
#include "imu/imu_data_searcher.h"
#include "lidar/pointcloud_cluster.h"
#include "optimization/g2o/loopclosure_optimizer.h"

class FrontEnd;

class PreProcessing;

class LoopClosure;

class Localization;

enum SLAM_MODE { UNKNOWN = 0, MAPPING = 1, LOCALIZATION = 2 };

class System : public rclcpp::Node {
 public:
  explicit System(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  System() = delete;

  ~System();

  void Run();

  void RunMapping();

  void RunLocalization();

 private:
  static void InitLidarModel();

  void InitMappingMode();

  void InitLocalizationMode();

  void InitSubscriber();

  void InitPublisher();

  void InitMappingPublisher();

  void InitLocalizationPublisher();

  void InitSaveMapServer();

  void InitConfigParameters();

  void PublishMappingKeyFramePath();

  void PublishLocalizationPath();

  void LidarStandardMsgCallBack(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_ros_ptr);

  void LidarLivoxMsgCallBack(
      const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& msg);

  void LocalizationInitPoseMsgCallBack(
      const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& msg);

  void ImuMsgCallBack(const sensor_msgs::msg::Imu::ConstSharedPtr& imu_ptr);

  bool HasLoopClosure();

  void PerformLoopclosureOptimization();

  void PublishMappingFrameCloud(const Frame::Ptr& frame, const Mat4d& pose);

  void PublishMappingKeyFrameCloud(const KeyFrame::Ptr& keyframe);

  void PublishTF(const Mat4d& pose, TimeStampUs timestamp);

  /*!
   * Process frame cache
   * @return if true, current frame is keyframe
   */
  bool ProcessMappingFrameCache();

  bool ProcessLocalizationResultCache();

  [[nodiscard]] bool IsKeyFrame(const Mat4d& delta_pose) const;

  static bool InitIMU(const IMUData& imu_data, Vec3d& init_mean_acc);

  void VisualizeGlobalMap();

  bool SaveMap(std::shared_ptr<funny_lidar_slam::srv::SaveMap::Request> req,
               std::shared_ptr<funny_lidar_slam::srv::SaveMap::Response> res);

 private:
  // ros2 subscriber
  rclcpp::CallbackGroup::SharedPtr imu_cb_group_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::CallbackGroup::SharedPtr lidar_cb_group_;
  rclcpp::SubscriptionBase::SharedPtr lidar_sub_;
  rclcpp::CallbackGroup::SharedPtr init_pose_cb_group_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      localization_init_pose_sub_;

  // ros server
  rclcpp::CallbackGroup::SharedPtr save_map_cb_group_;
  rclcpp::Service<funny_lidar_slam::srv::SaveMap>::SharedPtr save_map_server_;

  // ros publisher
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr localization_path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      localization_local_cloud_map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      localization_global_cloud_map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      localization_current_lidar_cloud_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mapping_keyframe_path_pub_;
  //
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      mapping_keyframe_path_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      mapping_curr_corner_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      mapping_curr_planer_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      mapping_curr_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      mapping_curr_keyframe_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      mapping_global_map_cloud_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster>
      tf_broadcaster_;  // lidar odometer
  rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
  rclcpp::TimerBase::SharedPtr system_timer_;  // 执行定时器

  // frontend
  std::shared_ptr<FrontEnd> front_end_ptr_ = nullptr;

  // pre-processing
  std::shared_ptr<PreProcessing> pre_processing_ptr_ = nullptr;

  // loopclosure
  std::shared_ptr<LoopClosure> loop_closure_ptr_ = nullptr;

  // localization
  std::shared_ptr<Localization> localization_ptr_ = nullptr;

  std::shared_ptr<std::thread> frontend_thread_ptr_ = nullptr;
  std::shared_ptr<std::thread> pre_processing_thread_ptr_ = nullptr;
  std::shared_ptr<std::thread> loop_closure_thread_ptr_ = nullptr;
  std::shared_ptr<std::thread> visualize_global_map_thread_ptr_ = nullptr;
  std::shared_ptr<std::thread> localization_thread_ptr_ = nullptr;

 public:
  // ros node handle
  // std::shared_ptr<ros::NodeHandle> node_handle_ptr_ = nullptr;

  // data searcher use to search imu data
  std::shared_ptr<IMUDataSearcher> imu_data_searcher_ptr_ = nullptr;
  std::shared_ptr<LoopClosureOptimizer> loop_closure_optimizer_ptr_ = nullptr;

  // condition variable
  std::condition_variable cv_frontend_;
  std::condition_variable cv_localization_;
  std::condition_variable cv_preprocessing_;

  std::atomic_bool has_imu_init_{false};
  std::atomic_bool need_update_global_map_visualization_{false};

  // mutex
  std::mutex mutex_keyframes_;
  std::mutex mutex_raw_cloud_deque_;
  std::mutex mutex_cloud_cluster_deque_;
  std::mutex mutex_loopclosure_results_;
  std::mutex mutex_frame_temp_deque_;
  std::mutex mutex_localization_results_deque_;

  // deque
  std::deque<NavStateData::Ptr> localization_results_deque_;
  std::deque<PointcloudClusterPtr> cloud_cluster_deque_;
  std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr> raw_cloud_deque_;
  std::deque<LoopClosureResult> loopclosure_result_deque_;
  std::deque<Frame::Ptr> frame_temp_deque_;

  // keyframes
  std::vector<KeyFrame::Ptr> keyframes_;

  // localization ros path
  nav_msgs::msg::Path localization_path_;

  SLAM_MODE slam_mode_ = SLAM_MODE::UNKNOWN;
};

#endif  // FUNNY_LIDAR_SLAM_SYSTEM_H
