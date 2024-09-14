//
// Created by Zhang Zhimeng on 22-10-28.
//

#ifndef FUNNY_LIDAR_SLAM_SYSTEM_H
#define FUNNY_LIDAR_SLAM_SYSTEM_H

#include "common/frame.h"
#include "common/keyframe.h"
#include "imu/imu_data_searcher.h"
#include "lidar/pointcloud_cluster.h"
#include "funny_lidar_slam/save_map.h"
#include "common/loopclosure_result.h"
#include "optimization/g2o/loopclosure_optimizer.h"

#include <livox_ros_driver/CustomMsg.h>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>

#include <condition_variable>
#include <thread>

class FrontEnd;

class PreProcessing;

class LoopClosure;

class Localization;

enum SLAM_MODE {
    UNKNOWN = 0,
    MAPPING = 1,
    LOCALIZATION = 2
};

class System {
public:
    explicit System(std::shared_ptr<ros::NodeHandle> node_handle_ptr);

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

    void LidarStandardMsgCallBack(const sensor_msgs::PointCloud2Ptr& cloud_ros_ptr);

    void LidarLivoxMsgCallBack(const livox_ros_driver::CustomMsg::ConstPtr& msg);

    void LocalizationInitPoseMsgCallBack(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);

    void ImuMsgCallBack(const sensor_msgs::ImuPtr& imu_ptr);

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

    bool SaveMap(funny_lidar_slam::save_map::Request& req,
                 funny_lidar_slam::save_map::Response& res);

private:
    // ros subscriber
    ros::Subscriber imu_sub_;
    ros::Subscriber lidar_sub_;
    ros::Subscriber localization_init_pose_sub_;

    // ros server
    ros::ServiceServer save_map_server_;

    // ros publisher
    ros::Publisher localization_path_pub_;
    ros::Publisher localization_local_cloud_map_pub_;
    ros::Publisher localization_global_cloud_map_pub_;
    ros::Publisher localization_current_lidar_cloud_pub_;
    ros::Publisher mapping_keyframe_path_pub_;
    ros::Publisher mapping_keyframe_path_cloud_pub_;
    ros::Publisher mapping_curr_corner_cloud_pub_;
    ros::Publisher mapping_curr_planer_cloud_pub_;
    ros::Publisher mapping_curr_cloud_pub_;
    ros::Publisher mapping_curr_keyframe_cloud_pub_;
    ros::Publisher mapping_global_map_cloud_pub_;
    tf::TransformBroadcaster tf_broadcaster_; // lidar odometer

    // frontend
    FrontEnd* front_end_ptr_ = nullptr;

    // pre-processing
    PreProcessing* pre_processing_ptr_ = nullptr;

    // loopclosure
    LoopClosure* loop_closure_ptr_ = nullptr;

    // localization
    Localization* localization_ptr_ = nullptr;

    std::thread* frontend_thread_ptr_ = nullptr;
    std::thread* pre_processing_thread_ptr_ = nullptr;
    std::thread* loop_closure_thread_ptr_ = nullptr;
    std::thread* visualize_global_map_thread_ptr_ = nullptr;
    std::thread* localization_thread_ptr_ = nullptr;

public:
    // ros node handle
    std::shared_ptr<ros::NodeHandle> node_handle_ptr_ = nullptr;

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
    std::deque<sensor_msgs::PointCloud2Ptr> raw_cloud_deque_;
    std::deque<LoopClosureResult> loopclosure_result_deque_;
    std::deque<Frame::Ptr> frame_temp_deque_;

    // keyframes
    std::vector<KeyFrame::Ptr> keyframes_;

    // localization ros path
    nav_msgs::Path localization_path_;

    SLAM_MODE slam_mode_ = SLAM_MODE::UNKNOWN;
};

#endif //FUNNY_LIDAR_SLAM_SYSTEM_H
