//
// Created by Zhang Zhimeng on 22-10-28.
//
#include "slam/system.h"
#include "common/file_manager.h"
#include "common/keyframe.h"

#include "3rd/backward.hpp"

#include <ros/ros.h>
#include <glog/logging.h>

namespace backward {
backward::SignalHandling sh;
}

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = true;
    FLAGS_colorlogtostderr = true;

    CHECK(MakeDirs(KeyFrame::kKeyFramePath) == 0) << "Failed to create folder: " << KeyFrame::kKeyFramePath;

    ros::init(argc, argv, "funny_lidar_slam");

    const auto node_handle_ptr = std::make_shared<ros::NodeHandle>();

    System system(node_handle_ptr);

    ros::Rate rate(1000);

    while (ros::ok()) {
        ros::spinOnce();

        system.Run();

        rate.sleep();
    }

    google::ShutdownGoogleLogging();

    return 0;
}
