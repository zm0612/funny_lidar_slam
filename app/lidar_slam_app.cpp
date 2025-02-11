//
// Created by Zhang Zhimeng on 22-10-28.
//
#include <glog/logging.h>

#include <rclcpp/rclcpp.hpp>

#include "3rd/backward.hpp"
#include "common/file_manager.h"
#include "common/keyframe.h"
#include "slam/system.h"

#ifdef DEBUG
namespace backward {
backward::SignalHandling sh;
}
#endif

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;
  FLAGS_colorlogtostderr = true;

  CHECK(MakeDirs(KeyFrame::kKeyFramePath) == 0)
      << "Failed to create folder: " << KeyFrame::kKeyFramePath;

  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions();
  auto node = std::make_shared<System>(options);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  google::ShutdownGoogleLogging();

  return 0;
}
