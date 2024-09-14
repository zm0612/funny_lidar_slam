//
// Created by Zhang Zhimeng on 2022/4/25.
//

#ifndef FUNNY_LIDAR_SLAM_TIMER_H
#define FUNNY_LIDAR_SLAM_TIMER_H

#include <string>
#include <iostream>
#include <chrono>

#include <glog/logging.h>

class Timer {
public:
    Timer() {
        Begin();
    }

    void Begin() {
        start_ = std::chrono::system_clock::now();
    }

    void End(const std::string &task_name) {
        end_ = std::chrono::system_clock::now();
        std::chrono::duration<double> use_time = end_ - start_;

        LOG(INFO) << task_name << " use time(ms): " << use_time.count() * 1000;
    }

    double End() {
        end_ = std::chrono::system_clock::now();
        std::chrono::duration<double> use_time = end_ - start_;
        return use_time.count() * 1000.0;
    }

private:
    std::chrono::time_point<std::chrono::system_clock> start_, end_;
};

#endif //FUNNY_LIDAR_SLAM_TIMER_H
