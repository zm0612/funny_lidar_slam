//
// Created by Zhang Zhimeng on 24-3-26.
//

#ifndef FUNNY_LIDAR_SLAM_FILE_MANAGER_H
#define FUNNY_LIDAR_SLAM_FILE_MANAGER_H

#include <string>
#include <dirent.h>
#include <unistd.h>
#include <sys/stat.h>
#include <glog/logging.h>

/*!
 * Creating one-level directory
 * 函数返回0： 目录创建成功、或目录已经存在；
 * 函数返回-1：目录创建失败；
 */
inline int MakeDir(const std::string &path_name) {
    int ret = 0;
    DIR *dir;
    dir = opendir(path_name.c_str()); //打开目录
    if (dir == nullptr) {
        ret = mkdir(path_name.c_str(), 0755);   //创建目录
        if (ret != 0) {
            LOG(ERROR) << "Make direction failed";
            return -1;
        }
        LOG(INFO) << "Make direction successfully";
    } else {
        LOG(INFO) << "Direction exist";
    }
    closedir(dir);//关闭目录
    return ret;
}

/*!
 * Create multi-level directories
 * 函数返回 0： 目录创建成功、或目录已经存在；
 * 函数返回-1：目录创建失败；
 */
inline int32_t MakeDirs(const std::string &path_name) {
    if (path_name.find('/') == std::string::npos) {
        LOG(ERROR) << "path error: " << path_name;
    }

    int ret = 0;
    DIR *dir;
    dir = opendir(path_name.c_str());
    if (dir == nullptr) {
        std::string string = path_name;
        const size_t string_len = path_name.size();

        for (size_t i = 1; i < string_len; i++) {
            if (string[i] == '/') {
                string[i] = '\0';
                if (access(string.c_str(), 0) != 0) {
                    ret = mkdir(string.c_str(), 0755);

                    if (ret != 0) {
                        LOG(ERROR) << "Make direction failed: " << path_name;
                        return -1;
                    }
                }
                string[i] = '/';
            }
        }

        if (string_len > 0 && access(string.c_str(), 0) != 0) {
            ret = mkdir(string.c_str(), 0755);

            if (ret != 0) {
                LOG(ERROR) << "Make direction failed: " << path_name;
                return -1;
            }
        }

        LOG(INFO) << "Make direction successfully";
    } else {
        LOG(INFO) << "Direction exist";
    }
    closedir(dir);
    return ret;
}

#endif //FUNNY_LIDAR_SLAM_FILE_MANAGER_H
