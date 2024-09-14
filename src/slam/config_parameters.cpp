//
// Created by Zhang Zhimeng on 23-11-26.
//

#include "slam/config_parameters.h"

#include <mutex>

std::unique_ptr<ConfigParameters> ConfigParameters::instance_ = nullptr;

ConfigParameters& ConfigParameters::Instance() {
    if (instance_ == nullptr) {
        static std::once_flag flag;
        std::call_once(flag, [&] {
            instance_.reset(new(std::nothrow) ConfigParameters());
        });
    }

    return *instance_;
}
