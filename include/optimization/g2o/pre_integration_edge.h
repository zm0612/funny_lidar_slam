//
// Created by Zhang Zhimeng on 23-10-21.
//

#ifndef FUNNY_LIDAR_SLAM_PRE_INTEGRATION_EDGE_H
#define FUNNY_LIDAR_SLAM_PRE_INTEGRATION_EDGE_H

#include "imu/pre_integration.h"

#include "common/data_type.h"
#include "common/math_function.h"

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_multi_edge.h>
#include <g2o/core/robust_kernel.h>

/*!
 * IMU预积分的优化边
 *
 * 顶点(优化变量)类型: i和j时刻的R,V,P，以及bias的增量，其顶点排列顺序如下：
 * [R_i, V_i, P_i, bg_i, ba_i, R_j, V_j, P_j]
 */
class EdgePreIntegration final : public g2o::BaseMultiEdge<9, Vec9d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit EdgePreIntegration(std::shared_ptr<const PreIntegration> pre_integration_ptr);

    bool read(std::istream &is) override {
        return false;
    }

    bool write(std::ostream &os) const override {
        return false;
    }

    void linearizeOplus() override;

    void computeError() override;

    Mat24d GetPosteriorInformation();

private:
    std::shared_ptr<const PreIntegration> pre_integration_ptr_ = nullptr;
};

#endif //FUNNY_LIDAR_SLAM_PRE_INTEGRATION_EDGE_H
