//
// Created by Zhang Zhimeng on 24-1-17.
//
#ifndef FUNNY_LIDAR_SLAM_ACCEL_BIAS_PRIOR_EDGE_H
#define FUNNY_LIDAR_SLAM_ACCEL_BIAS_PRIOR_EDGE_H

#include "common/data_type.h"
#include "optimization/g2o/vertex_type.h"

#include <g2o/core/base_unary_edge.h>

/*!
 *  accel bias prior edge
 *
 *  error = measure - estimate
 *
 *  error dim: 3
 *  vertex: P.  all dim 3 x 1
 *
 *  So, Jacobian dim: 3 x 3
 */
class EdgeAccelBiasPrior final : public g2o::BaseUnaryEdge<3, Vec3d, VertexBiasAccel> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeAccelBiasPrior() = default;

    bool read(std::istream &is) override {
        return false;
    }

    bool write(std::ostream &os) const override {
        return false;
    }

    void linearizeOplus() override;

    void computeError() override;

    Mat3d GetPosteriorInformation();
};

#endif //FUNNY_LIDAR_SLAM_ACCEL_BIAS_PRIOR_EDGE_H
