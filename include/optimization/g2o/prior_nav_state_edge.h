//
// Created by Zhang Zhimeng on 23-11-15.
//

#ifndef FUNNY_LIDAR_SLAM_PRIOR_NAV_STATE_EDGE_H
#define FUNNY_LIDAR_SLAM_PRIOR_NAV_STATE_EDGE_H

#include "common/data_type.h"
#include "common/sensor_data_type.h"
#include "optimization/g2o/vertex_type.h"

#include <g2o/core/base_multi_edge.h>

/*!
 * Prior navigation state edge
 *
 *  error = measure - estimate
 *
 *  error dim: 15
 *  vertex: R, V, P, delta_bg, delta_ba. all dim 3 x 1
 *
 *  So, Jacobian dim: 15 x 3
 */
class EdgePriorNavState final : public g2o::BaseMultiEdge<15, Vec15d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgePriorNavState(const NavStateData &nav_state, const Mat15d &info);

    bool read(std::istream &is) override {
        return false;
    }

    bool write(std::ostream &os) const override {
        return false;
    }

    void linearizeOplus() override;

    void computeError() override;

    Mat15d GetPosteriorInformation();

private:
    NavStateData nav_state_{};
};

#endif //FUNNY_LIDAR_SLAM_PRIOR_NAV_STATE_EDGE_H
