//
// Created by Zhang Zhimeng on 23-11-22.
//

#ifndef FUNNY_LIDAR_SLAM_RELATIVE_POSE_EDGE_H
#define FUNNY_LIDAR_SLAM_RELATIVE_POSE_EDGE_H

#include "common/data_type.h"
#include "optimization/g2o/vertex_type.h"

#include <g2o/core/base_multi_edge.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/types/slam3d/vertex_se3.h>

/*!
 * loopclosure edge
 *
 *  R_error = R_measure.transpose * R_i.transpose * R_j
 *  t_error = t_measure_ - (R_i.transpose * t_j - R_i.transpose * t_i);
 *
 *  error dim: 6
 *  vertex: R, P.  each dim 3 x 1
 *
 *  So, Jacobian 0 dim: 6 x 6
 *      Jacobian 1 dim: 6 x 6
 */
class EdgeRelativePose final : public g2o::BaseMultiEdge<6, Mat4d> {
public:
    using VertexPoseType = VertexPose;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeRelativePose() = delete;

    explicit EdgeRelativePose(const Mat4d &measure);

    bool read(std::istream &is) override {
        double data[7];
        for (int i = 0; i < 7; i++) {
            is >> data[i];
        }

        Eigen::Quaterniond q(data[6], data[3], data[4], data[5]);
        q.normalize();

        Mat4d T = Mat4d::Identity();
        T.block<3, 3>(0, 0) = q.toRotationMatrix();
        T.block<3, 1>(0, 3) = Vec3d(data[0], data[1], data[2]);

        setMeasurement(T);

        for (int i = 0; i < information().rows() && is.good(); i++) {
            for (int j = i; j < information().cols() && is.good(); j++) {
                is >> information()(i, j);
                if (i != j) information()(j, i) = information()(i, j);
            }
        }
        return true;
    }

    bool write(std::ostream &os) const override {
        os << "EDGE_SE3:QUAT ";
        auto *v1 = static_cast<VertexPose *>(_vertices[0]);
        auto *v2 = static_cast<VertexPose *>(_vertices[1]);
        os << v1->id() << " " << v2->id() << " ";

        const Mat4d &T = T_measure_;
        Eigen::Quaterniond q(T.block<3, 3>(0, 0));
        q.normalize();

        os << T.block<3, 1>(0, 3).transpose() << " ";
        os << q.coeffs()[0] << " " << q.coeffs()[1] << " " << q.coeffs()[2] << " " << q.coeffs()[3] << " ";

        for (int i = 0; i < information().rows(); i++) {
            for (int j = i; j < information().cols(); j++) {
                os << information()(i, j) << " ";
            }
        }
        os << std::endl;
        return true;
    }

    void linearizeOplus() override;

    void computeError() override;

private:
    Mat4d T_measure_ = Mat4d::Identity();
};


#endif //FUNNY_LIDAR_SLAM_RELATIVE_POSE_EDGE_H
