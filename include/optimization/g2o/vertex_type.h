//
// Created by Zhang Zhimeng on 23-10-21.
//

#ifndef FUNNY_LIDAR_SLAM_VERTEX_TYPE_H
#define FUNNY_LIDAR_SLAM_VERTEX_TYPE_H

#include "common/data_type.h"
#include "common/math_function.h"

#include <g2o/core/base_vertex.h>

class VertexPose : public g2o::BaseVertex<6, Mat4d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VertexPose() = default;

    bool read(std::istream &is) override {
        double data[7];
        for (int i = 0; i < 7; i++) {
            is >> data[i];
        }

        Mat4d T = Mat4d::Identity();
        T.block<3, 3>(0, 0) = Eigen::Quaterniond(data[6], data[3], data[4], data[5]).toRotationMatrix();
        T.block<3, 1>(0, 3) = Vec3d(data[0], data[1], data[2]);

        setEstimate(T);

        return true;
    }

    bool write(std::ostream &os) const override {
        os << "VERTEX_SE3:QUAT ";
        os << id() << " ";
        Eigen::Quaterniond q = Eigen::Quaterniond(_estimate.block<3, 3>(0, 0));
        q.normalize();
        os << _estimate.block<3, 1>(0, 3).transpose() << " ";
        os << q.coeffs()[0] << " " << q.coeffs()[1] << " " << q.coeffs()[2] << " " << q.coeffs()[3] << std::endl;

        if (fixed()) {
            os << "FIX " << id() << std::endl;
        }

        return true;
    }

    void setToOriginImpl() override {
        _estimate.setIdentity();
    }

    // Use left perturbation
    void oplusImpl(const double *update) override {
        Mat4d delta_T = SE3Exp(Eigen::Map<const Vec6d>(&update[0]));
        _estimate = delta_T * _estimate;
    }
};

class VertexRotation : public g2o::BaseVertex<3, Mat3d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VertexRotation() = default;

    bool read(std::istream &is) override {
        return false;
    }

    bool write(std::ostream &os) const override {
        return false;
    }

    void setToOriginImpl() override {
        _estimate.setIdentity();
    }

    void oplusImpl(const double *v) override {
        // Vertex right multiplication perturbation
        _estimate = _estimate * SO3Exp(Eigen::Map<const Vec3d>(v));
    }
};


class VertexVec3d : public g2o::BaseVertex<3, Vec3d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VertexVec3d() = default;

    bool read(std::istream &is) override {
        return false;
    }

    bool write(std::ostream &os) const override {
        return false;
    }

    void setToOriginImpl() override {
        _estimate.setZero();
    }

    void oplusImpl(const double *v) override {
        _estimate += Eigen::Map<const Vec3d>(v);
    }
};

class VertexPosition : public VertexVec3d {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VertexPosition() = default;
};

class VertexVelocity : public VertexVec3d {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VertexVelocity() = default;
};

class VertexBiasGyro : public VertexVec3d {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VertexBiasGyro() = default;
};

class VertexBiasAccel : public VertexVec3d {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VertexBiasAccel() = default;
};

#endif //FUNNY_LIDAR_SLAM_VERTEX_TYPE_H
