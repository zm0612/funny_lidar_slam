//
// Created by Zhang Zhimeng on 24-3-12.
//
#include "optimization/g2o/loopclosure_optimizer.h"
#include "common/constant_variable.h"

LoopClosureOptimizer::LoopClosureOptimizer() :
    optimizer_ptr_(std::make_unique<g2o::SparseOptimizer>()) {
    using BlockSolverType = g2o::BlockSolverX;
    using LinearSolverType = g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType>;
    auto* solver = new g2o::OptimizationAlgorithmLevenberg(
        std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>())
    );

    optimizer_ptr_->setAlgorithm(solver);
}

void LoopClosureOptimizer::AddVertex(const Mat4d& pose, int id, bool is_fix) {
    auto* vertex_pose = new VertexType();
    vertex_pose->setId(id);
    vertex_pose->setFixed(is_fix);
    vertex_pose->setEstimate(pose);
    optimizer_ptr_->addVertex(vertex_pose);
}

void LoopClosureOptimizer::AddEdge(const Mat4d& delta_pose, const Mat6d& info, int id_i, int id_j) {
    auto* vertex_pose_i = dynamic_cast<VertexType*>(optimizer_ptr_->vertex(id_i));
    auto* vertex_pose_j = dynamic_cast<VertexType*>(optimizer_ptr_->vertex(id_j));

    CHECK_NOTNULL(vertex_pose_i);
    CHECK_NOTNULL(vertex_pose_j);

    auto edge = new EdgeType(delta_pose);
    edge->setInformation(info);
    edge->setVertex(0, vertex_pose_i);
    edge->setVertex(1, vertex_pose_j);
    edge->setId(static_cast<int>(optimizer_ptr_->edges().size()));
    //    edge->setRobustKernel(new g2o::RobustKernelHuber());

    optimizer_ptr_->addEdge(edge);
}

void LoopClosureOptimizer::Optimize(int num_iter) {
    if (optimizer_ptr_->edges().empty()) {
        return;
    }

    //Save(kDataPath + "path_before_optimization.g2o");

    optimizer_ptr_->initializeOptimization();
    optimizer_ptr_->setVerbose(false);
    optimizer_ptr_->optimize(num_iter);
}

Mat4d LoopClosureOptimizer::GetVertexEstimate(int id) {
    CHECK_GT(optimizer_ptr_->vertices().size(), id);
    auto* vertex = dynamic_cast<VertexType*>(optimizer_ptr_->vertex(id));
    CHECK_NOTNULL(vertex);
    return vertex->estimate().matrix();
}

void LoopClosureOptimizer::Save(const std::string& file_path) {
    std::ofstream fout(file_path, std::ios::trunc);

    // save vertices
    std::vector<const VertexType*> vertices;
    for (const auto& v : optimizer_ptr_->vertices()) {
        vertices.push_back(dynamic_cast<VertexType*>(v.second));
    }
    std::sort(vertices.begin(), vertices.end(),
              [](const VertexType* left, const VertexType* right) {
                  return left->id() < right->id();
              }
    );
    for (const auto& vertex : vertices) {
        vertex->write(fout);
    }

    // save edges
    std::vector<EdgeType*> edges;
    for (const auto& e : optimizer_ptr_->edges()) {
        edges.push_back(dynamic_cast<EdgeType*>(e));
    }
    std::sort(edges.begin(), edges.end(),
              [](const EdgeType* left, const EdgeType* right) {
                  return left->id() < right->id();
              }
    );
    for (const auto& edge : edges) {
        edge->write(fout);
    }

    fout.close();
}
