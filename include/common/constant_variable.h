//
// Created by Zhang Zhimeng on 23-12-30.
//

#ifndef FUNNY_LIDAR_SLAM_CONSTANT_VARIABLE_H
#define FUNNY_LIDAR_SLAM_CONSTANT_VARIABLE_H

#include <string>

#define DoubleNaN (std::numeric_limits<double>::max())
#define FloatNaN (std::numeric_limits<float>::max())
#define Int64NaN (std::numeric_limits<int64_t>::max())
#define IntNaN (std::numeric_limits<int>::max())
#define SizeNaN (std::numeric_limits<unsigned long>::max())
#define StringEmpty (std::string(""))

///////////////////////////////////////////////////////
/// The following strings cannot be changed at will ///
///////////////////////////////////////////////////////

static const std::string kPointToPlane_IVOX = "PointToPlane_IVOX";
static const std::string kPointToPlane_KdTree = "PointToPlane_KdTree";
static const std::string kLoamFull_KdTree = "LoamFull_KdTree";
static const std::string kIcpOptimized = "IcpOptimized";
static const std::string kIncrementalNDT = "IncrementalNDT";

static const std::string kFusionLooseCoupling = "LooseCoupling";
static const std::string kFusionTightCouplingKF = "TightCouplingKF";
static const std::string kFusionTightCouplingOptimization = "TightCouplingOptimization";

static const std::string kLoopClosureByDistance = "LoopClosureByDistance";
static const std::string kLoopClosureByFeature = "LoopClosureByFeature";

static const std::string kDataPath = std::string(PROJECT_SOURCE_DIR) + "/data/";
static const std::string kTileMapFolder = std::string(PROJECT_SOURCE_DIR) + "/data/tile_map/";
static const std::string kTileMapIndicesFileName = "tile_map_indices.txt";
static const std::string kGlobalMapFileName = "map.pcd";

static const std::string kRosMapFrameID = "map";
static const std::string kRosLidarFrameID = "lidar";

#endif //FUNNY_LIDAR_SLAM_CONSTANT_VARIABLE_H
