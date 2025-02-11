//
// Created by Zhang Zhimeng on 24-6-2.
//
#include "slam/split_map.h"

#include <pcl/io/pcd_io.h>

#include <fstream>
#include <map>

#include "common/constant_variable.h"
#include "common/hash_function.h"

SplitMap::SplitMap(double grid_size) {
  grid_size_ = grid_size;
  grid_half_size_ = grid_size_ / 2.0;
}

void SplitMap::SetTileMapGridSize(double grid_size) {
  grid_size_ = grid_size;
  grid_half_size_ = grid_size_ / 2.0;
}

void SplitMap::Split(const PCLPointCloudXYZI& map_cloud) const {
  LOG(INFO) << "Start splitting the global cloud map ...";

  std::map<Vec2i, PCLPointCloudXYZI::Ptr, LessGridIndex> map_data;

  for (const auto& pt : map_cloud.points) {
    int gx = std::floor((pt.x - grid_half_size_) / grid_size_);
    int gy = std::floor((pt.y - grid_half_size_) / grid_size_);
    Vec2i key(gx, gy);
    auto iter = map_data.find(key);
    if (iter == map_data.end()) {
      PCLPointCloudXYZI::Ptr cloud(new PCLPointCloudXYZI);
      cloud->emplace_back(pt);
      map_data.emplace(key, cloud);
    } else {
      iter->second->emplace_back(pt);
    }
  }

  CHECK(MakeDirs(kTileMapFolder) == 0)
      << "Failed to open folder: " << kTileMapFolder;

  std::ofstream tile_map_indices_file(kTileMapFolder + kTileMapIndicesFileName,
                                      std::ios::trunc);

  for (auto& local_grid_map : map_data) {
    tile_map_indices_file << local_grid_map.first.x() << " "
                          << local_grid_map.first.y() << std::endl;

    pcl::io::savePCDFileBinary(
        kTileMapFolder + TILE_MAP_NAME(local_grid_map.first),
        *local_grid_map.second);
  }

  LOG(INFO) << "Split map done.";
}
