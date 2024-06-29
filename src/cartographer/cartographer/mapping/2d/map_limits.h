/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_MAPPING_2D_MAP_LIMITS_H_
#define CARTOGRAPHER_MAPPING_2D_MAP_LIMITS_H_

#include <utility>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/common/math.h"
#include "cartographer/mapping/2d/xy_index.h"
#include "cartographer/mapping/proto/map_limits.pb.h"
#include "cartographer/mapping/trajectory_node.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

///@class 管理地图边界
class MapLimits {
 public:
  MapLimits(const double resolution, const Eigen::Vector2d& max, const CellLimits& cell_limits)
      : resolution_(resolution), max_(max), cell_limits_(cell_limits) {
    CHECK_GT(resolution_, 0.);
    CHECK_GT(cell_limits.num_x_cells, 0.);
    CHECK_GT(cell_limits.num_y_cells, 0.);
  }

  explicit MapLimits(const proto::MapLimits& map_limits)
      : resolution_(map_limits.resolution()),
        max_(transform::ToEigen(map_limits.max())),
        cell_limits_(map_limits.cell_limits()) {}

  ///@brief Returns the cell size in meters. All cells are square and the resolution is the length of one side.
  double resolution() const { return resolution_; }

  ///@brief Returns the corner of the limits, i.e., all pixels have positions with smaller coordinates.
  const Eigen::Vector2d& max() const { return max_; }

  ///@brief Returns the limits of the grid in number of cells.
  const CellLimits& cell_limits() const { return cell_limits_; }

  ///@brief 物理坐标转像素坐标
  Eigen::Array2i GetCellIndex(const Eigen::Vector2f& point) const {
    /// Index values are row major and the top left has Eigen::Array2i::Zero() and contains (centered_max_x, centered_max_y).
    /// We need to flip and rotate.
    /// 物理坐标系和像素坐标系不同,需要翻转和旋转
    return Eigen::Array2i(common::RoundToInt((max_.y() - point.y()) / resolution_ - 0.5),
                          common::RoundToInt((max_.x() - point.x()) / resolution_ - 0.5));
  }

  ///@brief 像素坐标转物理坐标
  Eigen::Vector2f GetCellCenter(const Eigen::Array2i cell_index) const {
    return {max_.x() - resolution() * (cell_index[1] + 0.5), max_.y() - resolution() * (cell_index[0] + 0.5)};
  }

  ///@brief 当前栅格地图内部是否包含index
  bool Contains(const Eigen::Array2i& cell_index) const {
    return (Eigen::Array2i(0, 0) <= cell_index).all() &&
           (cell_index < Eigen::Array2i(cell_limits_.num_x_cells, cell_limits_.num_y_cells)).all();
  }

 private:
  double resolution_;       // 分辨率
  Eigen::Vector2d max_;     // 地图最大值(根据T_local_laser 增加物理尺寸获得)
  CellLimits cell_limits_;  // x y的格子数
};

inline proto::MapLimits ToProto(const MapLimits& map_limits) {
  proto::MapLimits result;
  result.set_resolution(map_limits.resolution());
  *result.mutable_max() = transform::ToProto(map_limits.max());
  *result.mutable_cell_limits() = ToProto(map_limits.cell_limits());
  return result;
}

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_MAP_LIMITS_H_
