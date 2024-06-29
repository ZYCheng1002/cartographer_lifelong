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

#include "cartographer/mapping/2d/probability_grid_range_data_inserter_2d.h"

#include <cstdlib>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/mapping/2d/xy_index.h"
#include "cartographer/mapping/internal/2d/ray_to_pixel_mask.h"
#include "cartographer/mapping/probability_values.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace {

constexpr int kSubpixelScale = 1000;  // 射线投射的起点和终点的亚像素精度因子

///@brief 获取包围盒
void GrowAsNeeded(const sensor::RangeData& range_data, ProbabilityGrid* const probability_grid) {
  Eigen::AlignedBox2f bounding_box(range_data.origin.head<2>());  // 2D空间的有向包围盒
  /// 填充包围盒
  constexpr float kPadding = 1e-6f;
  for (const sensor::RangefinderPoint& hit : range_data.returns) {
    bounding_box.extend(hit.position.head<2>());
  }
  for (const sensor::RangefinderPoint& miss : range_data.misses) {
    bounding_box.extend(miss.position.head<2>());
  }
  probability_grid->GrowLimits(bounding_box.min() - kPadding * Eigen::Vector2f::Ones());
  probability_grid->GrowLimits(bounding_box.max() + kPadding * Eigen::Vector2f::Ones());
}

///@brief 根据激光雷达数据进行数据更新
///@param hit_table 更新占用时的查找表
///@param miss_table 更新空闲时的查找表
///@param probability_grid 栅格地图
void CastRays(const sensor::RangeData& range_data,
              const std::vector<uint16>& hit_table,
              const std::vector<uint16>& miss_table,
              const bool insert_free_space,
              ProbabilityGrid* probability_grid) {
  /// 通过laser坐标和range_data获取包围狂
  GrowAsNeeded(range_data, probability_grid);

  const MapLimits& limits = probability_grid->limits();
  const double superscaled_resolution = limits.resolution() / kSubpixelScale;
  const MapLimits superscaled_limits(
      superscaled_resolution,
      limits.max(),
      CellLimits(limits.cell_limits().num_x_cells * kSubpixelScale, limits.cell_limits().num_y_cells * kSubpixelScale));
  const Eigen::Array2i begin = superscaled_limits.GetCellIndex(range_data.origin.head<2>());  // T_local_lidar
  std::vector<Eigen::Array2i> ends;
  ends.reserve(range_data.returns.size());
  /// 遍历所有的占据点
  for (const sensor::RangefinderPoint& hit : range_data.returns) {
    ends.push_back(superscaled_limits.GetCellIndex(hit.position.head<2>()));  // 终点坐标
    /// 栅格更新:刷黑
    probability_grid->ApplyLookupTable(ends.back() / kSubpixelScale, hit_table);
  }
  /// 是否插入空闲区域
  if (!insert_free_space) {
    return;
  }

  /// 射线刷白
  for (const Eigen::Array2i& end : ends) {
    std::vector<Eigen::Array2i> ray = RayToPixelMask(begin, end, kSubpixelScale);
    for (const Eigen::Array2i& cell_index : ray) {
      /// 栅格更新:刷白
      probability_grid->ApplyLookupTable(cell_index, miss_table);
    }
  }

  /// misses点刷白
  for (const sensor::RangefinderPoint& missing_echo : range_data.misses) {
    std::vector<Eigen::Array2i> ray =
        RayToPixelMask(begin, superscaled_limits.GetCellIndex(missing_echo.position.head<2>()), kSubpixelScale);
    for (const Eigen::Array2i& cell_index : ray) {
      probability_grid->ApplyLookupTable(cell_index, miss_table);
    }
  }
}
}  // namespace

proto::ProbabilityGridRangeDataInserterOptions2D CreateProbabilityGridRangeDataInserterOptions2D(
    common::LuaParameterDictionary* parameter_dictionary) {
  /// 获取相关option参数
  proto::ProbabilityGridRangeDataInserterOptions2D options;
  options.set_hit_probability(parameter_dictionary->GetDouble("hit_probability"));    // 占用概率
  options.set_miss_probability(parameter_dictionary->GetDouble("miss_probability"));  // 空闲概率
  options.set_insert_free_space(
      parameter_dictionary->HasKey("insert_free_space") ? parameter_dictionary->GetBool("insert_free_space") : true);
  CHECK_GT(options.hit_probability(), 0.5);
  CHECK_LT(options.miss_probability(), 0.5);
  return options;
}

ProbabilityGridRangeDataInserter2D::ProbabilityGridRangeDataInserter2D(
    const proto::ProbabilityGridRangeDataInserterOptions2D& options)
    : options_(options),
      hit_table_(ComputeLookupTableToApplyCorrespondenceCostOdds(Odds(options.hit_probability()))),
      miss_table_(ComputeLookupTableToApplyCorrespondenceCostOdds(Odds(options.miss_probability()))) {}

void ProbabilityGridRangeDataInserter2D::Insert(const sensor::RangeData& range_data, GridInterface* const grid) const {
  /// 做个上行转换
  ProbabilityGrid* const probability_grid = static_cast<ProbabilityGrid*>(grid);
  CHECK(probability_grid != nullptr);
  /// By not finishing the update after hits are inserted,
  /// we give hits priority (i.e. no hits will be ignored because of a miss in the same cell).
  CastRays(range_data, hit_table_, miss_table_, options_.insert_free_space(), probability_grid);
  probability_grid->FinishUpdate();
}

}  // namespace mapping
}  // namespace cartographer
