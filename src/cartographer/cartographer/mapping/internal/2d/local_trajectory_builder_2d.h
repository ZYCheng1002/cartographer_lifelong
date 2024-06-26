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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_2D_LOCAL_TRAJECTORY_BUILDER_2D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_2D_LOCAL_TRAJECTORY_BUILDER_2D_H_

#include <chrono>
#include <memory>

#include "cartographer/common/time.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/real_time_correlative_scan_matcher_2d.h"
#include "cartographer/mapping/internal/motion_filter.h"
#include "cartographer/mapping/internal/range_data_collator.h"
#include "cartographer/mapping/pose_extrapolator.h"
#include "cartographer/mapping/proto/local_trajectory_builder_options_2d.pb.h"
#include "cartographer/metrics/family_factory.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {

///@brief Wires up the local SLAM stack (i.e. pose extrapolator, scan matching, etc.) without loop closure.
class LocalTrajectoryBuilder2D {
 public:
  struct InsertionResult {
    std::shared_ptr<const TrajectoryNode::Data> constant_data;  // 帧信息(时间,点云,重力对其的点云,匹配位姿)
    std::vector<std::shared_ptr<const Submap2D>> insertion_submaps;  // 当前submap(1或2个)
  };
  struct MatchingResult {
    common::Time time;                      // 时间戳
    transform::Rigid3d local_pose;          // 匹配位姿
    sensor::RangeData range_data_in_local;  // 局部坐标系的data
    std::unique_ptr<const InsertionResult> insertion_result;
  };

  explicit LocalTrajectoryBuilder2D(const proto::LocalTrajectoryBuilderOptions2D& options,
                                    const std::vector<std::string>& expected_range_sensor_ids);
  ~LocalTrajectoryBuilder2D();

  LocalTrajectoryBuilder2D(const LocalTrajectoryBuilder2D&) = delete;
  LocalTrajectoryBuilder2D& operator=(const LocalTrajectoryBuilder2D&) = delete;

  ///@brief 2D激光数据处理
  ///@return 匹配结果
  std::unique_ptr<MatchingResult> AddRangeData(const std::string& sensor_id,
                                               const sensor::TimedPointCloudData& range_data);

  void AddImuData(const sensor::ImuData& imu_data);

  void AddOdometryData(const sensor::OdometryData& odometry_data);

  static void RegisterMetrics(metrics::FamilyFactory* family_factory);

 private:
  ///@brief 接收处理测距数据(laser)
  std::unique_ptr<MatchingResult> AddAccumulatedRangeData(common::Time time,
                                                          const sensor::RangeData& gravity_aligned_range_data,
                                                          const transform::Rigid3d& gravity_alignment,
                                                          const absl::optional<common::Duration>& sensor_duration);
  ///@brief 重力对齐
  sensor::RangeData TransformToGravityAlignedFrameAndFilter(
      const transform::Rigid3f& transform_to_gravity_aligned_frame, const sensor::RangeData& range_data) const;

  ///@brief 添加到submap中
  std::unique_ptr<InsertionResult> InsertIntoSubmap(common::Time time,
                                                    const sensor::RangeData& range_data_in_local,
                                                    const sensor::PointCloud& filtered_gravity_aligned_point_cloud,
                                                    const transform::Rigid3d& pose_estimate,
                                                    const Eigen::Quaterniond& gravity_alignment);

  // Scan matches 'filtered_gravity_aligned_point_cloud' and returns the
  // observed pose, or nullptr on failure.
  std::unique_ptr<transform::Rigid2d> ScanMatch(common::Time time,
                                                const transform::Rigid2d& pose_prediction,
                                                const sensor::PointCloud& filtered_gravity_aligned_point_cloud);

  ///@brief pose递推器初始化
  void InitializeExtrapolator(common::Time time);

  const proto::LocalTrajectoryBuilderOptions2D options_;
  ActiveSubmaps2D active_submaps_;                                                      // 当前的local submap
  MotionFilter motion_filter_;                                                          // 运动滤波器
  scan_matching::RealTimeCorrelativeScanMatcher2D real_time_correlative_scan_matcher_;  // 预测位姿优化器
  scan_matching::CeresScanMatcher2D ceres_scan_matcher_;                                // 激光匹配
  std::unique_ptr<PoseExtrapolator> extrapolator_;                                      // 位姿递推器

  int num_accumulated_ = 0;                   // 控制数据累积
  sensor::RangeData accumulated_range_data_;  // 存储距离信息(laser)

  absl::optional<std::chrono::steady_clock::time_point> last_wall_time_;
  absl::optional<double> last_thread_cpu_time_seconds_;
  absl::optional<common::Time> last_sensor_time_;

  RangeDataCollator range_data_collator_;  // 多个激光传感器融合
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_2D_LOCAL_TRAJECTORY_BUILDER_2D_H_
