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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_COLLATED_TRAJECTORY_BUILDER_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_COLLATED_TRAJECTORY_BUILDER_H_

#include <chrono>
#include <map>
#include <memory>
#include <set>
#include <string>

#include "cartographer/common/internal/rate_timer.h"
#include "cartographer/common/port.h"
#include "cartographer/mapping/internal/local_slam_result_data.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/mapping/trajectory_builder_interface.h"
#include "cartographer/sensor/collator_interface.h"
#include "cartographer/sensor/internal/dispatchable.h"

namespace cartographer {
namespace mapping {

///@class 收集传感器数据,分发到2D||3D SLAM, 即GlobalTrajectoryBuilder为其成员
class CollatedTrajectoryBuilder : public TrajectoryBuilderInterface {
 public:
  using SensorId = TrajectoryBuilderInterface::SensorId;

  ///@param sensor_collator 数据采集器
  ///@param trajectory_id 传感器类型和对应的topic
  CollatedTrajectoryBuilder(const proto::TrajectoryBuilderOptions& trajectory_options,
                            sensor::CollatorInterface* sensor_collator,
                            int trajectory_id,
                            const std::set<SensorId>& expected_sensor_ids,
                            std::unique_ptr<TrajectoryBuilderInterface> wrapped_trajectory_builder);
  ~CollatedTrajectoryBuilder() override {}

  CollatedTrajectoryBuilder(const CollatedTrajectoryBuilder&) = delete;
  CollatedTrajectoryBuilder& operator=(const CollatedTrajectoryBuilder&) = delete;

  void AddSensorData(const std::string& sensor_id, const sensor::TimedPointCloudData& timed_point_cloud_data) override {
    AddData(sensor::MakeDispatchable(sensor_id, timed_point_cloud_data));
  }

  void AddSensorData(const std::string& sensor_id, const sensor::ImuData& imu_data) override {
    /// 以imu为例:
    /// step1 统一数据类型
    /// step2 添加数据到collector
    AddData(sensor::MakeDispatchable(sensor_id, imu_data));
  }

  void AddSensorData(const std::string& sensor_id, const sensor::OdometryData& odometry_data) override {
    AddData(sensor::MakeDispatchable(sensor_id, odometry_data));
  }

  void AddSensorData(const std::string& sensor_id, const sensor::FixedFramePoseData& fixed_frame_pose_data) override {
    if (collate_fixed_frame_) {
      AddData(sensor::MakeDispatchable(sensor_id, fixed_frame_pose_data));
      return;
    }
    /// 调用GlobalTrajectory进行消息传播
    wrapped_trajectory_builder_->AddSensorData(sensor_id, fixed_frame_pose_data);
  }

  void AddSensorData(const std::string& sensor_id, const sensor::LandmarkData& landmark_data) override {
    if (collate_landmarks_) {
      AddData(sensor::MakeDispatchable(sensor_id, landmark_data));
      return;
    }
    wrapped_trajectory_builder_->AddSensorData(sensor_id, landmark_data);
  }

  void AddLocalSlamResultData(std::unique_ptr<mapping::LocalSlamResultData> local_slam_result_data) override {
    AddData(std::move(local_slam_result_data));
  }

 private:
  void AddData(std::unique_ptr<sensor::Data> data);
  ///@brief 数据处理的回调函数
  void HandleCollatedSensorData(const std::string& sensor_id, std::unique_ptr<sensor::Data> data);

  sensor::CollatorInterface* const sensor_collator_;
  const bool collate_landmarks_;
  const bool collate_fixed_frame_;
  const int trajectory_id_;
  std::unique_ptr<TrajectoryBuilderInterface> wrapped_trajectory_builder_; // 保存全局SLAM,GlobalTrajectoryBuilder

  // Time at which we last logged the rates of incoming sensor data.
  std::chrono::steady_clock::time_point last_logging_time_;
  std::map<std::string, common::RateTimer<>> rate_timers_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_COLLATED_TRAJECTORY_BUILDER_H_
