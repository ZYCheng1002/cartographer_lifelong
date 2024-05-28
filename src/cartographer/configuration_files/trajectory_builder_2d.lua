-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

TRAJECTORY_BUILDER_2D = {
  use_imu_data = true,                          -- 是否使用imu数据                                                -- 是否使用imu数据
  min_range = 0.,                               -- 最近距离                          -- laser最短距离
  max_range = 30.,                              -- 最远距离                                -- laser最远距离
  min_z = -0.8,                                 -- 最小高度                                          -- laser最低高度
  max_z = 2.,                                   -- 最大高度                                          -- laser最高高速
  missing_data_ray_length = 5.,                 -- 激光默认长度                                  --
  num_accumulated_range_data = 1,               -- 单个node累计激光帧数                                    --
  voxel_filter_size = 0.025,                    -- voxel leaf_size                                         -- 体素滤波分辨率

  adaptive_voxel_filter = {                     -- 自适应滤波
    max_length = 0.5,                           -- 栅格大小
    min_num_points = 200,                       -- 最少点云数目
    max_range = 50.,                            -- 最远点云距离
  },

  loop_closure_adaptive_voxel_filter = {        -- 闭环检测自适应滤波
    max_length = 0.9,
    min_num_points = 100,
    max_range = 50.,
  },

  use_online_correlative_scan_matching = false, -- 在线CSM激光匹配
  real_time_correlative_scan_matcher = {        -- 快速CSM匹配参数
    linear_search_window = 0.1,                 -- 平移搜索范围
    angular_search_window = math.rad(20.),      -- 旋转搜索角度
    translation_delta_cost_weight = 1e-1,       -- 平移得分权重
    rotation_delta_cost_weight = 1e-1,          -- 旋转得分权重
  },

  ceres_scan_matcher = {                        -- 精配准参数
    occupied_space_weight = 1.,                 -- 占据空间权重
    translation_weight = 10.,                   -- 平移权重
    rotation_weight = 40.,                      -- 旋转权重
    ceres_solver_options = {                    -- 优化器参数
      use_nonmonotonic_steps = false,
      max_num_iterations = 20,
      num_threads = 1,
    },
  },

  motion_filter = {                             -- 运动滤波
    max_time_seconds = 5.,                      -- 时间间隔
    max_distance_meters = 0.2,                  -- 距离间隔
    max_angle_radians = math.rad(1.),           -- 角度间隔
  },

  -- TODO(schwoere,wohe): Remove this constant. This is only kept for ROS.
  imu_gravity_time_constant = 10.,              -- imu重力常数
  pose_extrapolator = {
    use_imu_based = false,                      -- 基于imu的位姿递推
    constant_velocity = {
      imu_gravity_time_constant = 10.,          -- imu重力常数
      pose_queue_duration = 0.001,              -- 位姿时间间隔
    },
    imu_based = {
      pose_queue_duration = 5.,
      gravity_constant = 9.806,
      pose_translation_weight = 1.,
      pose_rotation_weight = 1.,
      imu_acceleration_weight = 1.,
      imu_rotation_weight = 1.,
      odometry_translation_weight = 1.,
      odometry_rotation_weight = 1.,
      solver_options = {
        use_nonmonotonic_steps = false;
        max_num_iterations = 10;
        num_threads = 1;
      },
    },
  },

  submaps = {
    num_range_data = 90,                          -- 子地图的node数量
    grid_options_2d = {
      grid_type = "PROBABILITY_GRID",             -- 概率栅格地图
      resolution = 0.05,                          -- 分辨率
    },
    range_data_inserter = {
      range_data_inserter_type = "PROBABILITY_GRID_INSERTER_2D",
      probability_grid_range_data_inserter = {
        insert_free_space = true,                 -- 是否改变占用网格概率
        hit_probability = 0.55,                   -- 占用时的概率
        miss_probability = 0.49,                  -- 非占据的概率
      },
      tsdf_range_data_inserter = {
        truncation_distance = 0.3,
        maximum_weight = 10.,
        update_free_space = false,
        normal_estimation_options = {
          num_normal_samples = 4,
          sample_radius = 0.5,
        },
        project_sdf_distance_to_scan_normal = true,
        update_weight_range_exponent = 0,
        update_weight_angle_scan_normal_to_ray_kernel_bandwidth = 0.5,
        update_weight_distance_cell_to_hit_kernel_bandwidth = 0.5,
      },
    },
  },
}
