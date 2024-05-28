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

POSE_GRAPH = {
  optimize_every_n_nodes = 90,              -- 整体优化node数
  constraint_builder = {
    sampling_ratio = 0.3,                   -- 对局部子地图进行回环检测时的计算频率
    max_constraint_distance = 15.,          -- 局部子地图回环检测的最大距离
    min_score = 0.55,                       -- 局部子地图检测时的最低分数阈值
    global_localization_min_score = 0.6,    -- 整体子地图回环时候的最低匹配阈值
    loop_closure_translation_weight = 1.1e4,-- 回环位移权重
    loop_closure_rotation_weight = 1e5,     -- 回环旋转权重
    log_matches = true,                     -- 是否打印匹配log
    fast_correlative_scan_matcher = {       -- 分支定界匹配
      linear_search_window = 7.,            -- 搜索窗口
      angular_search_window = math.rad(30.),-- 搜索角度
      branch_and_bound_depth = 7,           -- 分支定界深度
    },
    ceres_scan_matcher = {                  -- 基于ceres的精配准
      occupied_space_weight = 20.,          -- 占据空间权重
      translation_weight = 10.,             -- 平移权重
      rotation_weight = 1.,                 -- 旋转权重
      ceres_solver_options = {              -- ceres求解相关参数
        use_nonmonotonic_steps = true,      -- 梯度下降策略
        max_num_iterations = 10,            -- 迭代次数
        num_threads = 1,                    -- 线程
      },
    },
    fast_correlative_scan_matcher_3d = {    -- 3d同2d
      branch_and_bound_depth = 8,
      full_resolution_depth = 3,
      min_rotational_score = 0.77,
      min_low_resolution_score = 0.55,
      linear_xy_search_window = 5.,
      linear_z_search_window = 1.,
      angular_search_window = math.rad(15.),
    },
    ceres_scan_matcher_3d = {
      occupied_space_weight_0 = 5.,
      occupied_space_weight_1 = 30.,
      translation_weight = 10.,
      rotation_weight = 1.,
      only_optimize_yaw = false,
      ceres_solver_options = {
        use_nonmonotonic_steps = false,
        max_num_iterations = 10,
        num_threads = 1,
      },
    },
  },
  matcher_translation_weight = 5e2,
  matcher_rotation_weight = 1.6e3,
  optimization_problem = {                              -- pgo相关优化求解参数
    huber_scale = 1e1,                                  -- 鲁棒核函数?
    acceleration_weight = 1.1e2,                        -- 线加速度权重
    rotation_weight = 1.6e4,                            -- 旋转权重
    local_slam_pose_translation_weight = 1e5,           -- 前端t权重
    local_slam_pose_rotation_weight = 1e5,              -- 前端R权重
    odometry_translation_weight = 1e5,                  -- 里程计t权重
    odometry_rotation_weight = 1e5,                     -- 里程计r权重
    fixed_frame_pose_translation_weight = 1e1,          -- fixed
    fixed_frame_pose_rotation_weight = 1e2,
    fixed_frame_pose_use_tolerant_loss = false,
    fixed_frame_pose_tolerant_loss_param_a = 1,
    fixed_frame_pose_tolerant_loss_param_b = 1,
    log_solver_summary = false,                         -- log打印
    use_online_imu_extrinsics_in_3d = true,
    fix_z_in_3d = false,
    ceres_solver_options = {
      use_nonmonotonic_steps = false,
      max_num_iterations = 50,
      num_threads = 7,
    },
  },
  max_num_final_iterations = 200,
  global_sampling_ratio = 0.003,
  log_residual_histograms = true,
  global_constraint_search_after_n_seconds = 10.,
  --  overlapping_submaps_trimmer_2d = {
  --    fresh_submaps_count = 1,
  --    min_covered_area = 2,
  --    min_added_submaps_count = 5,
  --  },
}
