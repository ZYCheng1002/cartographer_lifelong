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

#include "cartographer/mapping/probability_values.h"

#include "absl/memory/memory.h"

namespace cartographer {
namespace mapping {

namespace {

constexpr int kValueCount = 32768;

/// 将[0,1-32767]映射成[0.9, 0.1-0.9]
float SlowValueToBoundedFloat(const uint16 value,          // 默认0->32767
                              const uint16 unknown_value,  // 默认 0
                              const float unknown_result,  // 默认0.9
                              const float lower_bound,     // 默认0.1
                              const float upper_bound) {   // 默认0.9
  CHECK_LT(value, kValueCount);
  /// 未知值返回0.9
  if (value == unknown_value) return unknown_result;
  const float kScale = (upper_bound - lower_bound) / (kValueCount - 2.f);
  /// 根据value值计算对应的概率值
  return value * kScale + (lower_bound - kScale);
}

///@brief 预先计算有界浮点值
std::unique_ptr<std::vector<float>> PrecomputeValueToBoundedFloat(const uint16 unknown_value,  // 默认0
                                                                  const float unknown_result,  // 默认0.9
                                                                  const float lower_bound,     // 默认0.1
                                                                  const float upper_bound) {   // 默认0.9
  auto result = absl::make_unique<std::vector<float>>();
  /// 重复两次,覆盖更新和不覆盖的值
  constexpr int kRepetitionCount = 2;
  result->reserve(kRepetitionCount * kValueCount);  // 覆盖两次,空间*2
  for (int repeat = 0; repeat != kRepetitionCount; ++repeat) {
    for (int value = 0; value != kValueCount; ++value) {
      /// 0->32767
      result->push_back(SlowValueToBoundedFloat(value, unknown_value, unknown_result, lower_bound, upper_bound));
    }
  }
  return result;
}

///@brief 返回范围[0,0.1-0.9]概率值对应的概率值
std::unique_ptr<std::vector<float>> PrecomputeValueToProbability() {
  return PrecomputeValueToBoundedFloat(kUnknownProbabilityValue, kMinProbability, kMinProbability, kMaxProbability);
}

///@brief 返回范围[0,0.1-0.9]代价值对应的概率值
std::unique_ptr<std::vector<float>> PrecomputeValueToCorrespondenceCost() {
  /// 默认值[0, 0.9, 0.1, 0.9]
  return PrecomputeValueToBoundedFloat(
      kUnknownCorrespondenceValue, kMaxCorrespondenceCost, kMinCorrespondenceCost, kMaxCorrespondenceCost);
}

}  // namespace

const std::vector<float>* const kValueToProbability = PrecomputeValueToProbability().release();

///@note 增加release防止内存泄漏
const std::vector<float>* const kValueToCorrespondenceCost = PrecomputeValueToCorrespondenceCost().release();

std::vector<uint16> ComputeLookupTableToApplyOdds(const float odds) {
  std::vector<uint16> result;
  result.reserve(kValueCount);
  result.push_back(ProbabilityToValue(ProbabilityFromOdds(odds)) + kUpdateMarker);
  for (int cell = 1; cell != kValueCount; ++cell) {
    result.push_back(ProbabilityToValue(ProbabilityFromOdds(odds * Odds((*kValueToProbability)[cell]))) +
                     kUpdateMarker);
  }
  return result;
}

std::vector<uint16> ComputeLookupTableToApplyCorrespondenceCostOdds(float odds) {
  std::vector<uint16> result;
  result.reserve(kValueCount);  // 分配空间
  /// 将概率值进行量化,转为uint类型,防止浮点数计算
  result.push_back(CorrespondenceCostToValue(ProbabilityToCorrespondenceCost(ProbabilityFromOdds(odds))) +
                   kUpdateMarker);
  /// 预先计算所有的栅格更新的可能结果,起加速的作用
  for (int cell = 1; cell != kValueCount; ++cell) {
    /// 获取所有uint值对应的概率值;根据cell索引计算代价值;计算占率比值(Odds);...总之计算完了所有概率对应的uint值
    result.push_back(CorrespondenceCostToValue(ProbabilityToCorrespondenceCost(ProbabilityFromOdds(
                         odds * Odds(CorrespondenceCostToProbability((*kValueToCorrespondenceCost)[cell]))))) +
                     kUpdateMarker);
  }
  return result;
}

}  // namespace mapping
}  // namespace cartographer
