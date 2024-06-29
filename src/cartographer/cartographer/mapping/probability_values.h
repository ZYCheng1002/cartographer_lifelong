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

#ifndef CARTOGRAPHER_MAPPING_PROBABILITY_VALUES_H_
#define CARTOGRAPHER_MAPPING_PROBABILITY_VALUES_H_

#include <cmath>
#include <vector>

#include "cartographer/common/math.h"
#include "cartographer/common/port.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

namespace {
/// 为了避免浮点运算, 将[0~0.9]的浮点数转成[1~32767]之间的值
inline uint16 BoundedFloatToValue(const float float_value, const float lower_bound, const float upper_bound) {
  /// 将概率值进行量化,使其成为[1-32767]之间的整数值
  const int value = common::RoundToInt((common::Clamp(float_value, lower_bound, upper_bound) - lower_bound) *
                                       (32766.f / (upper_bound - lower_bound))) +
                    1;
  DCHECK_GE(value, 1);
  DCHECK_LE(value, 32767);
  return value;
}

}  // namespace

/// 计算占用几率比
inline float Odds(float probability) { return probability / (1.f - probability); }

/// 实际上又计算出概率值,即Odds的输入
inline float ProbabilityFromOdds(const float odds) { return odds / (odds + 1.f); }

inline float ProbabilityToCorrespondenceCost(const float probability) { return 1.f - probability; }

///@brief 代价值转概率值
inline float CorrespondenceCostToProbability(const float correspondence_cost) { return 1.f - correspondence_cost; }

constexpr float kMinProbability = 0.1f;                          // 0.1
constexpr float kMaxProbability = 1.f - kMinProbability;         // 0.9
constexpr float kMinCorrespondenceCost = 1.f - kMaxProbability;  // 0.1
constexpr float kMaxCorrespondenceCost = 1.f - kMinProbability;  // 0.9

///@brief 概率值的上下界判断
inline float ClampProbability(const float probability) {
  return common::Clamp(probability, kMinProbability, kMaxProbability);
}

///@brief 代价值的上下界
inline float ClampCorrespondenceCost(const float correspondence_cost) {
  return common::Clamp(correspondence_cost, kMinCorrespondenceCost, kMaxCorrespondenceCost);
}

constexpr uint16 kUnknownProbabilityValue = 0;                            // 0
constexpr uint16 kUnknownCorrespondenceValue = kUnknownProbabilityValue;  // 0
constexpr uint16 kUpdateMarker = 1u << 15;                                // 32768

///@brief Converts a correspondence_cost to a uint16 in the [1, 32767] range.
inline uint16 CorrespondenceCostToValue(const float correspondence_cost) {
  /// 概率值的上限0.9, 下限0.1, 返回uint值
  return BoundedFloatToValue(correspondence_cost, kMinCorrespondenceCost, kMaxCorrespondenceCost);
}

///@brief Converts a probability to a uint16 in the [1, 32767] range.
inline uint16 ProbabilityToValue(const float probability) {
  return BoundedFloatToValue(probability, kMinProbability, kMaxProbability);
}

extern const std::vector<float>* const kValueToProbability;
extern const std::vector<float>* const kValueToCorrespondenceCost;

///@brief Converts a uint16 (which may or may not have the update marker set) to a probability in the range
///[kMinProbability, kMaxProbability].
inline float ValueToProbability(const uint16 value) { return (*kValueToProbability)[value]; }

///@brief Converts a uint16 (which may or may not have the update marker set) to a correspondence cost in the range
///[kMinCorrespondenceCost, kMaxCorrespondenceCost].
inline float ValueToCorrespondenceCost(const uint16 value) { return (*kValueToCorrespondenceCost)[value]; }

inline uint16 ProbabilityValueToCorrespondenceCostValue(uint16 probability_value) {
  if (probability_value == kUnknownProbabilityValue) {
    return kUnknownCorrespondenceValue;
  }
  bool update_carry = false;
  if (probability_value > kUpdateMarker) {
    probability_value -= kUpdateMarker;
    update_carry = true;
  }
  uint16 result = CorrespondenceCostToValue(ProbabilityToCorrespondenceCost(ValueToProbability(probability_value)));
  if (update_carry) result += kUpdateMarker;
  return result;
}

inline uint16 CorrespondenceCostValueToProbabilityValue(uint16 correspondence_cost_value) {
  if (correspondence_cost_value == kUnknownCorrespondenceValue) return kUnknownProbabilityValue;
  bool update_carry = false;
  if (correspondence_cost_value > kUpdateMarker) {
    correspondence_cost_value -= kUpdateMarker;
    update_carry = true;
  }
  uint16 result =
      ProbabilityToValue(CorrespondenceCostToProbability(ValueToCorrespondenceCost(correspondence_cost_value)));
  if (update_carry) result += kUpdateMarker;
  return result;
}

std::vector<uint16> ComputeLookupTableToApplyOdds(float odds);
std::vector<uint16> ComputeLookupTableToApplyCorrespondenceCostOdds(float odds);

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_PROBABILITY_VALUES_H_
