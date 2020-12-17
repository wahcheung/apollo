/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 **/

#include "modules/planning/tasks/deciders/st_bounds_decider/st_driving_limits.h"

namespace apollo {
namespace planning {

// Note: 设定adc起始状态，t0 = 0, s0 = 0, v0 = 规划起始点速度
void STDrivingLimits::Init(const double max_acc, const double max_dec,
                           const double max_v, double curr_v) {
  max_acc_ = max_acc;
  max_dec_ = max_dec;
  max_v_ = max_v;
  upper_t0_ = 0.0;
  upper_v0_ = curr_v;
  upper_s0_ = 0.0;
  lower_t0_ = 0.0;
  lower_v0_ = curr_v;
  lower_s0_ = 0.0;
}

// Note: 以最大加速度/减速度行驶，t时刻车的位置边界
std::pair<double, double> STDrivingLimits::GetVehicleDynamicsLimits(
    const double t) const {
  std::pair<double, double> dynamic_limits;
  // Process lower bound: (constant deceleration)
  // Note: 计算以最大减速度减速的话，adc在t时刻的位置
  double dec_time = lower_v0_ / max_dec_;
  if (t - lower_t0_ < dec_time) {
    dynamic_limits.first =
        lower_s0_ + (lower_v0_ - max_dec_ * (t - lower_t0_) + lower_v0_) *
                        (t - lower_t0_) * 0.5;
  } else {
    dynamic_limits.first = lower_s0_ + (lower_v0_ * dec_time) * 0.5;
  }

  // Process upper bound: (constant acceleration)
  // Note: 计算以最大加速度加速的话，adc在t时刻的位置
  double acc_time = (max_v_ - upper_v0_) / max_acc_;
  if (t - upper_t0_ < acc_time) {
    dynamic_limits.second =
        upper_s0_ + (upper_v0_ + max_acc_ * (t - upper_t0_) + upper_v0_) *
                        (t - upper_t0_) * 0.5;
  } else {
    dynamic_limits.second = upper_s0_ + (upper_v0_ + max_v_) * acc_time * 0.5 +
                            (t - upper_t0_ - acc_time) * max_v_;
  }

  return dynamic_limits;
}

// Remind(huachang): 这里更新各项参数的逻辑太奇怪了，感觉不应该要那两个if的判断条件
void STDrivingLimits::UpdateBlockingInfo(const double t, const double lower_s,
                                         const double lower_v,
                                         const double upper_s,
                                         const double upper_v) {
  // Note: 只有lower_s0_和upper_s0_是忠实地体现了最优s_gap(&driving_limit)做出的选择
  // Note: lower_v和upper_v完全没用到
  auto curr_bounds = GetVehicleDynamicsLimits(t);
  if (curr_bounds.first < lower_s) {
    // lower_v0_ = std::fmax(lower_v, 0.0);
    lower_v0_ = std::fmax(0.0, lower_v0_ - max_dec_ * (t - lower_t0_));
    lower_t0_ = t;
    lower_s0_ = lower_s;
  }
  if (curr_bounds.second > upper_s) {
    // upper_v0_ = std::fmax(upper_v, 0.0);
    upper_v0_ = std::fmin(max_v_, upper_v0_ + max_acc_ * (t - upper_t0_));
    upper_t0_ = t;
    upper_s0_ = upper_s;
  }
}

}  // namespace planning
}  // namespace apollo
