/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

/*
 *  @file: st_graph_point.h
 */

#pragma once

#include <limits>

#include "modules/planning/common/speed/st_point.h"

namespace apollo {
namespace planning {

class StGraphPoint {
 public:
  std::uint32_t index_s() const;
  std::uint32_t index_t() const;

  const STPoint& point() const;
  const StGraphPoint* pre_point() const;

  double reference_cost() const;
  double obstacle_cost() const;
  double spatial_potential_cost() const;
  double total_cost() const;

  void Init(const std::uint32_t index_t, const std::uint32_t index_s,
            const STPoint& st_point);

  // given reference speed profile, reach the cost, including position
  void SetReferenceCost(const double reference_cost);

  // given obstacle info, get the cost;
  void SetObstacleCost(const double obs_cost);

  // given potential cost for minimal time traversal
  void SetSpatialPotentialCost(const double spatial_potential_cost);

  // total cost
  void SetTotalCost(const double total_cost);

  void SetPrePoint(const StGraphPoint& pre_point);

  double GetOptimalSpeed() const;

  void SetOptimalSpeed(const double optimal_speed);

 private:
  STPoint point_;
  const StGraphPoint* pre_point_ = nullptr;
  std::uint32_t index_s_ = 0;
  std::uint32_t index_t_ = 0;

  // Note: 通过上一个点的位置和速度信息，需要达到当前点的话，当前点的预估速度
  double optimal_speed_ = 0.0;
  double reference_cost_ = 0.0;
  // Note: st_graph_point在t轴方向上距离障碍物越近则cost越大
  double obstacle_cost_ = 0.0;
  // Note: 对s做惩罚，效果就是追求更快地到达远处
  // Note: (total_s_ - point.point().s()) * config_.spatial_potential_penalty()
  double spatial_potential_cost_ = 0.0;
  double total_cost_ = std::numeric_limits<double>::infinity();
};

}  // namespace planning
}  // namespace apollo
