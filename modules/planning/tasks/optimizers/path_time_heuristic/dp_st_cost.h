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

/**
 * @file
 **/

#pragma once

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/proto/dp_st_speed_config.pb.h"

#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/speed/st_boundary.h"
#include "modules/planning/common/speed/st_point.h"
#include "modules/planning/proto/st_drivable_boundary.pb.h"
#include "modules/planning/tasks/optimizers/path_time_heuristic/st_graph_point.h"

namespace apollo {
namespace planning {

class DpStCost {
 public:
  DpStCost(const DpStSpeedConfig& config, const double total_t,
           const double total_s, const std::vector<const Obstacle*>& obstacles,
           const STDrivableBoundary& st_drivable_boundary,
           const common::TrajectoryPoint& init_point);

  double GetObstacleCost(const StGraphPoint& point);

  double GetSpatialPotentialCost(const StGraphPoint& point);

  double GetReferenceCost(const STPoint& point,
                          const STPoint& reference_point) const;

  double GetSpeedCost(const STPoint& first, const STPoint& second,
                      const double speed_limit,
                      const double cruise_speed) const;

  double GetAccelCostByTwoPoints(const double pre_speed, const STPoint& first,
                                 const STPoint& second);
  double GetAccelCostByThreePoints(const STPoint& first, const STPoint& second,
                                   const STPoint& third);

  double GetJerkCostByTwoPoints(const double pre_speed, const double pre_acc,
                                const STPoint& pre_point,
                                const STPoint& curr_point);
  double GetJerkCostByThreePoints(const double first_speed,
                                  const STPoint& first_point,
                                  const STPoint& second_point,
                                  const STPoint& third_point);

  double GetJerkCostByFourPoints(const STPoint& first, const STPoint& second,
                                 const STPoint& third, const STPoint& fourth);

 private:
  double GetAccelCost(const double accel);
  double JerkCost(const double jerk);

  void AddToKeepClearRange(const std::vector<const Obstacle*>& obstacles);
  static void SortAndMergeRange(
      std::vector<std::pair<double, double>>* keep_clear_range_);
  bool InKeepClearRange(double s) const;

  const DpStSpeedConfig& config_;
  // Note: 来自于reference_line_info->path_decision()->obstacles().Items()，在DpStCost初始化时传进来的
  const std::vector<const Obstacle*>& obstacles_;

  // Note: adc的ST行驶通道，来源于st_bounds_decider，在DpStCost初始化时传进来的
  STDrivableBoundary st_drivable_boundary_;

  // Note: 规划起始点，在DpStCost初始化时传进来的
  const common::TrajectoryPoint& init_point_;

  // Note: 初始化之后是1.0s
  double unit_t_ = 0.0;
  double total_s_ = 0.0;

  // Note: obstacle id对应的障碍物在obstacles_中的下标
  std::unordered_map<std::string, int> boundary_map_;
  // Note: outer size为obstacles_.size()
  // Note: inner size为dimension_t
  // Note: 内容都被初始化为make_pair(-1.0, -1.0)
  // Note: 这个实际上存放的是障碍物st_boundary在t时刻对应的s_lower和s_upper
  std::vector<std::vector<std::pair<double, double>>> boundary_cost_;

  // Note: 禁停区的区间
  std::vector<std::pair<double, double>> keep_clear_range_;

  // Note: 这是两个简单的hash map，用于复用已经计算过的特定accel和jerk的cost
  std::array<double, 200> accel_cost_;
  std::array<double, 400> jerk_cost_;
};

}  // namespace planning
}  // namespace apollo
