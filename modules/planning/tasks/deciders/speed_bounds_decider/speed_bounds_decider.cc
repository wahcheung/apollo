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

#include "modules/planning/tasks/deciders/speed_bounds_decider/speed_bounds_decider.h"

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/path/path_data.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/st_graph_data.h"
#include "modules/planning/common/util/common.h"
#include "modules/planning/tasks/deciders/speed_bounds_decider/speed_limit_decider.h"
#include "modules/planning/tasks/deciders/speed_bounds_decider/st_boundary_mapper.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::planning_internal::StGraphBoundaryDebug;
using apollo::planning_internal::STGraphDebug;

SpeedBoundsDecider::SpeedBoundsDecider(const TaskConfig &config)
    : Decider(config) {
  CHECK(config.has_speed_bounds_decider_config());
  speed_bounds_config_ = config.speed_bounds_decider_config();
}

// Note: 对于没做Longitudinal决策以及follow/overtake/yield的障碍物重新计算了STBoundary
// 对于yield的STBoundary还对s上下界做了扩充
// Note: 根据地图限速/向心加速度限制/Nudge决策导致的减速/预设的lowest_speed
// 来确定path各处的速度限制，即speed_limit
Status SpeedBoundsDecider::Process(
    Frame *const frame, ReferenceLineInfo *const reference_line_info) {
  // retrieve data from frame and reference_line_info
  const PathData &path_data = reference_line_info->path_data();
  const TrajectoryPoint &init_point = frame->PlanningStartPoint();
  const ReferenceLine &reference_line = reference_line_info->reference_line();
  PathDecision *const path_decision = reference_line_info->path_decision();

  // 1. Map obstacles into st graph
  auto time1 = std::chrono::system_clock::now();
  // STBoundaryMapper初始化就只是简单地给那些私有变量赋值
  STBoundaryMapper boundary_mapper(
      speed_bounds_config_, reference_line, path_data,
      path_data.discretized_path().Length(), speed_bounds_config_.total_time());

  if (!FLAGS_use_st_drivable_boundary) {
    // Note: 不使用ST_BOUNDS_DECIDER的数据，重置obstacle的path_st_boundary_
    path_decision->EraseStBoundaries();
  }

  // Note: 在函数里面，对于没做Longitudinal决策以及有follow/overtake/yield决策的障碍物重新计算了STBoundary
  // 并且对最近的stop_point做处理，计算一个STOP类型的STBoundary，这个STBoundary的上界扩充了一个boundary_buffer
  // 什么样的障碍物到这个流程是还没有Longitudinal决策的呢？
  // 前面的st_bounds_decider虽然把所有与path有overlap的障碍物的STBoundary都算了，boundary也给了类型(stop/overtake/yield之类的)
  // 但只对不在STGraph中的障碍物添加了横向和纵向的ignore decision，
  // 并没有给这些在STGraph中的障碍物做Longitudinal decision
  // 也就是说，在st_bounds_decider中算的STBoundary都没用
  if (boundary_mapper.ComputeSTBoundary(path_decision).code() ==
      ErrorCode::PLANNING_ERROR) {
    const std::string msg = "Mapping obstacle failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  auto time2 = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = time2 - time1;
  ADEBUG << "Time for ST Boundary Mapping = " << diff.count() * 1000
         << " msec.";

  std::vector<const STBoundary *> boundaries;
  for (auto *obstacle : path_decision->obstacles().Items()) {
    const auto &id = obstacle->Id();
    const auto &st_boundary = obstacle->path_st_boundary();
    if (!st_boundary.IsEmpty()) {
      if (st_boundary.boundary_type() == STBoundary::BoundaryType::KEEP_CLEAR) {
        // Note: 这个字段 for keep_clear usage only
        path_decision->Find(id)->SetBlockingObstacle(false);
      } else {
        path_decision->Find(id)->SetBlockingObstacle(true);
      }
      boundaries.push_back(&st_boundary);
    }
  }

  // Note: 差不多就是取了STGraph中st_boundary的s最小值，函数里面的处理有点问题，逻辑上不能理解
  const double min_s_on_st_boundaries = SetSpeedFallbackDistance(path_decision);

  // 2. Create speed limit along path
  SpeedLimitDecider speed_limit_decider(speed_bounds_config_, reference_line,
                                        path_data);

  // Note: 根据地图限速/向心加速度限制/Nudge决策导致的减速/预设的lowest_speed
  // 来确定path各处的速度限制
  SpeedLimit speed_limit;
  if (!speed_limit_decider
           .GetSpeedLimits(path_decision->obstacles(), &speed_limit)
           .ok()) {
    std::string msg("Getting speed limits failed!");
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // 3. Get path_length as s axis search bound in st graph
  const double path_data_length = path_data.discretized_path().Length();

  // 4. Get time duration as t axis search bound in st graph
  const double total_time_by_conf = speed_bounds_config_.total_time();

  // Load generated st graph data back to frame
  StGraphData *st_graph_data = reference_line_info_->mutable_st_graph_data();

  // Add a st_graph debug info and save the pointer to st_graph_data for
  // optimizer logging
  auto *debug = reference_line_info_->mutable_debug();
  STGraphDebug *st_graph_debug = debug->mutable_planning_data()->add_st_graph();

  st_graph_data->LoadData(boundaries, min_s_on_st_boundaries, init_point,
                          speed_limit, reference_line_info->GetCruiseSpeed(),
                          path_data_length, total_time_by_conf, st_graph_debug);

  // Create and record st_graph debug info
  // TODO(huachang): 这个和st_bounds_decider中的RecordSTGraphDebug对比一下
  // Remind(huachang): 这个speed_bounds_decider的st_graph不知道为什么没有在最终的ADCTrajectory中出现
  // Note: 这个debug信息的Name在speed_optimizer中的RecordDebugInfo被覆盖了
  RecordSTGraphDebug(*st_graph_data, st_graph_debug);

  return Status::OK();
}

// Remind(huachang): 这里取fallback的s的方法不合理，一个逆向行驶的车辆迎面而来，如果预测轨迹穿过adc，则这个fallback_s就是0了
// 不应该考虑超过一定时间之后的障碍物
double SpeedBoundsDecider::SetSpeedFallbackDistance(
    PathDecision *const path_decision) {
  // Set min_s_on_st_boundaries to guide speed fallback.
  static constexpr double kEpsilon = 1.0e-6;
  double min_s_non_reverse = std::numeric_limits<double>::infinity();
  double min_s_reverse = std::numeric_limits<double>::infinity();

  for (auto *obstacle : path_decision->obstacles().Items()) {
    const auto &st_boundary = obstacle->path_st_boundary();

    if (st_boundary.IsEmpty()) {
      continue;
    }

    const auto left_bottom_point_s = st_boundary.bottom_left_point().s();
    const auto right_bottom_point_s = st_boundary.bottom_right_point().s();
    const auto lowest_s = std::min(left_bottom_point_s, right_bottom_point_s);

    // Note: 这个障碍物应该是逆向行驶的才会有left_bottom_point_s > right_bottom_point_s
    // Remind(huachang): 下面这样处理的意义是什么，为什么要将min_s_non_reverse和min_s_reverse用if-elif来做互斥更新处理
    // Remind(huachang): 这个speed fallback distance直接取所有st_boundary的s最小值不好吗?
    if (left_bottom_point_s - right_bottom_point_s > kEpsilon) {
      if (min_s_reverse > lowest_s) {
        min_s_reverse = lowest_s;
      }
    } else if (min_s_non_reverse > lowest_s) {
      min_s_non_reverse = lowest_s;
    }
  }

  // Note: 所有逆向行驶的障碍物的lowest_s
  min_s_reverse = std::max(min_s_reverse, 0.0);
  // Note: 所有正向行驶的障碍物的lowest_s
  min_s_non_reverse = std::max(min_s_non_reverse, 0.0);

  // Note: 这个speed fallback distance直接取所有st_boundary的s最小值不好吗?
  return min_s_non_reverse > min_s_reverse ? 0.0 : min_s_non_reverse;
}

void SpeedBoundsDecider::RecordSTGraphDebug(
    const StGraphData &st_graph_data, STGraphDebug *st_graph_debug) const {
  if (!FLAGS_enable_record_debug || !st_graph_debug) {
    ADEBUG << "Skip record debug info";
    return;
  }

  for (const auto &boundary : st_graph_data.st_boundaries()) {
    auto boundary_debug = st_graph_debug->add_boundary();
    boundary_debug->set_name(boundary->id());
    switch (boundary->boundary_type()) {
      case STBoundary::BoundaryType::FOLLOW:
        boundary_debug->set_type(StGraphBoundaryDebug::ST_BOUNDARY_TYPE_FOLLOW);
        break;
      case STBoundary::BoundaryType::OVERTAKE:
        boundary_debug->set_type(
            StGraphBoundaryDebug::ST_BOUNDARY_TYPE_OVERTAKE);
        break;
      case STBoundary::BoundaryType::STOP:
        boundary_debug->set_type(StGraphBoundaryDebug::ST_BOUNDARY_TYPE_STOP);
        break;
      case STBoundary::BoundaryType::UNKNOWN:
        boundary_debug->set_type(
            StGraphBoundaryDebug::ST_BOUNDARY_TYPE_UNKNOWN);
        break;
      case STBoundary::BoundaryType::YIELD:
        boundary_debug->set_type(StGraphBoundaryDebug::ST_BOUNDARY_TYPE_YIELD);
        break;
      case STBoundary::BoundaryType::KEEP_CLEAR:
        boundary_debug->set_type(
            StGraphBoundaryDebug::ST_BOUNDARY_TYPE_KEEP_CLEAR);
        break;
    }

    for (const auto &point : boundary->points()) {
      auto point_debug = boundary_debug->add_point();
      point_debug->set_t(point.x());
      point_debug->set_s(point.y());
    }
  }

  for (const auto &point : st_graph_data.speed_limit().speed_limit_points()) {
    common::SpeedPoint *speed_point = st_graph_debug->add_speed_limit();
    speed_point->set_s(point.first);
    speed_point->set_v(point.second);
  }
}

}  // namespace planning
}  // namespace apollo
