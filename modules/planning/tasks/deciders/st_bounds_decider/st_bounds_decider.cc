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

#include "modules/planning/tasks/deciders/st_bounds_decider/st_bounds_decider.h"

#include <limits>

#include "modules/planning/common/st_graph_data.h"
#include "modules/planning/tasks/deciders/st_bounds_decider/st_obstacles_processor.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::planning_internal::StGraphBoundaryDebug;
using apollo::planning_internal::STGraphDebug;

namespace {
// STBoundPoint contains (t, s_min, s_max)
using STBoundPoint = std::tuple<double, double, double>;
// STBound is a vector of STBoundPoints
using STBound = std::vector<STBoundPoint>;
// ObsDecSet is a set of decision for new obstacles.
using ObsDecSet = std::vector<std::pair<std::string, ObjectDecisionType>>;
}  // namespace

STBoundsDecider::STBoundsDecider(const TaskConfig& config) : Decider(config) {
  CHECK(config.has_st_bounds_decider_config());
  st_bounds_config_ = config.st_bounds_decider_config();
}

// Note: STBoundsDecider输出的内容有
// 1. 计算所有动态障碍物的STBoundary和closest_stop_obstacle的STBoundary并存到path_decision中，
//    STBoundary都根据最优的gap给了boundary type
//    对不记录在STGraph中的障碍物添加了纵向&横向的ignore decision(这些障碍物与adc的path没有交集)
//    虽然给出了障碍物的STBoundary和STBoundary的类型，但并没有对这些非ignore、非stop的障碍物做decision
// 2. 计算了regular_st_bound和regular_vt_bound并写入st_graph_data中
// 3. STGraphDebug，可视化debug信息
Status STBoundsDecider::Process(Frame* const frame,
                                ReferenceLineInfo* const reference_line_info) {
  // Initialize the related helper classes.
  // Note: 构建st图，对无关障碍物添加ignore decision，初始化辅助工具类（st_guide_line_/st_driving_limits_）
  InitSTBoundsDecider(*frame, reference_line_info);

  // Sweep the t-axis, and determine the s-boundaries step by step.
  STBound regular_st_bound;
  STBound regular_vt_bound;
  // Note: regular_st_bound，对每个时刻t选出最优的s_gap，然后再结合driving_limit给出(交集)
  // Note: regular_vt_bound，根据s_gap给出的yield/stop/overtake的decision，对adc速度有制约的障碍物的速度
  // Note: regular_vt_bound准确来说应该是对应时刻下最近的boundary的斜率，实际上记录的是障碍物的速度
  Status ret = GenerateRegularSTBound(&regular_st_bound, &regular_vt_bound);
  if (!ret.ok()) {
    ADEBUG << "Cannot generate a regular ST-boundary.";
    return Status(ErrorCode::PLANNING_ERROR, ret.error_message());
  }
  if (regular_st_bound.empty()) {
    std::string msg = "Generated regular ST-boundary is empty.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  StGraphData* st_graph_data = reference_line_info_->mutable_st_graph_data();
  st_graph_data->SetSTDrivableBoundary(regular_st_bound, regular_vt_bound);

  // Record the ST-Graph for good visualization and easy debugging.
  auto all_st_boundaries = st_obstacles_processor_.GetAllSTBoundaries();
  std::vector<STBoundary> st_boundaries;
  for (const auto& st_boundary : all_st_boundaries) {
    st_boundaries.push_back(st_boundary.second);
  }
  ADEBUG << "Total ST boundaries = " << st_boundaries.size();
  STGraphDebug* st_graph_debug = reference_line_info->mutable_debug()
                                     ->mutable_planning_data()
                                     ->add_st_graph();
  RecordSTGraphDebug(st_boundaries, regular_st_bound, st_graph_debug);

  return Status::OK();
}

// Note: 构建st图，对无关障碍物添加ignore decision，初始化辅助工具类（st_guide_line_/st_driving_limits_）
void STBoundsDecider::InitSTBoundsDecider(
    const Frame& frame, ReferenceLineInfo* const reference_line_info) {
  const PathData& path_data = reference_line_info->path_data();
  PathDecision* path_decision = reference_line_info->path_decision();

  // Map all related obstacles onto ST-Graph.
  auto time1 = std::chrono::system_clock::now();
  st_obstacles_processor_.Init(path_data.discretized_path().Length(),
                               st_bounds_config_.total_time(), path_data,
                               path_decision);
  // Note: 计算障碍物的ST boundary，那些不在ST graph中的障碍物都被添加了ignore decision
  // Note: 动态障碍物的st boundary和最近的需要stop的静态障碍物的st boundary都写入了path_decision中
  st_obstacles_processor_.MapObstaclesToSTBoundaries(path_decision);
  auto time2 = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = time2 - time1;
  ADEBUG << "Time for ST Obstacles Processing = " << diff.count() * 1000
         << " msec.";

  // Initialize Guide-Line and Driving-Limits.
  // Remind(huachang): 初始化参数根据车辆配置来是不是合理一些
  static constexpr double desired_speed = 15.0;
  st_guide_line_.Init(desired_speed);
  static constexpr double max_acc = 2.5;
  static constexpr double max_dec = 5.0;
  static constexpr double max_v = desired_speed * 1.5;
  st_driving_limits_.Init(max_acc, max_dec, max_v,
                          frame.PlanningStartPoint().v());
}

Status STBoundsDecider::GenerateFallbackSTBound(STBound* const st_bound,
                                                STBound* const vt_bound) {
  // Initialize st-boundary.
  for (double curr_t = 0.0; curr_t <= st_bounds_config_.total_time();
       curr_t += kSTBoundsDeciderResolution) {
    st_bound->emplace_back(curr_t, std::numeric_limits<double>::lowest(),
                           std::numeric_limits<double>::max());
    vt_bound->emplace_back(curr_t, std::numeric_limits<double>::lowest(),
                           std::numeric_limits<double>::max());
  }

  // Sweep-line to get detailed ST-boundary.
  for (size_t i = 0; i < st_bound->size(); ++i) {
    double t, s_lower, s_upper, lower_obs_v, upper_obs_v;
    std::tie(t, s_lower, s_upper) = st_bound->at(i);
    std::tie(t, lower_obs_v, upper_obs_v) = vt_bound->at(i);
    ADEBUG << "Processing st-boundary at t = " << t;

    // Get Boundary due to driving limits
    // Note: t时刻车辆动力学范围内的s边界
    auto driving_limits_bound = st_driving_limits_.GetVehicleDynamicsLimits(t);
    s_lower = std::fmax(s_lower, driving_limits_bound.first);
    s_upper = std::fmin(s_upper, driving_limits_bound.second);
    ADEBUG << "Bounds for s due to driving limits are "
           << "s_upper = " << s_upper << ", s_lower = " << s_lower;

    // Get Boundary due to obstacles
    std::vector<std::pair<double, double>> available_s_bounds;
    std::vector<ObsDecSet> available_obs_decisions;
    if (!st_obstacles_processor_.GetSBoundsFromDecisions(
            t, &available_s_bounds, &available_obs_decisions)) {
      const std::string msg =
          "Failed to find a proper boundary due to obstacles.";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
    std::vector<std::pair<STBoundPoint, ObsDecSet>> available_choices;
    ADEBUG << "Available choices are:";
    for (int j = 0; j < static_cast<int>(available_s_bounds.size()); ++j) {
      ADEBUG << "  (" << available_s_bounds[j].first << ", "
             << available_s_bounds[j].second << ")";
      available_choices.emplace_back(
          std::make_tuple(0.0, available_s_bounds[j].first,
                          available_s_bounds[j].second),
          available_obs_decisions[j]);
    }
    RemoveInvalidDecisions(driving_limits_bound, &available_choices);

    // Always go for the most conservative option.
    if (!available_choices.empty()) {
      // Select the most conservative decision.
      auto top_choice_s_range = available_choices.front().first;
      auto top_choice_decision = available_choices.front().second;
      for (size_t j = 1; j < available_choices.size(); ++j) {
        if (std::get<1>(available_choices[j].first) <
            std::get<1>(top_choice_s_range)) {
          top_choice_s_range = available_choices[j].first;
          top_choice_decision = available_choices[j].second;
        }
      }

      // Set decision for obstacles without decisions.
      bool is_limited_by_upper_obs = false;
      bool is_limited_by_lower_obs = false;
      if (s_lower < std::get<1>(top_choice_s_range)) {
        s_lower = std::get<1>(top_choice_s_range);
        is_limited_by_lower_obs = true;
      }
      if (s_upper > std::get<2>(top_choice_s_range)) {
        s_upper = std::get<2>(top_choice_s_range);
        is_limited_by_upper_obs = true;
      }
      st_obstacles_processor_.SetObstacleDecision(top_choice_decision);

      // Update st-guide-line, st-driving-limit info, and v-limits.
      std::pair<double, double> limiting_speed_info;
      if (st_obstacles_processor_.GetLimitingSpeedInfo(t,
                                                       &limiting_speed_info)) {
        st_driving_limits_.UpdateBlockingInfo(
            t, s_lower, limiting_speed_info.first, s_upper,
            limiting_speed_info.second);
        st_guide_line_.UpdateBlockingInfo(t, s_lower, true);
        st_guide_line_.UpdateBlockingInfo(t, s_upper, false);
        if (is_limited_by_lower_obs) {
          lower_obs_v = limiting_speed_info.first;
        }
        if (is_limited_by_upper_obs) {
          upper_obs_v = limiting_speed_info.second;
        }
      }
    } else {
      const std::string msg = "No valid st-boundary exists.";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }

    // Update into st_bound
    st_bound->at(i) = std::make_tuple(t, s_lower, s_upper);
    vt_bound->at(i) = std::make_tuple(t, lower_obs_v, upper_obs_v);
  }

  return Status::OK();
}

// Note: 对与每一个时刻t，对s gap做排序，选最优的gap，然后结合车辆动力学极限，获得t时刻的s bound
// 根据t时刻最优的s gap，可以确定对障碍物的决策，根据决策可以给出一个速度范围，从而获得vt_bound
Status STBoundsDecider::GenerateRegularSTBound(STBound* const st_bound,
                                               STBound* const vt_bound) {
  // Initialize st-boundary.
  // Note: 初始化st和vt bound为无穷大，采样间隔为0.1秒，与Prediction给的预测线的t采样间隔相同
  for (double curr_t = 0.0; curr_t <= st_bounds_config_.total_time();
       curr_t += kSTBoundsDeciderResolution) {
    st_bound->emplace_back(curr_t, std::numeric_limits<double>::lowest(),
                           std::numeric_limits<double>::max());
    vt_bound->emplace_back(curr_t, std::numeric_limits<double>::lowest(),
                           std::numeric_limits<double>::max());
  }

  // Sweep-line to get detailed ST-boundary.
  // Note: 对每一个时间点做处理
  for (size_t i = 0; i < st_bound->size(); ++i) {
    double t, s_lower, s_upper, lower_obs_v, upper_obs_v;
    std::tie(t, s_lower, s_upper) = st_bound->at(i);
    std::tie(t, lower_obs_v, upper_obs_v) = vt_bound->at(i);
    ADEBUG << "Processing st-boundary at t = " << t;

    // Get Boundary due to driving limits
    auto driving_limits_bound = st_driving_limits_.GetVehicleDynamicsLimits(t);
    s_lower = std::fmax(s_lower, driving_limits_bound.first);
    s_upper = std::fmin(s_upper, driving_limits_bound.second);
    ADEBUG << "Bounds for s due to driving limits are "
           << "s_upper = " << s_upper << ", s_lower = " << s_lower;

    // Get Boundary due to obstacles
    std::vector<std::pair<double, double>> available_s_bounds;
    std::vector<ObsDecSet> available_obs_decisions;
    // Remind(huachang): GetSBoundsFromDecisions里面有一个需要关注的点，
    // 如果判断overtake，就将obstacle的path_st_boundary替换为alternative_st_boundary
    if (!st_obstacles_processor_.GetSBoundsFromDecisions(
            t, &available_s_bounds, &available_obs_decisions)) {
      const std::string msg =
          "Failed to find a proper boundary due to obstacles.";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
    std::vector<std::pair<STBoundPoint, ObsDecSet>> available_choices;
    ADEBUG << "Available choices are:";
    for (int j = 0; j < static_cast<int>(available_s_bounds.size()); ++j) {
      ADEBUG << "  (" << available_s_bounds[j].first << ", "
             << available_s_bounds[j].second << ")";
      available_choices.emplace_back(
          std::make_tuple(0.0, available_s_bounds[j].first,
                          available_s_bounds[j].second),
          available_obs_decisions[j]);
    }
    // Note: 剔除available_choices中超出车辆动力学限制的STBound
    RemoveInvalidDecisions(driving_limits_bound, &available_choices);

    if (!available_choices.empty()) {
      ADEBUG << "One decision needs to be made among "
             << available_choices.size() << " choices.";
      // Note: 通过两个简单的规则对gaps进行排序，然后选规则下最优的
      // 规则很简单，两个gap比较，如果其中一个gap宽度小于3m，选较宽的
      // 上面一条规则没排出胜负的话，选包含st_guide_line_指导位置的gap
      // Remind(huachang): 排序规则看看能不能优化一下，目前这个最优gap等于是根据desired_v来选的
      // 如果自车现在处于静止状态，会倾向于选中能在预定时间内跑到desired_v*delta_t所在s位置的gap
      // 但这个加速可能是不太合理的，特别是这个desired_v的设定值比较大的时候(或者是自车限速比较低的时候)
      RankDecisions(st_guide_line_.GetGuideSFromT(t), driving_limits_bound,
                    &available_choices);
      // Select the top decision.
      auto top_choice_s_range = available_choices.front().first;
      // Note: 边界是不是由障碍物限制的
      bool is_limited_by_upper_obs = false;
      bool is_limited_by_lower_obs = false;
      // Note: 通过下面的操作，s_lower/s_upper成为由最优gap和driving_limits_bound的交集的上下界
      if (s_lower < std::get<1>(top_choice_s_range)) {
        s_lower = std::get<1>(top_choice_s_range);
        is_limited_by_lower_obs = true;
      }
      if (s_upper > std::get<2>(top_choice_s_range)) {
        s_upper = std::get<2>(top_choice_s_range);
        is_limited_by_upper_obs = true;
      }
      // Note: 经过上面的一顿操作，现在(s_lower, s_upper)是实际可行的s_gap

      // Set decision for obstacles without decisions.
      // Note: top choice区间对障碍物的决策
      auto top_choice_decision = available_choices.front().second;
      // Note: 把对障碍物的决策写入obs_id_to_decision_
      // 同时通过obs_id_to_st_boundary_[obs_id].SetBoundaryType设置boundary类型
      st_obstacles_processor_.SetObstacleDecision(top_choice_decision);

      // Update st-guide-line, st-driving-limit info, and v-limits.
      std::pair<double, double> limiting_speed_info;
      // Note: t时刻最近的上下STBoundary的斜率
      // Remind(huachang): 这里的两个斜率与实际上自车速度的限制边界没有必然联系，明显缺陷
      // 上方障碍物A的st_boundary的斜率不一定就比下方障碍物B的st_boundary的斜率大
      if (st_obstacles_processor_.GetLimitingSpeedInfo(t,
                                                       &limiting_speed_info)) {
        // Note: 更新了st_driving_limits_的lower_s0_和upper_s0_
        // Note: 传进去的limiting_speed_info信息没被使用
        // Remind(huachang): 里面的更新操作不太合理，导致最终出来的limit非常的大(下限非常低，上限非常高)，起不了什么作用
        st_driving_limits_.UpdateBlockingInfo(
            t, s_lower, limiting_speed_info.first, s_upper,
            limiting_speed_info.second);
        // Note: 当t时刻的guide_line_s超出[s_lower, s_upper]的时候，更新st_guide_line_的s0_位置
        st_guide_line_.UpdateBlockingInfo(t, s_lower, true);
        st_guide_line_.UpdateBlockingInfo(t, s_upper, false);
        if (is_limited_by_lower_obs) {
          lower_obs_v = limiting_speed_info.first;
        }
        if (is_limited_by_upper_obs) {
          upper_obs_v = limiting_speed_info.second;
        }
      }
    } else {
      const std::string msg = "No valid st-boundary exists.";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }

    // Update into st_bound
    // Remind(huachang): 由于st_driving_limits_的很多参数是写死的，而不是根据vehicle_param配置
    // 可能导致这里的st_bound与实际能到达的值不一致，导致被高估或者低估
    st_bound->at(i) = std::make_tuple(t, s_lower, s_upper);
    // Note: 根据yield/stop/overtake的decision，决定车速上下界
    vt_bound->at(i) = std::make_tuple(t, lower_obs_v, upper_obs_v);
  }

  return Status::OK();
}

// Note: 剔除available_choices中超出车辆动力学限制的STBound
void STBoundsDecider::RemoveInvalidDecisions(
    std::pair<double, double> driving_limit,
    std::vector<std::pair<STBoundPoint, ObsDecSet>>* available_choices) {
  // Remove those choices that don't even fall within driving-limits.
  size_t i = 0;
  while (i < available_choices->size()) {
    double s_lower = 0.0;
    double s_upper = 0.0;
    std::tie(std::ignore, s_lower, s_upper) = available_choices->at(i).first;
    if (s_lower > driving_limit.second || s_upper < driving_limit.first) {
      // Invalid bound, should be removed.
      if (i != available_choices->size() - 1) {
        swap(available_choices->at(i),
             available_choices->at(available_choices->size() - 1));
      }
      available_choices->pop_back();
    } else {
      // Valid bound, proceed to the next one.
      ++i;
    }
  }
}

// Note: 对available_choices执行冒泡排序，
// 1. 大区间优先(带条件)
// 2. 包含s_guide_line的区间优先
// Note: 冒泡排序是稳定的
void STBoundsDecider::RankDecisions(
    double s_guide_line, std::pair<double, double> driving_limit,
    std::vector<std::pair<STBoundPoint, ObsDecSet>>* available_choices) {
  // Perform sorting of the existing decisions.
  // Note: 冒泡排序
  bool has_swaps = true;
  while (has_swaps) {
    has_swaps = false;
    for (int i = 0; i < static_cast<int>(available_choices->size()) - 1; ++i) {
      double A_s_lower = 0.0;
      double A_s_upper = 0.0;
      std::tie(std::ignore, A_s_lower, A_s_upper) =
          available_choices->at(i).first;
      double B_s_lower = 0.0;
      double B_s_upper = 0.0;
      std::tie(std::ignore, B_s_lower, B_s_upper) =
          available_choices->at(i + 1).first;

      ADEBUG << "    Range ranking: A has s_upper = " << A_s_upper
             << ", s_lower = " << A_s_lower;
      ADEBUG << "    Range ranking: B has s_upper = " << B_s_upper
             << ", s_lower = " << B_s_lower;

      // If not both are larger than passable-threshold, should select
      // the one with larger room.
      double A_room = std::fmin(driving_limit.second, A_s_upper) -
                      std::fmax(driving_limit.first, A_s_lower);
      double B_room = std::fmin(driving_limit.second, B_s_upper) -
                      std::fmax(driving_limit.first, B_s_lower);
      if (A_room < kSTPassableThreshold || B_room < kSTPassableThreshold) {
        if (A_room < B_room) {
          swap(available_choices->at(i + 1), available_choices->at(i));
          has_swaps = true;
          ADEBUG << "Swapping to favor larger room.";
        }
        continue;
      }

      // Should select the one with overlap to guide-line
      // Remind(huachang): 这里是不是应该考虑一下加速度，难道现在静止状态下一脚油门轰到s_guide_line
      // Note: s_guide_line是以desired_v匀速前进的参考位置，但由于自车上一个t可能处于静止状态，这个是不是不太好
      bool A_contains_guideline =
          A_s_upper >= s_guide_line && A_s_lower <= s_guide_line;
      bool B_contains_guideline =
          B_s_upper >= s_guide_line && B_s_lower <= s_guide_line;
      if (A_contains_guideline != B_contains_guideline) {
        if (!A_contains_guideline) {
          swap(available_choices->at(i + 1), available_choices->at(i));
          has_swaps = true;
          ADEBUG << "Swapping to favor overlapping with guide-line.";
        }
        continue;
      }
    }
  }
}

void STBoundsDecider::RecordSTGraphDebug(
    const std::vector<STBoundary>& st_graph_data, const STBound& st_bound,
    planning_internal::STGraphDebug* const st_graph_debug) {
  if (!FLAGS_enable_record_debug || !st_graph_debug) {
    ADEBUG << "Skip record debug info";
    return;
  }

  // Plot ST-obstacle boundaries.
  for (const auto& boundary : st_graph_data) {
    auto boundary_debug = st_graph_debug->add_boundary();
    boundary_debug->set_name(boundary.id());
    if (boundary.boundary_type() == STBoundary::BoundaryType::YIELD) {
      boundary_debug->set_type(StGraphBoundaryDebug::ST_BOUNDARY_TYPE_YIELD);
      ADEBUG << "Obstacle ID = " << boundary.id() << ", decision = YIELD";
    } else if (boundary.boundary_type() == STBoundary::BoundaryType::OVERTAKE) {
      boundary_debug->set_type(StGraphBoundaryDebug::ST_BOUNDARY_TYPE_OVERTAKE);
      ADEBUG << "Obstacle ID = " << boundary.id() << ", decision = OVERTAKE";
    } else {
      boundary_debug->set_type(StGraphBoundaryDebug::ST_BOUNDARY_TYPE_UNKNOWN);
      ADEBUG << "Obstacle ID = " << boundary.id() << ", decision = UNKNOWN";
    }

    for (const auto& point : boundary.points()) {
      auto point_debug = boundary_debug->add_point();
      point_debug->set_t(point.x());
      point_debug->set_s(point.y());
    }
  }

  // Plot the chosen ST boundary.
  auto boundary_debug = st_graph_debug->add_boundary();
  boundary_debug->set_name("Generated ST-Boundary");
  boundary_debug->set_type(
      StGraphBoundaryDebug::ST_BOUNDARY_TYPE_DRIVABLE_REGION);
  for (const auto& st_bound_pt : st_bound) {
    auto point_debug = boundary_debug->add_point();
    double t = 0.0;
    double s_lower = 0.0;
    std::tie(t, s_lower, std::ignore) = st_bound_pt;
    point_debug->set_t(t);
    point_debug->set_s(s_lower);
    ADEBUG << "(" << t << ", " << s_lower << ")";
  }
  for (int i = static_cast<int>(st_bound.size()) - 1; i >= 0; --i) {
    auto point_debug = boundary_debug->add_point();
    double t = 0.0;
    double s_upper = 0.0;
    std::tie(t, std::ignore, s_upper) = st_bound[i];
    point_debug->set_t(t);
    point_debug->set_s(s_upper);
    ADEBUG << "(" << t << ", " << s_upper << ")";
  }
}

}  // namespace planning
}  // namespace apollo
