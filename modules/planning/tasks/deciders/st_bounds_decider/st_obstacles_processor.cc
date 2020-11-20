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

#include "modules/planning/tasks/deciders/st_bounds_decider/st_obstacles_processor.h"

#include <algorithm>
#include <unordered_set>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/proto/decision.pb.h"

#include "cyber/common/log.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/line_segment2d.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/util/util.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::PathPoint;
using apollo::common::Status;
using apollo::common::math::Box2d;
using apollo::common::math::LineSegment2d;
using apollo::common::math::Vec2d;

namespace {
// ObsTEdge contains: (is_starting_t, t, s_min, s_max, obs_id).
using ObsTEdge = std::tuple<int, double, double, double, std::string>;
}  // namespace

void STObstaclesProcessor::Init(const double planning_distance,
                                const double planning_time,
                                const PathData& path_data,
                                PathDecision* const path_decision) {
  // Note: 规划时间，固定配置值，7s
  planning_time_ = planning_time;
  // Note: path长度
  planning_distance_ = planning_distance;
  path_data_ = path_data;
  vehicle_param_ = common::VehicleConfigHelper::GetConfig().vehicle_param();
  // Note: adc_path_init_s_一般就是0.0
  // 具体原因查看PathData::SetFrenetPath，将frenet path转成xy时，对第一个点的s置0了，后面累加
  adc_path_init_s_ = path_data_.discretized_path().front().s();
  path_decision_ = path_decision;

  obs_t_edges_.clear();
  obs_t_edges_idx_ = 0;

  obs_id_to_st_boundary_.clear();
  obs_id_to_decision_.clear();
  candidate_clear_zones_.clear();
  obs_id_to_alternative_st_boundary_.clear();
}

// Note: 算障碍物的精确的st-boundary，构建st-graph
// Note: 凡是与adc path会有overlap的障碍物，都会有STBoundary
// 对于那些没有与path overlap的障碍物，会被忽略，并添加ignore decision
// Note: 预备知识
// Prediction给的障碍物预测轨迹，t=0的点为障碍物所在位置点
Status STObstaclesProcessor::MapObstaclesToSTBoundaries(
    PathDecision* const path_decision) {
  // Sanity checks.
  if (path_decision == nullptr) {
    const std::string msg = "path_decision is nullptr";
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  if (planning_time_ < 0.0) {
    const std::string msg = "Negative planning time.";
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  if (planning_distance_ < 0.0) {
    const std::string msg = "Negative planning distance.";
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  if (path_data_.discretized_path().size() <= 1) {
    const std::string msg = "Number of path points is too few.";
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  obs_id_to_st_boundary_.clear();

  // Some preprocessing to save the adc_low_road_right segments.
  // Note: 找ADC的低路权路段
  // Note: 下面这一顿操作就是为了算哪些segments(start_s, end_s)是不在原车道上的
  // 不在原来车道上时，路权低(low road right)
  // Note: path_point_decision_guide是在path_assessment_decider中给出的
  bool is_adc_low_road_right_beginning = true;
  for (const auto& path_pt_info : path_data_.path_point_decision_guide()) {
    double path_pt_s = 0.0;
    PathData::PathPointType path_pt_type;
    std::tie(path_pt_s, path_pt_type, std::ignore) = path_pt_info;
    if (path_pt_type == PathData::PathPointType::OUT_ON_FORWARD_LANE ||
        path_pt_type == PathData::PathPointType::OUT_ON_REVERSE_LANE) {
      if (is_adc_low_road_right_beginning) {
        adc_low_road_right_segments_.emplace_back(path_pt_s, path_pt_s);
        is_adc_low_road_right_beginning = false;
      } else {
        adc_low_road_right_segments_.back().second = path_pt_s;
      }
    } else if (path_pt_type == PathData::PathPointType::IN_LANE) {
      if (!is_adc_low_road_right_beginning) {
        is_adc_low_road_right_beginning = true;
      }
    }
  }

  // Map obstacles into ST-graph.
  // Go through every obstacle and plot them in ST-graph.
  std::unordered_set<std::string> non_ignore_obstacles;
  // Note: lower_points.front().s()最小的静态障碍物
  // Note: 顾名思义，即最近的需要停车的(静态)障碍物
  std::tuple<std::string, STBoundary, Obstacle*> closest_stop_obstacle;
  std::get<0>(closest_stop_obstacle) = "NULL";
  for (const auto* obs_item_ptr : path_decision->obstacles().Items()) {
    // Sanity checks.
    Obstacle* obs_ptr = path_decision->Find(obs_item_ptr->Id());
    if (obs_ptr == nullptr) {
      const std::string msg = "Null obstacle pointer.";
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }

    // Draw the obstacle's st-boundary.
    std::vector<STPoint> lower_points;
    std::vector<STPoint> upper_points;
    // Note: 障碍物当前位置是否完全处于adc的低路权区间内
    bool is_caution_obstacle = false;
    // Note: 障碍物离开adc低路权区间的时间，过了这个时刻，adc重新处于高路权
    // Note: 如果adc不离开参考线所在的车道，就不会有低路权路段，这个obs_caution_end_t还是0.0
    double obs_caution_end_t = 0.0;
    if (!ComputeObstacleSTBoundary(*obs_ptr, &lower_points, &upper_points,
                                   &is_caution_obstacle, &obs_caution_end_t)) {
      // Obstacle doesn't appear on ST-Graph.
      // Note: Obstacle轨迹与自车Path不相交（ADC的左右两边分别扩充了0.1米作为buffer）
      // Remind(huachang): Obstacle轨迹与自车Path不相交则完全不管这个障碍物了
      // 导致障碍物cutin时如果没有预测线甩过来，完全不会刹车
      continue;
    }

/*
  是正向行驶障碍物的STBoundary大致形状，横轴为t，纵轴为s
                    .
              .     | 
         .          |
    .               .
    |         .
    |    .
    .
*/

/*
  是逆向行驶障碍物的STBoundary大致形状，横轴为t，纵轴为s
    .
    |    .
    |         .
    .               .
         .          |
              .     |
                    .
*/

    // Note: 根据boundary point创建STBoundary(这个继承自Polygon)
    // Note: 即使是逆行车辆，它的STBoundary也是可以正常创建的，并且最终的polygon与正常行驶的障碍物一样
    auto boundary =
        STBoundary::CreateInstanceAccurate(lower_points, upper_points);
    boundary.set_id(obs_ptr->Id());
    if (is_caution_obstacle) {
      // Note: 障碍物第一个点的overlap_s完全位于自车的低路权区间内，
      // 本意是障碍物目前位于自车path低路权区间内，但具体实现偏离了这个想法
      // Note: 障碍物在自车path低路权路段最晚的relative time(基于障碍物轨迹的timestamp)
      // Note: 这个obs_caution_end_t记录什么时候不需要再cautious?
      boundary.set_obstacle_road_right_ending_t(obs_caution_end_t);
    }
    // Update the trimmed obstacle into alternative st-bound storage
    // for later uses.
    // Note: 把obs_caution_end_t后面的ST point删除，作为alternative_boundary
    // Remind(huachang): 如果自车没有低路权区间，那么这个polygon的点几乎全被删了，只留下上下各两个点
    // Remind(huachang): 由于两个点列的点数是一样的，这里完全可以把两个while循环合成一个
    while (lower_points.size() > 2 &&
           lower_points.back().t() > obs_caution_end_t) {
      lower_points.pop_back();
    }
    while (upper_points.size() > 2 &&
           upper_points.back().t() > obs_caution_end_t) {
      upper_points.pop_back();
    }
    // Note: 根据lower_points和upper_points创建精确的polygon boundary
    // 这个boundary单纯就是由原始的这些点围成的polygon，不做RemoveRedundantPoints
    auto alternative_boundary =
        STBoundary::CreateInstanceAccurate(lower_points, upper_points);
    alternative_boundary.set_id(obs_ptr->Id());
    // Note: 比obs_id_to_st_boundary_多了静态障碍物
    obs_id_to_alternative_st_boundary_[obs_ptr->Id()] = alternative_boundary;
    ADEBUG << "Obstacle " << obs_ptr->Id()
           << " has an alternative st-boundary with "
           << lower_points.size() + upper_points.size() << " points.";

    // Store all Keep-Clear zone together.
    // Note: 禁停区单独处理
    if (obs_item_ptr->Id().find("KC") != std::string::npos) {
      candidate_clear_zones_.push_back(
          make_tuple(obs_ptr->Id(), boundary, obs_ptr));
      continue;
    }

    // Process all other obstacles than Keep-Clear zone.
    if (obs_ptr->Trajectory().trajectory_point().empty()) {
      // Obstacle is static.
      // Note: 尝试更新最近的需要停车的静止障碍物
      if (std::get<0>(closest_stop_obstacle) == "NULL" ||
          std::get<1>(closest_stop_obstacle).bottom_left_point().s() >
              boundary.bottom_left_point().s()) {
        // If this static obstacle is closer for ADC to stop, record it.
        closest_stop_obstacle =
            std::make_tuple(obs_ptr->Id(), boundary, obs_ptr);
      }
    } else {
      // Obstacle is dynamic.
      // Note: 在ADC后方的障碍物都被忽略了
      // Note: ST Point的s值都来自adc_path_points的s，
      // 这个值不可能小于adc_path_init_s_，最多就是等于
      // 也就是说，这里的逻辑就是如果障碍物在kTIgnoreThreshold之后才与adc_path_init_s_相交，则忽略这个障碍物
      // Remind(huachang): 原意就是在本车道行驶时不管后方车辆，但有些特殊情况会出现危险
      if (boundary.bottom_left_point().s() - adc_path_init_s_ <
              kSIgnoreThreshold &&
          boundary.bottom_left_point().t() > kTIgnoreThreshold) {
        // Ignore obstacles that are behind.
        // TODO(jiacheng): don't ignore if ADC is in dangerous segments.
        // Note: 这个障碍物在一段时间后会穿过adc的位置(adc_path_init_s_)
        // Remind(huachang): 这个处理有问题的，旁边的车怼过来了有可能因为这个被ignore掉
        // 经过仿真测试，有几种情况会导致障碍物因为这个判断而被ignore:
        // 1. 正后方障碍物一段时间后会经过adc_path_init_point
        // 2. 斜后方的车辆准备cutin，预测第一个与path有overlap的点为adc_path_init_point
        // 3. 斜前方的逆行车辆要cutin，预测线中第一个与path有overlap的点为adc_path_init_point
        continue;
      }
      // Note: 注意，静态障碍物目前没添加到这里
      // Note: 后面会把最近的需要stop的障碍物的boundary添加进来
      obs_id_to_st_boundary_[obs_ptr->Id()] = boundary;
      // Note: st_boundary从这里写进了path_decision中
      obs_ptr->set_path_st_boundary(boundary);
      non_ignore_obstacles.insert(obs_ptr->Id());
      ADEBUG << "Adding " << obs_ptr->Id() << " into the ST-graph.";
    }
  }
  // For static obstacles, only retain the closest one (also considers
  // Keep-Clear zone here).
  // Note: We only need to check the overlapping between the closest obstacle
  //       and all the Keep-Clear zones. Because if there is another obstacle
  //       overlapping with a Keep-Clear zone, which results in an even closer
  //       stop fence, then that very Keep-Clear zone must also overlap with
  //       the closest obstacle. (Proof omitted here)
  // Note: 这里为什么要把Keep-Clear zones单独拎出来处理，感觉可以与上面的静态障碍物合并处理
  if (std::get<0>(closest_stop_obstacle) != "NULL") {
    std::string closest_stop_obs_id;
    STBoundary closest_stop_obs_boundary;
    Obstacle* closest_stop_obs_ptr;
    std::tie(closest_stop_obs_id, closest_stop_obs_boundary,
             closest_stop_obs_ptr) = closest_stop_obstacle;
    ADEBUG << "Closest obstacle ID = " << closest_stop_obs_id;
    // Go through all Keep-Clear zones, and see if there is an even closer
    // stop fence due to them.
    if (!closest_stop_obs_ptr->IsVirtual()) {
      for (const auto& clear_zone : candidate_clear_zones_) {
        const auto& clear_zone_boundary = std::get<1>(clear_zone);
        if (closest_stop_obs_boundary.min_s() >= clear_zone_boundary.min_s() &&
            closest_stop_obs_boundary.min_s() <= clear_zone_boundary.max_s()) {
          // Note: closest_stop_obstacle位于禁停区内，那么这个禁停区成为新的closest_stop_obstacle
          std::tie(closest_stop_obs_id, closest_stop_obs_boundary,
                   closest_stop_obs_ptr) = clear_zone;
          ADEBUG << "Clear zone " << closest_stop_obs_id << " is closer.";
          break;
        }
      }
    }
    // Note: 所以obs_id_to_st_boundary_里面包含了所有不被ignore的动态障碍物以及closest_stop_obs_boundary
    obs_id_to_st_boundary_[closest_stop_obs_id] = closest_stop_obs_boundary;
    closest_stop_obs_ptr->set_path_st_boundary(closest_stop_obs_boundary);
    non_ignore_obstacles.insert(closest_stop_obs_id);
    ADEBUG << "Adding " << closest_stop_obs_ptr->Id() << " into the ST-graph.";
    ADEBUG << "min_s = " << closest_stop_obs_boundary.min_s();
  }

  // Set IGNORE decision for those that are not in ST-graph:
  for (const auto* obs_item_ptr : path_decision->obstacles().Items()) {
    Obstacle* obs_ptr = path_decision->Find(obs_item_ptr->Id());
    if (non_ignore_obstacles.count(obs_ptr->Id()) == 0) {
      ObjectDecisionType ignore_decision;
      ignore_decision.mutable_ignore();
      if (!obs_ptr->HasLongitudinalDecision()) {
        obs_ptr->AddLongitudinalDecision("st_obstacle_processor",
                                         ignore_decision);
      }
      if (!obs_ptr->HasLateralDecision()) {
        obs_ptr->AddLateralDecision("st_obstacle_processor", ignore_decision);
      }
    }
  }

  // Preprocess the obstacles for sweep-line algorithms.
  // Fetch every obstacle's beginning end ending t-edges only.
  // Note: 保存boundary的左右两边平行的边，即对应min_t和max_t的edge
  for (const auto& it : obs_id_to_st_boundary_) {
    obs_t_edges_.emplace_back(true, it.second.min_t(),
                              it.second.bottom_left_point().s(),
                              it.second.upper_left_point().s(), it.first);
    obs_t_edges_.emplace_back(false, it.second.max_t(),
                              it.second.bottom_right_point().s(),
                              it.second.upper_right_point().s(), it.first);
  }
  // Sort the edges.
  // Note: 根据t从小到大排序，如果t相同，则is_starting_t的edge优先
  std::sort(obs_t_edges_.begin(), obs_t_edges_.end(),
            [](const ObsTEdge& lhs, const ObsTEdge& rhs) {
              if (std::get<1>(lhs) != std::get<1>(rhs)) {
                return std::get<1>(lhs) < std::get<1>(rhs);
              } else {
                return std::get<0>(lhs) > std::get<0>(rhs);
              }
            });

  return Status::OK();
}

std::unordered_map<std::string, STBoundary>
STObstaclesProcessor::GetAllSTBoundaries() {
  return obs_id_to_st_boundary_;
}

// Note: 根据yield/stop/overtake的decision，以及boundary的斜率，决定车速上下界
bool STObstaclesProcessor::GetLimitingSpeedInfo(
    double t, std::pair<double, double>* const limiting_speed_info) {
  if (obs_id_to_decision_.empty()) {
    // If no obstacle, then no speed limits.
    return false;
  }

  double s_min = 0.0;
  double s_max = planning_distance_;
  for (auto it : obs_id_to_decision_) {
    auto obs_id = it.first;
    auto obs_decision = it.second;
    auto obs_st_boundary = obs_id_to_st_boundary_[obs_id];
    double obs_s_min = 0.0;
    double obs_s_max = 0.0;
    obs_st_boundary.GetBoundarySRange(t, &obs_s_max, &obs_s_min);
    double obs_ds_lower = 0.0;
    double obs_ds_upper = 0.0;
    obs_st_boundary.GetBoundarySlopes(t, &obs_ds_upper, &obs_ds_lower);
    if (obs_decision.has_yield() || obs_decision.has_stop()) {
      if (obs_s_min <= s_max) {
        s_max = obs_s_min;
        limiting_speed_info->second = obs_ds_lower;
      }
    } else if (it.second.has_overtake()) {
      if (obs_s_max >= s_min) {
        s_min = obs_s_max;
        limiting_speed_info->first = obs_ds_upper;
      }
    }
  }
  return s_min <= s_max;
}

// Note: ST图中在t位置做垂线，对垂线做line sweep，找出gaps，然后基于每种gap对obstacles做decision
bool STObstaclesProcessor::GetSBoundsFromDecisions(
    double t, std::vector<std::pair<double, double>>* const available_s_bounds,
    std::vector<std::vector<std::pair<std::string, ObjectDecisionType>>>* const
        available_obs_decisions) {
  // Sanity checks.
  available_s_bounds->clear();
  available_obs_decisions->clear();

  // Gather any possible change in st-boundary situations.
  ADEBUG << "There are " << obs_t_edges_.size() << " t-edges.";
  // Note: 新扫到的edges
  std::vector<ObsTEdge> new_t_edges;
  // Note: ObsTEdge contains: (is_starting_t, t, s_min, s_max, obs_id).
  // Note: 传进来的t以0.1秒的step递增，这就是一个sweep line从左往右扫obs_t_edges_
  // 这一轮扫到的obs_t_edge都放到new_t_edges中
  while (obs_t_edges_idx_ < static_cast<int>(obs_t_edges_.size()) &&
         std::get<1>(obs_t_edges_[obs_t_edges_idx_]) <= t) {
    if (std::get<0>(obs_t_edges_[obs_t_edges_idx_]) == 0 &&
        std::get<1>(obs_t_edges_[obs_t_edges_idx_]) == t) {
      // Note: t时刻之前(包含t时刻)已经没有剩余的入边了
      // Note: obs_t_edges_中的edge都是排好序的
      // Note: 这里直接break了是不是有bug，后面可能还会有符合这个判断的edge，
      //       即有多个障碍物的st boundary的右边界在同一个时刻t
      //       会导致下面这个障碍物的obs_id_to_decision_没被erase
      break;
    }
    ADEBUG << "Seeing a new t-edge at t = "
           << std::get<1>(obs_t_edges_[obs_t_edges_idx_]);
    new_t_edges.push_back(obs_t_edges_[obs_t_edges_idx_]);
    // 扫面线往右
    ++obs_t_edges_idx_;
  }

  // For st-boundaries that disappeared before t, remove them.
  for (const auto& obs_t_edge : new_t_edges) {
    if (std::get<0>(obs_t_edge) == 0) {
      // Note: 扫描线越过了这个障碍物的st-boundary，把对这个障碍物的决策清掉
      ADEBUG << "Obstacle id: " << std::get<4>(obs_t_edge)
             << " is leaving st-graph.";
      if (obs_id_to_decision_.count(std::get<4>(obs_t_edge)) != 0) {
        obs_id_to_decision_.erase(std::get<4>(obs_t_edge));
      }
    }
  }

  // For overtaken obstacles, remove them if we are after
  // their high right-of-road ending time (with a margin).
  // Note: overtake高路权障碍物一段时间之后
  // Remind(huachang): 这个路权设定不太合理，需要障碍物当前时刻在ADC的低路权路段内
  // 这个设定导致ADC在借道绕行时，几乎都不会考虑所借道的Lane后方的障碍物
  std::vector<std::string> obs_id_to_remove;
  // Note: 对前面的扫面已经做出的决策做处理
  for (const auto& obs_id_to_decision_pair : obs_id_to_decision_) {
    auto obs_id = obs_id_to_decision_pair.first;
    auto obs_decision = obs_id_to_decision_pair.second;
    auto obs_st_boundary = obs_id_to_st_boundary_[obs_id];
    if (obs_decision.has_overtake() &&
        // Note: adc越过overlap起始点已经过了一段时间
        obs_st_boundary.min_t() <= t - kOvertakenObsCautionTime &&
        // Note: adc越过低路权区间已经过了一段时间
        obs_st_boundary.obstacle_road_right_ending_t() <=
            t - kOvertakenObsCautionTime) {
      obs_id_to_remove.push_back(obs_id_to_decision_pair.first);
    }
  }
  for (const auto& obs_id : obs_id_to_remove) {
    obs_id_to_decision_.erase(obs_id);
    // Change the displayed st-boundary to the alternative one:
    if (obs_id_to_alternative_st_boundary_.count(obs_id) > 0) {
      Obstacle* obs_ptr = path_decision_->Find(obs_id);
      // Note: 把obstacle的高路权之后的st boundary删掉了
      // Note: 这个处理让绕行/变道变得很危险
      obs_id_to_st_boundary_[obs_id] =
          obs_id_to_alternative_st_boundary_[obs_id];
      obs_id_to_st_boundary_[obs_id].SetBoundaryType(
          STBoundary::BoundaryType::OVERTAKE);
      // Note: 如果判断overtake，就将obstacle的path_st_boundary替换为alternative_st_boundary
      obs_ptr->set_path_st_boundary(obs_id_to_alternative_st_boundary_[obs_id]);
    }
  }

  // Based on existing decisions, get the s-boundary.
  // Note: 根据决策更新可行驶的s范围[s_min, s_max]
  // Note: 如果对障碍物已有决策，
  // 如果是yield/stop的决策，则可行驶边界的上界小于该障碍物st-boundary的下界
  // 如果是overtake的决策，则可行驶边界的下界应该大于该障碍物st-boundary的上界
  double s_min = 0.0;
  double s_max = planning_distance_;
  for (auto it : obs_id_to_decision_) {
    auto obs_id = it.first;
    auto obs_decision = it.second;
    auto obs_st_boundary = obs_id_to_st_boundary_[obs_id];
    double obs_s_min = 0.0;
    double obs_s_max = 0.0;
    // Note: 计算obstacle对应时间的STBoundary的上下界，即这个polygon在横坐标为t时截面的下界和上界
    obs_st_boundary.GetBoundarySRange(t, &obs_s_max, &obs_s_min);
    // Note: 根据决策更新当前t时刻可以通行的s区间
    if (obs_decision.has_yield() || obs_decision.has_stop()) {
      s_max = std::fmin(s_max, obs_s_min);
    } else if (it.second.has_overtake()) {
      s_min = std::fmax(s_min, obs_s_max);
    }
  }
  // Note: 不可通行
  if (s_min > s_max) {
    return false;
  }
  ADEBUG << "S-boundary based on existing decisions = (" << s_min << ", "
         << s_max << ")";

  // For newly entering st_boundaries, determine possible new-boundaries.
  // For apparent ones, make decisions directly.
  // Note: ObsTEdge contains: (is_starting_t, t, s_min, s_max, obs_id)
  std::vector<ObsTEdge> ambiguous_t_edges;
  for (auto obs_t_edge : new_t_edges) {
    ADEBUG << "For obstacle id: " << std::get<4>(obs_t_edge)
           << ", its s-range = [" << std::get<2>(obs_t_edge) << ", "
           << std::get<3>(obs_t_edge) << "]";
    // 根据障碍物st-boundary的入边做决策，想象st图比较好理解
    if (std::get<0>(obs_t_edge) == 1) {
      if (std::get<2>(obs_t_edge) >= s_max) {
        ADEBUG << "  Apparently, it should be yielded.";
        obs_id_to_decision_[std::get<4>(obs_t_edge)] =
            // Note: 根本没必要调下面这个函数，这里直接给yield决策就是了
            DetermineObstacleDecision(std::get<2>(obs_t_edge),
                                      std::get<3>(obs_t_edge), s_max);
        obs_id_to_st_boundary_[std::get<4>(obs_t_edge)].SetBoundaryType(
            STBoundary::BoundaryType::YIELD);
      } else if (std::get<3>(obs_t_edge) <= s_min) {
        ADEBUG << "  Apparently, it should be overtaken.";
        obs_id_to_decision_[std::get<4>(obs_t_edge)] =
            // Note: 根本没必要调下面这个函数，这里直接给overtake决策就是了
            DetermineObstacleDecision(std::get<2>(obs_t_edge),
                                      std::get<3>(obs_t_edge), s_min);
        obs_id_to_st_boundary_[std::get<4>(obs_t_edge)].SetBoundaryType(
            STBoundary::BoundaryType::OVERTAKE);
      } else {
        ADEBUG << "  It should be further analyzed.";
        // Note: 待定，这些obs_t_edge与区间[s_min, s_max]有交集
        ambiguous_t_edges.push_back(obs_t_edge);
      }
    }
  }
  // For ambiguous ones, enumerate all decisions and corresponding bounds.
  // Note: t时刻垂直方向上[s_min, s_max]的Gaps
  auto s_gaps = FindSGaps(ambiguous_t_edges, s_min, s_max);
  if (s_gaps.empty()) {
    // Note: 没有可行的gap，不可通过
    return false;
  }
  for (auto s_gap : s_gaps) {
    available_s_bounds->push_back(s_gap);
    std::vector<std::pair<std::string, ObjectDecisionType>> obs_decisions;
    // Note: 决策待定的入边
    for (auto obs_t_edge : ambiguous_t_edges) {
      std::string obs_id = std::get<4>(obs_t_edge);
      double obs_s_min = std::get<2>(obs_t_edge);
      double obs_s_max = std::get<3>(obs_t_edge);
      obs_decisions.emplace_back(
          obs_id,
          DetermineObstacleDecision(obs_s_min, obs_s_max,
                                    (s_gap.first + s_gap.second) / 2.0));
    }
    // Note: 选择每个可能的gap时，对障碍物的决策是不一样的，把这些不一样的gap对应的obs_decisions记录下来
    available_obs_decisions->push_back(obs_decisions);
  }

  return true;
}

// Note: 将决策信息写入obs_id_to_decision_和obs_id_to_st_boundary_
// 同时将决策信息写入history_
void STObstaclesProcessor::SetObstacleDecision(
    const std::string& obs_id, const ObjectDecisionType& obs_decision) {
  obs_id_to_decision_[obs_id] = obs_decision;
  ObjectStatus object_status;
  object_status.mutable_motion_type()->mutable_dynamic();
  if (obs_decision.has_yield() || obs_decision.has_stop()) {
    obs_id_to_st_boundary_[obs_id].SetBoundaryType(
        STBoundary::BoundaryType::YIELD);
    object_status.mutable_decision_type()->mutable_yield();
  } else if (obs_decision.has_overtake()) {
    obs_id_to_st_boundary_[obs_id].SetBoundaryType(
        STBoundary::BoundaryType::OVERTAKE);
    object_status.mutable_decision_type()->mutable_overtake();
  }
  history_->mutable_history_status()->SetObjectStatus(obs_id, object_status);
}

void STObstaclesProcessor::SetObstacleDecision(
    const std::vector<std::pair<std::string, ObjectDecisionType>>&
        obstacle_decisions) {
  for (auto obs_decision : obstacle_decisions) {
    SetObstacleDecision(obs_decision.first, obs_decision.second);
  }
}

///////////////////////////////////////////////////////////////////////////////
// Private helper functions.
// Note: is_caution_obstacle: 障碍物第一个点与自车path在低路权区间内overlap
//       即障碍物目前位于自车path低路权区间内(只有自车离开离开参考线所在的lane时才会出现低路权的情况)
// obs_caution_end_t:
// 1. 静态障碍物的obs_caution_end_t = planning_time_
// 或者
// 2. 动态障碍物离开低路权区间的relative time
// Note: 如果一个障碍物的轨迹与自车Path没有任何overlap，则返回false
// Note: 这里设置的kADCSafetyLBuffer只有0.1米，是不是应该把buffer加大一些
bool STObstaclesProcessor::ComputeObstacleSTBoundary(
    const Obstacle& obstacle, std::vector<STPoint>* const lower_points,
    std::vector<STPoint>* const upper_points, bool* const is_caution_obstacle,
    double* const obs_caution_end_t) {
  lower_points->clear();
  upper_points->clear();
  *is_caution_obstacle = false;
  const auto& adc_path_points = path_data_.discretized_path();
  const auto& obs_trajectory = obstacle.Trajectory();

  if (obs_trajectory.trajectory_point().empty()) {
    // Processing a static obstacle.
    // Sanity checks.
    if (!obstacle.IsStatic()) {
      AWARN << "Non-static obstacle[" << obstacle.Id()
            << "] has NO prediction trajectory."
            << obstacle.Perception().ShortDebugString();
    }
    // Get the overlapping s between ADC path and obstacle's perception box.
    const Box2d& obs_box = obstacle.PerceptionBoundingBox();
    std::pair<double, double> overlapping_s;
    // Note: 找path与obs_box有overlap的s区间
    if (GetOverlappingS(adc_path_points, obs_box, kADCSafetyLBuffer,
                        &overlapping_s)) {
      // Note: 自车位于overlapping_s区间内时，会与障碍物有overlap
      lower_points->emplace_back(overlapping_s.first, 0.0);
      lower_points->emplace_back(overlapping_s.first, planning_time_);
      upper_points->emplace_back(overlapping_s.second, 0.0);
      upper_points->emplace_back(overlapping_s.second, planning_time_);
    }
    *is_caution_obstacle = true;
    // Note: 静态障碍物的obs_caution_end_t = planning_time_
    *obs_caution_end_t = planning_time_;
  } else {
    // Processing a dynamic obstacle.
    // Go through every occurrence of the obstacle at all timesteps, and
    // figure out the overlapping s-max and s-min one by one.
    bool is_obs_first_traj_pt = true;
    // Note: 想象一下如果障碍物是逆行车辆，下面的处理结果会与正常行驶的动态障碍物有什么不一样
    for (const auto& obs_traj_pt : obs_trajectory.trajectory_point()) {
      // TODO(jiacheng): Currently, if the obstacle overlaps with ADC at
      // disjoint segments (happens very rarely), we merge them into one.
      // In the future, this could be considered in greater details rather
      // than being approximated.
      // Note: 意思是，即使某个obs_traj_pt没有产生overlap，
      // 但由于前后的点有overlap而导致，ST图中这个位置是不可通行的(被包含在ST polygon里面)
      const Box2d& obs_box = obstacle.GetBoundingBox(obs_traj_pt);
      ADEBUG << obs_box.DebugString();
      std::pair<double, double> overlapping_s;
      if (GetOverlappingS(adc_path_points, obs_box, kADCSafetyLBuffer,
                          &overlapping_s)) {
        ADEBUG << "Obstacle instance is overlapping with ADC path.";
        // Note: 这里的ST boundary point的时间是按照障碍物的轨迹点的relative_time定的
        // Note: lower_points和upper_points都是一起添加的，t是一样的
        // Note: 如果障碍物行驶方向与自车方向相反，则这些lower_points/upper_points的s值是从大到小排的
        lower_points->emplace_back(overlapping_s.first,
                                   obs_traj_pt.relative_time());
        upper_points->emplace_back(overlapping_s.second,
                                   obs_traj_pt.relative_time());
        // Note: 对障碍物当前位置做判断
        // Note: 只有当障碍物当前就在adc低路权区间时，才会成为一个caution_obstacle
        if (is_obs_first_traj_pt) {
          // Note: 障碍物第一个轨迹点处于自车低路权区域
          // Note: 即障碍物目前处于高路权状态，adc需要避让该障碍物
          // Note: 这里要求overlapping_s完全位于自车的低路权区间内，非常不合理
          // 万一这个障碍物是个超级长的大货车，那么这个障碍物就因此无法成为一个需要caution的障碍物了
          // Remind(huachang): 这里是不是应该改成区间有交集就应该是is_caution_obstacle
          if (IsSWithinADCLowRoadRightSegment(overlapping_s.first) ||
              IsSWithinADCLowRoadRightSegment(overlapping_s.second)) {
            *is_caution_obstacle = true;
          }
        }
        if ((*is_caution_obstacle)) {
          if (IsSWithinADCLowRoadRightSegment(overlapping_s.first) ||
              IsSWithinADCLowRoadRightSegment(overlapping_s.second)) {
            *obs_caution_end_t = obs_traj_pt.relative_time();
          }
        }
      }
      is_obs_first_traj_pt = false;
    }
    // Note: 如果障碍物的轨迹只有一个点与adc的path是overlap的，就补一小截用于形成一个四边形
    if (lower_points->size() == 1) {
      lower_points->emplace_back(lower_points->front().s(),
                                 lower_points->front().t() + 0.1);
      upper_points->emplace_back(upper_points->front().s(),
                                 upper_points->front().t() + 0.1);
    }
  }

  return (!lower_points->empty() && !upper_points->empty());
}

// Note: 找障碍物某个时刻与ADC path相交的s区间，没有overlap则返回false
// Note: 这些overlapping_s指的都是adc path的s
bool STObstaclesProcessor::GetOverlappingS(
    const std::vector<PathPoint>& adc_path_points,
    const Box2d& obstacle_instance, const double adc_l_buffer,
    std::pair<double, double>* const overlapping_s) {
  // Locate the possible range to search in details.
  // Note: 为什么搜索范围用[0, adc_path_points.size()) - 2] ?
  // Note: 是为了能取到下一个点，用于确定朝向（GetSBoundingPathPointIndex里面用到朝向）
  // Note: 这里的逻辑要求adc_path_points.size不小于2
  // Note: 跟在obstacle屁股后面距离超过front_edge_to_center的临界点
  // Note: 由于传入的点不包含最后一个点，因此最后一个点不在考虑范围内
  int pt_before_idx = GetSBoundingPathPointIndex(
      adc_path_points, obstacle_instance, vehicle_param_.front_edge_to_center(),
      true, 0, static_cast<int>(adc_path_points.size()) - 2);
  ADEBUG << "The index before is " << pt_before_idx;
  // Note: 在障碍物前方超过back_edge_to_center距离的临界点
  int pt_after_idx = GetSBoundingPathPointIndex(
      adc_path_points, obstacle_instance, vehicle_param_.back_edge_to_center(),
      false, 0, static_cast<int>(adc_path_points.size()) - 2);
  ADEBUG << "The index after is " << pt_after_idx;
  // Note: adc_path_points的倒数第二个点在障碍物屁股后面至少front_edge_to_center的距离
  // 认为障碍物与path不会有overlap
  // Note: 但是，实际上，最后一个点可能与障碍物时有overlap的
  if (pt_before_idx == static_cast<int>(adc_path_points.size()) - 2) {
    return false;
  }
  // Note: adc_path_points的第一个点在障碍物前方至少back_edge_to_center的距离
  // 认为障碍物与path不会有overlap
  if (pt_after_idx == 0) {
    return false;
  }

  // Note: 没找到pt_before_idx，第一个点都不满足要求
  // 即所有path point都不满足在障碍物屁股后面距离大于front_edge_to_center的条件
  // 注意，这并不是说障碍物在s方向上与path没有overlap，可能在区间[0, front_edge_to_center)内overlap了
  if (pt_before_idx == -1) {
    pt_before_idx = 0;
  }
  // Note: 没找到pt_after_idx，倒数第二个点都不符合要求
  // 即所有path point都不满足在障碍物前方距离超过back_edge_to_center的条件
  // Note: 此时，障碍物可能与path末尾那些点有overlap
  if (pt_after_idx == -1) {
    pt_after_idx = static_cast<int>(adc_path_points.size()) - 2;
  }
  // Remind(huachang): 什么时候会出现这种情况?
  if (pt_before_idx >= pt_after_idx) {
    return false;
  }

  // Note: 注意，上面这一通操作并不理会障碍物是否真的与path有overlap，
  // 只是检查障碍物在s方向上是否接近path，算出来的这个[pt_before_idx, pt_after_idx]区间，
  // 可能在横向上与障碍物距离非常远，从而没有overlap
  // 上面这个过程只是为了筛选可能的overlap区间，减少调用IsADCOverlappingWithObstacle的次数(因为这个比较耗时)

  // Detailed searching.
  // Note: 找与obstacle_instance有overlap的区间边界(overlapping_s)
  // overlapping_s->first和second对应的点都是与obstacle_instance没有overlap的
  // Note: 注意，如果adc_path_points的第一个点都overlap的话，
  // 那么第一个点没有前置的点，则返回的overlapping_s->first对应的点也是overlap的
  // Note: 找到第一个overlap的点，然后把前一个点作为overlapping_s->first
  bool has_overlapping = false;
  for (int i = pt_before_idx; i <= pt_after_idx; ++i) {
    ADEBUG << "At ADC path index = " << i << " :";
    if (IsADCOverlappingWithObstacle(adc_path_points[i], obstacle_instance,
                                     adc_l_buffer)) {
      // Note: overlapping_s的first和second都是adc_path_points的s值
      // Note: 这些ST boundary的s值指的都是adc path的s值
      // Note: 这里尝试把下标往前移一位，试图获得真的没有overlap的区间起点
      overlapping_s->first = adc_path_points[std::max(i - 1, 0)].s();
      has_overlapping = true;
      ADEBUG << "There is overlapping.";
      break;
    }
  }
  if (!has_overlapping) {
    return false;
  }
  // Note: 找到最后一个overlap的点，然后把后一个点作为overlapping_s->second
  for (int i = pt_after_idx; i >= pt_before_idx; --i) {
    ADEBUG << "At ADC path index = " << i << " :";
    if (IsADCOverlappingWithObstacle(adc_path_points[i], obstacle_instance,
                                     adc_l_buffer)) {
      // Note: 这里把下标往后移一位，试图获得真的没有overlap的区间终点
      // 但是，上面的pt_before_idx如果刚好是倒数第二个点的话，由于前面从未考虑过最后一个点是否overlap，
      // 有可能这个overlapping_s->second代表的位置与障碍物时overlap的
      // 在这里，也算是把最后一个点考虑上了吧
      overlapping_s->second = adc_path_points[i + 1].s();
      ADEBUG << "There is overlapping.";
      break;
    }
  }
  return true;
}

// Note: 二分法查找s方向上距离obstacle_instance超过s_thresh的临界adc_path_point点，临界点的下标
// Note: 如果没找到符合要求的临界点，就返回-1
// Note: is_before表示要求path point在障碍物屁股后面
// Note: 如果传入的is_before为真，
//         则把adc_path_points中在obstacle_instance屁股后方至少s_thresh距离的最后一个点的下标返回
//       如果传入的is_before为假，
//         则把adc_path_points中在obstacle_instance前方至少s_thresh距离的第一个点的下标返回
// Note: 当path是是圆弧形状时(大拐弯)，可能会出现比较奇怪的结果
int STObstaclesProcessor::GetSBoundingPathPointIndex(
    const std::vector<PathPoint>& adc_path_points,
    const Box2d& obstacle_instance, const double s_thresh, const bool is_before,
    const int start_idx, const int end_idx) {
  if (start_idx == end_idx) {
    if (IsPathPointAwayFromObstacle(adc_path_points[start_idx],
                                    adc_path_points[start_idx + 1],
                                    obstacle_instance, s_thresh, is_before)) {
      return start_idx;
    } else {
      return -1;
    }
  }

  if (is_before) {
    int mid_idx = (start_idx + end_idx - 1) / 2 + 1;
    if (IsPathPointAwayFromObstacle(adc_path_points[mid_idx],
                                    adc_path_points[mid_idx + 1],
                                    obstacle_instance, s_thresh, is_before)) {
      return GetSBoundingPathPointIndex(adc_path_points, obstacle_instance,
                                        s_thresh, is_before, mid_idx, end_idx);
    } else {
      return GetSBoundingPathPointIndex(adc_path_points, obstacle_instance,
                                        s_thresh, is_before, start_idx,
                                        mid_idx - 1);
    }
  } else {
    int mid_idx = (start_idx + end_idx) / 2;
    if (IsPathPointAwayFromObstacle(adc_path_points[mid_idx],
                                    adc_path_points[mid_idx + 1],
                                    obstacle_instance, s_thresh, is_before)) {
      return GetSBoundingPathPointIndex(adc_path_points, obstacle_instance,
                                        s_thresh, is_before, start_idx,
                                        mid_idx);
    } else {
      return GetSBoundingPathPointIndex(adc_path_points, obstacle_instance,
                                        s_thresh, is_before, mid_idx + 1,
                                        end_idx);
    }
  }
}

// Note: 检查障碍物是否在由direction_point给定的朝向上远离PathPoint
// Note: 检查障碍物在纵向方向上是否远离path_point(至少s_thresh距离)
// Note: 这个不考虑横向距离，只是把障碍物Polygon的各个corner point投映到轨迹段的截面上
// 然后检查所有corner point到截面的距离，距离超过s_thresh的话认为是Far Away的
// Note: 具体来说就是，
//       如果is_before为真，则判断障碍物是否满足在path_point前方，并且距离不小于s_thresh
//       如果is_before为假，则判断障碍物是否满足在path_point后方，并且距离不小于s_thresh
bool STObstaclesProcessor::IsPathPointAwayFromObstacle(
    const PathPoint& path_point, const PathPoint& direction_point,
    const Box2d& obs_box, const double s_thresh, const bool is_before) {
  // Note: 待考察的点
  Vec2d path_pt(path_point.x(), path_point.y());
  // Note: 方向点，自车Path中考察点的下一个点，用于指示方向
  Vec2d dir_pt(direction_point.x(), direction_point.y());
  // Note: 轨迹段
  LineSegment2d path_dir_lineseg(path_pt, dir_pt);
  // Note: 轨迹段的垂线
  LineSegment2d normal_line_seg(path_pt, path_dir_lineseg.rotate(M_PI_2));

  auto corner_points = obs_box.GetAllCorners();
  for (const auto& corner_pt : corner_points) {

/*  pc
    .  <--这是corner_pt
    |
    |  <--这是perpendicular_vec
    |
    |   pd
    |   .
    |   |  <--这是轨迹段，由path的两个相邻点构成
----.---.  <--这条横线是normal_line_seg，垂直于轨迹段；
    pf  pp

各个点表示，
  pc(corner point), 即corner_pt
  pf(foot point), 即normal_line_ft_pt
  pp(path point), 即path_pt
  pd(direction point), 即direction_point
*/
    // Note: 障碍物corner_point引垂线到轨迹段的垂线
    Vec2d normal_line_ft_pt;
    // Note: 将障碍物的corner point投映到垂线上，得到normal_line_ft_pt
    normal_line_seg.GetPerpendicularFoot(corner_pt, &normal_line_ft_pt);
    // Note: 轨迹段朝向
    Vec2d path_dir_unit_vec = path_dir_lineseg.unit_direction();
    // Note: 障碍物corner point到轨迹做的垂线段
    Vec2d perpendicular_vec = corner_pt - normal_line_ft_pt;
    // Note: corner point在path_point前方多远的距离
    // Note: 如果corner point在path_point后面，则这个corner_pt_s_dist是负值
    double corner_pt_s_dist = path_dir_unit_vec.InnerProd(perpendicular_vec);
    if (is_before && corner_pt_s_dist < s_thresh) {
      // Note: 障碍物在path_point前方s_thresh距离以内
      return false;
    }
    if (!is_before && corner_pt_s_dist > -s_thresh) {
      // Note: 障碍物在path_point后方s_thresh距离以内
      return false;
    }
  }
  return true;
}

bool STObstaclesProcessor::IsADCOverlappingWithObstacle(
    const PathPoint& adc_path_point, const Box2d& obs_box,
    const double l_buffer) const {
  // Convert reference point from center of rear axis to center of ADC.
  Vec2d ego_center_map_frame((vehicle_param_.front_edge_to_center() -
                              vehicle_param_.back_edge_to_center()) *
                                 0.5,
                             (vehicle_param_.left_edge_to_center() -
                              vehicle_param_.right_edge_to_center()) *
                                 0.5);
  ego_center_map_frame.SelfRotate(adc_path_point.theta());
  ego_center_map_frame.set_x(ego_center_map_frame.x() + adc_path_point.x());
  ego_center_map_frame.set_y(ego_center_map_frame.y() + adc_path_point.y());

  // Compute the ADC bounding box.
  Box2d adc_box(ego_center_map_frame, adc_path_point.theta(),
                vehicle_param_.length(), vehicle_param_.width() + l_buffer * 2);

  ADEBUG << "    ADC box is: " << adc_box.DebugString();
  ADEBUG << "    Obs box is: " << obs_box.DebugString();

  // Check whether ADC bounding box overlaps with obstacle bounding box.
  return obs_box.HasOverlap(adc_box);
}

// Note: 通过扫面线算法找t时刻垂直方向上的Gaps
std::vector<std::pair<double, double>> STObstaclesProcessor::FindSGaps(
    const std::vector<ObsTEdge>& obstacle_t_edges, double s_min, double s_max) {
  std::vector<std::pair<double, int>> obs_s_edges;
  // Note: ObsTEdge contains: (is_starting_t, t, s_min, s_max, obs_id)
  for (auto obs_t_edge : obstacle_t_edges) {
    // Note: 入边
    obs_s_edges.emplace_back(std::get<2>(obs_t_edge), 1);
    // Note: 出边
    obs_s_edges.emplace_back(std::get<3>(obs_t_edge), 0);
  }
  // obs_s_edges.emplace_back(std::numeric_limits<double>::lowest(), 1);
  // Note: 出边
  obs_s_edges.emplace_back(s_min, 0);
  // Note: 入边
  obs_s_edges.emplace_back(s_max, 1);
  // obs_s_edges.emplace_back(std::numeric_limits<double>::max(), 0);
  std::sort(
      obs_s_edges.begin(), obs_s_edges.end(),
      [](const std::pair<double, int>& lhs, const std::pair<double, int>& rhs) {
        if (lhs.first != rhs.first) {
          // Note: 按照s位置从小到大排列
          return lhs.first < rhs.first;
        } else {
          // Note: 如果s位置相同，则入边在前
          return lhs.second > rhs.second;
        }
      });

  // Note: 扫描线算法获取垂直方向的gap
  std::vector<std::pair<double, double>> s_gaps;
  int num_st_obs = 1;
  double prev_open_s = 0.0;
  for (auto obs_s_edge : obs_s_edges) {
    if (obs_s_edge.second == 1) {
      num_st_obs++;
      if (num_st_obs == 1) {
        s_gaps.emplace_back(prev_open_s, obs_s_edge.first);
      }
    } else {
      num_st_obs--;
      if (num_st_obs == 0) {
        prev_open_s = obs_s_edge.first;
      }
    }
  }

  return s_gaps;
}

ObjectDecisionType STObstaclesProcessor::DetermineObstacleDecision(
    const double obs_s_min, const double obs_s_max, const double s) const {
  ObjectDecisionType decision;
  if (s <= obs_s_min) {
    decision.mutable_yield()->set_distance_s(0.0);
  } else if (s >= obs_s_max) {
    decision.mutable_overtake()->set_distance_s(0.0);
  }
  return decision;
}

bool STObstaclesProcessor::IsSWithinADCLowRoadRightSegment(
    const double s) const {
  for (const auto& seg : adc_low_road_right_segments_) {
    if (s >= seg.first && s <= seg.second) {
      return true;
    }
  }
  return false;
}

}  // namespace planning
}  // namespace apollo
