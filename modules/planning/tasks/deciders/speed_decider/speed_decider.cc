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

#include "modules/planning/tasks/deciders/speed_decider/speed_decider.h"

#include <algorithm>

#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/planning/proto/decision.pb.h"

#include "cyber/common/log.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/time/time.h"
#include "modules/common/util/util.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/tasks/utils/st_gap_estimator.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::VehicleConfigHelper;
using apollo::common::math::Vec2d;
using apollo::common::time::Clock;
using apollo::perception::PerceptionObstacle;

std::unordered_map<std::string, double> SpeedDecider::pedestrian_stop_timer_;

SpeedDecider::SpeedDecider(const TaskConfig& config) : Task(config) {}

common::Status SpeedDecider::Execute(Frame* frame,
                                     ReferenceLineInfo* reference_line_info) {
  Task::Execute(frame, reference_line_info);
  init_point_ = frame_->PlanningStartPoint();
  adc_sl_boundary_ = reference_line_info_->AdcSlBoundary();
  reference_line_ = &reference_line_info_->reference_line();
  if (!MakeObjectDecision(reference_line_info->speed_data(),
                          reference_line_info->path_decision())
           .ok()) {
    const std::string msg = "Get object decision by speed profile failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  return Status::OK();
}

// Note: 判断STPoint从障碍物STBoundary的什么位置经过
// ABOVE表示speed_profile从障碍物STBoundary的上方通过
// BELOW表示speed_profile从障碍物STBoundary的下方通过
// CROSS表示speed_profile与障碍物STBoundary有overlap
SpeedDecider::STLocation SpeedDecider::GetSTLocation(
    const PathDecision* const path_decision, const SpeedData& speed_profile,
    const STBoundary& st_boundary) const {
  if (st_boundary.IsEmpty()) {
    return BELOW;
  }

  STLocation st_location = BELOW;
  bool st_position_set = false;
  const double start_t = st_boundary.min_t();
  const double end_t = st_boundary.max_t();
  for (size_t i = 0; i + 1 < speed_profile.size(); ++i) {
    const STPoint curr_st(speed_profile[i].s(), speed_profile[i].t());
    const STPoint next_st(speed_profile[i + 1].s(), speed_profile[i + 1].t());
    // Remind(huachang): 判断curr_st.t() < start_t是多此一举
    // 因为后面的next_st.t() < start_t保证了curr_st.t() < start_t
    if (curr_st.t() < start_t && next_st.t() < start_t) {
      continue;
    }
    if (curr_st.t() > end_t) {
      break;
    }

    if (!FLAGS_use_st_drivable_boundary) {
      common::math::LineSegment2d speed_line(curr_st, next_st);
      if (st_boundary.HasOverlap(speed_line)) {
        ADEBUG << "speed profile cross st_boundaries.";
        st_location = CROSS;

        if (!FLAGS_use_st_drivable_boundary) {
          if (st_boundary.boundary_type() ==
              STBoundary::BoundaryType::KEEP_CLEAR) {
            if (!CheckKeepClearCrossable(path_decision, speed_profile,
                                         st_boundary)) {
              st_location = BELOW;
            }
          }
        }
        break;
      }
    }

    // note: st_position can be calculated by checking two st points once
    //       but we need iterate all st points to make sure there is no CROSS
    if (!st_position_set) {
      if (start_t < next_st.t() && curr_st.t() < end_t) {
        // Note: 障碍物st_boundary的左上方点
        STPoint bd_point_front = st_boundary.upper_points().front();
        // Note: 通过向量叉乘来判断两个线段的相对位置关系
        // 记bd_point_front, curr_st, next_st分别为A, B, C点
        // 通过判断向量AB与向量AC的叉乘来判断线段AC在AB的什么方向上
        // 如果AB×AC为负，则AC在AB的顺时针方向，否则在逆时针方向
        double side = common::math::CrossProd(bd_point_front, curr_st, next_st);
        st_location = side < 0.0 ? ABOVE : BELOW;
        st_position_set = true;
        // Remind(huachang): 为什么不break退出循环?
      }
    }
  }
  return st_location;
}

bool SpeedDecider::CheckKeepClearCrossable(
    const PathDecision* const path_decision, const SpeedData& speed_profile,
    const STBoundary& keep_clear_st_boundary) const {
  bool keep_clear_crossable = true;

  const auto& last_speed_point = speed_profile.back();
  double last_speed_point_v = 0.0;
  if (last_speed_point.has_v()) {
    last_speed_point_v = last_speed_point.v();
  } else {
    const size_t len = speed_profile.size();
    if (len > 1) {
      const auto& last_2nd_speed_point = speed_profile[len - 2];
      last_speed_point_v = (last_speed_point.s() - last_2nd_speed_point.s()) /
                           (last_speed_point.t() - last_2nd_speed_point.t());
    }
  }
  static constexpr double kKeepClearSlowSpeed = 2.5;  // m/s
  ADEBUG << "last_speed_point_s[" << last_speed_point.s()
         << "] st_boundary.max_s[" << keep_clear_st_boundary.max_s()
         << "] last_speed_point_v[" << last_speed_point_v << "]";
  if (last_speed_point.s() <= keep_clear_st_boundary.max_s() &&
      last_speed_point_v < kKeepClearSlowSpeed) {
    keep_clear_crossable = false;
  }
  return keep_clear_crossable;
}

// Note: 如果有个障碍物在禁停区的正前方近处，我们不能进入这个禁停区
bool SpeedDecider::CheckKeepClearBlocked(
    const PathDecision* const path_decision,
    const Obstacle& keep_clear_obstacle) const {
  bool keep_clear_blocked = false;

  // check if overlap with other stop wall
  for (const auto* obstacle : path_decision->obstacles().Items()) {
    if (obstacle->Id() == keep_clear_obstacle.Id()) {
      continue;
    }
    const double obstacle_start_s = obstacle->PerceptionSLBoundary().start_s();
    const double adc_length =
        VehicleConfigHelper::GetConfig().vehicle_param().length();
    const double distance =
        obstacle_start_s - keep_clear_obstacle.PerceptionSLBoundary().end_s();

    // Note: 有个障碍物在禁停区的正前方近处，我们不能进入这个禁停区
    if (obstacle->IsBlockingObstacle() && distance > 0 &&
        distance < (adc_length / 2)) {
      keep_clear_blocked = true;
      break;
    }
  }
  return keep_clear_blocked;
}

/**
 * @brief "too close" is determined by whether ego vehicle will hit the front
 * obstacle if the obstacle drive at current speed and ego vehicle use some
 * reasonable deceleration
 **/
// Note: 跟车太近的逻辑基于一个假设，
// 如果自车速度大于前方障碍物速度，自车通过最大减速度减速到前方障碍物同样的速度，这时候自车会不会距离前方障碍物小于预设距离
bool SpeedDecider::IsFollowTooClose(const Obstacle& obstacle) const {
  // Note: 只有禁停区(Keep Clear)不是BlockingObstacle
  if (!obstacle.IsBlockingObstacle()) {
    return false;
  }

  // Note: 障碍物目前不在自车path上
  if (obstacle.path_st_boundary().min_t() > 0.0) {
    return false;
  }
  const double obs_speed = obstacle.speed();
  const double ego_speed = init_point_.v();
  if (obs_speed > ego_speed) {
    return false;
  }
  const double distance =
      obstacle.path_st_boundary().min_s() - FLAGS_min_stop_distance_obstacle;
  static constexpr double lane_follow_max_decel = 3.0;
  static constexpr double lane_change_max_decel = 3.0;
  auto* planning_status = PlanningContext::Instance()
                              ->mutable_planning_status()
                              ->mutable_change_lane();
  double distance_numerator = std::pow((ego_speed - obs_speed), 2) * 0.5;
  double distance_denominator = lane_follow_max_decel;
  if (planning_status->has_status() &&
      planning_status->status() == ChangeLaneStatus::IN_CHANGE_LANE) {
    distance_denominator = lane_change_max_decel;
  }
  return distance < distance_numerator / distance_denominator;
}

Status SpeedDecider::MakeObjectDecision(
    const SpeedData& speed_profile, PathDecision* const path_decision) const {
  if (speed_profile.size() < 2) {
    const std::string msg = "dp_st_graph failed to get speed profile.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  for (const auto* obstacle : path_decision->obstacles().Items()) {
    auto* mutable_obstacle = path_decision->Find(obstacle->Id());
    const auto& boundary = mutable_obstacle->path_st_boundary();

    // Note: ignore不在STGraph中的障碍物，这些障碍物与path没有overlap
    // Note: 这一步在之前的task里面是不是已经做过?
    if (boundary.IsEmpty() || boundary.max_s() < 0.0 ||
        boundary.max_t() < 0.0 ||
        boundary.min_t() >= speed_profile.back().t()) {
      AppendIgnoreDecision(mutable_obstacle);
      continue;
    }
    // Note: 实质上相当于如果横向没有decision的话，就添加横向的ignore decision
    if (obstacle->HasLongitudinalDecision()) {
      AppendIgnoreDecision(mutable_obstacle);
      continue;
    }

    // for Virtual obstacle, skip if center point NOT "on lane"
    if (obstacle->IsVirtual()) {
      const auto& obstacle_box = obstacle->PerceptionBoundingBox();
      if (!reference_line_->IsOnLane(obstacle_box.center())) {
        continue;
      }
    }

    // always STOP for pedestrian
    // Remind(huachang): 下面对行人的处理导致近处的行人一下子把车逼停，体感非常不好
    // 这主要是由于参数kSDistanceStartTimer以及停车距离的设置的设置导致停车点就在车前方两三米，来不及刹车导致fallback
    // TODO(huachang): 这里stop fence的位置需要更灵活一点，应该能大幅改善出现行人时的体感
    if (CheckStopForPedestrian(*mutable_obstacle)) {
      ObjectDecisionType stop_decision;
      // Note: 行人的stop是所有stop中最近的，则添加stop_decision
      if (CreateStopDecision(*mutable_obstacle, &stop_decision,
                             -FLAGS_min_stop_distance_obstacle)) {
        mutable_obstacle->AddLongitudinalDecision("dp_st_graph/pedestrian",
                                                  stop_decision);
      }
      continue;
    }

    // Note: 判断STPoint从障碍物STBoundary的什么位置经过
    auto location = GetSTLocation(path_decision, speed_profile, boundary);

    if (!FLAGS_use_st_drivable_boundary) {
      if (boundary.boundary_type() == STBoundary::BoundaryType::KEEP_CLEAR) {
        // Note: 如果有个障碍物在禁停区的正前方近处，我们不能进入这个禁停区
        if (CheckKeepClearBlocked(path_decision, *obstacle)) {
          location = BELOW;
        }
      }
    }

    switch (location) {
      case BELOW:
        if (boundary.boundary_type() == STBoundary::BoundaryType::KEEP_CLEAR) {
          ObjectDecisionType stop_decision;
          if (CreateStopDecision(*mutable_obstacle, &stop_decision, 0.0)) {
            mutable_obstacle->AddLongitudinalDecision("dp_st_graph/keep_clear",
                                                      stop_decision);
          }
        // Note: 判断是否是应该Follow的障碍物
        } else if (CheckIsFollow(*obstacle, boundary)) {
          // stop for low_speed decelerating
          if (IsFollowTooClose(*mutable_obstacle)) {
            ObjectDecisionType stop_decision;
            if (CreateStopDecision(*mutable_obstacle, &stop_decision,
                                   -FLAGS_min_stop_distance_obstacle)) {
              mutable_obstacle->AddLongitudinalDecision("dp_st_graph/too_close",
                                                        stop_decision);
            }
          } else {  // high speed or low speed accelerating
            // FOLLOW decision
            ObjectDecisionType follow_decision;
            if (CreateFollowDecision(*mutable_obstacle, &follow_decision)) {
              mutable_obstacle->AddLongitudinalDecision("dp_st_graph",
                                                        follow_decision);
            }
          }
        } else {
          // YIELD decision
          ObjectDecisionType yield_decision;
          if (CreateYieldDecision(*mutable_obstacle, &yield_decision)) {
            mutable_obstacle->AddLongitudinalDecision("dp_st_graph",
                                                      yield_decision);
          }
        }
        break;
      case ABOVE:
        if (boundary.boundary_type() == STBoundary::BoundaryType::KEEP_CLEAR) {
          ObjectDecisionType ignore;
          ignore.mutable_ignore();
          mutable_obstacle->AddLongitudinalDecision("dp_st_graph", ignore);
        } else {
          // OVERTAKE decision
          ObjectDecisionType overtake_decision;
          if (CreateOvertakeDecision(*mutable_obstacle, &overtake_decision)) {
            mutable_obstacle->AddLongitudinalDecision("dp_st_graph/overtake",
                                                      overtake_decision);
          }
        }
        break;
      case CROSS:
        // Note: 针对普通障碍物，而不是禁停区
        if (mutable_obstacle->IsBlockingObstacle()) {
          ObjectDecisionType stop_decision;
          if (CreateStopDecision(*mutable_obstacle, &stop_decision,
                                 -FLAGS_min_stop_distance_obstacle)) {
            mutable_obstacle->AddLongitudinalDecision("dp_st_graph/cross",
                                                      stop_decision);
          }
          const std::string msg =
              "Failed to find a solution for crossing obstacle:" +
              mutable_obstacle->Id();
          AERROR << msg;
          return Status(ErrorCode::PLANNING_ERROR, msg);
        }
        break;
      default:
        AERROR << "Unknown position:" << location;
    }
    AppendIgnoreDecision(mutable_obstacle);
  }

  return Status::OK();
}

void SpeedDecider::AppendIgnoreDecision(Obstacle* obstacle) const {
  ObjectDecisionType ignore_decision;
  ignore_decision.mutable_ignore();
  if (!obstacle->HasLongitudinalDecision()) {
    obstacle->AddLongitudinalDecision("dp_st_graph", ignore_decision);
  }
  if (!obstacle->HasLateralDecision()) {
    obstacle->AddLateralDecision("dp_st_graph", ignore_decision);
  }
}

bool SpeedDecider::CreateStopDecision(const Obstacle& obstacle,
                                      ObjectDecisionType* const stop_decision,
                                      double stop_distance) const {
  const auto& boundary = obstacle.path_st_boundary();

  // TODO(all): this is a bug! Cannot mix reference s and path s!
  // Replace boundary.min_s() with computed reference line s
  // fence is set according to reference line s.
  // Remind(huachang): 这里混用各种s值了，逻辑不清晰，虽然最后的效果一般都没差
  double fence_s = adc_sl_boundary_.end_s() + boundary.min_s() + stop_distance;
  if (boundary.boundary_type() == STBoundary::BoundaryType::KEEP_CLEAR) {
    fence_s = obstacle.PerceptionSLBoundary().start_s();
  }
  const double main_stop_s =
      reference_line_info_->path_decision()->stop_reference_line_s();
  if (main_stop_s < fence_s) {
    ADEBUG << "Stop fence is further away, ignore.";
    return false;
  }

  const auto fence_point = reference_line_->GetReferencePoint(fence_s);

  // set STOP decision
  auto* stop = stop_decision->mutable_stop();
  stop->set_distance_s(stop_distance);
  auto* stop_point = stop->mutable_stop_point();
  stop_point->set_x(fence_point.x());
  stop_point->set_y(fence_point.y());
  stop_point->set_z(0.0);
  stop->set_stop_heading(fence_point.heading());

  if (boundary.boundary_type() == STBoundary::BoundaryType::KEEP_CLEAR) {
    stop->set_reason_code(StopReasonCode::STOP_REASON_CLEAR_ZONE);
  }

  PerceptionObstacle::Type obstacle_type = obstacle.Perception().type();
  ADEBUG << "STOP: obstacle_id[" << obstacle.Id() << "] obstacle_type["
         << PerceptionObstacle_Type_Name(obstacle_type) << "]";

  return true;
}

bool SpeedDecider::CreateFollowDecision(
    const Obstacle& obstacle, ObjectDecisionType* const follow_decision) const {
  const double follow_speed = init_point_.v();
  const double follow_distance_s =
      -StGapEstimator::EstimateProperFollowingGap(follow_speed);

  const auto& boundary = obstacle.path_st_boundary();
  // Remind(huachang): boundary.min_s()混用在这里十分不合理，这个处理是很不精确的，不可控
  const double reference_s =
      adc_sl_boundary_.end_s() + boundary.min_s() + follow_distance_s;
  const double main_stop_s =
      reference_line_info_->path_decision()->stop_reference_line_s();
  if (main_stop_s < reference_s) {
    ADEBUG << "Follow reference_s is further away, ignore.";
    return false;
  }

  auto ref_point = reference_line_->GetReferencePoint(reference_s);

  // set FOLLOW decision
  auto* follow = follow_decision->mutable_follow();
  follow->set_distance_s(follow_distance_s);
  auto* fence_point = follow->mutable_fence_point();
  fence_point->set_x(ref_point.x());
  fence_point->set_y(ref_point.y());
  fence_point->set_z(0.0);
  follow->set_fence_heading(ref_point.heading());

  PerceptionObstacle::Type obstacle_type = obstacle.Perception().type();
  ADEBUG << "FOLLOW: obstacle_id[" << obstacle.Id() << "] obstacle_type["
         << PerceptionObstacle_Type_Name(obstacle_type) << "]";

  return true;
}

bool SpeedDecider::CreateYieldDecision(
    const Obstacle& obstacle, ObjectDecisionType* const yield_decision) const {
  PerceptionObstacle::Type obstacle_type = obstacle.Perception().type();
  double yield_distance = StGapEstimator::EstimateProperYieldingGap();

  const auto& obstacle_boundary = obstacle.path_st_boundary();
  const double yield_distance_s =
      std::max(-obstacle_boundary.min_s(), -yield_distance);

  // Remind(huachang): speed decider到处混用
  const double reference_line_fence_s =
      adc_sl_boundary_.end_s() + obstacle_boundary.min_s() + yield_distance_s;
  const double main_stop_s =
      reference_line_info_->path_decision()->stop_reference_line_s();
  if (main_stop_s < reference_line_fence_s) {
    ADEBUG << "Yield reference_s is further away, ignore.";
    return false;
  }

  auto ref_point = reference_line_->GetReferencePoint(reference_line_fence_s);

  // set YIELD decision
  auto* yield = yield_decision->mutable_yield();
  yield->set_distance_s(yield_distance_s);
  yield->mutable_fence_point()->set_x(ref_point.x());
  yield->mutable_fence_point()->set_y(ref_point.y());
  yield->mutable_fence_point()->set_z(0.0);
  yield->set_fence_heading(ref_point.heading());

  ADEBUG << "YIELD: obstacle_id[" << obstacle.Id() << "] obstacle_type["
         << PerceptionObstacle_Type_Name(obstacle_type) << "]";

  return true;
}

bool SpeedDecider::CreateOvertakeDecision(
    const Obstacle& obstacle,
    ObjectDecisionType* const overtake_decision) const {
  const auto& velocity = obstacle.Perception().velocity();
  // Note: 障碍物在自车朝向上的速度分量
  const double obstacle_speed =
      common::math::Vec2d::CreateUnitVec2d(init_point_.path_point().theta())
          .InnerProd(Vec2d(velocity.x(), velocity.y()));

  // Remind(huachang): 这里overtake gap设置之后有什么用途
  const double overtake_distance_s =
      StGapEstimator::EstimateProperOvertakingGap(obstacle_speed,
                                                  init_point_.v());

  const auto& boundary = obstacle.path_st_boundary();
  const double reference_line_fence_s =
      adc_sl_boundary_.end_s() + boundary.min_s() + overtake_distance_s;
  const double main_stop_s =
      reference_line_info_->path_decision()->stop_reference_line_s();
  if (main_stop_s < reference_line_fence_s) {
    ADEBUG << "Overtake reference_s is further away, ignore.";
    return false;
  }

  auto ref_point = reference_line_->GetReferencePoint(reference_line_fence_s);

  // set OVERTAKE decision
  auto* overtake = overtake_decision->mutable_overtake();
  overtake->set_distance_s(overtake_distance_s);
  overtake->mutable_fence_point()->set_x(ref_point.x());
  overtake->mutable_fence_point()->set_y(ref_point.y());
  overtake->mutable_fence_point()->set_z(0.0);
  overtake->set_fence_heading(ref_point.heading());

  PerceptionObstacle::Type obstacle_type = obstacle.Perception().type();
  ADEBUG << "OVERTAKE: obstacle_id[" << obstacle.Id() << "] obstacle_type["
         << PerceptionObstacle_Type_Name(obstacle_type) << "]";

  return true;
}

// Note: 通过障碍物STBoundary检查是否应该follow该障碍物，下面这些障碍物都不follow
// 1. 不在本车道上的，
// 2. 还没进入本车道的，
// 3. 马上要离开本车道的，
// 4. 逆向行驶的障碍物，
// 5. 与adc path交叉时间不足2秒的障碍物
bool SpeedDecider::CheckIsFollow(const Obstacle& obstacle,
                                 const STBoundary& boundary) const {
  const double obstacle_l_distance =
      std::min(std::fabs(obstacle.PerceptionSLBoundary().start_l()),
               std::fabs(obstacle.PerceptionSLBoundary().end_l()));
  // Note: 认为障碍物不在本车道上(横向偏离参考线较远)
  if (obstacle_l_distance > FLAGS_follow_min_obs_lateral_distance) {
    return false;
  }

  // move towards adc
  // Note: 这个障碍物是逆行行驶的(相对于adc)
  if (boundary.bottom_left_point().s() > boundary.bottom_right_point().s()) {
    return false;
  }

  // Note: 对于还没进入本车道以及马上要离开本车道(adc的path)的障碍物不做跟车处理
  static constexpr double kFollowTimeEpsilon = 1e-3;
  static constexpr double kFollowCutOffTime = 0.5;
  if (boundary.min_t() > kFollowCutOffTime ||
      boundary.max_t() < kFollowTimeEpsilon) {
    return false;
  }

  // cross lane but be moving to different direction
  // Note: 障碍物预测轨迹与自车path交叉的时间少于2秒，认为这个障碍物只是横穿该车道(例如十字路口处)
  if (boundary.max_t() - boundary.min_t() < FLAGS_follow_min_time_sec) {
    return false;
  }

  return true;
}

// Note: 对十米外的行人obstacle一律返回true，对于十米内的行人，执行以下处理
// Note: 走得比较快的行人obstacle一律停车，对走得很慢的行人，等4秒之后就不等了
bool SpeedDecider::CheckStopForPedestrian(const Obstacle& obstacle) const {
  const auto& perception_obstacle = obstacle.Perception();
  if (perception_obstacle.type() != PerceptionObstacle::PEDESTRIAN) {
    return false;
  }

  // Note: 后方行人
  const auto& obstacle_sl_boundary = obstacle.PerceptionSLBoundary();
  if (obstacle_sl_boundary.end_s() < adc_sl_boundary_.start_s()) {
    return false;
  }

  const std::string& obstacle_id = obstacle.Id();

  // update stop timestamp on static pedestrian for watch timer
  // check on stop timer for static pedestrians
  static constexpr double kSDistanceStartTimer = 10.0;
  static constexpr double kMaxStopSpeed = 0.3;
  static constexpr double kPedestrianStopTimeout = 4.0;
  // Note: 自车在10米内会与行人(预测轨迹轨迹)相交
  if (obstacle.path_st_boundary().min_s() < kSDistanceStartTimer) {
    const auto obstacle_speed = std::hypot(perception_obstacle.velocity().x(),
                                           perception_obstacle.velocity().y());
    // Note: 走得比较快的与轨迹overlap的近处(10m)行人一律停车
    if (obstacle_speed > kMaxStopSpeed) {
      pedestrian_stop_timer_.erase(obstacle_id);
    } else {
      if (pedestrian_stop_timer_.find(obstacle_id) ==
          pedestrian_stop_timer_.end()) {
        // add timestamp
        pedestrian_stop_timer_[obstacle_id] = Clock::NowInSeconds();
        ADEBUG << "add timestamp: obstacle_id[" << obstacle_id << "] timestamp["
               << Clock::NowInSeconds() << "]";
      } else {
        // check timeout
        double stop_timer =
            Clock::NowInSeconds() - pedestrian_stop_timer_[obstacle_id];
        ADEBUG << "stop_timer: obstacle_id[" << obstacle_id << "] stop_timer["
               << stop_timer << "]";
        if (stop_timer >= kPedestrianStopTimeout) {
          return false;
        }
      }
    }
  }

  return true;
}

}  // namespace planning
}  // namespace apollo
