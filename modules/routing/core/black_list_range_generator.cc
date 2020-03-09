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

#include "modules/routing/core/black_list_range_generator.h"

namespace apollo {
namespace routing {

constexpr double S_GAP_FOR_BLACK = 0.01;

namespace {

double MoveSForward(double s, double upper_bound) {
  if (s > upper_bound) {
    AERROR << "Illegal s: " << s << ", upper bound: " << upper_bound;
    return s;
  }
  if (s + S_GAP_FOR_BLACK < upper_bound) {
    return (s + S_GAP_FOR_BLACK);
  } else {
    return ((s + upper_bound) / 2.0);
  }
}

// Note: 试图将s缩小1cm
double MoveSBackward(double s, double lower_bound) {
  if (s < lower_bound) {
    AERROR << "Illegal s: " << s << ", lower bound: " << lower_bound;
    return s;
  }
  if (s - S_GAP_FOR_BLACK > lower_bound) {
    return (s - S_GAP_FOR_BLACK);
  } else {
    return ((s + lower_bound) / 2.0);
  }
}

// Note: 将能从当前车道(递归)进入的平行车道的node添加到node_set
void GetOutParallelLane(const TopoNode* node,
                        std::unordered_set<const TopoNode*>* const node_set) {
  for (const auto* edge : node->OutToLeftOrRightEdge()) {
    const auto* to_node = edge->ToNode();
    if (node_set->count(to_node) == 0) {
      node_set->emplace(to_node);
      GetOutParallelLane(to_node, node_set);
    }
  }
}

void GetInParallelLane(const TopoNode* node,
                       std::unordered_set<const TopoNode*>* const node_set) {
  for (const auto* edge : node->InFromLeftOrRightEdge()) {
    const auto* from_node = edge->FromNode();
    if (node_set->count(from_node) == 0) {
      node_set->emplace(from_node);
      GetInParallelLane(from_node, node_set);
    }
  }
}

// for new navigator
// Note: 添加road黑名单, routing结果不能包含这些road中的lane
void AddBlackMapFromRoad(const RoutingRequest& request, const TopoGraph* graph,
                         TopoRangeManager* const range_manager) {
  for (const auto& road_id : request.blacklisted_road()) {
    std::unordered_set<const TopoNode*> road_nodes_set;
    graph->GetNodesByRoadId(road_id, &road_nodes_set);
    for (const auto& node : road_nodes_set) {
      range_manager->Add(node, 0.0, node->Length());
    }
  }
}

// for new navigator
// Note: 添加LaneSegment黑名单, routing结果不能包含这些LaneSegment
void AddBlackMapFromLane(const RoutingRequest& request, const TopoGraph* graph,
                         TopoRangeManager* const range_manager) {
  // Note: request.blacklisted_lane()的类型是LaneSegment
  for (const auto& lane : request.blacklisted_lane()) {
    const auto* node = graph->GetNode(lane.id());
    if (node) {
      range_manager->Add(node, lane.start_s(), lane.end_s());
    }
  }
}

void AddBlackMapFromOutParallel(const TopoNode* node, double cut_ratio,
                                TopoRangeManager* const range_manager) {
  std::unordered_set<const TopoNode*> par_node_set;
  GetOutParallelLane(node, &par_node_set);
  par_node_set.erase(node);
  for (const auto* par_node : par_node_set) {
    double par_cut_s = cut_ratio * par_node->Length();
    range_manager->Add(par_node, par_cut_s, par_cut_s);
  }
}

void AddBlackMapFromInParallel(const TopoNode* node, double cut_ratio,
                               TopoRangeManager* const range_manager) {
  std::unordered_set<const TopoNode*> par_node_set;
  GetInParallelLane(node, &par_node_set);
  par_node_set.erase(node);
  for (const auto* par_node : par_node_set) {
    double par_cut_s = cut_ratio * par_node->Length();
    range_manager->Add(par_node, par_cut_s, par_cut_s);
  }
}

}  // namespace

// Note: 黑名单里面的Lane和Road对应的s range不能通过
void BlackListRangeGenerator::GenerateBlackMapFromRequest(
    const RoutingRequest& request, const TopoGraph* graph,
    TopoRangeManager* const range_manager) const {
  AddBlackMapFromLane(request, graph, range_manager);
  AddBlackMapFromRoad(request, graph, range_manager);
  // Note: 对range_map_里面的TopoNode的NodeSRange进行排序, 然后进行区间合并
  range_manager->SortAndMerge();
}

// Note: 在start_s的后面(start_s - 1cm)和end_s的前面(end_s + 1cm)设置黑名单
// 注意, 可能进入的平行车道中的对应位置也会设置黑名单
// 这里添加的黑名单NodeSRange的长度都是0
// 添加了0长度的黑名单后会对黑名单区间进行拼接和合并
void BlackListRangeGenerator::AddBlackMapFromTerminal(
    const TopoNode* src_node, const TopoNode* dest_node, double start_s,
    double end_s, TopoRangeManager* const range_manager) const {
  double start_length = src_node->Length();
  double end_length = dest_node->Length();
  if (start_s < 0.0 || start_s > start_length) {
    AERROR << "Illegal start_s: " << start_s << ", length: " << start_length;
    return;
  }
  if (end_s < 0.0 || end_s > end_length) {
    AERROR << "Illegal end_s: " << end_s << ", length: " << end_length;
    return;
  }

  // Note: 对能从当前车道递归进入的所有平行车道设置黑名单,
  // 黑名单位于start_s后方1cm处(start_s - 1cm)
  // 注意, 这里的黑名单路段长度是0
  // 这样做是为了将起点处的TopoNode切割成两部分
  double start_cut_s = MoveSBackward(start_s, 0.0);
  range_manager->Add(src_node, start_cut_s, start_cut_s);
  AddBlackMapFromOutParallel(src_node, start_cut_s / start_length,
                             range_manager);

  // Note: 对能进入当前车道的所有平行车道设置黑名单
  // 黑名单位于end_s前方1cm处(end_s + 1cm)
  // 这样做是为了将终点处的TopoNode切割成两部分
  double end_cut_s = MoveSForward(end_s, end_length);
  range_manager->Add(dest_node, end_cut_s, end_cut_s);
  AddBlackMapFromInParallel(dest_node, end_cut_s / end_length, range_manager);
  range_manager->SortAndMerge();
}

}  // namespace routing
}  // namespace apollo
