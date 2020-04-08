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

#include <algorithm>
#include <limits>
#include <queue>

#include "modules/routing/common/routing_gflags.h"
#include "modules/routing/graph/sub_topo_graph.h"
#include "modules/routing/graph/topo_graph.h"
#include "modules/routing/strategy/a_star_strategy.h"

namespace apollo {
namespace routing {
namespace {

struct SearchNode {
  const TopoNode* topo_node = nullptr;
  // Note: f(n) = g(n) + h(n)
  // g(n): the exact cost of the path from the starting point to any vertex n
  // h(n): the heuristic estimated cost from vertex n to the goal.
  double f = std::numeric_limits<double>::max();

  SearchNode() = default;
  explicit SearchNode(const TopoNode* node)
      : topo_node(node), f(std::numeric_limits<double>::max()) {}
  SearchNode(const SearchNode& search_node) = default;

  bool operator<(const SearchNode& node) const {
    // in order to let the top of priority queue is the smallest one!
    return f > node.f;
  }

  bool operator==(const SearchNode& node) const {
    return topo_node == node.topo_node;
  }
};

// Note: 到达下一节点的代价(包含连接边的代价和下一节点的代价)
double GetCostToNeighbor(const TopoEdge* edge) {
  return (edge->Cost() + edge->ToNode()->Cost());
}

// Note: 从多个节点中选出长度最长的节点
const TopoNode* GetLargestNode(const std::vector<const TopoNode*>& nodes) {
  double max_range = 0.0;
  const TopoNode* largest = nullptr;
  for (const auto* node : nodes) {
    const double temp_range = node->EndS() - node->StartS();
    if (temp_range > max_range) {
      max_range = temp_range;
      largest = node;
    }
  }
  return largest;
}

// Note: 从后往前检查，尝试将变道的地点往终点方向移动到更合适的位置(更长的Lane)
bool AdjustLaneChangeBackward(
    std::vector<const TopoNode*>* const result_node_vec) {
  for (int i = static_cast<int>(result_node_vec->size()) - 2; i > 0; --i) {
    // Note: 行驶路径是base_node->from_node->to_node
    const auto* from_node = result_node_vec->at(i);
    const auto* to_node = result_node_vec->at(i + 1);
    const auto* base_node = result_node_vec->at(i - 1);
    // Note: 从from_node到to_node的edge
    const auto* from_to_edge = from_node->GetOutEdgeTo(to_node);
    if (from_to_edge == nullptr) {
      // may need to recalculate edge,
      // because only edge from origin node to subnode is saved
      from_to_edge = to_node->GetInEdgeFrom(from_node);
    }
    if (from_to_edge == nullptr) {
      AERROR << "Get null ptr to edge:" << from_node->LaneId() << " ("
             << from_node->StartS() << ", " << from_node->EndS() << ")"
             << " --> " << to_node->LaneId() << " (" << to_node->StartS()
             << ", " << to_node->EndS() << ")";
      return false;
    }
    // Note: 只处理变道类型的边(from_node到to_node)
    // Note: 尝试从base_node处开始变道
    // Note: 检查base_node--(变道)->candidate_node--(直行)->to_node是不是
    // 比base_node--(直行)->from_node--(变道)->to_node更适合变道
    if (from_to_edge->Type() != TopoEdgeType::TET_FORWARD) {
      // Note: 如果base_node长度比from_node短，
      // 那么原来的从from_node变道到to_node更不容易出错
      if (base_node->EndS() - base_node->StartS() <
          from_node->EndS() - from_node->StartS()) {
        continue;
      }
      std::vector<const TopoNode*> candidate_set;
      candidate_set.push_back(from_node);
      // Note: 如果base_node是一个origin TopoNode
      // 则这种获取out_edges的方法会漏掉那些与base_node连接的子TopoNode
      // 但在黑名单区域内不进行这个变道优化也合理，忽略这样的TopoNode优化也完全OK
      const auto& out_edges = base_node->OutToLeftOrRightEdge();
      for (const auto* edge : out_edges) {
        const auto* candidate_node = edge->ToNode();
        if (candidate_node == from_node) {
          continue;
        }
        if (candidate_node->GetOutEdgeTo(to_node) != nullptr) {
          candidate_set.push_back(candidate_node);
        }
      }
      // Note: 选出最长的中间节点用于替换from_node
      const auto* largest_node = GetLargestNode(candidate_set);
      if (largest_node == nullptr) {
        return false;
      }
      if (largest_node != from_node) {
        // 新的行驶流程变为base_node --> largest_node --> to_node
        result_node_vec->at(i) = largest_node;
      }
    }
  }
  return true;
}

// Note: 从前往后检查，尝试将变道的地点往起点方向移动到更合适的位置(更长的Lane)
bool AdjustLaneChangeForward(
    std::vector<const TopoNode*>* const result_node_vec) {
  for (size_t i = 1; i < result_node_vec->size() - 1; ++i) {
    // Note: 行驶路径是from_node->to_node->base_node
    const auto* from_node = result_node_vec->at(i - 1);
    const auto* to_node = result_node_vec->at(i);
    const auto* base_node = result_node_vec->at(i + 1);
    const auto* from_to_edge = from_node->GetOutEdgeTo(to_node);
    if (from_to_edge == nullptr) {
      // may need to recalculate edge,
      // because only edge from origin node to subnode is saved
      from_to_edge = to_node->GetInEdgeFrom(from_node);
    }
    if (from_to_edge == nullptr) {
      AERROR << "Get null ptr to edge:" << from_node->LaneId() << " ("
             << from_node->StartS() << ", " << from_node->EndS() << ")"
             << " --> " << to_node->LaneId() << " (" << to_node->StartS()
             << ", " << to_node->EndS() << ")";
      return false;
    }
    // Note: 变道类型的边才作处理(from_node->to_node)
    // Note: 检查from_node--(直行)->candidate_node--(变道)->base_node是不是
    // 比from_node--(变道)->to_node--(直行)->base_node更适合变道
    if (from_to_edge->Type() != TopoEdgeType::TET_FORWARD) {
      if (base_node->EndS() - base_node->StartS() <
          to_node->EndS() - to_node->StartS()) {
        continue;
      }
      std::vector<const TopoNode*> candidate_set;
      candidate_set.push_back(to_node);
      const auto& in_edges = base_node->InFromLeftOrRightEdge();
      for (const auto* edge : in_edges) {
        // 看看从from_node到base_node有没有别的路可以走
        const auto* candidate_node = edge->FromNode();
        if (candidate_node == to_node) {
          continue;
        }
        if (candidate_node->GetInEdgeFrom(from_node) != nullptr) {
          candidate_set.push_back(candidate_node);
        }
      }
      // Note: 选长度最长的节点(更容易实现变道)
      const auto* largest_node = GetLargestNode(candidate_set);
      if (largest_node == nullptr) {
        return false;
      }
      if (largest_node != to_node) {
        result_node_vec->at(i) = largest_node;
      }
    }
  }
  return true;
}

// Note: 在发生变道的路由段做处理，选择更长的节点作为变道区间
bool AdjustLaneChange(std::vector<const TopoNode*>* const result_node_vec) {
  if (result_node_vec->size() < 3) {
    return true;
  }
  // Note: 从后往前检查，尝试将变道的地点往终点方向移动到更合适的位置(更长的Lane)
  if (!AdjustLaneChangeBackward(result_node_vec)) {
    AERROR << "Failed to adjust lane change backward";
    return false;
  }
  // Note: 从前往后检查，尝试将变道的地点往起点方向移动到更合适的位置(更长的Lane)
  if (!AdjustLaneChangeForward(result_node_vec)) {
    AERROR << "Failed to adjust lane change backward";
    return false;
  }
  return true;
}

// Note: 根据came_from信息抽取完整的行车路径
// Note: 对发生变道的路由段做处理，选择前方/后方更长的节点作为变道区间
bool Reconstruct(
    const std::unordered_map<const TopoNode*, const TopoNode*>& came_from,
    const TopoNode* dest_node, std::vector<NodeWithRange>* result_nodes) {
  std::vector<const TopoNode*> result_node_vec;
  result_node_vec.push_back(dest_node);

  auto iter = came_from.find(dest_node);
  while (iter != came_from.end()) {
    result_node_vec.push_back(iter->second);
    iter = came_from.find(iter->second);
  }
  std::reverse(result_node_vec.begin(), result_node_vec.end());
  // Note: 对发生变道的路由段做处理，选择前方/后方更长的节点作为变道区间
  if (!AdjustLaneChange(&result_node_vec)) {
    AERROR << "Failed to adjust lane change";
    return false;
  }
  result_nodes->clear();
  for (const auto* node : result_node_vec) {
    // Note: result_nodes中的TopoNode都是origin TopoNode
    result_nodes->emplace_back(node->OriginNode(), node->StartS(),
                               node->EndS());
  }
  return true;
}

}  // namespace

AStarStrategy::AStarStrategy(bool enable_change)
    : change_lane_enabled_(enable_change) {}

void AStarStrategy::Clear() {
  closed_set_.clear();
  open_set_.clear();
  came_from_.clear();
  enter_s_.clear();
  g_score_.clear();
}

// Note: heuristic cost是两个TopoNode的anchor points的欧式距离
// anchor point是TopoNode表示的LaneSegment的中点
double AStarStrategy::HeuristicCost(const TopoNode* src_node,
                                    const TopoNode* dest_node) {
  const auto& src_point = src_node->AnchorPoint();
  const auto& dest_point = dest_node->AnchorPoint();
  double distance = fabs(src_point.x() - dest_point.x()) +
                    fabs(src_point.y() - dest_point.y());
  return distance;
}

// Note: 搜索两个waypoint(TopoNode)之间的可行路径
// graph: 拓扑图
// sub_graph: 子拓扑图(子图)
// src_node: 起点waypoint所在的子TopoNode,
// 一般来说，src_node区间范围是[start_s - 1cm, length]
// dest_node: 终点waypoint所在的子TopoNode,
// 一般来说，dest_node区间范围是[0, end_s + 1cm]
// 最终输出从src_node到dest_node的NodeWithRange序列
// 这个result_nodes记录的是从src_node到dest_node经过的LaneSegment
// 这些LaneSegment从搜索结果的TopoNode信息中抽取而来
// result_nodes中的NodeWithRange里面的TopoNode都是origin TopoNode
bool AStarStrategy::Search(const TopoGraph* graph,
                           const SubTopoGraph* sub_graph,
                           const TopoNode* src_node, const TopoNode* dest_node,
                           std::vector<NodeWithRange>* const result_nodes) {
  Clear();
  AINFO << "Start A* search algorithm.";

  std::priority_queue<SearchNode> open_set_detail;

  SearchNode src_search_node(src_node);
  // Note: f = g + h, 起点的g = 0
  src_search_node.f = HeuristicCost(src_node, dest_node);
  open_set_detail.push(src_search_node);

  open_set_.insert(src_node);
  g_score_[src_node] = 0.0;
  enter_s_[src_node] = src_node->StartS();

  SearchNode current_node;
  // Note: 所有从当前node出发的edge
  std::unordered_set<const TopoEdge*> next_edge_set;
  std::unordered_set<const TopoEdge*> sub_edge_set;
  while (!open_set_detail.empty()) {
    current_node = open_set_detail.top();
    const auto* from_node = current_node.topo_node;
    // Note: 搜索到终点
    if (current_node.topo_node == dest_node) {
      // Note: 根据搜索的中间信息输出路径规划结果
      if (!Reconstruct(came_from_, from_node, result_nodes)) {
        AERROR << "Failed to reconstruct route.";
        return false;
      }
      return true;
    }
    open_set_.erase(from_node);
    open_set_detail.pop();

    if (closed_set_.count(from_node) != 0) {
      // if showed before, just skip...
      continue;
    }
    closed_set_.emplace(from_node);

    // if residual_s is less than FLAGS_min_length_for_lane_change, only move
    // forward
    // 根据剩余距离是否足够变道来筛选下一个节点
    const auto& neighbor_edges =
        (GetResidualS(from_node) > FLAGS_min_length_for_lane_change &&
         change_lane_enabled_)
            ? from_node->OutToAllEdge()
            : from_node->OutToSucEdge();
    double tentative_g_score = 0.0;
    next_edge_set.clear();
    for (const auto* edge : neighbor_edges) {
      sub_edge_set.clear();
      // Note: 本质上只是获取从from_node出发的Edge
      sub_graph->GetSubInEdgesIntoSubGraph(edge, &sub_edge_set);
      next_edge_set.insert(sub_edge_set.begin(), sub_edge_set.end());
    }

    // Note: 所有从当前node出发的edge
    for (const auto* edge : next_edge_set) {
      const auto* to_node = edge->ToNode();
      // Note: 已经访问过, 访问过的node的cost不会比当前的cost高
      if (closed_set_.count(to_node) == 1) {
        continue;
      }
      if (GetResidualS(edge, to_node) < FLAGS_min_length_for_lane_change) {
        continue;
      }
      tentative_g_score =
          g_score_[current_node.topo_node] + GetCostToNeighbor(edge);
      // Note: 如果是变道类型的Edge，两条道不是都走完全程的，节点自身的cost只取一半
      // 通俗一点的说法就是，我从A变道到B，我的行驶路程并没有Lane A + Lane B的总长度这么长
      // 大概就是一半这样子
      if (edge->Type() != TopoEdgeType::TET_FORWARD) {
        tentative_g_score -=
            (edge->FromNode()->Cost() + edge->ToNode()->Cost()) / 2;
      }
      double f = tentative_g_score + HeuristicCost(to_node, dest_node);
      // Note: 如果发现to_node已经处于搜索边界且已知的cost比从当前节点出发的cost要小
      if (open_set_.count(to_node) != 0 && f >= g_score_[to_node]) {
        continue;
      }

      // Note: 估计进入to_node时的位置
      // if to_node is reached by forward, reset enter_s to start_s
      if (edge->Type() == TopoEdgeType::TET_FORWARD) {
        enter_s_[to_node] = to_node->StartS();
      } else {
        // else, add enter_s with FLAGS_min_length_for_lane_change
        // Note: 估算变道进入to_node的位置
        double to_node_enter_s =
            (enter_s_[from_node] + FLAGS_min_length_for_lane_change) /
            from_node->Length() * to_node->Length();
        // enter s could be larger than end_s but should be less than length
        to_node_enter_s = std::min(to_node_enter_s, to_node->Length());
        // if enter_s is larger than end_s and to_node is dest_node
        if (to_node_enter_s > to_node->EndS() && to_node == dest_node) {
          continue;
        }
        enter_s_[to_node] = to_node_enter_s;
      }

      g_score_[to_node] = f;
      SearchNode next_node(to_node);
      next_node.f = f;
      open_set_detail.push(next_node);
      // Note: 记录父节点
      came_from_[to_node] = from_node;
      if (open_set_.count(to_node) == 0) {
        open_set_.insert(to_node);
      }
    }
  }
  AERROR << "Failed to find goal lane with id: " << dest_node->LaneId();
  return false;
}

// Note: 计算从进入当前TopoNode的位置开始，到TopoNode结束，这段距离的长度
// 这段距离用于判断剩余距离是否足够变道，然后筛选下一个节点
double AStarStrategy::GetResidualS(const TopoNode* node) {
  double start_s = node->StartS();
  const auto iter = enter_s_.find(node);
  if (iter != enter_s_.end()) {
    if (iter->second > node->EndS()) {
      return 0.0;
    }
    start_s = iter->second;
  } else {
    AWARN << "lane " << node->LaneId() << "(" << node->StartS() << ", "
          << node->EndS() << "not found in enter_s map";
  }
  double end_s = node->EndS();
  const TopoNode* succ_node = nullptr;
  // Note: waypoint处的Lane会被切割成连接的两段TopoNode
  // 这种情况下，同一Lane中的下一段LaneSegment的s也算到剩余可变道区间里面
  for (const auto* edge : node->OutToAllEdge()) {
    if (edge->ToNode()->LaneId() == node->LaneId()) {
      succ_node = edge->ToNode();
      break;
    }
  }
  if (succ_node != nullptr) {
    end_s = succ_node->EndS();
  }
  return (end_s - start_s);
}

double AStarStrategy::GetResidualS(const TopoEdge* edge,
                                   const TopoNode* to_node) {
  if (edge->Type() == TopoEdgeType::TET_FORWARD) {
    return std::numeric_limits<double>::max();
  }
  double start_s = to_node->StartS();
  const auto* from_node = edge->FromNode();
  const auto iter = enter_s_.find(from_node);
  if (iter != enter_s_.end()) {
    double temp_s = iter->second / from_node->Length() * to_node->Length();
    start_s = std::max(start_s, temp_s);
  } else {
    AWARN << "lane " << from_node->LaneId() << "(" << from_node->StartS()
          << ", " << from_node->EndS() << "not found in enter_s map";
  }
  double end_s = to_node->EndS();
  const TopoNode* succ_node = nullptr;
  // Note: waypoint处的Lane会被切割成连接的两段TopoNode
  // 这种情况下，同一Lane中的下一段LaneSegment的s也算到剩余可变道区间里面
  for (const auto* edge : to_node->OutToAllEdge()) {
    if (edge->ToNode()->LaneId() == to_node->LaneId()) {
      succ_node = edge->ToNode();
      break;
    }
  }
  if (succ_node != nullptr) {
    end_s = succ_node->EndS();
  }
  return (end_s - start_s);
}

}  // namespace routing
}  // namespace apollo
