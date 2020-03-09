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

#include "modules/routing/graph/sub_topo_graph.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include "modules/routing/graph/range_utils.h"

namespace apollo {
namespace routing {

namespace {

const double MIN_DIFF_LENGTH = 0.1e-6;             // in meters
const double MIN_INTERNAL_FOR_NODE = 0.01;         // in meters
const double MIN_POTENTIAL_LANE_CHANGE_LEN = 3.0;  // in meters

bool IsCloseEnough(double s1, double s2) {
  return std::fabs(s1 - s2) < MIN_DIFF_LENGTH;
}

// Note: 对topo_node的多个NodeSRange区间进行合并和拼接
// [1, 3]和[2, 4]会被合并成[1, 4]
// [1, 2]和[2, 3]会被拼接成[1, 3]
void MergeBlockRange(const TopoNode* topo_node,
                     const std::vector<NodeSRange>& origin_range,
                     std::vector<NodeSRange>* block_range) {
  std::vector<NodeSRange> sorted_origin_range;
  sorted_origin_range.insert(sorted_origin_range.end(), origin_range.begin(),
                             origin_range.end());
  sort(sorted_origin_range.begin(), sorted_origin_range.end());
  int cur_index = 0;
  int total_size = static_cast<int>(sorted_origin_range.size());
  while (cur_index < total_size) {
    NodeSRange range(sorted_origin_range[cur_index]);
    ++cur_index;
    while (cur_index < total_size &&
           range.MergeRangeOverlap(sorted_origin_range[cur_index])) {
      ++cur_index;
    }
    if (range.EndS() < topo_node->StartS() ||
        range.StartS() > topo_node->EndS()) {
      continue;
    }
    range.SetStartS(std::max(topo_node->StartS(), range.StartS()));
    range.SetEndS(std::min(topo_node->EndS(), range.EndS()));
    block_range->push_back(std::move(range));
  }
}

// Note: 过滤TopoNode处于黑名单的NodeSRange, 从而获得TopoNode的有效NodeSRange
// origin_range是topo_node对应的黑名单列表
// 如果整条Lane(从[0, lane's length]都是黑名单), 那么这个方法获取到的valid_range
// 会是[0, 0]和[lane's length, lane's length]
// 但是没关系，在后面通过valid range生成sub TopoNode时会过滤长度过小的Range
// Note: 一条正常的普通Lane，如果同向的neighbour Lane或者本Lane上有waypoint
// 则这个Lane的会被分成两段连续的valid range，例如[0, m], [m, length]
void GetSortedValidRange(const TopoNode* topo_node,
                         const std::vector<NodeSRange>& origin_range,
                         std::vector<NodeSRange>* valid_range) {
  std::vector<NodeSRange> block_range;
  MergeBlockRange(topo_node, origin_range, &block_range);
  double start_s = topo_node->StartS();
  double end_s = topo_node->EndS();
  std::vector<double> all_value;
  all_value.push_back(start_s);
  for (const auto& range : block_range) {
    all_value.push_back(range.StartS());
    all_value.push_back(range.EndS());
  }
  all_value.push_back(end_s);
  for (size_t i = 0; i < all_value.size(); i += 2) {
    NodeSRange new_range(all_value[i], all_value[i + 1]);
    valid_range->push_back(std::move(new_range));
  }
}

// Node: 判断from_node和to_node的对应的相邻Lane的边界重合部分是否足够用于变道
// 这个计算结果需要以一个基本假设为基础, 这两条Lane的起始位置是差不多的
bool IsReachable(const TopoNode* from_node, const TopoNode* to_node) {
  // Note: 计算to_node->StartS()在from_node的origin TopoNode中的等比例对应位置
  // 注意, 这里的Length()是routing map里面的Node的长度(即Lane长度)
  double start_s = to_node->StartS() / to_node->Length() * from_node->Length();
  start_s = std::max(start_s, from_node->StartS());
  // Note: 计算to_node->EndS()在from_node的origin TopoNode中的等比例对应位置
  // 注意, 这里的Length()是routing map里面的Node的长度(即Lane长度)
  double end_s = to_node->EndS() / to_node->Length() * from_node->Length();
  end_s = std::min(end_s, from_node->EndS());
  return (end_s - start_s > MIN_POTENTIAL_LANE_CHANGE_LEN);
}

}  // namespace

// Note: 建子图, 过滤黑名单然后创建valid range的TopoNode(即子TopoNode)
// 相关的子TopoEdge都会被创建, 子TopoEdge的信息会添加到子TopoNode中
// black_map是origin TopoNode对应的黑名单NodeSRange
// 没有NodeSRange黑名单的TopoNode不在black_map里面
// Note: black_map包含了那些与waypoint和routing request中的黑名单相关的TopoNode
// Note: 这个初始化之后，SubTopoGraph给valid range创建了子TopoNode，
// 对可以连接的<子TopoNode-子TopoNode>以及<子TopoNode-origin TopoNode>添加了子TopoEdge
// 子TopoEdge的信息被添加到子TopoNode中,
// 但origin TopoNode不会添加子TopoEdge到InEdge或OutEdge信息中
SubTopoGraph::SubTopoGraph(
    const std::unordered_map<const TopoNode*, std::vector<NodeSRange> >&
        black_map) {
  std::vector<NodeSRange> valid_range;
  for (const auto& map_iter : black_map) {
    valid_range.clear();
    // Note: 过滤TopoNode处于黑名单的NodeSRange, 从而获得TopoNode的有效NodeSRange
    GetSortedValidRange(map_iter.first, map_iter.second, &valid_range);
    // Note: 为TopoNode的每个valid range都创建一个对应range的子TopoNode
    // 如果两个子TopoNode是紧邻的, 则在两个TopoNode之间添加Edge
    // 使得之前被分割的waypoint起点和终点前后的平行车道的子TopoNode重新连接起来了
    InitSubNodeByValidRange(map_iter.first, valid_range);
  }

  // Note: 创建黑名单中的origin TopoNode的有效子TopoNode的连接边(子TopoEdge)
  for (const auto& map_iter : black_map) {
    InitSubEdge(map_iter.first);
  }

  // 与InitSubEdge里面判断能否变道的标准有一些不同, AddPotentialEdge的变道区间下限为3m
  // 把一些可能在InitSubEdge因变道区间不足而不被添加的边添加进来
  for (const auto& map_iter : black_map) {
    AddPotentialEdge(map_iter.first);
  }
}

SubTopoGraph::~SubTopoGraph() {}

// Note: 这个接口用于(尝试)获取与当前节点(edge->FromNode())连接的子图的Edge
// 如果没有子图的连接边则直接返回原来的边(这当然就是一条原TopoGraph的连接边了)
// 换句话说，这个接口其实就是为了获取从当前节点出发的所有的Edge
// 那为什么不是直接使用edge呢，因为原TopoGraph没有任何关于子图的连接信息
// 在SubTopoGraph构造函数中进行子图构造时，所有的与子图相关的信息都只存在于子图中
// 当原TopoNode与子TopoNode相连时，In/Out Edge信息只是放在子TopoNode中
// 这个接口充当了连接原图和子图的作用
// Note: edge: from_node->OutToAllEdge()中的一条边
// edge: A -> B 如果是sub edge的话(edge其中一端是sub node)
//    return edge A -> B
// edge: A -> B 如果不是sub edge(即A和B都不是子TopoNode), 分以下情况
// 1. B没有sub node
//    return origin edge A -> B
// 2. B有sub node
//    return all edge A -> B'(B'指B的sub node)
void SubTopoGraph::GetSubInEdgesIntoSubGraph(
    const TopoEdge* edge,
    std::unordered_set<const TopoEdge*>* const sub_edges) const {
  const auto* from_node = edge->FromNode();
  const auto* to_node = edge->ToNode();
  std::unordered_set<TopoNode*> sub_nodes;
  // Note: 如果edge其中一端的TopoNode是子TopoNode,
  // 则这个edge是在SubTopoGraph中创建的，否则就是原图TopoGraph中的边
  // Note: edge是子图里面的边或者edge的to_node是没有sub TopoNode的origin TopoNode
  if (from_node->IsSubNode() || to_node->IsSubNode() ||
      !GetSubNodes(to_node, &sub_nodes)) {
    sub_edges->insert(edge);
    return;
  }
  // Note: edge是一条origin TopoEdge, 且to_node有子TopoNode
  for (const auto* sub_node : sub_nodes) {
    for (const auto* in_edge : sub_node->InFromAllEdge()) {
      if (in_edge->FromNode() == from_node) {
        sub_edges->insert(in_edge);
      }
    }
  }
}

void SubTopoGraph::GetSubOutEdgesIntoSubGraph(
    const TopoEdge* edge,
    std::unordered_set<const TopoEdge*>* const sub_edges) const {
  const auto* from_node = edge->FromNode();
  const auto* to_node = edge->ToNode();
  std::unordered_set<TopoNode*> sub_nodes;
  if (from_node->IsSubNode() || to_node->IsSubNode() ||
      !GetSubNodes(from_node, &sub_nodes)) {
    sub_edges->insert(edge);
    return;
  }
  for (const auto* sub_node : sub_nodes) {
    for (const auto* out_edge : sub_node->OutToAllEdge()) {
      if (out_edge->ToNode() == to_node) {
        sub_edges->insert(out_edge);
      }
    }
  }
}

const TopoNode* SubTopoGraph::GetSubNodeWithS(const TopoNode* topo_node,
                                              double s) const {
  const auto& map_iter = sub_node_range_sorted_map_.find(topo_node);
  if (map_iter == sub_node_range_sorted_map_.end()) {
    return topo_node;
  }
  const auto& sorted_vec = map_iter->second;
  // sorted vec can't be empty!
  int index = BinarySearchForStartS(sorted_vec, s);
  if (index < 0) {
    return nullptr;
  }
  return sorted_vec[index].GetTopoNode();
}

// Note: 为origin TopoNode的每个valid range都创建一个对应range的子TopoNode
// valid_range是已经排序的
// 如果两个子TopoNode是紧邻的, 则在两个TopoNode之间添加Edge
void SubTopoGraph::InitSubNodeByValidRange(
    const TopoNode* topo_node, const std::vector<NodeSRange>& valid_range) {
  // Attention: no matter topo node has valid_range or not,
  // create map value first;
  // Note: 由原TopoNode的每一段valid range生成的子TopoNode列表
  auto& sub_node_vec = sub_node_range_sorted_map_[topo_node];
  // Note: 由原TopoNode的每一段valid range生成的子TopoNode集合
  auto& sub_node_set = sub_node_map_[topo_node];

  // Note: 为每个valid range都创建一个对应range的TopoNode
  std::vector<TopoNode*> sub_node_sorted_vec;
  for (const auto& range : valid_range) {
    // Note: 把[0, 0]和[length, length]这种过滤了
    if (range.Length() < MIN_INTERNAL_FOR_NODE) {
      continue;
    }
    std::shared_ptr<TopoNode> sub_topo_node_ptr;
    sub_topo_node_ptr.reset(new TopoNode(topo_node, range));
    sub_node_vec.emplace_back(sub_topo_node_ptr.get(), range);
    sub_node_set.insert(sub_topo_node_ptr.get());
    sub_node_sorted_vec.push_back(sub_topo_node_ptr.get());
    topo_nodes_.push_back(std::move(sub_topo_node_ptr));
  }

  // Note: 如果两个valid range是挨着的, 则在对应的TopoNode之间添加Edge表示连通
  // 这里将那些由于waypoint被分割的Lane的子TopoNode重新被连接起来了
  for (size_t i = 1; i < sub_node_sorted_vec.size(); ++i) {
    auto* pre_node = sub_node_sorted_vec[i - 1];
    auto* next_node = sub_node_sorted_vec[i];
    if (IsCloseEnough(pre_node->EndS(), next_node->StartS())) {
      Edge edge;
      edge.set_from_lane_id(topo_node->LaneId());
      edge.set_to_lane_id(topo_node->LaneId());
      edge.set_direction_type(Edge::FORWARD);
      edge.set_cost(0.0);
      std::shared_ptr<TopoEdge> topo_edge_ptr;
      topo_edge_ptr.reset(new TopoEdge(edge, pre_node, next_node));
      pre_node->AddOutEdge(topo_edge_ptr.get());
      next_node->AddInEdge(topo_edge_ptr.get());
      topo_edges_.push_back(std::move(topo_edge_ptr));
    }
  }
}

// Note: 创建黑名单中的子TopoNode的连接边(子TopoEdge)
// topo_node是黑名单中的origin TopoNode
// Note: 为子TopoNode添加TopoEdge，这里添加的TopoEdge的其中一端是子TopoNode，
// 另一端可能是origin TopoNode也可能是子TopoNode
// 子TopoNode从此被赋予了InEdge和OutEdge属性，
// 但这新的Edge信息没有被添加到origin TopoNode的InEdge/OutEdge属性中
void SubTopoGraph::InitSubEdge(const TopoNode* topo_node) {
  std::unordered_set<TopoNode*> sub_nodes;
  // Note: 获取TopoNode的所有valid range对应的子TopoNode集合
  if (!GetSubNodes(topo_node, &sub_nodes)) {
    return;
  }

  // Note: 那些被waypoint分割的Lane和neighbour Lane至少有两个sub_node
  for (auto* sub_node : sub_nodes) {
    // Note: origin topo_node的InFromAllEdge()和OutToAllEdge()
    // 在加载完拓扑地图(routing map)时就已经确定
    // Note: 给sub_node添加(子)TopoEdge
    // 主要是判断origin TopoEdge是否能在sub_node的valid range范围内变道或者直行
    // 然后创建新的TopoEdge, 通过AddInEdge或AddOutEdge添加到sub_node的信息中
    // Note: 可以以起点和终点作为为topo_node例子来考察
    InitInSubNodeSubEdge(sub_node, topo_node->InFromAllEdge());
    InitOutSubNodeSubEdge(sub_node, topo_node->OutToAllEdge());
  }
}

// Note: 对于一个子TopoNode, 判断其origin TopoNode关联的
// 那些TopoEdge连接的另一边的origin TopoNode的子TopoNode
// 是否可能与当前的子TopoNode产生连接
// 如果另一边没有子TopoNode, 直接在子图新建一条子TopoEdge, 连接到另一边的origin TopoNode
// Note: 注意, 并没有在origin node中添加新的sub edge信息
// sub edge信息只在sub node中添加
// Note: 创建子图的TopoEdge, 子图的TopoNode从此可以通过子图的TopoEdge连接到origin TopoNode
// Note: origin_edge是进入sub_node所在origin_node的所有In类型的TopoEdge
void SubTopoGraph::InitInSubNodeSubEdge(
    TopoNode* const sub_node,
    const std::unordered_set<const TopoEdge*> origin_edge) {
  std::unordered_set<TopoNode*> other_sub_nodes;
  // Note: 每一条进入原始TopoNode的原始TopoEdge
  for (const auto* in_edge : origin_edge) {
    // Note: 判断in_edge的FromNode是否在黑名单中
    // 如果在, 将FromNode(origin TopoNode)的子TopoNode集合放到other_sub_nodes
    // other_sub_nodes是FromNode的valid range的子TopoNode集合

    // Note: 判断有没有可以进入当前子TopoNode的别的子TopoNode
    if (GetSubNodes(in_edge->FromNode(), &other_sub_nodes)) {
      for (auto* sub_from_node : other_sub_nodes) {
        // Note: 判断是否能从sub_from_node行驶到sub_node(变道或者直行)
        if (!sub_from_node->IsOverlapEnough(sub_node, in_edge)) {
          continue;
        }
        std::shared_ptr<TopoEdge> topo_edge_ptr;
        topo_edge_ptr.reset(
            new TopoEdge(in_edge->PbEdge(), sub_from_node, sub_node));
        sub_node->AddInEdge(topo_edge_ptr.get());
        sub_from_node->AddOutEdge(topo_edge_ptr.get());
        topo_edges_.push_back(std::move(topo_edge_ptr));
      }

    // Note: in_edge的FromNode不在子图中，情况分以下两种
    // 1. in_edge是那些从普通Lane进入带waypoint的Lane的连接边
    // 2. in_edge是那些从普通Lane进入带黑名单LaneSegment的Lane的连接边
    // 子图和原来从routing map加载后建立的拓扑地图由此关联起来
    } else if (in_edge->FromNode()->IsOverlapEnough(sub_node, in_edge)) {
      std::shared_ptr<TopoEdge> topo_edge_ptr;
      topo_edge_ptr.reset(
          new TopoEdge(in_edge->PbEdge(), in_edge->FromNode(), sub_node));
      // Note: 注意, 并没有在origin node中添加新的sub edge信息
      sub_node->AddInEdge(topo_edge_ptr.get());
      topo_edges_.push_back(std::move(topo_edge_ptr));
    }
  }
}

void SubTopoGraph::InitOutSubNodeSubEdge(
    TopoNode* const sub_node,
    const std::unordered_set<const TopoEdge*> origin_edge) {
  std::unordered_set<TopoNode*> other_sub_nodes;
  for (const auto* out_edge : origin_edge) {
    // Note: 如果in_edge的FromNode是否在黑名单中
    // 如果在, 将FromNode(origin TopoNode)的子TopoNode集合放到other_sub_nodes
    if (GetSubNodes(out_edge->ToNode(), &other_sub_nodes)) {
      for (auto* sub_to_node : other_sub_nodes) {
        if (!sub_node->IsOverlapEnough(sub_to_node, out_edge)) {
          continue;
        }
        std::shared_ptr<TopoEdge> topo_edge_ptr;
        topo_edge_ptr.reset(
            new TopoEdge(out_edge->PbEdge(), sub_node, sub_to_node));
        sub_node->AddOutEdge(topo_edge_ptr.get());
        sub_to_node->AddInEdge(topo_edge_ptr.get());
        topo_edges_.push_back(std::move(topo_edge_ptr));
      }
    } else if (sub_node->IsOverlapEnough(out_edge->ToNode(), out_edge)) {
      std::shared_ptr<TopoEdge> topo_edge_ptr;
      topo_edge_ptr.reset(
          new TopoEdge(out_edge->PbEdge(), sub_node, out_edge->ToNode()));
      sub_node->AddOutEdge(topo_edge_ptr.get());
      topo_edges_.push_back(std::move(topo_edge_ptr));
    }
  }
}

// Note: 获取origin TopoNode中的所有valid range对应的子TopoNode集合
bool SubTopoGraph::GetSubNodes(
    const TopoNode* node,
    std::unordered_set<TopoNode*>* const sub_nodes) const {
  const auto& iter = sub_node_map_.find(node);
  if (iter == sub_node_map_.end()) {
    return false;
  }
  // Note: 注意, 这里最终获得的sub_nodes可能是个空集(没有valid range)
  // 举个例子, 黑名单Road里面的Lane对应的TopoNode就没有子TopoNode
  sub_nodes->clear();
  sub_nodes->insert(iter->second.begin(), iter->second.end());
  return true;
}

// Note: topo_node是黑名单里面的origin TopoNode
// 将TopoEdge信息添加到子TopoNode中
void SubTopoGraph::AddPotentialEdge(const TopoNode* topo_node) {
  std::unordered_set<TopoNode*> sub_nodes;
  if (!GetSubNodes(topo_node, &sub_nodes)) {
    return;
  }
  // Note: 根据有效的origin_edge创建新的TopoEdge
  // 新的TopoEdge信息被添加到sub_node的信息中, 同时将TopoEdge放到topo_edges_
  for (auto* sub_node : sub_nodes) {
    AddPotentialInEdge(sub_node, topo_node->InFromLeftOrRightEdge());
    AddPotentialOutEdge(sub_node, topo_node->OutToLeftOrRightEdge());
  }
}

// Note: 为sub_node创建TopoEdge
// 这里创建TopoEdge的条件与InitInSubNodeSubEdge的条件稍有不同
// 区别就是判断能否变道的标准改变了(变道区间范围是3m，这是一个相当小的值)
// sub_node是origin TopoNode的子TopoNode
// origin_edge是origin TopoNode的In Edge
void SubTopoGraph::AddPotentialInEdge(
    TopoNode* const sub_node,
    const std::unordered_set<const TopoEdge*> origin_edge) {
  std::unordered_set<TopoNode*> other_sub_nodes;
  for (const auto* in_edge : origin_edge) {
    // Note: 判断in_edge的FromNode是否在黑名单中
    // 如果在, 将FromNode(origin TopoNode)的子TopoNode集合放到other_sub_nodes
    // other_sub_nodes是FromNode的valid range的子TopoNode集合

    // Note: 判断有没有可以进入当前子TopoNode的别的子TopoNode
    if (GetSubNodes(in_edge->FromNode(), &other_sub_nodes)) {
      for (auto* sub_from_node : other_sub_nodes) {
        if (sub_node->GetInEdgeFrom(sub_from_node) != nullptr) {
          continue;
        }
        // Note: 判断sub_from_node与sub_node对应的Lane片段的重合边界
        // 是否足够用于变道
        if (!IsReachable(sub_from_node, sub_node)) {
          continue;
        }
        std::shared_ptr<TopoEdge> topo_edge_ptr;
        topo_edge_ptr.reset(
            new TopoEdge(in_edge->PbEdge(), sub_from_node, sub_node));
        sub_node->AddInEdge(topo_edge_ptr.get());
        sub_from_node->AddOutEdge(topo_edge_ptr.get());
        topo_edges_.push_back(std::move(topo_edge_ptr));
      }
    } else {
      if (sub_node->GetInEdgeFrom(in_edge->FromNode()) != nullptr) {
        continue;
      }
      std::shared_ptr<TopoEdge> topo_edge_ptr;
      topo_edge_ptr.reset(
          new TopoEdge(in_edge->PbEdge(), in_edge->FromNode(), sub_node));
      sub_node->AddInEdge(topo_edge_ptr.get());
      topo_edges_.push_back(std::move(topo_edge_ptr));
    }
  }
}

void SubTopoGraph::AddPotentialOutEdge(
    TopoNode* const sub_node,
    const std::unordered_set<const TopoEdge*> origin_edge) {
  std::unordered_set<TopoNode*> other_sub_nodes;
  for (const auto* out_edge : origin_edge) {
    if (GetSubNodes(out_edge->ToNode(), &other_sub_nodes)) {
      for (auto* sub_to_node : other_sub_nodes) {
        if (sub_node->GetOutEdgeTo(sub_to_node) != nullptr) {
          continue;
        }
        if (!IsReachable(sub_node, sub_to_node)) {
          continue;
        }
        std::shared_ptr<TopoEdge> topo_edge_ptr;
        topo_edge_ptr.reset(
            new TopoEdge(out_edge->PbEdge(), sub_node, sub_to_node));
        sub_node->AddOutEdge(topo_edge_ptr.get());
        sub_to_node->AddInEdge(topo_edge_ptr.get());
        topo_edges_.push_back(std::move(topo_edge_ptr));
      }
    } else {
      if (sub_node->GetOutEdgeTo(out_edge->ToNode()) != nullptr) {
        continue;
      }
      std::shared_ptr<TopoEdge> topo_edge_ptr;
      topo_edge_ptr.reset(
          new TopoEdge(out_edge->PbEdge(), sub_node, out_edge->ToNode()));
      sub_node->AddOutEdge(topo_edge_ptr.get());
      topo_edges_.push_back(std::move(topo_edge_ptr));
    }
  }
}

}  // namespace routing
}  // namespace apollo
