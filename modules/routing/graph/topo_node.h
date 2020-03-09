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

#pragma once

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "modules/routing/graph/topo_range.h"
#include "modules/routing/proto/topo_graph.pb.h"

namespace apollo {
namespace routing {

class TopoEdge;

class TopoNode {
 public:
  static bool IsOutRangeEnough(const std::vector<NodeSRange>& range_vec,
                               double start_s, double end_s);

 public:
  explicit TopoNode(const Node& node);
  TopoNode(const TopoNode* topo_node, const NodeSRange& range);

  ~TopoNode();

  const Node& PbNode() const;
  double Length() const;
  double Cost() const;
  bool IsVirtual() const;

  const std::string& LaneId() const;
  const std::string& RoadId() const;
  const hdmap::Curve& CentralCurve() const;
  const common::PointENU& AnchorPoint() const;
  const std::vector<NodeSRange>& LeftOutRange() const;
  const std::vector<NodeSRange>& RightOutRange() const;

  const std::unordered_set<const TopoEdge*>& InFromAllEdge() const;
  const std::unordered_set<const TopoEdge*>& InFromLeftEdge() const;
  const std::unordered_set<const TopoEdge*>& InFromRightEdge() const;
  const std::unordered_set<const TopoEdge*>& InFromLeftOrRightEdge() const;
  const std::unordered_set<const TopoEdge*>& InFromPreEdge() const;
  const std::unordered_set<const TopoEdge*>& OutToAllEdge() const;
  const std::unordered_set<const TopoEdge*>& OutToLeftEdge() const;
  const std::unordered_set<const TopoEdge*>& OutToRightEdge() const;
  const std::unordered_set<const TopoEdge*>& OutToLeftOrRightEdge() const;
  const std::unordered_set<const TopoEdge*>& OutToSucEdge() const;

  const TopoEdge* GetInEdgeFrom(const TopoNode* from_node) const;
  const TopoEdge* GetOutEdgeTo(const TopoNode* to_node) const;

  const TopoNode* OriginNode() const;
  double StartS() const;
  double EndS() const;
  bool IsSubNode() const;
  bool IsInFromPreEdgeValid() const;
  bool IsOutToSucEdgeValid() const;
  bool IsOverlapEnough(const TopoNode* sub_node,
                       const TopoEdge* edge_for_type) const;
  void AddInEdge(const TopoEdge* edge);
  void AddOutEdge(const TopoEdge* edge);

 private:
  // Note: 找lane中心线的中间点作为锚点
  // 找左右虚线边界最长的range, 并简单地通过长度判断是否足够用于lane change
  void Init();
  // Note: anchor point在lane中心线的中间位置点
  bool FindAnchorPoint();
  void SetAnchorPoint(const common::PointENU& anchor_point);

  // Note: TopoNode的来源Node, 即拓扑地图中的Node
  Node pb_node_;
  // Note: 就是TopoNode表示的LaneSegment的中心线的中间点
  // 这个用来评估当前LaneSegment到终点的距离(作为heuristic cost)
  common::PointENU anchor_point_;

  // Note: TopoNode的起始位置(相对于Lane起点s距离)
  double start_s_;
  // Note: TopoNode的结束位置(相对于Lane起点s距离)
  double end_s_;
  // Note: prefer_range大于设置的最小变道区间则认为长度足够
  bool is_left_range_enough_;
  // Note: lane左边界最大的虚线range的left_out_sorted_range_下标
  int left_prefer_range_index_;
  // Note: prefer_range大于设置的最小变道区间则认为长度足够
  bool is_right_range_enough_;
  // Note: lane右边界最大的虚线range的right_out_sorted_range_下标
  int right_prefer_range_index_;

  // out range按start_s从小到大排序
  std::vector<NodeSRange> left_out_sorted_range_;
  std::vector<NodeSRange> right_out_sorted_range_;

  // Note: 从任意方向进入当前TopoNode的TopoEdge集合
  std::unordered_set<const TopoEdge*> in_from_all_edge_set_;
  // Note: 从左(left)边进入(in)当前TopoNode的TopoEdge集合
  std::unordered_set<const TopoEdge*> in_from_left_edge_set_;
  // Note: 从右(right)边进入(in)当前TopoNode的TopoEdge集合
  std::unordered_set<const TopoEdge*> in_from_right_edge_set_;
  std::unordered_set<const TopoEdge*> in_from_left_or_right_edge_set_;
  std::unordered_set<const TopoEdge*> in_from_pre_edge_set_;
  std::unordered_set<const TopoEdge*> out_to_all_edge_set_;
  std::unordered_set<const TopoEdge*> out_to_left_edge_set_;
  std::unordered_set<const TopoEdge*> out_to_right_edge_set_;
  std::unordered_set<const TopoEdge*> out_to_left_or_right_edge_set_;
  std::unordered_set<const TopoEdge*> out_to_suc_edge_set_;

  // Note: 对于某个TopoNode A，可以查询this TopoNode能否进入A
  // 例如, out_edge_map_[A]可以获取从this TopoNode进入A的TopoEdge
  std::unordered_map<const TopoNode*, const TopoEdge*> out_edge_map_;
  // Note: 对于某个TopoNode A，可以查询A是否能进入this TopoNode
  // 例如, in_edge_map_[A]可以获取从A进入this TopoNode的TopoEdge
  std::unordered_map<const TopoNode*, const TopoEdge*> in_edge_map_;

  const TopoNode* origin_node_;
};

enum TopoEdgeType {
  TET_FORWARD,
  TET_LEFT,
  TET_RIGHT,
};

class TopoEdge {
 public:
  TopoEdge(const Edge& edge, const TopoNode* from_node,
           const TopoNode* to_node);

  ~TopoEdge();

  const Edge& PbEdge() const;
  double Cost() const;
  const std::string& FromLaneId() const;
  const std::string& ToLaneId() const;
  TopoEdgeType Type() const;

  const TopoNode* FromNode() const;
  const TopoNode* ToNode() const;

 private:
  Edge pb_edge_;
  const TopoNode* from_node_ = nullptr;
  const TopoNode* to_node_ = nullptr;
};

}  // namespace routing
}  // namespace apollo
