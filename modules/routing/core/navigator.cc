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

#include "modules/routing/core/navigator.h"

#include "cyber/common/file.h"
#include "modules/routing/common/routing_gflags.h"
#include "modules/routing/graph/sub_topo_graph.h"
#include "modules/routing/strategy/a_star_strategy.h"

namespace apollo {
namespace routing {

namespace {

using apollo::common::ErrorCode;

// Note: 打印routing request信息
bool ShowRequestInfo(const RoutingRequest& request, const TopoGraph* graph) {
  for (const auto& wp : request.waypoint()) {
    const auto* node = graph->GetNode(wp.id());
    if (node == nullptr) {
      AERROR << "Way node is not found in topo graph! ID: " << wp.id();
      return false;
    }
    AINFO << "Way point:\tlane id: " << wp.id() << " s: " << wp.s()
          << " x: " << wp.pose().x() << " y: " << wp.pose().y()
          << " length: " << node->Length();
  }

  for (const auto& bl : request.blacklisted_lane()) {
    const auto* node = graph->GetNode(bl.id());
    if (node == nullptr) {
      AERROR << "Black list node is not found in topo graph! ID: " << bl.id();
      return false;
    }
    AINFO << "Black point:\tlane id: " << bl.id()
          << " start_s: " << bl.start_s() << " end_s: " << bl.end_s()
          << " length: " << node->Length();
  }

  return true;
}

// Note: 获取waypoints对应的TopoNode，抽取waypoint在对应Lane中的s
bool GetWayNodes(const RoutingRequest& request, const TopoGraph* graph,
                 std::vector<const TopoNode*>* const way_nodes,
                 std::vector<double>* const way_s) {
  for (const auto& point : request.waypoint()) {
    const auto* cur_node = graph->GetNode(point.id());
    if (cur_node == nullptr) {
      AERROR << "Cannot find way point in graph! Id: " << point.id();
      return false;
    }
    way_nodes->push_back(cur_node);
    way_s->push_back(point.s());
  }
  return true;
}

void SetErrorCode(const common::ErrorCode& error_code_id,
                  const std::string& error_string,
                  common::StatusPb* const error_code) {
  error_code->set_error_code(error_code_id);
  error_code->set_msg(error_string);
  if (error_code_id == common::ErrorCode::OK) {
    ADEBUG << error_string.c_str();
  } else {
    AERROR << error_string.c_str();
  }
}

void PrintDebugData(const std::vector<NodeWithRange>& nodes) {
  AINFO << "Route lane id\tis virtual\tstart s\tend s";
  for (const auto& node : nodes) {
    AINFO << node.GetTopoNode()->LaneId() << "\t"
          << node.GetTopoNode()->IsVirtual() << "\t" << node.StartS() << "\t"
          << node.EndS();
  }
}

}  // namespace

// Note: 加载原始(origin)拓扑地图
Navigator::Navigator(const std::string& topo_file_path) {
  Graph graph;
  if (!cyber::common::GetProtoFromFile(topo_file_path, &graph)) {
    AERROR << "Failed to read topology graph from " << topo_file_path;
    return;
  }

  graph_.reset(new TopoGraph());
  // Note: 原始(origin)拓扑地图
  if (!graph_->LoadGraph(graph)) {
    AINFO << "Failed to init navigator graph failed! File path: "
          << topo_file_path;
    return;
  }
  black_list_generator_.reset(new BlackListRangeGenerator);
  result_generator_.reset(new ResultGenerator);
  is_ready_ = true;
  AINFO << "The navigator is ready.";
}

Navigator::~Navigator() {}

bool Navigator::IsReady() const { return is_ready_; }

void Navigator::Clear() { topo_range_manager_.Clear(); }

// Note: 获取waypoints对应的origin TopoNode和在对应Lane中的s
// 添加request指定的黑名单
bool Navigator::Init(const RoutingRequest& request, const TopoGraph* graph,
                     std::vector<const TopoNode*>* const way_nodes,
                     std::vector<double>* const way_s) {
  Clear();
  // Note: 获取waypoints对应的TopoNode和waypoint在对应Lane中的s
  if (!GetWayNodes(request, graph_.get(), way_nodes, way_s)) {
    AERROR << "Failed to find search terminal point in graph!";
    return false;
  }
  // Note: 从request中获取黑名单
  black_list_generator_->GenerateBlackMapFromRequest(request, graph_.get(),
                                                     &topo_range_manager_);
  return true;
}

bool Navigator::MergeRoute(
    const std::vector<NodeWithRange>& node_vec,
    std::vector<NodeWithRange>* const result_node_vec) const {
  for (const auto& node : node_vec) {
    if (result_node_vec->empty() ||
        result_node_vec->back().GetTopoNode() != node.GetTopoNode()) {
      result_node_vec->push_back(node);
    } else {
      if (result_node_vec->back().EndS() < node.StartS()) {
        AERROR << "Result route is not continuous.";
        return false;
      }
      // Note: 连接被waypoint分割的路由结果
      result_node_vec->back().SetEndS(node.EndS());
    }
  }
  return true;
}

// Note: 搜索依次经过所有waypoints的行驶路径
// result_nodes里面NodeWithRange的TopoNode都是origin TopoNode
bool Navigator::SearchRouteByStrategy(
    const TopoGraph* graph, const std::vector<const TopoNode*>& way_nodes,
    const std::vector<double>& way_s,
    std::vector<NodeWithRange>* const result_nodes) const {
  std::unique_ptr<Strategy> strategy_ptr;
  strategy_ptr.reset(new AStarStrategy(FLAGS_enable_change_lane_in_result));

  result_nodes->clear();
  std::vector<NodeWithRange> node_vec;
  // Note: 分段进行路由搜索
  // Note: 在每轮循环中都建立拓扑子图，然后进行路由搜索
  for (size_t i = 1; i < way_nodes.size(); ++i) {
    const auto* way_start = way_nodes[i - 1];
    const auto* way_end = way_nodes[i];
    double way_start_s = way_s[i - 1];
    double way_end_s = way_s[i];

    // Note: 黑名单管理
    // topo_range_manager_只有从routing request中制定的LaneSegment和Road黑名单
    // full_range_manager会在waypoint所在车道&平行车道前后添加新的黑名单
    TopoRangeManager full_range_manager = topo_range_manager_;
    // Note: 在当前Lane和平行车道Lane的
    // way_start_s的后面(-1cm)和way_end_s的前面(+1cm)设置黑名单
    // 但这里通过AddBlackMapFromTerminal加入的黑名单NodeSRange的长度都是0
    // 举个例子, [3.21, 3.21]
    // Note: way_start这个TopoNode会产生两个子TopoNode，
    // 分别是[0, way_start_s - 1cm]和[way_start_s - 1cm, length]
    // 这里单纯就为了将waypoint所在Lane(以及同向车道)分割成两部分子TopoNode
    // 从而获得搜索起点start和搜索终点end
    black_list_generator_->AddBlackMapFromTerminal(
        way_start, way_end, way_start_s, way_end_s, &full_range_manager);

    // Note: 创建子图, 里面包含黑名单origin TopoNode的有效子TopoNode的信息和关联的TopoEdge信息
    // 子图与origin TopoNode是相关联的,
    // 子图里面的TopoNode都是黑名单里面那些valid range生成的TopoNode(称为子TopoNode或sub TopoNode)
    // 在创建子TopoEdge时, 根据sub TopoNode所在的origin TopoNode连接的所有的connected origin TopoNode来创建,
    // 如果connected origin TopoNode没有sub TopoNode,
    // 则直接创建sub TopoEdge连接sub TopoNode和connected origin TopoNode
    // 如果connected origin TopoNode有sub TopoNode,
    // 则判断每个sub TopoNode是否可能与当前的sub TopoNode连接上(符合变道或直行的要求)
    // 总的来说, SubTopoGraph包含了所有的sub TopoNode和所有的sub TopoEdge,
    // 这些sub TopoEdge连接的其中一端肯定是sub TopoNode, 另一端则有可能是origin/sub TopoNode
    // 子TopoEdge的信息被添加到子TopoNode中,
    // 但origin TopoNode不会添加子TopoEdge到InEdge或OutEdge信息中
    SubTopoGraph sub_graph(full_range_manager.RangeMap());
    // Note: 起点处的子TopoNode
    // 这个子TopoNode包含了起点waypoint的点
    const auto* start = sub_graph.GetSubNodeWithS(way_start, way_start_s);
    if (start == nullptr) {
      AERROR << "Sub graph node is nullptr, origin node id: "
             << way_start->LaneId() << ", s:" << way_start_s;
      return false;
    }
    // Note: 终点处的子TopoNode
    // 这个子TopoNode包含了终点处的waypoint
    const auto* end = sub_graph.GetSubNodeWithS(way_end, way_end_s);
    if (end == nullptr) {
      AERROR << "Sub graph node is nullptr, origin node id: "
             << way_end->LaneId() << ", s:" << way_end_s;
      return false;
    }

    std::vector<NodeWithRange> cur_result_nodes;
    if (!strategy_ptr->Search(graph, &sub_graph, start, end,
                              &cur_result_nodes)) {
      AERROR << "Failed to search route with waypoint from " << start->LaneId()
             << " to " << end->LaneId();
      return false;
    }

    node_vec.insert(node_vec.end(), cur_result_nodes.begin(),
                    cur_result_nodes.end());
  }

  // Note: 合并多段waypoint之间的路由结果
  // Note: 这里把被middle waypoints分割的LaneSegment重新组合成一段LaneSegment了
  if (!MergeRoute(node_vec, result_nodes)) {
    AERROR << "Failed to merge route.";
    return false;
  }
  return true;
}

bool Navigator::SearchRoute(const RoutingRequest& request,
                            RoutingResponse* const response) {
  // Note: 打印routing request信息
  if (!ShowRequestInfo(request, graph_.get())) {
    SetErrorCode(ErrorCode::ROUTING_ERROR_REQUEST,
                 "Error encountered when reading request point!",
                 response->mutable_status());
    return false;
  }

  if (!IsReady()) {
    SetErrorCode(ErrorCode::ROUTING_ERROR_NOT_READY, "Navigator is not ready!",
                 response->mutable_status());
    return false;
  }
  std::vector<const TopoNode*> way_nodes;
  std::vector<double> way_s;
  // Note: 将routing request的waypoints信息转化为origin TopoNode和对应的s
  // 实际上就是去graph_里面去找waypoint的(Lane)id对应的TopoNode
  // 还添加了request指定的黑名单
  if (!Init(request, graph_.get(), &way_nodes, &way_s)) {
    SetErrorCode(ErrorCode::ROUTING_ERROR_NOT_READY,
                 "Failed to initialize navigator!", response->mutable_status());
    return false;
  }

  std::vector<NodeWithRange> result_nodes;
  if (!SearchRouteByStrategy(graph_.get(), way_nodes, way_s, &result_nodes)) {
    SetErrorCode(ErrorCode::ROUTING_ERROR_RESPONSE,
                 "Failed to find route with request!",
                 response->mutable_status());
    return false;
  }
  if (result_nodes.empty()) {
    SetErrorCode(ErrorCode::ROUTING_ERROR_RESPONSE, "Failed to result nodes!",
                 response->mutable_status());
    return false;
  }
  // 把起点和终点的前后1cm的多余距离抹除了
  result_nodes.front().SetStartS(request.waypoint().begin()->s());
  result_nodes.back().SetEndS(request.waypoint().rbegin()->s());

  // Note: result_nodes中的NodeWithRange的TopoNode都用的是原TopoGraph中的origin TopoNode

  // Note: 注意，这里传入的是routing request中黑名单建立的黑名单管理器
  if (!result_generator_->GeneratePassageRegion(
          graph_->MapVersion(), request, result_nodes, topo_range_manager_,
          response)) {
    SetErrorCode(ErrorCode::ROUTING_ERROR_RESPONSE,
                 "Failed to generate passage regions based on result lanes",
                 response->mutable_status());
    return false;
  }
  SetErrorCode(ErrorCode::OK, "Success!", response->mutable_status());

  PrintDebugData(result_nodes);
  return true;
}

}  // namespace routing
}  // namespace apollo
