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

#include "modules/map/proto/map.pb.h"
#include "modules/routing/proto/routing_config.pb.h"
#include "modules/routing/proto/topo_graph.pb.h"

namespace apollo {
namespace routing {

class GraphCreator {
 public:
  GraphCreator(const std::string& base_map_file_path,
               const std::string& dump_topo_file_path,
               const RoutingConfig& routing_conf);

  ~GraphCreator() = default;

  bool Create();

 private:
  void InitForbiddenLanes();
  std::string GetEdgeID(const std::string& from_id, const std::string& to_id);

  void AddEdge(
      const Node& from_node,
      const ::google::protobuf::RepeatedPtrField<hdmap::Id>& to_node_vec,
      const Edge::DirectionType& type);

  static bool IsValidUTurn(const hdmap::Lane& lane, const double radius);

 private:
  // Note: base_map的文件路径, base_map是创建Routing地图的信息来源
  std::string base_map_file_path_;
  // Note: Routing地图的输出路径
  std::string dump_topo_file_path_;
  // Note: base_map的内容
  hdmap::Map pbmap_;
  // Note: 最终要输出的Routing地图的内容
  Graph graph_;
  // Note: lane_id对应的Node的下标, 即Lane对应Node在graph_.node的下标
  std::unordered_map<std::string, int> node_index_map_;
  // Note: 从地图中直接获取的所有Lane的id及其对应的Road的id, <lane_id, road_id>
  // 这里包含所有的Lane(含禁止通行的非机动车道等)
  // 用于查询Lane所在的Road
  std::unordered_map<std::string, std::string> road_id_map_;
  // Note: 已经添加到graph_中的Edge的id, 用于避免重复添加Edge
  std::unordered_set<std::string> showed_edge_id_set_;
  // Note: Lane's type is not CITY_DRIVING which is not allowed for driving
  std::unordered_set<std::string> forbidden_lane_id_set_;

  // Note: Routing地图的配置信息, 主要是一些会影响Node和Edge的cost的配置
  const RoutingConfig& routing_conf_;
};

}  // namespace routing
}  // namespace apollo
