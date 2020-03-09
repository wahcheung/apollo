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

#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "cyber/common/log.h"
#include "modules/routing/graph/topo_node.h"

namespace apollo {
namespace routing {

class TopoGraph {
 public:
  TopoGraph() = default;
  ~TopoGraph() = default;

  bool LoadGraph(const Graph& filename);

  const std::string& MapVersion() const;
  const std::string& MapDistrict() const;
  const TopoNode* GetNode(const std::string& id) const;
  void GetNodesByRoadId(
      const std::string& road_id,
      std::unordered_set<const TopoNode*>* const node_in_road) const;

 private:
  void Clear();
  bool LoadNodes(const Graph& graph);
  bool LoadEdges(const Graph& graph);

 private:
  std::string map_version_;
  std::string map_district_;
  // Note: 存放所有从routing_map加载来的TopoNode
  // lane_id对应的TopoNode的下标可以通过lane_id查表node_index_map_得到
  std::vector<std::shared_ptr<TopoNode> > topo_nodes_;
  // Note: TopoEdge列表
  std::vector<std::shared_ptr<TopoEdge> > topo_edges_;
  // Note: lane_id对应的TopoNode在topo_nodes_中的下标
  std::unordered_map<std::string, int> node_index_map_;
  // Note: road_id对应的TopoNode的集合
  std::unordered_map<std::string, std::unordered_set<const TopoNode*> >
      road_node_map_;
};

}  // namespace routing
}  // namespace apollo
