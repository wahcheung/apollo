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

#include "modules/routing/graph/topo_range_manager.h"

#include <algorithm>
#include <utility>

#include "cyber/common/log.h"

namespace apollo {
namespace routing {
namespace {

// Note: 对topo_node的多个NodeSRange区间进行合并和拼接
// [1, 3]和[2, 4]会被合并成[1, 4]
// [1, 2]和[2, 3]会被拼接成[1, 3]
void merge_block_range(const TopoNode* topo_node,
                       const std::vector<NodeSRange>& origin_range,
                       std::vector<NodeSRange>* block_range) {
  std::vector<NodeSRange> sorted_origin_range(origin_range);
  // Note: NodeSRange重载了<运算符, 这里将NodeSRange按照start_s从小到大排序
  std::sort(sorted_origin_range.begin(), sorted_origin_range.end());
  size_t cur_index = 0;
  auto total_size = sorted_origin_range.size();
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

}  // namespace

const std::unordered_map<const TopoNode*, std::vector<NodeSRange>>&
TopoRangeManager::RangeMap() const {
  return range_map_;
}
const std::vector<NodeSRange>* TopoRangeManager::Find(
    const TopoNode* node) const {
  auto iter = range_map_.find(node);
  if (iter == range_map_.end()) {
    return nullptr;
  } else {
    return &(iter->second);
  }
}

void TopoRangeManager::PrintDebugInfo() const {
  for (const auto& map : range_map_) {
    for (const auto& range : map.second) {
      AINFO << "black lane id: " << map.first->LaneId()
            << ", start s: " << range.StartS() << ", end s: " << range.EndS();
    }
  }
}

void TopoRangeManager::Clear() { range_map_.clear(); }

// Note: 在black_list_range_generator中会通过这个接口添加LaneSegment黑名单
void TopoRangeManager::Add(const TopoNode* node, double start_s, double end_s) {
  NodeSRange range(start_s, end_s);
  range_map_[node].push_back(range);
}

// Note: 对黑名单区间进行排序, 然后合并区间,
// Note: 对黑名单NodeSRange区间会进行合并和拼接
// [1, 3]和[2, 4]会被合并成[1, 4]
// [1, 2]和[2, 3]会被拼接成[1, 3]
void TopoRangeManager::SortAndMerge() {
  for (auto& iter : range_map_) {
    std::vector<NodeSRange> merged_range_vec;
    merge_block_range(iter.first, iter.second, &merged_range_vec);
    iter.second.assign(merged_range_vec.begin(), merged_range_vec.end());
  }
}

}  // namespace routing
}  // namespace apollo
