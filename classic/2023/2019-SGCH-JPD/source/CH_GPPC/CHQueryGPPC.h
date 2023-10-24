#ifndef APPS_SUBGOALGRAPH_METHODMANAGERS_CHQUERYGPPC_H_
#define APPS_SUBGOALGRAPH_METHODMANAGERS_CHQUERYGPPC_H_

#define USE_STALL_ON_DEMAND
//#define USE_NODE_REORDER
//#define QUICK_CONTRACT

#include <algorithm>
#include <cstdio>
#include <vector>

#include "adj_graph.h"
#include "arc_gppc.h"
#include "bidirectional_dijkstra_gppc.h"
#include "contraction_hierarchy.h"
#include "dijkstra_gppc.h"
#include "mapper.h"
#include "range_gppc.h"
#include "timestamp_flags.h"


namespace ch_gppc {

class CHQueryGPPC {

  Mapper mapper;
  AdjGraph<ShortcutArcHeadGPPC> search_graph;
  BidirectionalDijkstraGPPC<TimestampFlags, AdjGraph<ShortcutArcHeadGPPC>,
      AdjGraph<ShortcutArcHeadGPPC>>*bidij;
  std::string filename_;

 public:
  typedef SgchXyLoc State;
  CHQueryGPPC(const std::vector<bool> &bits, int width, int height) {
    bidij = NULL;
    mapper = Mapper(bits, width, height);
  }
  ~CHQueryGPPC() {
    if (bidij != NULL)
      delete bidij;
  }

  AdjGraph<ShortcutArcHeadGPPC>* GetOriginalGraph() {
    return &search_graph;
  }

  void load(std::string preprocess_file) {
    search_graph = AdjGraph<ShortcutArcHeadGPPC>();
    std::FILE *file = fopen(preprocess_file.c_str(), "rb");
    search_graph.load(file);
    fclose(file);
    bidij = new BidirectionalDijkstraGPPC<TimestampFlags,
        AdjGraph<ShortcutArcHeadGPPC>, AdjGraph<ShortcutArcHeadGPPC>>(search_graph,
                                                              search_graph);
  }

  void Preprocess(std::string preprocess_file) {
    //printf("width = %d, height = %d\n", width, height);
    auto arc_list = extract_graph(mapper);
    //printf("node count: %d, arc count: %d\n", mapper.node_count(),
    //       arc_list.size());

    const int node_count = mapper.node_count();
    std::vector<ShortcutArcGPPC> search_arc_list;
    auto order = compute_online_contraction_order(
        node_count, arc_list, 100000000, [&](WeightedArcGPPC a, int m) {
          // T: On new forward arc: add to search_arc_list
        search_arc_list.push_back( {a.source, a.target, a.weight, m});
      },
        [&](WeightedArcGPPC, int) {}
        // T: On new backward arc: do nothing (This might be because of undirected graphs?)
        );

#ifdef USE_NODE_REORDER
    auto rank = inverse_permutation(order);

    for(auto&x:search_arc_list) {
      x.source = rank[x.source];
      x.target = rank[x.target];
      if(x.mid != -1)
      x.mid = rank[x.mid];
    }
#endif

    search_graph = AdjGraph<ShortcutArcHeadGPPC>(node_count, search_arc_list);

    std::FILE *file = fopen(preprocess_file.c_str(), "wb");
    search_graph.save(file);
    fclose(file);

    bidij = new BidirectionalDijkstraGPPC<TimestampFlags,
        AdjGraph<ShortcutArcHeadGPPC>, AdjGraph<ShortcutArcHeadGPPC>>(search_graph,
                                                              search_graph);

    //printf("search graph arc count = %d\n", (int) search_graph.arc_count());
  }

  Distance FindPath(xyLoc s, xyLoc t, std::vector<xyLoc> & path) {
    int source_node = mapper(SgchXyLoc(s.x, s.y));
    int target_node = mapper(SgchXyLoc(t.x, t.y));

    auto get_forward_weight = [&](ShortcutArcGPPC a) {
      return a.weight;
    };

    auto get_backward_weight = [&](ShortcutArcGPPC a) {
      return a.weight;
    };

#ifndef USE_STALL_ON_DEMAND
    auto should_forward_search_continue = [&](int v) {
      return true;
    };

    auto should_backward_search_continue = [&](int v) {
      return true;
    };
#else
    auto should_forward_search_continue =
        [&](int v) {
          for(auto a:search_graph.out(v)) {
            if(bidij->forward.extract_distance(a.target) < bidij->forward.extract_distance(v) - a.weight) {
              return false;
            }
          }
          return true;
        };
    auto should_backward_search_continue =
        [&](int v) {
          for(auto a:search_graph.out(v)) {
            if(bidij->backward.extract_distance(a.target) < bidij->backward.extract_distance(v) - a.weight) {
              return false;
            }
          }
          return true;
        };
#endif

    bidij->start(source_node, target_node);
    while (bidij->get_current_shortest_path_distance()
        > bidij->forward.get_current_radius()
        || bidij->get_current_shortest_path_distance()
            > bidij->backward.get_current_radius()) {
      bidij->settle_next(get_forward_weight, get_backward_weight,
                         should_forward_search_continue,
                         should_backward_search_continue);
    }

    auto up_down_path = bidij->get_current_shortest_path();


    if (up_down_path.empty())
      return kMaxDistance;

    auto get_mid = [&](int x, int y) {
#ifdef USE_NODE_REORDER
        if(y < x)
        std::swap(x, y);
        for(auto h:state->search_graph.out(x))
        if(h.target == y)
        return h.mid;
#else
        for(auto h:search_graph.out(x))
        if(h.target == y)
        return h.mid;
        for(auto h:search_graph.out(y))
        if(h.target == x)
        return h.mid;
#endif
        return -1;
      };

    while (up_down_path.size() != 1) {
      int last = up_down_path[up_down_path.size() - 1];
      int second_last = up_down_path[up_down_path.size() - 2];
      auto mid = get_mid(second_last, last);
      if (mid == -1) {
        up_down_path.pop_back();
        SgchXyLoc place = mapper(last);
        path.push_back((xyLoc) {place.x, place.y});
      } else {
        up_down_path.back() = mid;
        up_down_path.push_back(last);
      }
    }

    path.push_back(s);

    std::reverse(path.begin(), path.end());

    return bidij->get_current_shortest_path_distance()/1000.0;
  }

};
}
#endif /* APPS_SUBGOALGRAPH_METHODMANAGERS_CHQUERYGPPC_H_ */
