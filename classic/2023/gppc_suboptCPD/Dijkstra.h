#pragma once

#include <algorithm>
#include <vector>
#include <limits>

#include "adj_graph.h"
#include "heap.h"
#include "mapper.h"
#include "constants.h"
#include "Hsymbol.h"
#include "square_wildcard.h"

using namespace std;

namespace H = Hsymbol;

class Dijkstra{
public:
  Dijkstra(const AdjGraph&g, const Mapper& mapper):
    g(g), q(g.node_count()), dist(g.node_count()), allowed(g.node_count()), 
    inv_allowed(g.node_count()), mapper(mapper){}

  const std::vector<unsigned short>&run(int source_node, int hLevel){
    std::fill(dist.begin(), dist.end(), std::numeric_limits<int>::max());
    std::fill(allowed.begin(), allowed.end(), 0);
    fill(inv_allowed.begin(), inv_allowed.end(), 0);

    dist[source_node] = 0;
    allowed[source_node] = 0;
    inv_allowed[source_node] = warthog::ALLMOVE;

    auto reach = [&](const OutArc& a, int d, int fmove, int inv_fmove){
      int v = a.target;
      if(d < dist[v]){
        q.push_or_decrease_key(v, d);
        dist[v] = d;
        allowed[v] = fmove;
        inv_allowed[v] = inv_fmove;
      }else if(d == dist[v]){
        allowed[v] |= fmove;
        inv_allowed[v] |= inv_fmove;
      }
    };

    for(int i=0; i<g.out_deg(source_node); ++i){
      auto a = g.out(source_node, i);
      reach(a, a.weight, 1<<a.direction, 1<<(warthog::INV_MOVE[a.direction]));
    }

    while(!q.empty()){
      int x = q.pop();

      int ds = inv_allowed[x];
      uint32_t neighbors = warthog::ALLMOVE;
      while (ds > 0) {
        int lowb = warthog::lowb(ds);
        int d = 1 << warthog::INV_MOVE[warthog::m2i.at(lowb)];
        neighbors &= warthog::jps::compute_successors((warthog::jps::direction)d, mapper.get_jps_tiles(x));
        ds -= lowb;
      }

      for(auto a:g.out(x)) {
        if (neighbors & (1 << a.direction))
        reach(a, dist[x] + a.weight, allowed[x], 1 << (warthog::INV_MOVE[a.direction]));
      }
    }

    if (hLevel) {
      H::encode(source_node, allowed, mapper, hLevel);
      H::encode_inv(source_node, inv_allowed, mapper, hLevel);
    }
    #ifndef NDEBUG
    for(int u=0; u<g.node_count(); ++u)
      for(auto uv : g.out(u)){
        int v = uv.target;
        assert(dist[u] >= dist[v] - uv.weight);
      }
    #endif
    return allowed;
  }

  int distance(int target)const{
    return dist[target];
  }

  int get_directions(int t) {
    return allowed[t];
  }

  int get_inv_directions(int t) {
    return inv_allowed[t];
  }

  const vector<unsigned short>& get_allowed() const {
    return allowed;
  }

  const vector<unsigned short>& get_inv_allowed() const {
    return inv_allowed;
  }

  const std::vector<unsigned short>& run(int source_node, int hLevel, vector<int>&square_side){
    allowed = run(source_node, hLevel);
    int side = SquareWildcard(mapper, mapper(source_node)).computeMaxSquare(allowed);
    square_side.push_back(side);
    H::add_extr_inv_move(source_node, inv_allowed, mapper);
    return allowed;
  }

  const vector<unsigned short>& run_extra(int source_node, int hLevel) {
    allowed = run(source_node, hLevel);
    H::add_extr_inv_move(source_node, inv_allowed, mapper);
    return allowed;
  }

  const vector<int>& get_dist() const { return dist; }

private:
  const AdjGraph&g;
  min_id_heap<int>q;
  std::vector<int>dist;
  std::vector<unsigned short>allowed;
  std::vector<unsigned short>inv_allowed;
  const Mapper& mapper;
};
