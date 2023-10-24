#ifndef LIST_GRAPH_H
#define LIST_GRAPH_H
#include <vector>
#include <algorithm>
#include <string>
#include <cassert>
#include "range.h"

struct Arc{
  int source, target, weight, direction;
};

inline bool operator==(Arc l, Arc r){
  return l.source == r.source && l.target == r.target && l.direction == r.direction;
}

inline bool operator!=(Arc l, Arc r){
  return !(l == r);
}



struct ListGraph{
  ListGraph():n(0){}
  explicit ListGraph(int node_count):n(node_count){}

  std::vector<Arc>arc;

  int node_count()const{
    return n;
  }

  bool is_valid()const{
    bool ok = true;
    for(auto a:arc){
      ok &= (a.source != -1);
      ok &= (a.source >= 0);
      ok &= (a.source < node_count());
      ok &= (a.target != -1);
      ok &= (a.target >= 0);
      ok &= (a.target < node_count());
    }
    return ok;
  }

  int n;
};

inline bool operator==(const ListGraph&l, const ListGraph&r){
  return l.arc == r.arc && l.n == r.n;
}

inline bool operator!=(const ListGraph&l, const ListGraph&r){
  return !(l == r);
}
#endif
