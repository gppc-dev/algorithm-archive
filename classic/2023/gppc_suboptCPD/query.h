#pragma once
#include <cassert>
#include <vector>
#include "cpd_base.h"
using namespace std;

struct Counter {
  int access_cnt;
  int steps;
  double pathcost;
};

struct Extracter {
  vector<xyLoc> nodes;
  // vector<int> vis;
  int steps;
  int scenid;
  int last;

  void init(int size) {
    nodes.resize(size);
    // vis.resize(size);
    // fill(vis.begin(), vis.end(), -1);
    scenid = -1;
    steps = 0;
  }

  void reset(int id) {
    steps = 0;
    scenid = id;
  }

  void add(xyLoc loc) {
    nodes[steps++] = xyLoc{loc.x, loc.y};
    assert(steps <= (int)nodes.size());
  }

  // void mark(int locid) {
  //   vis[locid] = scenid;
  // }
  //
  // inline bool isVis(int locid) {
  //   return vis[locid] == scenid;
  // }
};

struct EntryData {
  CPDBASE cpd;
  Mapper mapper;
  AdjGraph graph;
  vector<int> row_ordering;
  vector<int> square_sides;
  Counter c;
  Extracter e1, e2;
  int scenid;
  EntryData(){}
};

double GetInvCentroidCost(const EntryData& data, xyLoc s, xyLoc g, int hLevel, Counter& c,
    Extracter& e1, Extracter& e2, vector<xyLoc>& path, int limit=-1);
