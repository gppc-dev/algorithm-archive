#include "query.h"
#include "Hsymbol.h"

double InvNodeToCentroid(const EntryData& data, xyLoc cent, xyLoc t, int hLevel, Counter& c, Extracter& e, int limit) {
  // cent is a centroid location
  int ranks = data.mapper.get_centroid_rank(data.mapper(cent));
  assert(ranks != -1);
  int (*heuristic_func)(int, int, const Mapper&);
  if (hLevel == 1)
    heuristic_func = Hsymbol::get_heuristic_move1;
  else if (hLevel == 2)
    heuristic_func = Hsymbol::get_heuristic_move2;
  else if (hLevel == 3)
    heuristic_func = Hsymbol::get_heuristic_move3;

  int curs = data.mapper(cent);
  int curt = data.mapper(t);
  int lhs = -1, rhs = -1, cur_move = -1, move = warthog::NOMOVE;
  const int16_t* dx = warthog::dx;
  const int16_t* dy = warthog::dy;
  double cost = 0.0;
  auto it = data.cpd.get_entry().end();
  auto to_next_pos = [&](xyLoc& source, xyLoc& target, int& sid, int& tid) {
    if (!(tid >= lhs && tid <= rhs)) {
      it = data.cpd.get_interval(ranks, tid, lhs, rhs, cur_move, it, data.mapper);
      c.access_cnt++;
    }
    move = cur_move;
    if ((1<<move) == warthog::NOMOVE) {
      e.steps = -1;
      return;
    }

    int pruned = data.mapper.get_pruned_neighbor(tid);
    if ((1<<move) == warthog::HMASK) {
      move = Hsymbol::decode(tid, sid, data.mapper, heuristic_func);
    } else if (!(pruned & (1 << move))) { // pseudo obstacle move 
      // if not move to target, then decode
      bool reached = (target.x + dx[move] == source.x && target.y + dy[move] == source.y);
      if (!reached || !((1<<move) & data.mapper.get_neighbor(tid)))
        move = Hsymbol::get_closest_valid_move(tid, move, data.mapper);
    }
    cost += warthog::doublew[move];
    target.x += dx[move];
    target.y += dy[move];
    e.add(target);
    tid = data.mapper(target);
  };

  e.last = curt;
  while (data.mapper.get_fa()[curt] != curs) {
    to_next_pos(cent, t, curs, curt);
    e.last = curt;
    if ((1<<move)== warthog::NOMOVE) {
      e.steps = -1;
      break;
    }
    c.steps++;
    if (limit != -1 && limit <= c.steps) break;
  }
  return cost;
}

double InvNodeToNodeInSameCentroid(const EntryData& data, Extracter& ea, Extracter& eb, 
    Counter& c, int hLevel=3) {
  xyLoc a = data.mapper(ea.last);
  xyLoc b = data.mapper(eb.last);
  int cura = ea.last;// cura2 = ea.last;
  int curb = eb.last;// curb2 = eb.last;
  int cid = data.mapper.get_fa()[cura];
  xyLoc cent = data.mapper(cid);
  assert(data.mapper.get_fa()[curb] == cid);
  int ranks = data.mapper.get_centroid_rank(cid);
  assert(ranks != -1);

  int (*heuristic_func)(int, int, const Mapper&);
  if (hLevel == 1)
    heuristic_func = Hsymbol::get_heuristic_move1;
  else if (hLevel == 2)
    heuristic_func = Hsymbol::get_heuristic_move2;
  else if (hLevel == 3)
    heuristic_func = Hsymbol::get_heuristic_move3;

  int move = warthog::NOMOVE;
  const int16_t* dx = warthog::dx;
  const int16_t* dy = warthog::dy;
  double cost = 0.0;
  // double cost2 = 0.0;

  auto to_next_pos = [&](xyLoc& target, int& tid, Extracter& e) {
    if (tid == cid) return;
    move = data.cpd.get_first_move(ranks, tid);
    if ((1<<move) == warthog::NOMOVE) return;

    int pruned = data.mapper.get_pruned_neighbor(tid);
    if ((1<<move) == warthog::HMASK) {
      move = Hsymbol::decode(tid, cid, data.mapper, heuristic_func);
    } else if (!(pruned & (1 << move))) { // pseudo obstacle move 
      // if not move to target, then decode
      bool reached = (target.x + dx[move] == cent.x && target.y + dy[move] == cent.y);
      if (!reached || !((1<<move) & data.mapper.get_neighbor(tid)))
        move = Hsymbol::get_closest_valid_move(tid, move, data.mapper);
    }
    cost += warthog::doublew[move];
    target.x += dx[move];
    target.y += dy[move];
    tid = data.mapper(target);
    e.last = tid;
    e.add(target);
  };

  xyLoc ta, tb;
  ta = a, tb = b;
  cura = data.mapper(ta), curb = data.mapper(tb);
  while (cura != curb) {
    to_next_pos(ta, cura, ea);
    if ((1<<move) == warthog::NOMOVE && cid != cura) {
      ea.steps = -1;
      break;
    }
    to_next_pos(tb, curb, eb);
    if ((1<<move) == warthog::NOMOVE && cid != curb) {
      eb.steps = -1;
      break;
    }
    c.steps++;
  }
  return cost;

  // while (cura2 != curb2) {
  //   move = Hsymbol::decode(cura2, curb2, data.mapper, 3);
  //   if ((1<<move)== warthog::NOMOVE) {
  //     cost2 = 1e10;
  //     break;
  //   }
  //   ta2.x += dx[move];
  //   ta2.y += dy[move];
  //   cost2 += warthog::doublew[move];
  //   cura2 = data.mapper(ta2);
  //   if (ea.isVis(cura2) || cost2 >= cost) {
  //     cost2 = 1e10;
  //     break;
  //   }
  //   ea.mark(cura2);
  // }
  // return min(cost, cost2);
}


double GetInvCentroidCost(const EntryData& data, xyLoc s, xyLoc g, int hLevel, Counter& c, 
    Extracter& eg, Extracter& es, vector<xyLoc>& path, int limit) {
  int gid = data.mapper(g);
  int cgid = data.mapper.get_fa()[gid];
  xyLoc cg = data.mapper(cgid);
  eg.last = gid;

  int sid = data.mapper(s);
  int csid = data.mapper.get_fa()[sid];
  xyLoc cs = data.mapper(csid);
  es.last = sid;

  double cost0 = 0, cost1 = 0;
  if (abs(s.x - cg.x) + abs(s.y - cg.y) < abs(g.x - cs.x) + abs(g.y - cs.y)) {
    cost0 = InvNodeToCentroid(data, cg, s, hLevel, c, es, limit);
    if (es.steps == -1) return 0;
  }
  else {
    cost0 = InvNodeToCentroid(data, cs, g, hLevel, c, eg, limit);
    if (eg.steps == -1) return 0;
  } 
  assert(cost0 <= warthog::EPS || data.mapper.get_fa()[es.last] == data.mapper.get_fa()[eg.last]);
  if (data.mapper.get_fa()[es.last] == data.mapper.get_fa()[eg.last]) { // if there is a path
    cost1 = InvNodeToNodeInSameCentroid(data, es, eg, c, hLevel);
    if (es.steps == -1 || eg.steps == -1) return 0;
  }

  path.push_back(s);
  for (int i=0; i<es.steps; i++) {
    if (es.nodes[i] == path.back()) continue;
    path.push_back(es.nodes[i]);
  }
  for (int i=eg.steps-1; i>=0; i--) {
    if (path.size() > 0 && path.back() == eg.nodes[i]) continue;
    path.push_back(eg.nodes[i]);
  }
  if (path.back() != g)
    path.push_back(g);
  return cost0 + cost1;
}
