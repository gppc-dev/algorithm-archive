#pragma once
#include <stdint.h>
#include "mapper.h"
using namespace std;
using namespace GPPC;
const uint32_t MASK = ((uint32_t)1<<31) - 1;
//#define signbit(x) (((x) >> 31) - (-(x) >> 31))
//#define iabs(x) ( (((x) >> 31) ^ (x)) - (((x) >> 31)) )
//#define iabs2(x) (((x) & MASK) - ((x) >> 31))
namespace Hsymbol {
/*            y+ 
 *
 *            S
 *
 *            |
 *         3  |  0
 * x- W ------|-------- E x+
 *         2  |  1
 *            |
 *
 *            N
 *
 *            y-
 * get quadrant: quadrant[dx+1][dy+1]
 */
const static int quadrant[3][3] = {
      {2, 3, 3},
      {2, -1, 0},
      {1, 1, 0}
};

const static int qDirection[3][4]  = {
// q0, q1, q2, q3
  {2,  2,  3,  3},// go x
  {1,  0,  0,  1},// go y
  {6,  4,  5,  7},// go diag
};

static inline void avoid_corner_cutting(const xyLoc& sloc, const Mapper& mapper, int& dx, int& dy) {
  xyLoc p1 = xyLoc{sloc.x, static_cast<int16_t>(sloc.y+dy)};
  xyLoc p2 = xyLoc{static_cast<int16_t>(sloc.x+dx), sloc.y};
  if (mapper(p1) == -1) dy = 0;
  if (mapper(p2) == -1) dx = 0;
}

static inline int closestDirection(int dx, int dy) {
  int quad = quadrant[signbit(dx) + 1][signbit(dy) + 1];
  bool gox = iabs(dx) > (iabs(dy) << 1);
  bool goy = iabs(dy) > (iabs(dx) << 1);
  bool diag = (!gox) && (!goy);
  int direct;
  switch ((diag << 2) | (goy << 1) | gox) {
    case 1: direct = qDirection[0][quad];
               break;
    case 1<<1: direct = qDirection[1][quad];
               break;
    case 1<<2 : direct = qDirection[2][quad];
                break;
    default: direct = - 1;
             break;
  };
  return direct;
}

static inline int get_coord_part(int x, int y) {
  bool p0 = (iabs(x) << 1) <= iabs(y);
  bool p1 = ((iabs(x) << 1) > iabs(y)) && (iabs(x) <= iabs(y));
  bool p2 = (iabs(x) > iabs(y)) && ((iabs(y) << 1) >= iabs(x));
  bool p3 = (iabs(y) << 1) < iabs(x);
  //bool p0 = iabs(x) == 0;
  //bool p1 = (iabs(x) > 0) & (iabs(x) <= iabs(y));
  //bool p2 = (iabs(y) > 0) & (iabs(x) > iabs(y));
  //bool p3 = iabs(y) == 0;
  int res = -1;
  switch ( p0 | (p1 << 1) | (p2 << 2) | (p3 << 3)) {
    case 1<<0: res = 0;
               break;
    case 1<<1: res = 1;
               break;
    case 1<<2: res = 2;
               break;
    case 1<<3: res = 3;
               break;
    default: res = -1;
             break;
  };
  return res;
}

static inline int get_heuristic_move2(int s, int t, const Mapper& mapper) {
    double min_cost = warthog::INF;
    int hmove = warthog::INVALID_MOVE;
    xyLoc sloc = mapper(s);
    xyLoc tloc = mapper(t);

    for (int d=0; d<8; d++) {
      xyLoc nxt = xyLoc{(int16_t)(sloc.x + warthog::dx[d]), (int16_t)(sloc.y + warthog::dy[d])};
      xyLoc p1 = xyLoc{sloc.x, (int16_t)(sloc.y + warthog::dy[d])};
      xyLoc p2 = xyLoc{(int16_t)(sloc.x + warthog::dx[d]), sloc.y};
      // avoid corner cutting
      if (mapper(nxt) == -1 || mapper(p1) == -1 || mapper(p2) == -1) 
        continue;
      //int common = min(abs(tloc.x - nxt.x), abs(tloc.y - nxt.y));
      //double cost = common * warthog::DBL_ROOT_TWO + abs(tloc.x - nxt.x) + abs(tloc.y - nxt.y) - 2.0 * common + warthog::doublew[d];
      double cost = sqrt((tloc.x - nxt.x) * (tloc.x - nxt.x) + (tloc.y - nxt.y) * (tloc.y - nxt.y)) + warthog::doublew[d];
      if (cost < min_cost) {
        min_cost = cost;
        hmove = d;
      }
    }
    return hmove;
}

static inline int get_heuristic_move1(int s, int t, const Mapper& mapper) {
  xyLoc sloc = mapper(s);
  xyLoc tloc = mapper(t);
  int dx = signbit(tloc.x - sloc.x);
  int dy = signbit(tloc.y - sloc.y);
  avoid_corner_cutting(sloc, mapper, dx, dy);
  return warthog::v2i[dx+1][dy+1];
}

static inline int get_heuristic_move3(int s, int t, const Mapper& mapper) {
  int move = warthog::INVALID_MOVE;
  xyLoc sloc = mapper(s);
  xyLoc tloc = mapper(t);
  int dx = (int)tloc.x - (int)sloc.x;
  int dy = (int)tloc.y - (int)sloc.y;
  if (iabs(dx) <= 1 && iabs(dy) <= 1 && (mapper.get_neighbor(s) & (1<<warthog::v2i[dx+1][dy+1])))
    return warthog::v2i[dx+1][dy+1];
  int quad = quadrant[signbit(dx) + 1][signbit(dy) + 1];
  int part = get_coord_part(dx, dy);
  //int no_diagonal= (dx == 0) | (dy == 0);
  int no_diagnonal = 1;
  move = mapper.get_valid_move(s, quad, part, no_diagnonal);
  return move;
}


static inline int get_heuristic_move(int s, int t, const Mapper& mapper, int hLevel) {
  int move = warthog::INVALID_MOVE;
  switch (hLevel) {
    case 1: move = get_heuristic_move1(s, t, mapper);
            break;
    case 2: move = get_heuristic_move2(s, t, mapper);
            break;
    case 3: move = get_heuristic_move3(s, t, mapper);
            break;
    default:
            cerr << "undefined hlevel" << endl;
            break;
  }
  return move;
}

static inline void encode(const int source, vector<unsigned short>& allowed, const Mapper& mapper,
    int hLevel) {
  for (int v=0; v<(int)allowed.size(); v++) if (v != source) {
    int hmove = get_heuristic_move(source, v, mapper, hLevel);
    if (allowed[v] & (1 << hmove))
      allowed[v] |= warthog::HMASK;
  }
}

static inline void encode_inv(const int target, vector<unsigned short>& inv_allowed, const Mapper& mapper,
    int hLevel) {
  for (int v=0; v<(int)inv_allowed.size(); v++) if (v != target) {
    int hmove = get_heuristic_move(v, target, mapper, hLevel);
    if (inv_allowed[v] & (1<<hmove))
      inv_allowed[v] |= warthog::HMASK;
  }
}

static inline int decode(int s, int t, const Mapper& mapper, int hLevel) {
  return get_heuristic_move(s, t, mapper, hLevel);
}

static inline int decode(int s, int t, const Mapper& mapper, int (*func) (int, int, const Mapper&) ) {
  return func(s, t, mapper);
}

static inline int decode_extr_move(xyLoc s, xyLoc t, int move, const Mapper& mapper) {
  const int* order;
  if (t.y > s.y) order = warthog::CCW;
  else order = warthog::CW;
  int begin = -1;
  for (int i=0; i<8; i++) if (order[i] == move) {
    begin = i;
    break;
  }
  for (int i=1; i<8; i++) {
    int m = order[(begin + i) % 8];
    int nxtx = s.x + warthog::dx[m];
    int nxty = s.y + warthog::dy[m];
    int id = mapper(xyLoc{(int16_t)nxtx, (int16_t)nxty});
    if (id == -1) continue;
    else return m;
  }
  return move;
}


static inline int get_closest_valid_move(int source, int move, const Mapper& mapper) {
  int pruned = mapper.get_pruned_neighbor(source);
  if (pruned & (1<<move)) return move;
  int idx = -1;
  for (int i=0; i<8; i++) if (warthog::CCW[i] == move) {
    idx = i;
    break;
  }
  assert(idx != -1);
  if (idx == -1)
    cerr << "can't find move" << endl;
  for (int i=1; i<=4; i++) {
    int m1, m2;
    m1 = warthog::CCW[(idx+i) % 8];
    if ((pruned & (1<<m1))) return m1;
    m2 = warthog::CCW[(idx-i+8) % 8];
    if ((pruned & (1<<m2))) return m2;
  }
  return -1;
}


static inline void add_extr_inv_move(int target, vector<unsigned short>& inv_allowed, const Mapper& mapper) {
  for (int s=0; s<(int)inv_allowed.size(); s++) if (s != target) {
    int pruned = mapper.get_pruned_neighbor(s);
    int extra_mask = 0;
    if (pruned == 0) continue;
    for (int j=0; j<8; j++) if (!((1<<j) & pruned)) {
      int m = get_closest_valid_move(s, j, mapper);
      assert(m != -1);
      if ((1<<m) & inv_allowed[s]) extra_mask |= 1<<j;
    }
    inv_allowed[s] |= extra_mask;
  }
}

};
