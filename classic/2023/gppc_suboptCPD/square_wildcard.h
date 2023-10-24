#pragma once
#include "mapper.h"

class SquareWildcard {
  const Mapper& mapper;
  const xyLoc& loc;

  public:
  SquareWildcard(const Mapper& m, const xyLoc& l):
    mapper(m), loc(l) {}

  inline bool check(int side, int& cnt, const vector<unsigned short>& allowed) {
    /*
     * 11112
     * 4...2
     * 4.c.2
     * 4...2
     * 43333
     */
    int x = (int)loc.x - side, y = (int)loc.y - side;
    xyLoc cur;
    while (x<loc.x+side) { // 1
      cur.x = x, cur.y = y;
      if (x<0 || x>=mapper.width() || y<0 || y>=mapper.height()) {x++; continue;}
      if (mapper(cur) == -1) {x++; continue;}
      if (! (allowed[mapper(cur)] & warthog::HMASK)) return false;
      x++;
      cnt++;
    }
    while (y<loc.y+side) { // 2
      cur.x = x, cur.y = y;
      if (x<0 || x>=mapper.width() || y<0 || y>=mapper.height()) {y++; continue;}
      if (mapper(cur) == -1) {y++; continue;}
      if (! (allowed[mapper(cur)] & warthog::HMASK)) return false;
      y++;
      cnt++;
    }
    while (x>loc.x-side) { // 3
      cur.x = x, cur.y = y;
      if (x<0 || x>=mapper.width() || y<0 || y>=mapper.height()) {x--; continue;}
      if (mapper(cur) == -1) {x--; continue;}
      if (! (allowed[mapper(cur)] & warthog::HMASK)) return false;
      x--;
      cnt++;
    }
    while (y>loc.y-side) { // 4
      cur.x = x, cur.y = y;
      if (x<0 || x>=mapper.width() || y<0 || y>=mapper.height()) {y--; continue;}
      if (mapper(cur) == -1) {y--; continue;}
      if (! (allowed[mapper(cur)] & warthog::HMASK)) return false;
      y--;
      cnt++;
    }
    return true;
  }

  int computeMaxSquare(const vector<unsigned short>& allowed) {
    int16_t side = 1;
    int cnt = 1;
    while (true) {
      if (check(side, cnt, allowed)) side++;
      else break;
      assert(cnt <= mapper.node_count());
      if (cnt == mapper.node_count()) break;
    }
    return (((int)side-1) << 1) + 1;
  }
};
