#include "cpd_base.h"

#include <fstream>
#include <stdexcept>
#include <cassert>

// compile with -O3 -DNDEBUG

void CPDBASE::append_row(
  int source_node, const vector<unsigned short>&allowed_first_move,
  const Mapper& mapper, const int side) {

  auto get_allowed_local = [&](int x) {
    return get_allowed(x, source_node, side, allowed_first_move, mapper);
  };

  int node_begin = 0;
  unsigned short allowed_up_to_now = get_allowed_local(0);

  for(int i=1; i<(int)allowed_first_move.size(); ++i){
    int allowed_next = allowed_up_to_now & get_allowed_local(i);
    if(allowed_next == 0){
      entry.push_back((node_begin << 4) | find_first_allowed_out_arc(allowed_up_to_now));
      node_begin = i;
      allowed_up_to_now = get_allowed_local(i);
    }else
      allowed_up_to_now = allowed_next;
  }
  entry.push_back((node_begin << 4) | find_first_allowed_out_arc(allowed_up_to_now));

  begin.push_back(entry.size());
}

void CPDBASE::append_rows(const CPDBASE&other){
  int offset = begin.back();
  for(auto x:make_range(other.begin.begin()+1, other.begin.end()))
    begin.push_back(x + offset);
  std::copy(other.entry.begin(), other.entry.end(), back_inserter(entry));
}

void CPDBASE::append_compressed_cpd_row(vector<int> compressed_row) {
  std::copy(compressed_row.begin(), compressed_row.end(), back_inserter(entry));
  begin.push_back(entry.size());
}

vector<int>::const_iterator CPDBASE::get_first_iter(int lhs, int rhs, int t) const {
  t <<= 4;
  t |= 0xF;
  return binary_find_last_true(
      entry.begin() + lhs,
      entry.begin() + rhs,
      [=](int x){return x <= t;}
  );
}

vector<int>::const_iterator CPDBASE::get_interval(int s, int t, int& lhs, int& rhs, int& move,
    vector<int>::const_iterator pre, const Mapper& mapper) const {
  vector<int>::const_iterator it;
  if (pre == entry.end()) {
    it = get_first_iter(begin[s], begin[s+1], t);
  }
  else if (t > rhs) {
    int lb = pre - entry.begin();
    it = get_first_iter(lb+1, begin[s+1], t);
  }
  else if (t < lhs) {
    int ub = pre - entry.begin();
    it = get_first_iter(begin[s], ub, t);
  }
  else {
    return pre;
  }

  lhs = (*it) >> 4;
  if (std::next(it) == entry.end() || std::next(it) == entry.begin() + begin[s+1])
    rhs = mapper.node_count();
  else
    rhs = ((*std::next(it))>>4)-1;
  move = (*it)&0xF;
  return it;
}

bool CPDBASE::is_in_square(int x, int side, int source_node, const Mapper& mapper) const {
  xyLoc loc_source = mapper.operator()(source_node);
  xyLoc loc_x = mapper.operator()(x);
  int dx = abs((int)loc_source.x - (int)loc_x.x);
  int dy = abs((int)loc_source.y- (int)loc_x.y);
  assert(side >= 1);
  if((dx <= (side-1)/2)&&(dy<= (side-1)/2))
  {
    return true;
  }
  return false;
};

int CPDBASE::get_allowed (
    int x, int s, int side,
    const vector<unsigned short>& fmoves,
    const Mapper& mapper) const {
  if(x == s)
    return warthog::ALLMOVE;
  else if(is_in_square(x, side, s, mapper))
    return warthog::ALLMOVE;
  else if(fmoves[x] == 0)
    return warthog::NOMOVE;
  else
    return fmoves[x];
}
