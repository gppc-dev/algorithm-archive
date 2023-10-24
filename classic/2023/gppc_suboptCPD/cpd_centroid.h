#pragma once
#include "cpd_base.h"
using namespace std;

class CPD_CENTROID: public CPDBASE {
  public:
  void append_row_forward(
    int source_node, const vector<unsigned short>&first_move,
    const Mapper& mapper, const int side) {

    auto get_allowed_local = [&](int x){
      return get_allowed_forward(x, source_node, side, first_move, mapper);
    };

    int node_begin = 0;
    unsigned short allowed_up_to_now = get_allowed_local(0);

    for(int i=1; i<(int)first_move.size(); ++i){
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

  inline void append_row_inv(
    int s, const vector<unsigned short>& first_move,
    const Mapper& mapper) {
    this->append_row(s, first_move, mapper, 0);
  }

  int get_allowed_forward(int x, int s, int side, const vector<unsigned short>& fmoves, const Mapper& mapper) {
    if(x == s)
      return warthog::ALLMOVE;
    else if(fmoves[x] == 0)
      return warthog::NOMOVE;
    else if(is_in_square(x, side, s, mapper))
      return warthog::ALLMOVE;
    // if x is not a centroid, ignore
    else if (mapper.get_fa()[x] != x)
      return warthog::ALLMOVE;
    else
      return fmoves[x];
  }
};
