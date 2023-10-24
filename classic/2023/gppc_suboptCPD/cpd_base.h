#pragma once
#include <vector>
#include <algorithm>
#include <string>
#include "adj_graph.h"
#include "binary_search.h"
#include "range.h"
#include "vec_io.h"
#include "mapper.h"
using namespace std;
//! Compressed Path database. Allows to quickly query the first out arc id of
//! any shortest source-target-path. There may be at most 15 outgoing arcs for
//! any node.
class CPDBASE {
public:
  CPDBASE():begin{0}{}

  //! Adds a new node s to the CPD. first_move should be an array that 
  //! maps every target node onto a 15-bit bitfield that has a bit set
  //! for every valid first move. get_first_move is free to return any of
  //! them.
  void append_row(int source_node, const std::vector<unsigned short>&first_move,
                  const Mapper& mapper, const int side);
  void append_rows(const CPDBASE&other);
  void append_compressed_cpd_row(vector<int> compressed_row);

  //! Get the first move. 
  //! An ID of 0xF means that there is no path. 
  //! If source_node == target_node then return value is undefined. 
  unsigned char get_first_move(int source_node, int target_node)const{
    assert(source_node != -1);
    assert(target_node != -1);
    target_node <<= 4;
    target_node |= 0xF;
    return *binary_find_last_true(
      entry.begin() + begin[source_node],
      entry.begin() + begin[source_node+1],
      [=](int x){return x <= target_node;}
    )&0xF;
  }

  vector<int>::const_iterator get_first_iter(int lhs, int rhs, int t) const;
  vector<int>::const_iterator get_interval(
      int s, int t, int& lhs, int& rhs, int& move,
      vector<int>::const_iterator pre, const Mapper& mapper) const;

  int node_count() const{
    // get the number of rows
    // in inverse centroid cpd, this returns the number of centroids.
    return begin.size()-1;
  }

  size_t entry_count()const{
    return entry.size();
  }

  friend bool operator==(const CPDBASE&l, const CPDBASE&r){
    return l.begin == r.begin && l.entry == r.entry;
  }

  friend bool operator!=(const CPDBASE&l, const CPDBASE&r){
    return !(l == r);
  }
  void save(std::FILE*f)const{
    save_vector(f, begin);
    save_vector(f, entry);
  }

  void load(std::FILE*f){
    begin = load_vector<int>(f);
    entry = load_vector<int>(f);
  }

  size_t get_entry_size() {
    return entry.size();
  }
  const vector<int>& get_entry() const {
    return entry;
  }

  const vector<int>& get_begin() const {
    return begin;
  }

  vector<int> get_ith_compressed_row(int i) {
    return vector<int>(entry.begin()+begin[i], entry.begin()+begin[i+1]);
  }

  int get_heuristic_cnt(int row) const {
    int hcnt = 0;
    for (int i=begin[row]; i<begin[row+1]; i++) {
      if ((1<<(entry[i]&0xF) == warthog::HMASK)) {
        int node_begin = entry[i]>>4;
        int node_end = i+1==begin[row+1]?node_count(): entry[i+1]>>4;
        hcnt += node_end - node_begin;
      }
    }
    return hcnt;
  }

  int get_centroid_cnt(int row) const {
    int ccnt = 0;
    for (int i=begin[row]; i<begin[row+1]; i++) {
      if ((1<<(entry[i]&0xF) == warthog::CENTMASK)) {
        int node_begin = entry[i]>>4;
        int node_end = i+1==begin[row+1]?node_count(): entry[i+1]>>4;
        ccnt += node_end - node_begin;
      }
    }
    return ccnt;
  }

  int get_heuristic_run(int row) const {
    int hrun= 0;
    for (int i=begin[row]; i<begin[row+1]; i++) {
      if ((1<<(entry[i]&0xF) == warthog::HMASK)) hrun++;
    }
    return hrun;
  }

  int get_centroid_run(int row) const {
    int crun= 0;
    for (int i=begin[row]; i<begin[row+1]; i++) {
      if ((1<<(entry[i]&0xF) == warthog::CENTMASK)) crun++;
    }
    return crun;
  }

  static unsigned short find_first_allowed_out_arc(unsigned short allowed) {
    return warthog::m2i.at(warthog::lowb(allowed));
  };

  bool is_in_square(
    int x, int side, int source_node,
    const Mapper& mapper) const;
 
protected:
  std::vector<int>begin;
  std::vector<int>entry;
 
  int get_allowed(
    int x, int s, int side,
    const vector<unsigned short>& fmoves,
    const Mapper& mapper) const;
};
