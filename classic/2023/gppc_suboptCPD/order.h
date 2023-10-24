#pragma once
#include "list_graph.h"
#include "adj_array.h"
#include "adj_graph.h"
#include "vec_io.h"
#include <cassert>
#include <string>
#include <utility>
#include <stdexcept>
#include <stack>

class Mapper;

class NodeOrdering{
public:
  NodeOrdering(){}
  explicit NodeOrdering(int node_count):to_new_array(node_count, -1), to_old_array(node_count, -1){}

  void clear(){
    std::fill(to_new_array.begin(), to_new_array.end(), -1);
    std::fill(to_old_array.begin(), to_old_array.end(), -1);
  }

  int node_count()const{
    return to_new_array.size();
  }

  int to_new(int x)const{return to_new_array[x];}
  int to_old(int x)const{return to_old_array[x];}

  void map(int old_id, int new_id){
    assert(old_id != -1);
    assert(new_id != -1);

    if(to_new_array[old_id] != -1)
      to_old_array[to_new_array[old_id]] = -1;
    if(to_old_array[new_id] != -1)
      to_new_array[to_old_array[new_id]] = -1;
    to_new_array[old_id] = new_id;
    to_old_array[new_id] = old_id;
  }

  bool validate() const {
    std::vector<bool> vis(node_count());
    for (int i=0; i<node_count(); i++) {
      if (to_old_array[i] == -1) return false;
      if (vis[to_old_array[i]]) return false;
      vis[to_old_array[i]] = true;
    }
    for (int i=0; i<node_count(); i++) {
      int new_id = to_new(i);
      if (to_old(new_id) != i) return false;
    }
    return true;
  }

  bool is_complete()const{
    return std::find(to_new_array.begin(), to_new_array.end(), -1) == to_new_array.end();
  }

  void sort_range(int new_id_begin, int new_id_end){
    std::sort(
      to_old_array.begin() + new_id_begin, 
      to_old_array.begin() + new_id_end
    );
    for(int i=new_id_begin; i<new_id_end; ++i)
      to_new_array[to_old_array[i]] = i;
  }
  
  bool next_range_permutation(int new_id_begin, int new_id_end){
    bool ret = std::next_permutation(
      to_old_array.begin() + new_id_begin, 
      to_old_array.begin() + new_id_end
    );
    for(int i=new_id_begin; i<new_id_end; ++i)
      to_new_array[to_old_array[i]] = i;
    return ret;
  }

  std::vector<int>store_range(int new_id_begin, int new_id_end){
    return std::vector<int>(
      to_old_array.begin() + new_id_begin, 
      to_old_array.begin() + new_id_end
    );
  }

  void load_range(int new_id_begin, const std::vector<int>&v){
    std::copy(v.begin(), v.end(), to_old_array.begin() + new_id_begin);
    for(int i=new_id_begin; i<new_id_begin+(int)v.size(); ++i)
      to_new_array[to_old_array[i]] = i;
  }

  void save(std::FILE*f)const{
    save_vector(f, to_new_array);
    save_vector(f, to_old_array);
  }

  void load(std::FILE*f){
    to_new_array = load_vector<int>(f);
    to_old_array = load_vector<int>(f);
  }

  friend bool operator==(const NodeOrdering&l, const NodeOrdering&r){
    return l.to_new_array == r.to_new_array && l.to_old_array == r.to_old_array;
  }

  void check_for_errors(){
    for(auto x:to_new_array){
      if(x < 0)
        throw std::runtime_error("to_new_array may not have 0 entries");
      if(x >= node_count())
        throw std::runtime_error("to_new_array may not have entries >= node_count");
    }
    for(auto x:to_old_array){
      if(x < 0)
        throw std::runtime_error("to_new_array may not have 0 entries");
      if(x >= node_count())
        throw std::runtime_error("to_new_array may not have entries >= node_count");
    }
    for(int i=0; i<node_count(); ++i){
      if(to_old_array[to_new_array[i]] != i)
        throw std::runtime_error("node ordering is no bijection");
    }
  }

  std::vector<int> get_old_array(){
    return to_old_array;
  }

private:
  std::vector<int>to_new_array, to_old_array;
};

inline
bool operator!=(const NodeOrdering&l, const NodeOrdering&r){
  return !(l == r);
}
inline NodeOrdering compute_real_dfs_order(const ListGraph&g){

  std::vector<int>out_begin, out_dest;
  auto source_node = [&](int x){
    return g.arc[x].source;
  };

  auto target_node = [&](int x){
    return g.arc[x].target;
  };

  build_adj_array(
    out_begin, out_dest, 
    g.node_count(), g.arc.size(),
    source_node, target_node
  );

  NodeOrdering order(g.node_count());

  std::vector<bool>was_pushed(g.node_count(), false);
  std::vector<bool>was_popped(g.node_count(), false);
  std::vector<int>next_out = out_begin;
  std::stack<int>to_visit;
  int next_id = 0;
  for(int source_node=0; source_node<g.node_count(); ++source_node){
    if(!was_pushed[source_node]){

      to_visit.push(source_node);
      was_pushed[source_node] = true;

      while(!to_visit.empty()){
        int x = to_visit.top();
        to_visit.pop();
        if(!was_popped[x]){
          order.map(x, next_id++);
          was_popped[x] = true;
        }

        while(next_out[x] != out_begin[x+1]){
          int y = out_dest[next_out[x]];
          if(was_pushed[y])
            ++next_out[x];
          else{
            was_pushed[y] = true;
            to_visit.push(x);
            to_visit.push(y);
            ++next_out[x];
            break;
          }
        }
      }
    }
  }
  assert(order.is_complete());
  return order;
}
