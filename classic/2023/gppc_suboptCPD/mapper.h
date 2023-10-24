#pragma once
#include <vector>
#include <iostream>
#include "constants.h"
#include "list_graph.h"
#include "order.h"
#include "jps.h"
#include "GPPC.h"
using namespace std;
using namespace GPPC;

struct ClosestMove {
  int move[4][4][2];
};

class Mapper{
public:
  Mapper(){}
  Mapper(const std::vector<bool>&field, int width, int height):
    width_(width), 
    height_(height),
    node_count_(0),
    pos_to_node_(width*height, -1)
  {
    for(int y=0; y<height_; ++y)
      for(int x=0; x<width_; ++x)
        if(field[x+y*width_]){
          node_to_pos_.push_back({static_cast<int16_t>(x), static_cast<int16_t>(y)});
          pos_to_node_[x+y*width_] = node_count_++;
        }else{
          pos_to_node_[x+y*width_] = -1;
        }
    init_jps_tiles();
    init_neighbors();
    init_pruned_neighbors();
    initClosestMove();
  }

  int width()const{
    return width_;
  }

  int height()const{
    return height_;
  }

  int node_count()const{
    return node_count_;
  } 

  xyLoc operator()(int x)const{ return node_to_pos_[x]; }

  inline int operator()(const int& x, const int& y) const {
    if (x < 0 || x >= width_ || y < 0 || y >= height_)
      return -1;
    else
      return pos_to_node_[x + y*width_];
  }

  inline int operator()(const xyLoc& p) const {
    return (*this)(p.x, p.y);
  }

  void reorder(const NodeOrdering&order){
    for(auto&x:pos_to_node_){
      if(x != -1){
        x = order.to_new(x);
      }
    }
    std::vector<xyLoc>new_node_to_pos_(node_count_);
    std::vector<int> new_tiles(node_count_);
    std::vector<int> new_neighbors(node_count_);
    vector<int> new_pruned_neighbors(node_count_);
    vector<int> new_rank(centroids_rank.size());
    for(int new_node=0; new_node<node_count(); ++new_node){
      int old_node = order.to_old(new_node);
      new_node_to_pos_[new_node] = node_to_pos_[old_node];
      new_neighbors[new_node] = neighbors[old_node];
      new_tiles[new_node] = jps_tiles[old_node];
      new_pruned_neighbors[new_node] = pruned_neighbors[old_node];
    }
    new_node_to_pos_.swap(node_to_pos_);
    new_tiles.swap(jps_tiles);
    new_neighbors.swap(neighbors);
    new_pruned_neighbors.swap(pruned_neighbors);
  }

  int get_pseudo_obs(int s) const {
    int mask = 0;
    xyLoc sloc = this->operator()(s);
    for (int i=0; i<8; i++) if ((1<<i) & get_neighbor(s)) {
      int x = sloc.x + warthog::dx[i];
      int y = sloc.y + warthog::dy[i];
      int nxt = this->operator()({(int16_t)x, (int16_t)y});
      assert(nxt != -1);
      int d = 1<<i;
      int tile = get_jps_tiles(nxt);
      int suc = warthog::jps::compute_successors((warthog::jps::direction)d, tile);
      if (suc == 0)
        mask |= d;
    }
    return mask;
  }

  static inline uint32_t str2tiles(const vector<string>& map) {
    uint32_t tiles = 0;
    for (int i=0; i<3; i++) {
      for (int j=0; j<3; j++) if (map[i][j] != '#') {
        tiles |= (1 << j) << (i * 8);
      }
    }
    return tiles;
  }

  static inline vector<string> tiles2str(uint32_t tiles) {
    vector<string> map = {
      "...",
      ".x.",
      "..."
    };
    for (int i=0; i<3; i++, tiles >>= 8) {
      for (int j=0; j<3; j++) {
        if (tiles & (1 << j)) map[i][j] = '.';
        else map[i][j] = '#';
      }
    }
    map[1][1] = 'x';
    return map;
  }

  static inline void set2direct(uint32_t moveset) {
    if (moveset & warthog::jps::NORTH) cout << "north ";
    if (moveset & warthog::jps::SOUTH) cout << "south ";
    if (moveset & warthog::jps::EAST) cout << "east ";
    if (moveset & warthog::jps::WEST) cout << "west ";
    if (moveset & warthog::jps::NORTHEAST) cout << "northeast ";
    if (moveset & warthog::jps::NORTHWEST) cout << "northwest ";
    if (moveset & warthog::jps::SOUTHEAST) cout << "southeast ";
    if (moveset & warthog::jps::SOUTHWEST) cout << "southwest ";
    cout << endl;
  }

  static inline int str2neighbors(vector<string> data) {
    int mask = 0;
    for (int i=0; i<8; i++) {
      int dx = warthog::dx[i];
      int dy = warthog::dy[i];
      if (data[1 + dx][1 + dy] == '.') mask &= 1<<i;
    }
    return mask;
  }

  static inline vector<string> neighbors2str(int mask) {
    vector<string> data = {
      "###",
      "#.#",
      "###",
    };
    for (int i=0; i<8; i++) if (mask & (1<<i)) {
      int dx = warthog::dx[i];
      int dy = warthog::dy[i];
      data[1 + dx][1 + dy] = '.';
    }
    return data;
  }

  int get_jps_tiles(int x) const {
    return this->jps_tiles[x];
  }

  int get_neighbor(int x) const {
    return this->neighbors[x];
  }

  int get_pruned_neighbor(int x) const {
    return this->pruned_neighbors[x];
  }

  int get_centroid_rank(int x) const {
    return this->centroids_rank[x];
  }

  const vector<int>& get_fa() const {
    return fa;
  }

  inline int get_valid_move(int s, int quad, int part, int no_diagnonal) const {
    //return getClosestMove(this->neighbors[s], quad, part, onaxis);
    return this->mem[this->pruned_neighbors[s]].move[quad][part][no_diagnonal];
  }

  int centroid_nums() const {
    return centroids.size();
  }

  void set_centroids(const vector<int>& fa) {
    this->fa = vector<int>(fa.begin(), fa.end());
    centroids.clear();
    for (int i=0; i<(int)fa.size(); i++) if (fa[i] == i) centroids.push_back(i);
    centroids_rank = vector<int>(node_count_, -1);
    for (int i=0; i<(int)centroids.size(); i++) {
      centroids_rank[centroids[i]] = i;
    }
  }

private:
  int width_, height_, node_count_;
  std::vector<int>pos_to_node_;
  std::vector<xyLoc>node_to_pos_;
  vector<int> jps_tiles;
  vector<int> neighbors;
  vector<int> pruned_neighbors;
  vector<int> fa;
  vector<int> centroids;
  vector<int> centroids_rank;
  vector<ClosestMove> mem;

  void init_neighbors() {
    neighbors.resize(node_count_);
    for (int i=0; i<node_count_; i++) {
      xyLoc cur = this->operator()(i);
      int mask = 0;
      for (int d=0; d<8; d++) {
        xyLoc nxt, p1, p2;
        nxt.x = cur.x + warthog::dx[d];
        nxt.y = cur.y + warthog::dy[d];

        p1.x = cur.x, p1.y = cur.y + warthog::dy[d];
        p2.x = cur.x + warthog ::dx[d], p2.y = cur.y;
        if (this->operator()(nxt) != -1 &&
            this->operator()(p1) != -1 &&
            this->operator()(p2) != -1 ) mask |= 1<<d;
      }
      neighbors[i] = mask;
    }
  }

  void init_pruned_neighbors() {
    pruned_neighbors.resize(node_count_);
    for (int i=0; i<node_count_; i++) {
      int pseudo_obs = get_pseudo_obs(i);
      pruned_neighbors[i] = get_neighbor(i) ^ pseudo_obs;
    }
  }

  void init_jps_tiles() {
    jps_tiles.resize(node_count_);
    for (int i=0; i<node_count_; i++) {
      vector<string> data = {
        "...",
        ".x.",
        "..."
      };
      xyLoc cur = this->operator()(i);
      for(int j=0; j<8; j++) {
        xyLoc neighbor = cur;
        neighbor.x += warthog::dx[j];
        neighbor.y += warthog::dy[j];
        if (this->operator()(neighbor) != -1) {
          data[1 + warthog::dy[j]][1 + warthog::dx[j]] = '.';
        } else 
          data[1 + warthog::dy[j]][1 + warthog::dx[j]] = '#';
      } 
      jps_tiles[i] = str2tiles(data);
    }
  }

  int getClosestMove(int mask, int quad, int part, bool no_diagnonal=1) const {
    int dx, dy;
 
    switch (part) {
      case 0:dx=25, dy=100;
                break; 
      case 1:dx=75, dy=100;
                break;
      case 2:dx=100, dy=75;
                break;
      case 3:dx=100, dy=25;
    }

    if (quad == 3 || quad == 2) dx *= -1;
    if (quad == 2 || quad == 1) dy *= -1;

    if (!no_diagnonal) {// try diagonal first
      // get signbit
      int vx = dx, vy = dy;
      if (vx > 0) vx = 1;
      if (vx < 0) vx = -1;
      if (vy > 0) vy = 1;
      if (vy < 0) vy = -1;
      int move = warthog::v2i[1+vx][1+vy];
      if (mask & (1<<move))
        return move;
    }

    auto dist = [&](xyLoc a, xyLoc b) {
      //int common = min(abs(a.x - b.x), abs(a.y - b.y));
      //double res = warthog::DBL_ROOT_TWO * common + abs(a.x - b.x) + abs(a.y - b.y) - 2.0 * common;
      //return res;
      return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
    };

    //double cos = -1;
    double cost = warthog::INF;
    int move = warthog::INVALID_MOVE;
    for (int i=7; i>=0; i--) if (mask & (1<<i)) {
      xyLoc nxt;
      nxt.x = warthog::dx[i];
      nxt.y = warthog::dy[i];
      //nxt.x *= 100, nxt.y *= 100;
      double t = dist(nxt, xyLoc{(int16_t)dx, (int16_t)dy}) + warthog::doublew[i];
      if (t < cost) {
        cost = t;
        move = i;
      }
      /*
      double crossp = nxt.x * dx + nxt.y * dy;
      double nxt_mag = sqrt((double)(nxt.x * nxt.x) + (double)(nxt.y * nxt.y));
      double d_mg = sqrt((double)(dx * dx) + (double)(dy * dy));
      double cosi = crossp / (nxt_mag * d_mg);
      if (cosi > cos) {
        cos = cosi;
        move = i;
      }
      */
    }
    return move;
  }

  void initClosestMove() {
    mem.resize(1<<8);
    for (int i=0; i<(1<<8); i++) {
      for (int quad=0; quad<4; quad++) {
        for (int part=0; part<4; part++) {
          for (int axis=0; axis<=1; axis++)
            mem[i].move[quad][part][axis] = getClosestMove(i, quad, part, axis);
        }
      }
    } 
  }
};

inline
ListGraph extract_graph(const Mapper&mapper){
  static const int16_t* dx = warthog::dx;
  static const int16_t* dy = warthog::dy;
  static const int* dw = warthog::dw;

  ListGraph g(mapper.node_count());
  for(int u=0; u<mapper.node_count(); ++u){
    auto u_pos = mapper(u);
    for(int d: {4,5,6,7,0,1,2,3}){
      xyLoc v_pos = {static_cast<int16_t>(u_pos.x + dx[d]), static_cast<int16_t>(u_pos.y + dy[d])};
      int v = mapper(v_pos);
      xyLoc p1 = xyLoc{u_pos.x, static_cast<int16_t>(u_pos.y+dy[d])};
      xyLoc p2 = xyLoc{static_cast<int16_t>(u_pos.x+dx[d]), u_pos.y};
      if(v != -1 && mapper(p1) != -1 &&  mapper(p2) != -1) { // obstacle cut
        g.arc.push_back({u, v, dw[d], d});
      }
    }
  }
  return g;
}

static inline int signbit(int num) {
  return (num >> 31) - (-num >> 31);
}

static inline int iabs(int num) {
  int mask = num >> 31;
  return (num ^ mask) - mask;
}

static inline void dump_map(const Mapper&map, const char*file){
  FILE* f = fopen(file, "w");

  for(int y=0; y<map.height(); ++y){
    for(int x=0; x<map.width(); ++x){
      fprintf(f, "%5d", map(xyLoc{static_cast<int16_t>(x), static_cast<int16_t>(y)}));
    }
    fprintf(f, "\n");
  }
  fclose(f);
}
