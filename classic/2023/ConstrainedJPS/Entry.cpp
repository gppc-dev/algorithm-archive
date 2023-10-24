/*
Copyright (c) 2023 Grid-based Path Planning Competition and Contributors <https://gppc.search-conference.org/>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "Entry.h"

/**
 * User code used during preprocessing of a map.  Can be left blank if no pre-processing is required.
 * It will not be called in the same program execution as `PrepareForSearch` is called,
 * all data must be shared through file.
 * 
 * Called with command below:
 * ./run -pre file.map
 * 
 * @param[in] bits Array of 2D table.  (0,0) is located at top-left corner.  bits.size() = height * width
 *                 Packed as 1D array, row-by-ray, i.e. first width bool's give row y=0, next width y=1
 *                 bits[i] returns `true` if (x,y) is traversable, `false` otherwise
 * @param[in] width Give the map's width
 * @param[in] height Give the map's height
 * @param[in] filename The filename you write the preprocessing data to.  Open in write mode.
 */
void PreprocessMap(const std::vector<bool> &bits, int width, int height, const std::string &filename) {}

/**
 * User code used to setup search before queries.  Can also load pre-processing data from file to speed load.
 * It will not be called in the same program execution as `PreprocessMap` is called,
 * all data must be shared through file.
 * 
 * Called with any commands below:
 * ./run -run file.map file.map.scen
 * ./run -check file.map file.map.scen
 * 
 * @param[in] bits Array of 2D table.  (0,0) is located at top-left corner.  bits.size() = height * width
 *                 Packed as 1D array, row-by-ray, i.e. first width bool's give row y=0, next width y=1
 *                 bits[i] returns `true` if (x,y) is traversable, `false` otherwise
 * @param[in] width Give the map's width
 * @param[in] height Give the map's height
 * @param[in] filename The filename you write the preprocessing data to.  Open in write mode.
 * @returns Pointer to data-structure used for search.  Memory should be stored on heap, not stack.
 */
void *PrepareForSearch(const std::vector<bool> &bits, int width, int height, const std::string &filename) {

  typedef warthog::flexible_astar<
	  warthog::octile_heuristic,
	  warthog::jps2_expansion_policy_prune2,
    warthog::pqueue_min> ALG;

  auto map = new warthog::gridmap(bits, height, width);
  // warthog::gridmap map(bits, height, width);
  auto expander = new warthog::jps2_expansion_policy_prune2(map);
	// warthog::jps2_expansion_policy_prune2 expander(&map);
  auto heuristic = new warthog::octile_heuristic(map->width(), map->height());
	// warthog::octile_heuristic heuristic(map.width(), map.height());
  auto open = new warthog::pqueue_min();
  // warthog::pqueue_min open;

  namespace G = global;
	auto astar = new ALG(heuristic, expander, open);
  G::query::map = map;
  G::query::open = open;
  G::nodepool = expander->get_nodepool();
  return (void*)astar;
}

/**
 * User code used to setup search before queries.  Can also load pre-processing data from file to speed load.
 * It will not be called in the same program execution as `PreprocessMap` is called,
 * all data must be shared through file.
 * 
 * Called with any commands below:
 * ./run -run file.map file.map.scen
 * ./run -check file.map file.map.scen
 * 
 * @param[in,out] data Pointer to data returned from `PrepareForSearch`.  Can static_cast to correct data type.
 * @param[in] s The start (x,y) coordinate of search query
 * @param[in] g The goal (x,y) coordinate of search query
 * @param[out] path The points that forms the shortest path from `s` to `g` computed by search algorithm.
 *                  Shortest path length will calculated by summation of Euclidean distance
 *                  between consecutive pairs path[i]--path[i+1].  Collinear points are allowed.
 *                  Return an empty path if no shortest path exists.
 * @returns `true` if search is complete, including if no-path-exists.  `false` if search only partially completed.
 *          if `false` then `GetPath` will be called again until search is complete.
 */
bool GetPath(void *data, xyLoc s, xyLoc g, std::vector<xyLoc> &path) {
  typedef warthog::flexible_astar<
	  warthog::octile_heuristic,
	  warthog::jps2_expansion_policy_prune2,
    warthog::pqueue_min> ALG;

  ALG* astar = (ALG*)(data);
  namespace G = global;
  uint32_t startid = s.y * G::query::map->header_width() + s.x;
  uint32_t goalid = g.y * G::query::map->header_width() + g.x;
  warthog::problem_instance pi(startid, goalid, false);
  warthog::solution sol;
  astar->get_path(pi, sol);

  path.push_back(s);
  for (size_t i=1; i<sol.path_.size(); i++) {
    uint32_t x, y;
    G::query::map->to_unpadded_xy(sol.path_[i], x, y);

    if ((int16_t)x == path.back().x || (int16_t)y == path.back().y) {
      path.push_back({(int16_t)x, (int16_t)y});
    }
    else {
      int px = path.back().x, py = path.back().y;
      int dx = x - px, dy = y - py;
      int l = min(abs(dx), abs(dy));
      int vx = dx / abs(dx), vy = dy / abs(dy);
      path.push_back({(int16_t)(px + l * vx), (int16_t)(py + l * vy)});
      path.push_back({(int16_t)x, (int16_t)y});
    }
  }
  return true;
}

/**
 * The algorithm name.  Please update std::string and ensure name is immutable.
 * 
 * @returns the name of the algorithm
 */
std::string GetName() { return "ConstrainedJPS"; }
