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
#include "Astar.h"
#include "cpd_centroid.h"
#include "centroid.h"
#include "Dijkstra.h"
#include "query.h"
#include <chrono>
#include <iomanip>

static const int hLevel = 3;

double evaluate_tcost(Dijkstra& dij, int source) {
  auto stime = std::chrono::steady_clock::now();
  dij.run(source, hLevel);
  auto etime = std::chrono::steady_clock::now();
  double tcost = std::chrono::duration_cast<std::chrono::nanoseconds>(etime - stime).count();
  return tcost / 1000000000.0;
}


void reportProgress(int& progress, int tot) {
  ++progress;
  if(progress % 100 == 0) {
    double ratio = (double)progress / tot * 100.0;
    cout << "Progress: [" << progress << "/" << tot << "] "
         << setprecision(3) << ratio << "% \r";
    cout.flush();
  }
}

void PreprocessRevCentroid(Mapper& mapper, NodeOrdering& order, vector<int>& cents, AdjGraph& g, const string& fname) {
  CPD_CENTROID cpd;
  
  const static int tnum = 4;
  printf("Using %d threads\n", tnum);
  vector<CPD_CENTROID>thread_cpd(tnum);

  int progress = 0;

  #pragma omp parallel num_threads(tnum)
  {
    const int thread_count = omp_get_num_threads();
    const int tid = omp_get_thread_num();
    const int node_count = g.node_count();

    int node_begin = (node_count*tid) / thread_count;
    int node_end = (node_count*(tid+1)) / thread_count;

    AdjGraph thread_adj_g(g);
    Dijkstra thread_dij(thread_adj_g, mapper);
    Mapper thread_mapper = mapper;

    for(int source_node=node_begin; source_node < node_end; source_node++){
      if (mapper.get_fa()[source_node] == source_node) {
        thread_dij.run_extra(source_node, hLevel);
        thread_cpd[tid].append_row(source_node, thread_dij.get_inv_allowed(), thread_mapper, 1);
        #pragma omp critical 
        reportProgress(progress, cents.size());
      }
    }
  }

  for (auto&x: thread_cpd)
    cpd.append_rows(x);

  FILE*f;
  printf("Saving data to %s\n", fname.c_str());
  printf("begin size: %zu, entry size: %zu\n", cpd.entry_count(), cpd.get_entry_size());
  f = fopen(fname.c_str(), "wb");
  save_vector(f, mapper.get_fa());
  order.save(f);
  cpd.save(f);
  fclose(f);
  printf("Done\n");
}

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
void PreprocessMap(const std::vector<bool> &bits, int width, int height, const std::string &filename) {

  int hLevel = 3;
  int r = 2;
  int size = width * height;
  if (size <= (1<<13)) r = 1;
  if (size > (1<<17)) {
    r <<= 1;
  }
  if (size > (1<<20)) {
    r <<= 1;
  }
  if (size > (1<<22)) {
    r = 96;
  }

  Mapper mapper(bits, width, height);
  printf("width = %d, height = %d, node_count = %d\n", width, height, mapper.node_count());

  printf("Computing node order\n");
  NodeOrdering order = compute_real_dfs_order(extract_graph(mapper));

  mapper.reorder(order);

  vector<int> cents;
  cents = compute_centroid(mapper, r);

  AdjGraph g(extract_graph(mapper));

  Dijkstra temp_dij(g, mapper);
  double tots = evaluate_tcost(temp_dij, 0);
  tots *= cents.size();
  printf("Estimated sequential running time : %fmin\n", tots / 60.0);

  printf("Computing first-move matrix, hLevel: %d\n", hLevel);
  PreprocessRevCentroid(mapper, order, cents, g, filename);
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
 * @param[in] bits Array of 2D table.  (0,0) is located at top-left corner.  bits.size() = height * width
 *                 Packed as 1D array, row-by-ray, i.e. first width bool's give row y=0, next width y=1
 *                 bits[i] returns `true` if (x,y) is traversable, `false` otherwise
 * @param[in] width Give the map's width
 * @param[in] height Give the map's height
 * @param[in] filename The filename you write the preprocessing data to.  Open in write mode.
 * @returns Pointer to data-structure used for search.  Memory should be stored on heap, not stack.
 */

inline void LoadInvCentroidsCPD(EntryData& data, FILE* f) {
  vector<int> centroids;
  //data.square_sides = load_vector<int>(f);
  centroids = load_vector<int>(f);
  NodeOrdering order;
  order.load(f);
  data.cpd.load(f);

  data.mapper.reorder(order);
  data.mapper.set_centroids(centroids);
  data.graph = AdjGraph(extract_graph(data.mapper));
}

void *PrepareForSearch(const std::vector<bool> &bits, int width, int height, const std::string &filename) {
  EntryData* entrydata = new EntryData();
  entrydata->mapper = Mapper(bits, width, height);
  FILE* f = fopen(filename.c_str(), "rb");
  LoadInvCentroidsCPD(*entrydata, f);
  entrydata->e1.init(entrydata->graph.node_count());
  entrydata->e2.init(entrydata->graph.node_count());
  entrydata->scenid = 0;
  fclose(f);
  return (void*)entrydata;
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
  EntryData* entrydata = (EntryData*)data;
  entrydata->scenid++;
  entrydata->e1.reset(entrydata->scenid);
  entrydata->e2.reset(entrydata->scenid);
  entrydata->c = Counter{0, 0, 0};
  GetInvCentroidCost(*entrydata, s, g, 3, entrydata->c, entrydata->e1, entrydata->e2, path, -1);
  return true;
}

/**
 * The algorithm name.  Please update std::string and ensure name is immutable.
 * 
 * @returns the name of the algorithm
 */
std::string GetName() { return "subOptCPD"; }
