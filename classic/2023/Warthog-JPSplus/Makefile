CXX       = g++
DEVFLAGS = -W -Wall -ggdb -O0 -std=c++17 -Wno-narrowing -Iwarthog/src/ -Iwarthog/src/contraction -Iwarthog/src/cpd -Iwarthog/src/domains -Iwarthog/src/experimental -Iwarthog/src/heuristics -Iwarthog/src/jps -Iwarthog/src/label -Iwarthog/src/mapf -Iwarthog/src/memory -Iwarthog/src/search -Iwarthog/src/sipp -Iwarthog/src/sys -Iwarthog/src/util
CXXFLAGS   = -W -Wall -O3 -std=c++17 -Wno-narrowing -march=native -DNDEBUG -Iwarthog/src/ -Iwarthog/src/contraction -Iwarthog/src/cpd -Iwarthog/src/domains -Iwarthog/src/experimental -Iwarthog/src/heuristics -Iwarthog/src/jps -Iwarthog/src/label -Iwarthog/src/mapf -Iwarthog/src/memory -Iwarthog/src/search -Iwarthog/src/sipp -Iwarthog/src/sys -Iwarthog/src/util
# CXXFLAGS   = $(DEVFLAGS)
EXEC     = run

SOURCES := ./Entry.cpp ./ScenarioLoader.cpp ./Timer.cpp ./main.cpp ./warthog/src/util/dimacs_parser.cpp ./warthog/src/util/experiment.cpp ./warthog/src/util/file_utils.cpp ./warthog/src/util/geography.cpp ./warthog/src/util/geom.cpp ./warthog/src/util/gm_parser.cpp ./warthog/src/util/helpers.cpp ./warthog/src/util/pqueue.cpp ./warthog/src/util/workload_manager.cpp ./warthog/src/util/scenario_manager.cpp ./warthog/src/util/timer.cpp ./warthog/src/contraction/ch_data.cpp ./warthog/src/contraction/contraction.cpp ./warthog/src/contraction/bch_bb_expansion_policy.cpp ./warthog/src/contraction/bch_expansion_policy.cpp ./warthog/src/contraction/fch_bb_expansion_policy.cpp ./warthog/src/contraction/fch_expansion_policy.cpp ./warthog/src/contraction/lazy_graph_contraction.cpp ./warthog/src/cpd/cast.cpp ./warthog/src/cpd/cpd.cpp ./warthog/src/cpd/graph_oracle.cpp ./warthog/src/cpd/cpd_graph_expansion_policy.cpp ./warthog/src/domains/blockmap.cpp ./warthog/src/domains/labelled_gridmap.cpp ./warthog/src/domains/xy_graph.cpp ./warthog/src/domains/gridmap.cpp ./warthog/src/experimental/cuckoo_table.cpp ./warthog/src/experimental/hash_table.cpp ./warthog/src/experimental/kway_pqueue.cpp ./warthog/src/heuristics/cpd_heuristic.cpp ./warthog/src/heuristics/road_heuristic.cpp ./warthog/src/heuristics/euclidean_heuristic.cpp ./warthog/src/jps/four_connected_jps_locator.cpp ./warthog/src/jps/jps.cpp ./warthog/src/jps/offline_jump_point_locator.cpp ./warthog/src/jps/offline_jump_point_locator2.cpp ./warthog/src/jps/online_jump_point_locator.cpp ./warthog/src/jps/jps2_expansion_policy.cpp ./warthog/src/jps/jps2plus_expansion_policy.cpp ./warthog/src/jps/jps4c_expansion_policy.cpp ./warthog/src/jps/jps_expansion_policy.cpp ./warthog/src/jps/jpsplus_expansion_policy.cpp ./warthog/src/jps/online_jump_point_locator2.cpp ./warthog/src/label/bb_labelling.cpp ./warthog/src/label/dfs_labelling.cpp ./warthog/src/mapf/plan.cpp ./warthog/src/mapf/reservation_table.cpp ./warthog/src/mapf/cbs_ll_expansion_policy.cpp ./warthog/src/mapf/cbs_ll_heuristic.cpp ./warthog/src/mapf/ll_expansion_policy.cpp ./warthog/src/memory/node_pool.cpp ./warthog/src/search/apriori_filter.cpp ./warthog/src/search/bidirectional_graph_expansion_policy.cpp ./warthog/src/search/bitfield_filter.cpp ./warthog/src/search/cpd_search.cpp ./warthog/src/search/expansion_policy.cpp ./warthog/src/search/flexible_astar.cpp ./warthog/src/search/graph_expansion_policy.cpp ./warthog/src/search/search_node.cpp ./warthog/src/search/gridmap_expansion_policy.cpp ./warthog/src/search/problem_instance.cpp ./warthog/src/search/search_metrics.cpp ./warthog/src/search/solution.cpp ./warthog/src/search/vl_gridmap_expansion_policy.cpp ./warthog/src/sipp/jpst_gridmap.cpp ./warthog/src/sipp/jpst_locator.cpp ./warthog/src/sipp/sipp_expansion_policy.cpp ./warthog/src/sipp/sipp_gridmap.cpp ./warthog/src/sipp/temporal_jps_expansion_policy.cpp
OBJECTS := $(patsubst %.cpp, %.o, $(SOURCES))

.PHONY: all clean

all: $(EXEC)

$(EXEC): $(OBJECTS)
	$(CXX) $(CXXFLAGS) $^ -o $@

%.o: %.c
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm $(EXEC) $(OBJECTS)
