#include <algorithm>
#include <map>
#include "Entry.h"
#include <domains/gridmap.h>
#include <search/gridmap_expansion_policy.h>
#include <search/flexible_astar.h>
#include <heuristics/octile_heuristic.h>
#include <util/pqueue.h>

struct WarthogAlg
{
	warthog::gridmap map;
	warthog::gridmap_expansion_policy expander;
	warthog::octile_heuristic heuristic;
	warthog::pqueue_min open;

	warthog::flexible_astar<
	warthog::octile_heuristic,
	warthog::gridmap_expansion_policy, 
	warthog::pqueue_min> 
	astar;
	warthog::solution sol;

	WarthogAlg(const std::vector<bool>& bits, int width, int height) : map(bits.begin(), bits.end(), height, width)
		,expander(&map)
		,heuristic(map.width(), map.height())
		,astar(&heuristic, &expander, &open)
	{ }
};

void PreprocessMap(const std::vector<bool> &bits, int width, int height, const std::string &filename) {
	
}

void *PrepareForSearch(const std::vector<bool> &bits, int width, int height, const std::string &filename) {
	return new WarthogAlg(bits, width, height);
}

bool GetPath(void *data, xyLoc s, xyLoc g, std::vector<xyLoc> &path) {
	WarthogAlg* alg = static_cast<WarthogAlg*>(data);
	
	const auto width = static_cast<warthog::sn_id_t>(alg->map.header_width());
	warthog::sn_id_t startid =static_cast<warthog::sn_id_t>(s.y) * width + static_cast<warthog::sn_id_t>(s.x);
	warthog::sn_id_t goalid = static_cast<warthog::sn_id_t>(g.y) * width + static_cast<warthog::sn_id_t>(g.x);
	warthog::problem_instance pi(startid, goalid, false);

	alg->astar.get_path(pi, alg->sol);
	if (alg->sol.path_.empty())
		return true;
	
	path.resize(alg->sol.path_.size());
	auto jt = alg->sol.path_.begin();
	for (auto it = path.begin(), ite = path.end(); it != ite; ++it, ++jt) {
		xyLoc xy;
		auto j = alg->map.to_unpadded_id(*jt);
		xy.x = static_cast<int16_t>(j % width);
		xy.y = static_cast<int16_t>(j / width);
		*it = xy;
	}
	return true;
}

std::string GetName() { return "Warthog-A*"; }
