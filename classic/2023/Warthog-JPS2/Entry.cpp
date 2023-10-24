#include <algorithm>
#include <map>
#include "Entry.h"
#include <domains/gridmap.h>
#include <jps/jps2_expansion_policy.h>
#include <jps/jps2plus_expansion_policy.h>
#include <search/flexible_astar.h>
#include <heuristics/octile_heuristic.h>
#include <util/pqueue.h>

// #define JPS_PLUS

struct WarthogAlg
{
	warthog::gridmap map;
#ifdef JPS_PLUS
	warthog::jps2plus_expansion_policy expander;
#else
	warthog::jps2_expansion_policy expander;
#endif
	warthog::octile_heuristic heuristic;
	warthog::pqueue_min open;

	warthog::flexible_astar<
	warthog::octile_heuristic,
#ifdef JPS_PLUS
	warthog::jps2plus_expansion_policy,
#else
	warthog::jps2_expansion_policy,
#endif
	warthog::pqueue_min> 
	astar;
	warthog::solution sol;

	WarthogAlg(const std::vector<bool>& bits, int width, int height, const std::string& filename) : map(bits.begin(), bits.end(), height, width)
#ifdef JPS_PLUS
		,expander(&map, filename)
#else
		,expander(&map)
#endif
		,heuristic(map.width(), map.height())
		,astar(&heuristic, &expander, &open)
	{ }
};

void PreprocessMap(const std::vector<bool> &bits, int width, int height, const std::string& filename)
{
#ifdef JPS_PLUS
	warthog::gridmap map(bits.begin(), bits.end(), height, width);
	warthog::jps2plus_expansion_policy expander(&map, filename);
#endif
}

void *PrepareForSearch(const std::vector<bool> &bits, int width, int height, const std::string& filename) {
	return new WarthogAlg(bits, width, height, filename);
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
	
	path.reserve(2*alg->sol.path_.size());
	for (auto jid : alg->sol.path_) {
		xyLoc xy;
		auto j = alg->map.to_unpadded_id(jid);
		xy.x = static_cast<int16_t>(j % width);
		xy.y = static_cast<int16_t>(j / width);
		if (!path.empty()) {
			xyLoc prev = path.back();
			xyLoc diff{xy.x - prev.x, xy.y - prev.y};
			xyLoc adiff{std::abs(diff.x), std::abs(diff.y)};
			if (adiff.x != 0 && adiff.y != 0 && adiff.x != adiff.y) {
				xyLoc ins = xy;
				int cardinalDiff = adiff.x - adiff.y;
				if (cardinalDiff > 0) {
					// adiff.x > adiff.y
					ins.x = diff.x > 0 ? (ins.x - cardinalDiff) : (ins.x + cardinalDiff);
				} else {
					// adiff.y > adiff.x
					ins.y = diff.y > 0 ? (ins.y + cardinalDiff) : (ins.y - cardinalDiff);
				}
				path.push_back(ins);
			}
		}
		path.push_back(xy);
	}
	return true;
}

std::string GetName() { return "Warthog-JPS2p"; }
