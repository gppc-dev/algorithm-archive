#ifndef RAYSCAN_SEARCH_EXPANSION_HPP
#define RAYSCAN_SEARCH_EXPANSION_HPP

#include "fwd.hpp"
#include "Scan.hpp"
#include "RayShoot.hpp"
#include "Vertex.hpp"
#include "DebugStruct.hpp"

#ifndef NDEBUG
#include <unordered_set>
#endif

namespace rayscan::search {

class Expansion
{
public:
	using vertex = Vertex;
	Expansion();

	void setup(Search& search);
	/**
	 * Setup the search
	 * @return The starting vertex
	*/
	Vertex* search_setup(geo::Point s, geo::Point t);

	const std::vector<Vertex*>& expand_s(Vertex& u);
	const std::vector<Vertex*>& expand(Vertex& u);
	void setup_expand(Vertex& u);

	shoot_res shoot(geo::Point u, geo::Point uv);
	shoot_res shoot(Vertex& u, Vertex& v);

	void push(Vertex& u, Vertex& v);
	void push_target(Vertex&u, Vertex& t);
	bool target_vis(shoot_res res, geo::Point uv)
	{
		if ( (res.second & SEG_CLEAR) || (!(res.second & EDGE) && m_vU->p + res.first == m_vT.p) ) [[unlikely]] {
			return (m_vT.ambig == Vertex::Ambig::NONE ||
				(m_vT.ambig == Vertex::Ambig::AMBIG_XY_SIGN && (uv.x | uv.y) < 0));
		}
		return false;
	}
	template <typename Vector>
	void get_path(Vector& v)
	{
		v.clear();
		DEBUG_SEARCH_CMD(m_sid.id,m_debugSearch.init_expansion(m_grid->getTable()[0], -1, m_vT.p))
#ifndef NDEBUG
		std::unordered_set<const Vertex*> loop_detect;
#endif
		for (auto* node = &m_vT; node != nullptr; node = node->pred) {
#ifndef NDEBUG
			assert(!loop_detect.contains(node));
			loop_detect.insert(node);
#endif
			v.push_back(node->p);
		}
		std::reverse(v.begin(), v.end());
#if DEBUG_SEARCH >= 0
		for (auto p : v) {
			DEBUG_SEARCH_CMD(m_sid.id, m_debugSearch.fill_point(p.x, p.y, 'x'))
		}
		DEBUG_SEARCH_CMD(m_sid.id, m_debugSearch.fill_point(m_vS.p.x, m_vS.p.y, 's'))
		DEBUG_SEARCH_CMD(m_sid.id, m_debugSearch.fill_point(m_vT.p.x, m_vT.p.y, 't'))
		DEBUG_SEARCH_CMD(m_sid.id, m_debugSearch.print())
#endif
	}

	/**
	 * Scan within a region
	*/
	void region_scan(geo::BasicAngleSector sector, geo::Point up, geo::Point cw_start, geo::Point ccw_start);
	/**
	 * Scan within a region, allows sector to be wider than 180 thus introduces additional logic
	*/
	void region_scan_s(geo::Region sector, geo::Point up, geo::Point cw_start, geo::Point ccw_start);
	template <geo::Ori D>
	geo::Point subregion_scan(geo::BasicAngleSector sector, geo::Point up, geo::Point start);

	// return D-far ray, bool: true if D-scan was performed (tp is blocked)
	template <geo::Ori D>
	std::pair<geo::Point, bool> shoot_backward_turning_point(geo::BasicAngleSector sector, geo::Point up, geo::Point tp);

	static geo::Region ray_split(shoot_res r) noexcept;

	bool done() const noexcept { return m_done; }

protected:
	void search_node_st(Vertex& v, geo::Point p);
	void expand_s_full(Vertex& u);
	void expand_s_bound(Vertex& u);

public:
	Vertex& getVertex(geo::Point p);

public:
	env::Grid* m_grid;
	SearchId m_sid;
	ExpId m_eid;
	Scan<SimpleScanner> m_scan;
	RayShoot m_ray;
	Vertex m_vS, m_vT;
	std::vector<Vertex*> m_nodePush;
	Vertex* m_vU;
	bool m_done;
#if DEBUG_SEARCH >= 0
	DebugSearch m_debugSearch;
#endif
};

extern template std::pair<geo::Point, bool> Expansion::shoot_backward_turning_point<geo::CW>(geo::BasicAngleSector sector, geo::Point up, geo::Point tp);
extern template std::pair<geo::Point, bool> Expansion::shoot_backward_turning_point<geo::CW>(geo::BasicAngleSector sector, geo::Point up, geo::Point tp);

inline geo::Region Expansion::ray_split(shoot_res r) noexcept
{
	switch (static_cast<uint32_t>(r.second) & ( inx::make_mask_v<uint32_t, extra::HV_COUNT, extra::HV_SHIFT> | inx::make_mask_v<uint32_t, extra::POS_COUNT, extra::POS_SHIFT> )) {
	case POINT | HORI_POS:
	case POINT | HORI_NEG:
	case POINT | VERT_POS:
	case POINT | VERT_NEG:
		return geo::Region(r.first, r.first);
	case EDGE | HORI_POS: return geo::Region(r.first + geo::Point(1, 0), r.first);
	case EDGE | HORI_NEG: return geo::Region(r.first, r.first + geo::Point(1, 0));
	case EDGE | VERT_POS: return geo::Region(r.first, r.first + geo::Point(0, 1));
	case EDGE | VERT_NEG: return geo::Region(r.first + geo::Point(0, 1), r.first);
	default:
	assert(false);
	return geo::Region();
	}
}

} // namespace rayscan::search

#endif // RAYSCAN_SEARCH_EXPANSION_HPP
