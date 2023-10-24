#include "Expansion.hpp"
#include "Search.hpp"
#if DEBUG_SEARCH >= 0
#include <iostream>
#endif

namespace rayscan::search {

Expansion::Expansion() : m_grid{}, m_sid{}, m_eid{}, m_vU(nullptr), m_done{}
{ }

void Expansion::setup(Search& search)
{
	m_grid = search.get_grid();
	m_sid.id = 0;
	m_grid->construct_vertices<vertex>();
	m_scan.setup(search);
	m_ray.setup(search);
#if DEBUG_SEARCH >= 0
	m_debugSearch.setup(m_grid->getTable()[0]);
#endif
}

Vertex* Expansion::search_setup(geo::Point s, geo::Point t)
{
	m_sid.id += 1;
	search_node_st(m_vS, s);
	search_node_st(m_vT, t);
	m_vS.g = m_vS.f = 0;
	m_done = false;
	return &m_vS;
}

void Expansion::search_node_st(Vertex& v, geo::Point p)
{
	v.p = p;
	v.fill_mask(m_grid->getTable()[0].region<1,1,2,2>(p.x, p.y));
	v.init(m_sid);
}

Vertex& Expansion::getVertex(geo::Point p)
{
	Vertex* v = static_cast<Vertex*>(m_grid->getVertex(p));
	v->search(m_sid);
	return *v;
}

void Expansion::setup_expand(Vertex& u)
{
	m_vU = &u;
	m_eid.id += 1;
	m_nodePush.clear();
	DEBUG_SEARCH_CMD(m_sid.id,m_debugSearch.init_expansion(m_grid->getTable()[0], m_eid.id, u.p))
	DEBUG_SEARCH_CMD(m_sid.id, m_debugSearch.fill_point(m_vS.p.x, m_vS.p.y, 's'))
	DEBUG_SEARCH_CMD(m_sid.id, m_debugSearch.fill_point(m_vT.p.x, m_vT.p.y, 't'))
	DEBUG_SEARCH_CMD(m_sid.id, if (u.pred != nullptr) m_debugSearch.fill_point(u.pred->p.x, u.pred->p.y, 'p'))
	DEBUG_SEARCH_CMD(m_sid.id, m_debugSearch.fill_point(u.p.x, u.p.y, 'u'))
}

const std::vector<Vertex*>& Expansion::expand_s(Vertex& u)
{
	setup_expand(u);
	switch (u.ambig) {
	case Vertex::Ambig::NONE:
		if (!u.corner) {
			expand_s_full(u);
			break;
		}
	[[fallthrough]];
	case Vertex::Ambig::AMBIG_XY_SIGN:
		expand_s_bound(u);
		break;
	case Vertex::Ambig::AMBIG_XY_NOSIGN:
		assert(false);
		break;
	}
	DEBUG_SEARCH_CMD(m_sid.id,m_debugSearch.print())
	return m_nodePush;
}
void Expansion::expand_s_full(Vertex& u)
{
	geo::Point st = m_vT.p - u.p;
	shoot_res st_res = shoot(u.p, st);
	if (target_vis(st_res, st)) {
		push_target(u, m_vT);
		return;
	}
	shoot_res ts_res = shoot(u.p, -st);
	auto st_start = ray_split(st_res);
	auto ts_start = ray_split(ts_res);
	region_scan(geo::BasicAngleSector(st, -st), u.p, st_start[1], ts_start[0]);
	region_scan(geo::BasicAngleSector(-st, st), u.p, ts_start[1], st_start[0]);
}
void Expansion::expand_s_bound(Vertex& u)
{
	geo::Point st = m_vT.p - u.p;
	geo::Region projField(Vertex::corner_pt_x<true>(u.corner), Vertex::corner_pt_x<false>(u.corner));
	if (!geo::is_strict_between_ccw_any(st, projField[0], projField[1])) {
		// target within bound, shoot
		shoot_res st_res = shoot(u.p, st);
		if (target_vis(st_res, st)) {
			push_target(u, m_vT);
			return;
		}
		if (geo::is_strict_between_cw_any(st, projField[0], projField[1])) { // st lies strictly within projField
			// split sector by target
			auto ray_start = ray_split(st_res);
			region_scan_s(geo::Region(projField[0], st), u.p, projField[0], ray_start[0]);
			region_scan_s(geo::Region(st, projField[1]), u.p, ray_start[1], projField[1]);
			return;
		}
	}
	// target is not strictly within projField
	region_scan_s(projField, u.p, projField[0], projField[1]);
}

const std::vector<Vertex*>& Expansion::expand(Vertex& u)
{
	setup_expand(u);
	geo::Point st = m_vT.p - u.p;
	assert(u.turn != geo::CO);
	const int projBoundId = geo::Region::ori_id(u.turn);
	// geo::Region projStart(Vertex::corner_pt_x<true>(u.corner), Vertex::corner_pt_x<false>(u.corner));
	geo::Region projStart;
	geo::BasicAngleSector projField = u.projection_field();
	projStart[projBoundId] = projField[projBoundId];
	projStart[projBoundId^1] = ray_split(u.get_pred_cache())[projBoundId];
	if (projField.within(st)) {
		shoot_res st_res = shoot(u.p, st);
		if (target_vis(st_res, st)) {
			push_target(u, m_vT);
			return m_nodePush;
		}
		if (projField.strictly_within(st)) {
			// perform 2 scans
			auto st_start = ray_split(st_res);
			auto projAltStart = projStart;
			// do scan along projection field
			projAltStart[projBoundId] = st_start[projBoundId^1];
			auto projAltField = projField;
			projAltField[projBoundId] = st;
			region_scan(projAltField, u.p, projAltStart[0], projAltStart[1]);
			// setup for next scan with boundary set to st
			projField[projBoundId^1] = st;
			projStart[projBoundId^1] = st_start[projBoundId];
		}
	}
	// perform scan
	region_scan(projField, u.p, projStart[0], projStart[1]);
	DEBUG_SEARCH_CMD(m_sid.id,m_debugSearch.print())
	return m_nodePush;
}

void Expansion::region_scan(geo::BasicAngleSector sector, geo::Point up, geo::Point cw_start, geo::Point ccw_start)
{
	// perform CW scan first, then CCW
	// assert(sector.within(cw_start));
	// assert(sector.within(ccw_start));
	// assert(!geo::is_ccw(cw_start, ccw_start));
	if (sector.within(cw_start)) {
		cw_start = subregion_scan<geo::CW>(sector, up, cw_start);
		if (!cw_start.is_zero())
			sector = sector.split_sector<geo::CW>(cw_start);
	}
	if (sector.within(ccw_start)) { // as ccw_start is not always aligned with sector boundary, it can lay outside full cw scan, so check
		subregion_scan<geo::CCW>(sector, up, ccw_start);
	}
}
void Expansion::region_scan_s(geo::Region sector, geo::Point up, geo::Point cw_start, geo::Point ccw_start)
{
	// is larger than 180 deg, thus shoot ray and split sector
	if (geo::is_ccw(sector[0], sector[1])) {
		geo::Point split_ray = static_cast<geo::coord_op>(sector[0].x) * sector[0].y == 0 ? -sector[0] : -sector[1]; // try for horizontal/vertical rays for faster rayshooting
		auto split_res = m_ray.shoot(up, split_ray);
		auto split_start = ray_split(split_res);
		region_scan(geo::BasicAngleSector(sector[0], split_ray), up, cw_start, split_start[0]);
		region_scan(geo::BasicAngleSector(split_ray, sector[1]), up, split_start[1], ccw_start);
	} else {
		region_scan(sector, up, cw_start, ccw_start);
	}
}
template <geo::Ori D>
geo::Point Expansion::subregion_scan(geo::BasicAngleSector sector, geo::Point up, geo::Point start)
{
	assert(sector.within(start));
	geo::Point far_ray = sector[geo::BasicAngleSector::ori_id_v<inv_ori(D)>];
	while (true) {
		geo::Point turning_point = m_scan.full_scan<D>(sector, up, start, far_ray);
		if (turning_point.is_zero())
			return far_ray;
		assert(sector.within(turning_point));
		far_ray = turning_point;
		Vertex& v = getVertex(up + turning_point);
		shoot_res res = shoot(*m_vU, v);
		if (res.second & RAY_CACHED) // ray shot previously, end
			return far_ray;
		auto res_start = ray_split(res);
		assert(!sector.within(res_start[geo::BasicAngleSector::ori_id_v<geo::CW>]) || sector.split_sector<geo::Ori::CW>(turning_point).within(res_start[geo::BasicAngleSector::ori_id_v<geo::CW>]));
		assert(!sector.within(res_start[geo::BasicAngleSector::ori_id_v<geo::CCW>]) || sector.split_sector<geo::Ori::CCW>(turning_point).within(res_start[geo::BasicAngleSector::ori_id_v<geo::CCW>]));
		if (res.second & SEG_CLEAR) {
			push(*m_vU, v);
		} else if (sector.within(res_start[geo::BasicAngleSector::ori_id_v<inv_ori(D)>])) {
			subregion_scan<inv_ori(D)>(sector.split_sector<inv_ori(D)>(turning_point), up, res_start[geo::BasicAngleSector::ori_id_v<inv_ori(D)>]);
		}
		if (!sector.within(res_start[geo::BasicAngleSector::ori_id_v<D>])) {
			return far_ray;
		}
		sector = sector.split_sector<D>(turning_point);
		start = res_start[geo::BasicAngleSector::ori_id_v<D>];
	}
}

template <geo::Ori D>
std::pair<geo::Point, bool> Expansion::shoot_backward_turning_point(geo::BasicAngleSector sector, geo::Point up, geo::Point tp)
{
	assert(sector.within(tp));
	using return_type = std::pair<geo::Point, bool>;
	Vertex& v = getVertex(up + tp);
	shoot_res res = shoot(*m_vU, v);
	if (res.second & RAY_CACHED) // ray shot previously, end
		return return_type(geo::Point::zero(), false);
	auto res_start = ray_split(res);
	assert(!sector.within(res_start[geo::BasicAngleSector::ori_id_v<geo::CW>]) || sector.split_sector<geo::Ori::CW>(tp).within(res_start[geo::BasicAngleSector::ori_id_v<geo::CW>]));
	assert(!sector.within(res_start[geo::BasicAngleSector::ori_id_v<geo::CCW>]) || sector.split_sector<geo::Ori::CCW>(tp).within(res_start[geo::BasicAngleSector::ori_id_v<geo::CCW>]));
	if (res.second & SEG_CLEAR) {
		// no forward
		push(*m_vU, v);
	}
	if (sector.within(res_start[geo::BasicAngleSector::ori_id_v<inv_ori(D)>])) {
		subregion_scan<inv_ori(D)>(sector.split_sector<inv_ori(D)>(tp), up, res_start[geo::BasicAngleSector::ori_id_v<inv_ori(D)>]);
	}
	if (!(res.second & SEG_CLEAR)) {
		if (sector.within(res_start[geo::BasicAngleSector::ori_id_v<D>]))
			return return_type(subregion_scan<D>(sector.split_sector<D>(tp), up, res_start[geo::BasicAngleSector::ori_id_v<D>]), true); // forward scan
		else
			return return_type(geo::Point::zero(), true);
	}
	return return_type(geo::Point::zero(), false);
}
template std::pair<geo::Point, bool> Expansion::shoot_backward_turning_point<geo::CW>(geo::BasicAngleSector sector, geo::Point up, geo::Point tp);
template std::pair<geo::Point, bool> Expansion::shoot_backward_turning_point<geo::CCW>(geo::BasicAngleSector sector, geo::Point up, geo::Point tp);

shoot_res Expansion::shoot(geo::Point u, geo::Point uv)
{
	return m_ray.shoot(u, uv);
}
shoot_res Expansion::shoot(Vertex& u, Vertex& v)
{
	assert(m_vU == &u);
	if (v.rayId.id == m_eid.id)
		return v.get_ray_cache();
	v.rayId.id = m_eid.id;
	shoot_res res = m_ray.shoot(u.p, v.p - u.p);
	v.set_ray_cache( shoot_res(res.first, static_cast<Shoot>(res.second | RAY_CACHED)) );
	return res;
}

void Expansion::push(Vertex& u, Vertex& v)
{
	const auto uv = v.p - u.p;
	assert(uv.square() <= v.get_ray_cache().first.square());
	double g = u.g + uv.length();
	if (!v.close & (g < v.g)) {
		shoot_res predRay = v.get_ray_cache();
		predRay.first = predRay.first - uv;
#ifndef NDEBUG
		auto debug_shoot = shoot(v.p, uv);
		assert(predRay.first == debug_shoot.first);
#endif
		v.set_pred_cache(predRay);
		v.pred = &u;
		v.g = g;
		v.h = v.p.length(m_vT.p);
		v.f = g + v.h;
		v.turn = geo::is_strict_between_cw_ge(uv, -Vertex::corner_pt_x<false, 0>(v.corner), Vertex::corner_pt_x<true, 0>(v.corner)) ? geo::CW : geo::CCW;
		m_nodePush.push_back(&v);
		DEBUG_SEARCH_CMD(m_sid.id, std::cerr << "push: " << geo::bracket_point(u.p) << " -> " << geo::bracket_point(v.p) << " g: " << v.g << " f: " << v.f << std::endl)
	}
	DEBUG_SEARCH_CMD(m_sid.id, m_debugSearch.fill_point(v.p.x, v.p.y, 'x'))
}
void Expansion::push_target(Vertex&u, Vertex& t)
{
	const auto ut = t.p - u.p;
	t.pred = &u;
	t.g = u.g + ut.length();
	m_done = true;
	DEBUG_SEARCH_CMD(m_sid.id, std::cerr << "push target: " << geo::bracket_point(u.p) << " -> " << geo::bracket_point(t.p) << " g: " << t.g << " f: " << t.f << std::endl)
	DEBUG_SEARCH_CMD(m_sid.id, m_debugSearch.fill_point(t.p.x, t.p.y, 't'))
}

} // namespace rayscan::search
