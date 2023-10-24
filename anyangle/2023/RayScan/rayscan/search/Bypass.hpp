#ifndef RAYSCAN_SEARCH_BYPASS_HPP
#define RAYSCAN_SEARCH_BYPASS_HPP

#include "fwd.hpp"
#include "Scan.hpp"

namespace rayscan::search {

template <int8_t MaxJumps>
struct Bypass
{
	static_assert(MaxJumps >= 1, "MaxJumps must be greater than 1");

	/**
	 * @param sector Scan angle sector
	 * @param uv Bypass of v from u
	 * @param next_uv Bypass next uv 
	 * @param vt Target from v
	 * @param dir Next dir to progress bypass
	 * @return A pair of points from CW and CCW scan that was reached, or zero.
	 */
	template <geo::Ori D>
	static std::tuple<geo::Point, geo::Compass, geo::Compass> bypass(const env::Grid::table::pack_type** tables, env::Grid::table::adj_index* idx, geo::BasicAngleSector sector, geo::Point uv, geo::Point next_uv, geo::Point ut, geo::Compass dir, geo::Compass next_dir)
	{
		static_assert(D == geo::CW || D == geo::CCW, "D must be CW or CCW");
		assert(sector.within(uv));
		// const geo::Point turning_point = uv;
		geo::BasicAngleSector proj_field(geo::BasicAngleSector::create_sector<D>(geo::compass_point(dir), uv));
		// bool target = !geo::is_strict_between_ge<D>(ut - turning_point, turning_point, geo::compass_point(dir));
		const bool target = proj_field.within(ut - uv);
		int8_t remaining_jumps = MaxJumps;
		do {
			if (!sector.within(next_uv))
				return {geo::Point::zero(), geo::Compass::NORTH, geo::Compass::NORTH}; // all points must fall within scan's sector
			dir = next_dir;
			next_dir = scan::scan_trace_progress(tables, idx, next_uv, next_dir, D);
			if (target) {
				// check if target is reachable
				if (auto uvXnext = geo::cross(proj_field[geo::BasicAngleSector::ori_id_v<D>], uv, next_uv); geo::is_co(uvXnext)) [[unlikely]] {
					// points V, A, B, T
					// uvXnext = area of VAB; triangles below:           VAT                           ABT                              BVT
					geo::coord_op sub_triangle_area = geo::cross(proj_field[geo::BasicAngleSector::ori_id_v<D>], uv, ut) + geo::cross(uv, next_uv, ut) + geo::cross(next_uv, uv, ut);
					if (sub_triangle_area == uvXnext) {
						// target is on or inside triangle
						return {geo::Point::zero(), geo::Compass::NORTH, geo::Compass::NORTH}; // target within
					}
				} // no need to check colin, will be discovered by one of neighboring triangles
			}
			if (auto nuv_adj = next_uv - proj_field[geo::BasicAngleSector::ori_id_v<D>]; !proj_field.within(nuv_adj)) {
				if (!geo::is_ori<D>(proj_field[geo::BasicAngleSector::ori_id_v<D>], nuv_adj)) {
					return {geo::Point::zero(), geo::Compass::NORTH, geo::Compass::NORTH};
				} else {
					return {next_uv, dir, next_dir};
				}
			}
		} while (--remaining_jumps >= 0);
		return {geo::Point::zero(), geo::Compass::NORTH, geo::Compass::NORTH};
	}
};

} // namespace rayscan::search

#endif // RAYSCAN_SEARCH_BYPASS_HPP
