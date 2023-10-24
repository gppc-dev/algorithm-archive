#include "Scan.hpp"
#include "Search.hpp"
#include "Bypass.hpp"

namespace rayscan::search {

const env::Grid* scan::get_grid(const Search& search) noexcept
{
	return search.get_grid();
}

void SimpleScanner::setup(Search& search)
{
	m_exp = &search.get_expansion();
}

template <geo::Ori D>
geo::Point SimpleScanner::scan_trace(std::array<const env::Grid::table::pack_type*, 2> tables, std::array<env::Grid::table::adj_index, 2> idx, geo::BasicAngleSector sector, geo::Point u, geo::Point uv, geo::Point ray, geo::Compass dir, uint8_t)
{
	assert(!geo::is_ori<inv_ori(D)>(uv, sector[geo::BasicAngleSector::ori_id_v<D>]));
	assert(!geo::is_ori<inv_ori(D)>(ray, uv));
	geo::Point ut = m_exp->m_vT.p - m_exp->m_vU->p;
	geo::Compass next_dir;
	ray = uv;
	while (true) {
		next_dir = scan::scan_trace_progress(tables.data(), idx.data(), uv, dir, D);
		if (auto rayXuv = geo::cross(ray, uv); !geo::is_ori<D>(rayXuv)) {
			if (geo::is_ori<inv_ori(D)>(rayXuv)) { // scan reverse, turning-point
				std::tie(uv, std::ignore, next_dir) = Bypass<RAYSCAN_BYPASS_COUNT>::bypass<D>(tables.data(), idx.data(), sector, ray, uv, ut, dir, next_dir);
				if (uv.is_zero())
					return ray;
			} else if (geo::is_opposite_or_zero_dir(ray, uv)) {
				return geo::Point::zero();
			}
		}
		if (geo::is_ori<D>(sector[geo::BasicAngleSector::ori_id_v<D>], uv)) {
			return geo::Point::zero(); // end search with nothing
		}
		ray = uv;
		dir = next_dir;
	}
}
template geo::Point SimpleScanner::scan_trace<geo::CW>(std::array<const env::Grid::table::pack_type*, 2> tables, std::array<env::Grid::table::adj_index, 2> idx, geo::BasicAngleSector sector, geo::Point u, geo::Point uv, geo::Point ray, geo::Compass dir, uint8_t);
template geo::Point SimpleScanner::scan_trace<geo::CCW>(std::array<const env::Grid::table::pack_type*, 2> tables, std::array<env::Grid::table::adj_index, 2> idx, geo::BasicAngleSector sector, geo::Point u, geo::Point uv, geo::Point ray, geo::Compass dir, uint8_t);

void ConvexScanner::setup(Search& search)
{
	m_exp = &search.get_expansion();
}

template <geo::Ori D>
geo::Point ConvexScanner::scan_trace(std::array<const env::Grid::table::pack_type*, 2> tables, std::array<env::Grid::table::adj_index, 2> idx, geo::BasicAngleSector sector, geo::Point u, geo::Point uv, geo::Point ray, geo::Compass dir, uint8_t start_region)
{
	assert(sector.within(uv));
	assert(!geo::is_ori<inv_ori(D)>(ray, uv));
	geo::Point ut = m_exp->m_vT.p - m_exp->m_vU->p;
	ray = uv;
	bool backwards_scan = false;
	bool ray_convex_point = std::popcount(start_region) == 1;
	geo::Compass next_dir;
	while (true) {
		next_dir = scan::scan_trace_progress(tables.data(), idx.data(), uv, dir, D);
		if (auto rayXuv = geo::cross(ray, uv); geo::is_co(rayXuv)) [[unlikely]] {
			if (geo::is_opposite_or_zero_dir(ray, uv)) [[unlikely]] {
				return geo::Point::zero(); // passed point u, quit
			}
		} else if (geo::is_ori<inv_ori(D)>(rayXuv) != backwards_scan) {
			// LOGIC [ first: scanning in reverse, second: scan is going backwards ]
			// [false, false] = no-op, forward scan
			// [false, true] = op, forward scan after a backward scan
			// [true, false] = op, backward scan, may be a turning-point if v is a convex point
			// [true, true] = no-op, backward scan after backward scan
			if (ray_convex_point) { // is a turning-point
				if (!backwards_scan) {
					// forward turning-point
					std::tie(uv, dir, next_dir) = Bypass<4>::bypass<D>(tables.data(), idx.data(), sector, ray, uv, ut - ray, dir, next_dir);
					if (uv.is_zero())
						return ray;
				} else {
					// backward turning-point
					if (auto ptr = m_exp->shoot_backward_turning_point<D>(sector, u, ray); !ptr.second) {
						backwards_scan = false;
						sector = sector.split_sector<D>(ray);
					} else {
						return ptr.first;
					}
				}
			} else {
				backwards_scan = true;
			}
		}
		if (!sector.within(uv)) {
			return geo::Point::zero(); // end search with nothing
		}
		ray = uv;
		ray_convex_point = D == geo::CCW ? scan::is_convex_turn(dir, next_dir) : scan::is_convex_turn(next_dir, dir);
		dir = next_dir;
	}
}
template geo::Point ConvexScanner::scan_trace<geo::CW>(std::array<const env::Grid::table::pack_type*, 2> tables, std::array<env::Grid::table::adj_index, 2> idx, geo::BasicAngleSector sector, geo::Point u, geo::Point uv, geo::Point ray, geo::Compass dir, uint8_t start_region);
template geo::Point ConvexScanner::scan_trace<geo::CCW>(std::array<const env::Grid::table::pack_type*, 2> tables, std::array<env::Grid::table::adj_index, 2> idx, geo::BasicAngleSector sector, geo::Point u, geo::Point uv, geo::Point ray, geo::Compass dir, uint8_t start_region);

} // namespace rayscan::search
