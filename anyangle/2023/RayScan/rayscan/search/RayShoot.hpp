#ifndef RAYSCAN_SEARCH_RAYSHOOT_HPP
#define RAYSCAN_SEARCH_RAYSHOOT_HPP

#include "fwd.hpp"
#include <env/Grid.hpp>
#include "DebugStruct.hpp"

namespace rayscan::search {

class RayShoot
{
public:
	using table_type = env::Grid::table;
	RayShoot();

	void setup(Search& search);
	shoot_res shoot(geo::Point u, geo::Point uv);

protected:
	shoot_res shoot_dir(const table_type& table, geo::Point u, geo::Point uv);
	int32_t shoot_cardinal_pos(const table_type& table, geo::Point u);
	int32_t shoot_cardinal_neg(const table_type& table, geo::Point u);
	shoot_res shoot_ordinal(const table_type& table, geo::Point u, geo::Point uv);

private:
	const env::Grid* m_grid;
#if DEBUG_RAYSHOOT != 0
	DebugRayShoot m_debugRay;
#endif
};

} // namespace rayscan::search

#endif // RAYSCAN_SEARCH_RAYSHOOT_HPP
