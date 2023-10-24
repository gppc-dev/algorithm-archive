#ifndef RAYSCAN_SEARCH_VERTEX_HPP
#define RAYSCAN_SEARCH_VERTEX_HPP

#include "fwd.hpp"
#include <env/Grid.hpp>

namespace rayscan::search {

struct Vertex : env::GridVertex
{
	using env::GridVertex::GridVertex;
	Vertex(const env::GridVertex& gv) noexcept : GridVertex(gv), sid{}, f{}, g{}, h{}, pred{}, ray_point{}, pred_point{}, ray_shoot{}, pred_shoot{}, open{}, close{}, turn{} { }

	SearchId sid;
	ExpId rayId;
	double f;
	double g;
	double h;
	Vertex* pred;
	shoot_res::first_type ray_point, pred_point;
	shoot_res::second_type ray_shoot, pred_shoot;
	bool open;
	bool close;
	geo::Ori turn;

	void init(SearchId l_sid) noexcept
	{
		sid = l_sid;
		f = g = inx::inf<double>;
		h = 0;
		pred = nullptr;
		open = close = false;
		turn = geo::Ori::CO;
	}
	void search(SearchId l_sid) noexcept
	{
		if (sid.id != l_sid.id)
			init(l_sid);
	}

	geo::BasicAngleSector projection_field() const noexcept
	{
		assert(pred != nullptr && turn != geo::CO);
		const int projBoundId = geo::Region::ori_id(turn);
		geo::BasicAngleSector as;
		as[projBoundId] = corner_pt_x(corner, static_cast<bool>(projBoundId ^ 1));
		as[projBoundId^1] = p - pred->p;
		return as;
	}

	void set_ray_cache(shoot_res r) noexcept
	{
		ray_point = r.first;
		ray_shoot = r.second;
	}
	shoot_res get_ray_cache() const noexcept
	{
		return shoot_res(ray_point, ray_shoot);
	}
	void set_pred_cache(shoot_res r) noexcept
	{
		pred_point = r.first;
		pred_shoot = r.second;
	}
	shoot_res get_pred_cache() const noexcept
	{
		return shoot_res(pred_point, pred_shoot);
	}
};

} // namespace rayscan::search

#endif // RAYSCAN_SEARCH_VERTEX_HPP
