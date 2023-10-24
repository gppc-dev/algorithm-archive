#include <search/RayShoot.hpp>
#include "Search.hpp"

namespace rayscan::search
{

	RayShoot::RayShoot()
	{
	}

	void RayShoot::setup(Search& search)
	{
		m_grid = search.get_grid();
#if DEBUG_RAYSHOOT != 0
		m_debugRay.setup(l_grid.getTable()[0]);
#endif
	}

#if DEBUG_RAYSHOOT != 0
#define DEBUG_RAYSHOOT_INIT(x, t) m_debugRay.init_ray(x, t)
#define DEBUG_RAYSHOOT_PUT(x, y, c) m_debugRay.fill_point(x, y, c)
#else
#define DEBUG_RAYSHOOT_INIT(x, t)
#define DEBUG_RAYSHOOT_PUT(x, y, c)
#endif

	auto RayShoot::shoot(geo::Point u, geo::Point uv) -> shoot_res
	{
#ifndef NDEBUG
		// checks that the ray is not going to imminently intersect an obstacle (or is inside one), that is now allowed
		using gv = env::GridVertex;
		gv vdebug;
		auto debug_region = m_grid->getTable()[0].region<1,1,2,2>(u.x, u.y);
		switch (debug_region) {
		case 0b0000: // non-traversable
			assert(false);
			break;
		case 0b1111: // fully-traversable
			break;
		default:
			vdebug.fill_mask(debug_region);
			assert(vdebug.ambig != gv::Ambig::AMBIG_XY_NOSIGN); // this ambig case is never a point
			bool Q1within = !geo::is_strict_between_cw_any(uv, gv::corner_pt_x<false>(vdebug.corner), gv::corner_pt_x<true>(vdebug.corner));
			bool Q2within = vdebug.ambig == gv::Ambig::AMBIG_XY_SIGN && !geo::is_strict_between_cw_any(uv, gv::corner_pt_x<false, 1>(vdebug.corner), gv::corner_pt_x<true, 1>(vdebug.corner));
			assert(Q1within || Q2within);
			break;
		}
#endif
		shoot_res res;
		if (std::abs(uv.y) <= std::abs(uv.x))
		{
			DEBUG_RAYSHOOT_INIT(m_grid->getTable()[0], false);
			res = shoot_dir(m_grid->getTable()[0], u, uv);
		}
		else
		{
			DEBUG_RAYSHOOT_INIT(m_grid->getTable()[1], true);
			res = shoot_dir(m_grid->getTable()[1], u.transpose(), uv.transpose());
			res.first = res.first.transpose();
			res.second = static_cast<Shoot>(res.second ^ HV_HORI);
		}
#if DEBUG_RAYSHOOT != 0
		m_debugRay.print();
#endif
#ifndef NDEBUG
		auto rect = m_grid->getTable()[0].region<1,1,2,2>(u.x + res.first.x, u.y + res.first.y);
		assert(rect != 0b0000 && rect != 0b1111);
#endif
		return res;
	}

	auto RayShoot::shoot_dir(const table_type &table, geo::Point u, geo::Point uv) -> shoot_res
	{
		DEBUG_RAYSHOOT_PUT(u.x, u.y, '*');
		DEBUG_RAYSHOOT_PUT(u.x - 1, u.y, '*');
		DEBUG_RAYSHOOT_PUT(u.x, u.y - 1, '*');
		DEBUG_RAYSHOOT_PUT(u.x - 1, u.y - 1, '*');
		if (uv.y == 0)
		{
			if (uv.x < 0)
			{
				int len = shoot_cardinal_neg(table, u);
				return shoot_res(geo::Point(-len, 0), static_cast<Shoot>(POINT | VERT_NEG | (static_cast<uint8_t>(len > -uv.x) << extra::SEG_SHIFT)));
			}
			else
			{
				int len = shoot_cardinal_pos(table, u);
				return shoot_res(geo::Point(len, 0), static_cast<Shoot>(POINT | VERT_POS | (static_cast<uint8_t>(len > uv.x) << extra::SEG_SHIFT)));
			}
		}
		else
		{
			return shoot_ordinal(table, u, uv);
		}
	}

#if 0
int RayShoot::shoot_cardinal_pos(const env::Grid::table& table, geo::Point u)
{
	constexpr int32_t word_adj = static_cast<uint8_t>(table_type::item_count - 1);
	auto it = table.bit_adj_index(u.x, u.y);
	const uint32_t words_row = it.row_word();
	int32_t length;
	const auto* words = table.data() + (it.word() - words_row); // words[0] = above row, words[words_row] = below row
	std::array<table_type::pack_type, 2> word_value;
	std::array<uint8_t, 2> prev_msb{}; // previous words most-significant-bit
	{
		const int bt = it.bit();
		const auto mask = ~inx::make_mask_limit<table_type::pack_type>(bt);
		length = -bt;
		word_value[0] = ~words[0] & mask;
		word_value[1] = ~words[words_row] & mask;
	}
	// begin scanning (shooting)
	while (true) {
		table_type::pack_type value = ( word_value[0] & word_value[1] ) // detect wall
		                            | ( ((word_value[0] >> 1) | (static_cast<table_type::pack_type>(prev_msb[0]) << word_adj)) & word_value[1] ) // detect ambig
		                            | ( word_value[0] & ((word_value[1] >> 1) | (static_cast<table_type::pack_type>(prev_msb[1]) << word_adj)) ); // detect ambig
		prev_msb[0] = static_cast<uint8_t>(word_value[0] >> word_adj);
		prev_msb[1] = static_cast<uint8_t>(word_value[1] >> word_adj);
		if (value == 0) {
			length += word_adj+1;
			++words;
			word_value[0] = ~words[0];
			word_value[1] = ~words[words_row];
		} else {
			length += std::countr_zero(value);
			assert(length >= 0);
			return length;
		}
	}
}

int RayShoot::shoot_cardinal_neg(const env::Grid::table& table, geo::Point u)
{
	constexpr int32_t word_adj = static_cast<uint8_t>(table_type::item_count - 1);
	auto it = table.bit_adj_index(u.x-1, u.y);
	const uint32_t words_row = it.row_word();
	int32_t length;
	const auto* words = table.data() + (it.word() - words_row); // words[0] = above row, words[words_row] = below row
	std::array<table_type::pack_type, 2> word_value;
	std::array<uint8_t, 2> prev_msb{}; // previous words most-significant-bit
	{
		const int bt = word_adj - it.bit();
		const auto mask = ~inx::make_mask<table_type::pack_type>(bt, word_adj - bt);
		length = -bt;
		word_value[0] = ~words[0] & mask;
		word_value[1] = ~words[words_row] & mask;
	}
	// begin scanning (shooting)
	while (true) {
		table_type::pack_type value = ( word_value[0] & word_value[1] ) // detect wall
		                            | ( ((word_value[0] << 1) | static_cast<table_type::pack_type>(prev_msb[0])) & word_value[1] ) // detect ambig
		                            | ( word_value[0] & ((word_value[1] << 1) | static_cast<table_type::pack_type>(prev_msb[1])) ); // detect ambig
		prev_msb[0] = static_cast<uint8_t>(word_value[0] & 1);
		prev_msb[1] = static_cast<uint8_t>(word_value[1] & 1);
		if (value == 0) {
			length += word_adj+1;
			--words;
			word_value[0] = ~words[0];
			word_value[1] = ~words[words_row];
		} else {
			length += std::countl_zero(value);
			assert(length >= 0);
			return length;
		}
	}
}
#else
	// non-aligned access
	int RayShoot::shoot_cardinal_pos(const env::Grid::table &table, geo::Point u)
	{
		constexpr int32_t word_size = static_cast<int32_t>(sizeof(table_type::pack_type) - 1);
		constexpr int32_t word_adj = inx::byte_size * word_size;
		auto it = table.bit_adj_index(u.x, u.y - 1);
		const uint32_t words_row = (word_size + 1) * it.row_word();
		int32_t length;
		const std::byte *words = std::bit_cast<const std::byte *>(table.data() + it.word()); // words[0-7] = above row, words[words_row-7] = below row
		std::array<table_type::pack_type, 2> word_value;
		{
			int bt = it.bit();
			words += (bt >> inx::byte_cnt) - 1;
			bt = bt & inx::make_mask_v<int, inx::byte_cnt>;
			length = -bt;
			std::memcpy(&word_value[0], words, sizeof(table_type::pack_type));
			word_value[0] = ~word_value[0] & ~inx::make_mask_limit<table_type::pack_type>(inx::byte_size + bt);
			std::memcpy(&word_value[1], words + words_row, sizeof(table_type::pack_type));
			word_value[1] = ~word_value[1] & ~inx::make_mask_limit<table_type::pack_type>(inx::byte_size + bt);
		}
		// begin scanning (shooting)
		while (true)
		{
			table_type::pack_type value = (word_value[0] & word_value[1])			// detect wall
										  | ((word_value[0] << 1) & word_value[1])	// detect ambig
										  | (word_value[0] & (word_value[1] << 1)); // detect ambig
			if (value == 0)
			{
				length += word_adj;
				words += word_size;
				std::memcpy(&word_value[0], words, sizeof(table_type::pack_type));
				word_value[0] = ~word_value[0] & ~inx::make_mask_v<table_type::pack_type, inx::byte_size - 1>;
				std::memcpy(&word_value[1], words + words_row, sizeof(table_type::pack_type));
				word_value[1] = ~word_value[1] & ~inx::make_mask_v<table_type::pack_type, inx::byte_size - 1>;
			}
			else
			{
				length += std::countr_zero(value) - inx::byte_size;
				assert(length >= 0);
				for (int32_t i = 0; i < length; ++i) {
					DEBUG_RAYSHOOT_PUT(u.x + i, u.y, '-');
					DEBUG_RAYSHOOT_PUT(u.x + i, u.y-1, '-');
				}
				return length;
			}
		}
	}

	int RayShoot::shoot_cardinal_neg(const env::Grid::table &table, geo::Point u)
	{
		constexpr int32_t word_size = static_cast<int32_t>(sizeof(table_type::pack_type) - 1);
		constexpr int32_t word_adj = inx::byte_size * word_size;
		auto it = table.bit_adj_index(u.x - 1, u.y - 1); // --it.id;
		const uint32_t words_row = (word_size + 1) * it.row_word();
		int32_t length;
		const std::byte *words = std::bit_cast<const std::byte *>(table.data() + it.word()); // words[0-7] = above row, words[words_row-7] = below row
		std::array<table_type::pack_type, 2> word_value;
		{
			int bt = it.bit();
			words += (bt >> inx::byte_cnt) - (word_size - 1);
			bt = bt & inx::make_mask_v<int, inx::byte_cnt>;
			length = bt - (inx::byte_size - 1);
			std::memcpy(&word_value[0], words, sizeof(table_type::pack_type));
			word_value[0] = ~word_value[0] & ~inx::make_msb_mask_limit<table_type::pack_type>(inx::byte_size - length);
			std::memcpy(&word_value[1], words + words_row, sizeof(table_type::pack_type));
			word_value[1] = ~word_value[1] & ~inx::make_msb_mask_limit<table_type::pack_type>(inx::byte_size - length);
		}
		// begin scanning (shooting)
		while (true)
		{
			table_type::pack_type value = (word_value[0] & word_value[1])			// detect wall
										  | ((word_value[0] >> 1) & word_value[1])	// detect ambig
										  | (word_value[0] & (word_value[1] >> 1)); // detect ambig
			if (value == 0)
			{
				length += word_adj;
				words -= word_size;
				std::memcpy(&word_value[0], words, sizeof(table_type::pack_type));
				word_value[0] = ~word_value[0] & ~inx::make_msb_mask_v<table_type::pack_type, inx::byte_size - 1>;
				std::memcpy(&word_value[1], words + words_row, sizeof(table_type::pack_type));
				word_value[1] = ~word_value[1] & ~inx::make_msb_mask_v<table_type::pack_type, inx::byte_size - 1>;
			}
			else
			{
				length += std::countl_zero(value) - inx::byte_size;
				assert(length >= 0);
				for (int32_t i = 0; i < length; ++i) {
					DEBUG_RAYSHOOT_PUT(u.x-1 - i, u.y, '-');
					DEBUG_RAYSHOOT_PUT(u.x-1 - i, u.y-1, '-');
				}
				return length;
			}
		}
	}
#endif

	auto RayShoot::shoot_ordinal(const table_type &table, geo::Point u, geo::Point uv) -> shoot_res
	{
		// dia restrcted to +-45 and 180+-45 angles
		assert(uv.x != 0 && uv.y != 0 && std::abs(uv.x) >= std::abs(uv.y));
		auto it = table.bit_adj_index(u.x, u.y);
		// will iterate along the x-axis, and adjust the y-axis when crossing
		uint16_t y_frac_step, y_frac_a, y_frac_b;
		int16_t len;
		int16_t x_adj;
		int16_t y_adj;
		int32_t y_id_adj;
		geo::Point intersect = geo::Point::zero();
#if DEBUG_RAYSHOOT != 0
		geo::Point intersect_adj = geo::Point(uv.x > 0 ? 0 : -1, uv.y > 0 ? 0 : -1);
#endif
		// skip first step, assumed free
		if (uv.x > 0)
		{
			y_frac_b = static_cast<uint16_t>(uv.x);
			len = -uv.x;
			x_adj = 1;
		}
		else
		{
			y_frac_b = static_cast<uint16_t>(-uv.x);
			len = uv.x;
			x_adj = -1;
			it.id += x_adj;
		}
		if (uv.y > 0)
		{
			y_frac_step = static_cast<uint16_t>(uv.y);
			y_frac_a = 0;
			y_adj = 1;
			y_id_adj = it.row;
		}
		else
		{
			y_frac_step = static_cast<uint16_t>(-uv.y);
			y_frac_a = 0;
			y_adj = -1;
			y_id_adj = -it.row;
			it.id += y_id_adj;
		}
		// q = u + (l / uv.x) * uv
		// q.x = u.x + l
		// q.y = u.y + (1/uv.x) * uv.y
		while (true)
		{
			DEBUG_RAYSHOOT_PUT(u.x + intersect.x + intersect_adj.x, u.y + intersect.y + intersect_adj.y, '-');
			if (!table.bit_test<0>(it))
			{
				intersect.y = y_adj >= 0 ? intersect.y : (intersect.y - 1);
				return shoot_res(intersect, static_cast<Shoot>((static_cast<uint8_t>(y_frac_a != 0) << extra::POS_SHIFT) | (uv.x < 0 ? VERT_NEG : VERT_POS) | (static_cast<uint8_t>(len > 0) << extra::SEG_SHIFT))); // collision
			}
			y_frac_a += y_frac_step;
			if (y_frac_a > y_frac_b)
			{
				// passes y-axis on non-corner
				y_frac_a -= y_frac_b;
				assert(y_frac_a < y_frac_b);
				it.id += y_id_adj;
				intersect.y += y_adj;
				DEBUG_RAYSHOOT_PUT(u.x + intersect.x + intersect_adj.x, u.y + intersect.y + intersect_adj.y, '|');
				if (!table.bit_test<0>(it))
				{
					intersect.x = x_adj >= 0 ? intersect.x : (intersect.x - 1);
					return shoot_res(intersect, static_cast<Shoot>(EDGE | (uv.y < 0 ? HORI_NEG : HORI_POS) | (static_cast<uint8_t>(len > 0) << extra::SEG_SHIFT))); // collision
				}
			}
			else if (y_frac_a == y_frac_b)
			{
				y_frac_a -= y_frac_b;
				// 0b3210
				// 0 1
				// 2 3
				// logic: ambig case 0b1001 and 0b0110
				// if x > 0:
				//    if y > 0: region & 0b1000 || 0b0110
				//    if y < 0: region & 0b0010 || 0b1001
				// if x < 0:
				//    if y > 0: region & 0b0100 || 0b1001
				//    if y < 0: region & 0b0001 || 0b0110
				// thus, as intersecting ambig case can never appear, we only need to test one ambig case
				DEBUG_RAYSHOOT_PUT(u.x + intersect.x + intersect_adj.x + x_adj, u.y + intersect.y + intersect_adj.y, '/');
				DEBUG_RAYSHOOT_PUT(u.x + intersect.x + intersect_adj.x, u.y + intersect.y + intersect_adj.y + y_adj, '/');
				if (
					(~(table.bit_get(table_type::index_t(static_cast<uint64_t>(static_cast<int64_t>(it.id) + x_adj))) | table.bit_get(table_type::index_t(static_cast<uint64_t>(static_cast<int64_t>(it.id) + y_id_adj)))) // diagonal
					| ~table.bit_get(table_type::index_t(static_cast<uint64_t>(static_cast<int64_t>(it.id) + x_adj + y_id_adj)))) // other dia
					& 1)
				{
					intersect.x += x_adj;
					intersect.y += y_adj;
					DEBUG_RAYSHOOT_PUT(u.x + intersect.x + intersect_adj.x, u.y + intersect.y + intersect_adj.y, '/');
					return shoot_res(intersect, static_cast<Shoot>(POINT | (static_cast<uint8_t>(len + 1 > 0) << extra::SEG_SHIFT))); // collision ambig
				}
				it.id += y_id_adj;
				intersect.y += y_adj;
			}
			it.id += x_adj;
			intersect.x += x_adj;
			len += 1;
		}
	}

} // namespace rayscan::search
