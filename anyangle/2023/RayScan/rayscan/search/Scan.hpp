#ifndef RAYSCAN_SEARCH_SCAN_HPP
#define RAYSCAN_SEARCH_SCAN_HPP

#include "fwd.hpp"
#include <geo/Point.hpp>
#include <geo/AngleSector.hpp>
#include <env/Grid.hpp>

namespace rayscan::search {

namespace scan {

const env::Grid* get_grid(const Search& search) noexcept;

inline bool is_convex_turn(geo::Compass from, geo::Compass to) noexcept
{
	// NORTH = 0,
	// SOUTH = 1,
	// EAST = 3,
	// WEST = 2
	// 0 -> 3 00 -> 11
	// 3 -> 1 11 -> 01
	// 1 -> 2 01 -> 10
	// 2 -> 0 10 -> 00
	return to == static_cast<geo::Compass>((static_cast<uint8_t>(0b01'00'10'11u) >> (2*static_cast<uint8_t>(from))) & 0b11);
}

template <geo::Compass C>
inline static std::pair<int16_t, geo::Compass> scan_jump(const env::Grid::table::pack_type* words, env::Grid::table::adj_index* idx, geo::Ori D)
{
	constexpr int16_t word_len = static_cast<int16_t>(env::Grid::table::pack_bits);
	const int16_t mult = geo::is_cw(D) ? 1 : -1;
	int16_t len;
	int16_t word_adj; // the adj of word_id for the non-obstacle
	geo::Compass turn_dir;
	env::Grid::table::pack_type word_check; // starts with mask
	if constexpr (C == geo::Compass::NORTH) {
		env::Grid::table::adj_index cidx = idx[1];
		cidx.adj_col(-1);
		len = static_cast<int16_t>(cidx.bit()) - (word_len-1);
		words += static_cast<int32_t>(cidx.word()); // - static_cast<int32_t>(cidx.bit() == 0);
		word_adj = static_cast<int16_t>(cidx.row_word());
		words = (geo::is_cw(D) ? words - word_adj : words);
		word_adj = (geo::is_cw(D) ? word_adj : -word_adj);
		word_check = inx::make_msb_mask_limit<decltype(word_check)>(-len);
	} else if constexpr (C == geo::Compass::SOUTH) {
		env::Grid::table::adj_index cidx = idx[1];
		len = -static_cast<int16_t>(cidx.bit());
		words += static_cast<int32_t>(cidx.word());
		word_adj = static_cast<int16_t>(cidx.row_word());
		words = (geo::is_ccw(D) ? words - word_adj : words);
		word_adj = (geo::is_ccw(D) ? word_adj : -word_adj);
		word_check = inx::make_mask_limit<decltype(word_check)>(-len);
	} else if constexpr (C == geo::Compass::EAST) {
		env::Grid::table::adj_index cidx = idx[0];
		len = -static_cast<int16_t>(cidx.bit());
		words += static_cast<int32_t>(cidx.word());
		word_adj = static_cast<int16_t>(cidx.row_word());
		words = (geo::is_cw(D) ? words - word_adj : words);
		word_adj = (geo::is_cw(D) ? word_adj : -word_adj);
		word_check = inx::make_mask_limit<decltype(word_check)>(-len);
	} else { // geo::Compass::WEST
		env::Grid::table::adj_index cidx = idx[0];
		cidx.adj_col(-1);
		len = static_cast<int16_t>(cidx.bit()) - (word_len-1);
		words += static_cast<int32_t>(cidx.word()); // - static_cast<int32_t>(cidx.bit() == 0);
		word_adj = static_cast<int16_t>(cidx.row_word());
		words = (geo::is_ccw(D) ? words - word_adj : words);
		word_adj = (geo::is_ccw(D) ? word_adj : -word_adj);
		word_check = inx::make_msb_mask_limit<decltype(word_check)>(-len);
	}
	if constexpr (C == geo::Compass::EAST || C == geo::Compass::SOUTH) { // positive increase
		word_check = ~(*words & ~word_check) & (words[word_adj] | word_check);
		while (~word_check == 0) { // full line scan, skip
			len += word_len;
			++words;
			word_check = ~*words & words[word_adj];
		}
		int16_t count = std::countr_one(word_check);
		len += count;
		if constexpr (C == geo::Compass::EAST) {
			turn_dir = (words[word_adj] & (static_cast<env::Grid::table::pack_type>(1) << count)) ? geo::Compass::NORTH : geo::Compass::SOUTH;
			turn_dir = geo::is_cw(D) ? turn_dir : static_cast<geo::Compass>(static_cast<int>(turn_dir) ^ 1);
		} else {
			turn_dir = (words[word_adj] & (static_cast<env::Grid::table::pack_type>(1) << count)) ? geo::Compass::EAST : geo::Compass::WEST;
			turn_dir = geo::is_cw(D) ? turn_dir : static_cast<geo::Compass>(static_cast<int>(turn_dir) ^ 1);
		}
	} else { // geo::Compass::NORTH || C == geo::Compass::WEST
		word_check = ~(*words & ~word_check) & (words[word_adj] | word_check);
		while (~word_check == 0) { // full line scan, skip
			len += word_len;
			--words;
			word_check = ~*words & words[word_adj];
		}
		int16_t count = std::countl_one(word_check);
		len += count;
		if constexpr (C == geo::Compass::WEST) {
			turn_dir = (words[word_adj] & (inx::bit_shift_set_v<0, env::Grid::table::pack_bits-1, static_cast<env::Grid::table::pack_type>(1)> >> count)) ? geo::Compass::SOUTH : geo::Compass::NORTH;
			turn_dir = geo::is_cw(D) ? turn_dir : static_cast<geo::Compass>(static_cast<int>(turn_dir) ^ 1);
		} else {
			turn_dir = (words[word_adj] & (inx::bit_shift_set_v<0, env::Grid::table::pack_bits-1, static_cast<env::Grid::table::pack_type>(1)> >> count)) ? geo::Compass::WEST : geo::Compass::EAST;
			turn_dir = geo::is_cw(D) ? turn_dir : static_cast<geo::Compass>(static_cast<int>(turn_dir) ^ 1);
		}
	}
	assert(len >= 0);
	if constexpr (C == geo::Compass::NORTH) {
		idx[0].adj_row(-len);
		idx[1].adj_col(-len);
	} else if constexpr (C == geo::Compass::SOUTH) {
		idx[0].adj_row(len);
		idx[1].adj_col(len);
	} else if constexpr (C == geo::Compass::EAST) {
		idx[0].adj_col(len);
		idx[1].adj_row(len);
	} else { // geo::Compass::WEST
		idx[0].adj_col(-len);
		idx[1].adj_row(-len);
	}
	return {len, turn_dir};
}

inline geo::Compass scan_trace_progress(const env::Grid::table::pack_type** tables, env::Grid::table::adj_index* idx, geo::Point& uv, geo::Compass dir, geo::Ori D) noexcept
{
	std::pair<int16_t, geo::Compass> r;
	switch (dir) {
	case geo::Compass::NORTH:
		r = scan::scan_jump<geo::Compass::NORTH>(tables[1], idx, D);
		uv.y -= r.first;
		break;
	case geo::Compass::SOUTH:
		r = scan::scan_jump<geo::Compass::SOUTH>(tables[1], idx, D);
		uv.y += r.first;
		break;
	case geo::Compass::EAST:
		r = scan::scan_jump<geo::Compass::EAST>(tables[0], idx, D);
		uv.x += r.first;
		break;
	case geo::Compass::WEST:
		r = scan::scan_jump<geo::Compass::WEST>(tables[0], idx, D);
		uv.x -= r.first;
		break;
	}
	return r.second;
}

template <typename T>
concept ScannerSectorPart = requires(T scanner, std::array<const env::Grid::table::pack_type*, 2> tables, std::array<env::Grid::table::adj_index, 2> idx, geo::Point p, geo::Compass d, uint8_t reg)
{
	{ scanner.template scan_trace<geo::Ori::CW>(tables, idx, p, p, p, p, d, reg) } -> std::convertible_to<geo::Point>;
};
template <typename T>
concept ScannerSectorFull = requires(T scanner, std::array<const env::Grid::table::pack_type*, 2> tables, std::array<env::Grid::table::adj_index, 2> idx, geo::BasicAngleSector as, geo::Point p, geo::Compass d, uint8_t reg)
{
	{ scanner.template scan_trace<geo::Ori::CW>(tables, idx, as, p, p, p, d, reg) } -> std::convertible_to<geo::Point>;
};
template <typename T>
concept Scanner = ScannerSectorPart<T> || ScannerSectorFull<T>;

} // namespace scan

template <scan::Scanner S>
class Scan final : public S
{
public:
	Scan() : m_table(nullptr)
	{ }

	void setup(Search& search)
	{
		m_table = scan::get_grid(search)->getTable().data();
		S::setup(search);
	}

	/**
	 * @param sector Scan angle sector
	 * @param u Scan origin point
	 * @param v Scan start point along obstacle
	 * @param ray The uv of the ray, can differ slightly as ray may not intersect on a lattice point
	 * @return A pair of points from CW and CCW scan that was reached, or zero.
	 */
	template <geo::Ori D>
	geo::Point full_scan(geo::BasicAngleSector sector, geo::Point u, geo::Point uv, geo::Point ray)
	{
		static_assert(D == geo::CW || D == geo::CCW, "D must be CW or CCW");
		assert(sector.within(ray));
		assert(sector.within(uv));
		const uint8_t region = ~m_table->region<1,1,2,2>(u.x + uv.x, u.y + uv.y) & 0b1111;
		geo::Compass dir;
		if constexpr (D == geo::CW) {
			// 0b3210
			// 0 1
			// 2 3
			switch (region) {
			case 0b0001:
			case 0b0101:
			case 0b1101:
				dir = geo::Compass::NORTH;
				break;
			case 0b0010:
			case 0b0011:
			case 0b0111:
				dir = geo::Compass::EAST;
				break;
			case 0b0100:
			case 0b1100:
			case 0b1110:
				dir = geo::Compass::WEST;
				break;
			case 0b1000:
			case 0b1010:
			case 0b1011:
				dir = geo::Compass::SOUTH;
				break;
			case 0b1001:
				// assert(!geo::is_strict_between_cw_le(uv, {-1,0}, {0,-1}) && !geo::is_strict_between_cw_le(uv, {1,0}, {0,1}));
				// #.  10
				// .#  23
				if (uv.y < 0) {
					// assert(uv.x >= 0 || std::abs(uv.y) > std::abs(uv.x));
					dir = geo::Compass::SOUTH;
				} else if (uv.y > 0) {
					// assert(uv.x <= 0 || std::abs(uv.y) > std::abs(uv.x));
					dir = geo::Compass::NORTH;
				} else[[unlikely]] {
					dir = uv.x < 0 ? geo::Compass::NORTH : geo::Compass::SOUTH;
				}
				break;
			case 0b0110:
				// assert(!geo::is_strict_between_cw_le(uv, {0,1}, {-1,0}) && !geo::is_strict_between_cw_le(uv, {0,-1}, {1,0}));
				// .#  10
				// #.  23
				if (uv.x < 0) {
					// assert(uv.y <= 0 || std::abs(uv.x) > std::abs(uv.y));
					dir = geo::Compass::EAST;
				} else if (uv.x > 0) {
					// assert(uv.y >= 0 || std::abs(uv.x) > std::abs(uv.y));
					dir = geo::Compass::WEST;
				} else[[unlikely]] {
					dir = uv.y < 0 ? geo::Compass::EAST : geo::Compass::WEST;
				}
				break;
			case 0b0000:
			case 0b1111:
			default:
				assert(false);
			}
		}
		if constexpr (D == geo::CCW) {
			// 0b3210
			// 0 1
			// 2 3
			switch (region) {
			case 0b0010:
			case 0b1010:
			case 0b1110:
				dir = geo::Compass::NORTH;
				break;
			case 0b1000:
			case 0b1100:
			case 0b1101:
				dir = geo::Compass::EAST;
				break;
			case 0b0001:
			case 0b0011:
			case 0b1011:
				dir = geo::Compass::WEST;
				break;
			case 0b0100:
			case 0b0101:
			case 0b0111:
				dir = geo::Compass::SOUTH;
				break;
			case 0b1001:
				// assert(!geo::is_strict_between_cw_le(uv, {-1,0}, {0,-1}) && !geo::is_strict_between_cw_le(uv, {1,0}, {0,1}));
				// #.  10
				// .#  23
				if (uv.x < 0) {
					// assert(uv.y >= 0 || std::abs(uv.x) > std::abs(uv.y));
					dir = geo::Compass::EAST;
				} else if (uv.x > 0) {
					// assert(uv.y <= 0 || std::abs(uv.x) > std::abs(uv.y));
					dir = geo::Compass::WEST;
				} else[[unlikely]] {
					dir = uv.y < 0 ? geo::Compass::WEST : geo::Compass::EAST;
				}
				break;
			case 0b0110:
				// assert(!geo::is_strict_between_cw_le(uv, {0,1}, {-1,0}) && !geo::is_strict_between_cw_le(uv, {0,-1}, {1,0}));
				// .#  10
				// #.  23
				if (uv.y < 0) {
					// assert(uv.x <= 0 || std::abs(uv.y) > std::abs(uv.x));
					dir = geo::Compass::SOUTH;
				} else if (uv.y > 0) {
					// assert(uv.x >= 0 || std::abs(uv.x) > std::abs(uv.y));
					dir = geo::Compass::NORTH;
				} else[[unlikely]] {
					dir = uv.x < 0 ? geo::Compass::SOUTH : geo::Compass::NORTH;
				}
				break;
			case 0b0000:
			case 0b1111:
			default:
				assert(false);
			}
		}
		if constexpr (scan::ScannerSectorPart<S>) {
			return this->template scan_trace<D>({m_table[0].data(), m_table[1].data()}, {m_table[0].bit_adj_index(u.x+uv.x, u.y+uv.y), m_table[1].bit_adj_index(u.y+uv.y, u.x+uv.x)},
				sector[geo::BasicAngleSector::ori_id_v<D>], u, uv, ray, dir, region);
		} else {
			return this->template scan_trace<D>({m_table[0].data(), m_table[1].data()}, {m_table[0].bit_adj_index(u.x+uv.x, u.y+uv.y), m_table[1].bit_adj_index(u.y+uv.y, u.x+uv.x)},
				sector, u, uv, ray, dir, region);
		}
	}

protected:
	const env::Grid::table* m_table;
};

struct SimpleScanner
{
	void setup(Search& search);

	template <geo::Ori D>
	geo::Point scan_trace(std::array<const env::Grid::table::pack_type*, 2> tables, std::array<env::Grid::table::adj_index, 2> idx, geo::BasicAngleSector sector, geo::Point u, geo::Point uv, geo::Point ray, geo::Compass dir, uint8_t);

	Expansion* m_exp;
};

extern template geo::Point SimpleScanner::scan_trace<geo::CW>(std::array<const env::Grid::table::pack_type*, 2> tables, std::array<env::Grid::table::adj_index, 2> idx, geo::BasicAngleSector sector, geo::Point u, geo::Point uv, geo::Point ray, geo::Compass dir, uint8_t);
extern template geo::Point SimpleScanner::scan_trace<geo::CCW>(std::array<const env::Grid::table::pack_type*, 2> tables, std::array<env::Grid::table::adj_index, 2> idx, geo::BasicAngleSector sector, geo::Point u, geo::Point uv, geo::Point ray, geo::Compass dir, uint8_t);

struct ConvexScanner
{
	ConvexScanner() noexcept : m_exp(nullptr)
	{ }

	void setup(Search& search);

	template <geo::Ori D>
	geo::Point scan_trace(std::array<const env::Grid::table::pack_type*, 2> tables, std::array<env::Grid::table::adj_index, 2> idx, geo::BasicAngleSector sector, geo::Point u, geo::Point uv, geo::Point ray, geo::Compass dir, uint8_t start_region);

	Expansion* m_exp;
};

extern template geo::Point ConvexScanner::scan_trace<geo::CW>(std::array<const env::Grid::table::pack_type*, 2> tables, std::array<env::Grid::table::adj_index, 2> idx, geo::BasicAngleSector sector, geo::Point u, geo::Point uv, geo::Point ray, geo::Compass dir, uint8_t start_region);
extern template geo::Point ConvexScanner::scan_trace<geo::CCW>(std::array<const env::Grid::table::pack_type*, 2> tables, std::array<env::Grid::table::adj_index, 2> idx, geo::BasicAngleSector sector, geo::Point u, geo::Point uv, geo::Point ray, geo::Compass dir, uint8_t start_region);

// extern template void Scan::full_scan<geo::CW>(geo::BasicAngleSector sector, geo::Point u, geo::Point v, geo::Point ray);
// extern template void Scan::full_scan<geo::CCW>(geo::BasicAngleSector sector, geo::Point u, geo::Point v, geo::Point ray);
// extern template void Scan::full_scan<geo::CO>(geo::BasicAngleSector sector, geo::Point u, geo::Point v, geo::Point ray);

// extern template geo::Point scan_trace<geo::CW>(geo::Point sector_outer, geo::Point ray, geo::Point uv, geo::Compass dir);
// extern template geo::Point scan_trace<geo::CCW>(geo::Point sector_outer, geo::Point ray, geo::Point uv, geo::Compass dir);

} // namespace rayscan::search

#endif // RAYSCAN_SEARCH_SCAN_HPP
