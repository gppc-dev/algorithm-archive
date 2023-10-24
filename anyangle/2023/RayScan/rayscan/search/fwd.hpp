#ifndef RAYSCAN_SEARCH_FWD_HPP
#define RAYSCAN_SEARCH_FWD_HPP

#include <inx/inx.hpp>
#include <geo/Point.hpp>

#ifndef RAYSCAN_BYPASS_COUNT
#define RAYSCAN_BYPASS_COUNT 6
#endif

// 0 = disable, -1 = all, >0 = rayid
#define DEBUG_RAYSHOOT 0
#ifdef DEBUG_RAYSHOOT
#ifdef NDEBUG
#undef DEBUG_RAYSHOOT
#define DEBUG_RAYSHOOT 0
#endif // NDEBUG
#else // !defined(DEBUG_RAYSHOOT)
#define DEBUG_RAYSHOOT 0
#endif

// debug instance id, -1 for disable
#define DEBUG_SEARCH -1
#ifdef DEBUG_SEARCH
#ifdef NDEBUG
#undef DEBUG_SEARCH
#define DEBUG_SEARCH -1
#endif // NDEBUG
#else // !defined(DEBUG_SEARCH)
#define DEBUG_SEARCH -1
#endif
#if DEBUG_SEARCH >= 0
#define DEBUG_SEARCH_CMD(i,cmd) {if ((i)-1 == DEBUG_SEARCH) {cmd;}}
#else
#define DEBUG_SEARCH_CMD(i,cmd) {}
#endif

namespace rayscan::search {

namespace extra {
constexpr int HV_SHIFT = 0;
constexpr int HV_COUNT = 2;
constexpr int POS_SHIFT = HV_SHIFT + HV_COUNT;
constexpr int POS_COUNT = 1;
constexpr int SEG_SHIFT = POS_SHIFT + POS_COUNT;
constexpr int SEG_COUNT = 1;
constexpr int CACHE_SHIFT = SEG_SHIFT + SEG_COUNT;
constexpr int CACHE_COUNT = 1;
} // namespace extra

enum Shoot : uint8_t
{
	HORI_POS  = 0 << extra::HV_SHIFT,
	HORI_NEG  = 2 << extra::HV_SHIFT,
	VERT_POS  = 1 << extra::HV_SHIFT,
	VERT_NEG  = 3 << extra::HV_SHIFT,
	HV_HORI = 1 << extra::HV_SHIFT,
	HV_POS = 2 << extra::HV_SHIFT,
	POINT = 0 << extra::POS_SHIFT,
	EDGE  = 1 << extra::POS_SHIFT,
	SEG_INT = 0 << extra::SEG_SHIFT,
	SEG_CLEAR = 1 << extra::SEG_SHIFT,
	RAY_CACHED = 1 << extra::CACHE_SHIFT,
};

using shoot_res = std::pair<geo::Point, Shoot>;

struct SearchId
{
	uint32_t id;
};

struct ExpId
{
	uint32_t id;
};

class BasicQueue;
class Expansion;
class RayShoot;
class Search;

} // namespace rayscan::search

#endif // RAYSCAN_SEARCH_FWD_HPP
