#ifndef RAYSCAN_GEO_ANGLE_HXX
#define RAYSCAN_GEO_ANGLE_HXX

#include "Point.hpp"

namespace rayscan::geo {

enum Ori : int8_t {
	CCW = -1,
	CO = 0,
	CW = 1,
};
constexpr Ori inv_ori(Ori o) { return static_cast<Ori>(-o); }

enum class Compass : uint8_t {
	NORTH = 0,
	SOUTH = 1,
	EAST = 3,
	WEST = 2
};

constexpr Point compass_point(Compass c) noexcept
{
	switch (c) {
	case Compass::NORTH: return Point(0, -1);
	case Compass::SOUTH: return Point(0, 1);
	case Compass::EAST: return Point(1, 0);
	case Compass::WEST: return Point(-1, 0);
	default: assert(false); return Point::zero();
	}
}

enum class Angle : int8_t {
	LE90 = -1,
	ANY = 0,
	GE90 = 1,
};

constexpr std::pair<coord_op, coord_op> dot(Point l, Point r) noexcept { return std::pair<coord_op, coord_op>(static_cast<coord_op>(l.x) * static_cast<coord_op>(r.x), static_cast<coord_op>(l.y) * static_cast<coord_op>(r.y)); }
constexpr std::pair<coord_op, coord_op> dot(Point o, Point l, Point r) noexcept { return dot(l - o, r - o); }

constexpr coord_op cross(Point l, Point r) noexcept { return static_cast<coord_op>(l.x) * static_cast<coord_op>(r.y) - static_cast<coord_op>(r.x) * static_cast<coord_op>(l.y); }
constexpr coord_op cross(Point o, Point l, Point r) noexcept { return cross(l - o, r - o); }

constexpr bool is_ori_cw(Ori d) noexcept { return d > 0; }
constexpr bool is_ori_ccw(Ori d) noexcept { return d < 0; }
constexpr bool is_ori_co(Ori d) noexcept { return d == 0; }
template <Ori D>
constexpr bool is_ori_dir(Ori d) noexcept
{
	if constexpr (D == Ori::CW) return d > 0;
	else if constexpr (D == Ori::CCW) return d < 0;
	else return d == 0;
}

constexpr bool is_cw(coord_op x) noexcept { return x > 0; }
constexpr bool is_ccw(coord_op x) noexcept { return x < 0; }
constexpr bool is_co(coord_op x) noexcept { return x == 0; }

constexpr bool is_cw(Point l, Point r) noexcept { return is_cw(cross(l, r)); }
constexpr bool is_ccw(Point l, Point r) noexcept { return is_ccw(cross(l, r)); }
constexpr bool is_co(Point l, Point r) noexcept { return is_co(cross(l, r)); }
constexpr bool is_cw(Point o, Point l, Point r) noexcept { return is_cw(cross(o, l, r)); }
constexpr bool is_ccw(Point o, Point l, Point r) noexcept { return is_ccw(cross(o, l, r)); }
constexpr bool is_co(Point o, Point l, Point r) noexcept { return is_co(cross(o, l, r)); }

template <Ori D>
constexpr bool is_ori(coord_op x) noexcept
{
	static_assert(D == Ori::CW || D == Ori::CCW || D == Ori::CO, "D must be CW, CCW or CO");
	if constexpr(D == Ori::CW) return is_cw(x);
	else if constexpr(D == Ori::CCW) return is_ccw(x);
	else return is_co(x);
}
template <Ori D>
constexpr bool is_ori(Point l, Point r) noexcept
{
	static_assert(D == Ori::CW || D == Ori::CCW || D == Ori::CO, "D must be CW, CCW or CO");
	if constexpr(D == Ori::CW) return is_cw(l, r);
	else if constexpr(D == Ori::CCW) return is_ccw(l, r);
	else return is_co(l, r);
}
template <Ori D>
constexpr bool is_ori(Point o, Point l, Point r) noexcept
{
	static_assert(D == Ori::CW || D == Ori::CCW || D == Ori::CO, "D must be CW, CCW or CO");
	if constexpr(D == Ori::CW) return is_cw(o, l, r);
	else if constexpr(D == Ori::CCW) return is_ccw(o, l, r);
	else return is_co(o, l, r);
}

constexpr bool is_same_dir(Point l, Point r) noexcept
{
	assert(is_co(l, r));
	return ((l.x * r.x) | (l.y * r.y)) > 0;
}
constexpr bool is_same_or_zero_dir(Point l, Point r) noexcept
{
	assert(is_co(l, r));
	return ((l.x * r.x) | (l.y * r.y)) >= 0;
}
constexpr bool is_opposite_dir(Point l, Point r) noexcept
{
	assert(is_co(l, r));
	return ((l.x * r.x) | (l.y * r.y)) < 0;
}
constexpr bool is_opposite_or_zero_dir(Point l, Point r) noexcept
{
	assert(is_co(l, r));
	return ((l.x * r.x) | (l.y * r.y)) <= 0;
}

// between base functions, only single branch (if not optimized out)
constexpr bool is_strict_between_ccw_le(Point angle, Point pivot, Point turn) noexcept {
	assert(!is_cw(pivot, turn));
	return (cross(pivot, angle) & cross(angle, turn)) < 0;
}
constexpr bool is_strict_between_ccw_ge(Point angle, Point pivot, Point turn) noexcept {
	assert(!is_ccw(pivot, turn));
	return (cross(pivot, angle) | cross(angle, turn)) < 0;
}
constexpr bool is_strict_between_ccw_any(Point angle, Point pivot, Point turn) noexcept {
	const coord_op pa = cross(pivot, angle), at = cross(angle, turn), pt = cross(pivot, turn);
	// CCW = true, otherwise false
	// (pt & pa & at) 'ccw_le'  |  (~pt & (pa | at)) 'cw_le'
	// becomes
	// DNF: (pa & at) | (pa & ~pt) | (at & ~pt)
	// or
	// CNF: (pa | at) & (pa | ~pt) & (at | ~pt)

	// return ( (pt & pa & at) | (~pt & (pa | at)) ) < 0; // base
	return ( (pa & at) | (pa & ~pt) | (at & ~pt) ) < 0; // DNF
	// return ( (pa | at) & (pa | ~pt) & (at | ~pt) ) < 0; // CNF
}

constexpr bool is_strict_between_cw_le(Point angle, Point pivot, Point turn) noexcept { return is_strict_between_ccw_le(angle, turn, pivot); }
constexpr bool is_strict_between_cw_ge(Point angle, Point pivot, Point turn) noexcept { return is_strict_between_ccw_ge(angle, turn, pivot); }
constexpr bool is_strict_between_cw_any(Point angle, Point pivot, Point turn) noexcept { return is_strict_between_ccw_any(angle, turn, pivot); }

template <Angle Ang>
constexpr bool is_strict_between_ccw(Point angle, Point pivot, Point turn) noexcept
{
	if constexpr (Ang == Angle::LE90) {
		return is_strict_between_ccw_le(angle, pivot, turn);
	} else if constexpr (Ang == Angle::GE90) {
		return is_strict_between_ccw_ge(angle, pivot, turn);
	} else {
		return is_strict_between_ccw_any(angle, pivot, turn);
	}
}
template <Angle Ang>
constexpr bool is_strict_between_cw(Point angle, Point pivot, Point turn) noexcept
{
	if constexpr (Ang == Angle::LE90) {
		return is_strict_between_cw_le(angle, pivot, turn);
	} else if constexpr (Ang == Angle::GE90) {
		return is_strict_between_cw_ge(angle, pivot, turn);
	} else {
		return is_strict_between_cw_any(angle, pivot, turn);
	}
}

template <Ori Orien>
constexpr bool is_strict_between_le(Point angle, Point pivot, Point turn) noexcept
{
	static_assert(Orien == Ori::CCW || Orien == Ori::CW, "Orien must be CW or CCW");
	if constexpr (Orien == Ori::CCW) {
		return is_strict_between_ccw_le(angle, pivot, turn);
	} else {
		return is_strict_between_cw_le(angle, pivot, turn);
	}
}
template <Ori Orien>
constexpr bool is_strict_between_ge(Point angle, Point pivot, Point turn) noexcept
{
	static_assert(Orien == Ori::CCW || Orien == Ori::CW, "Orien must be CW or CCW");
	if constexpr (Orien == Ori::CCW) {
		return is_strict_between_ccw_ge(angle, pivot, turn);
	} else {
		return is_strict_between_cw_ge(angle, pivot, turn);
	}
}
template <Ori Orien>
constexpr bool is_strict_between_any(Point angle, Point pivot, Point turn) noexcept
{
	static_assert(Orien == Ori::CCW || Orien == Ori::CW, "Orien must be CW or CCW");
	if constexpr (Orien == Ori::CCW) {
		return is_strict_between_ccw_any(angle, pivot, turn);
	} else {
		return is_strict_between_cw_any(angle, pivot, turn);
	}
}

template <Ori Orien, Angle Ang>
constexpr bool is_strict_between(Point angle, Point pivot, Point turn) noexcept
{
	static_assert(Orien == Ori::CCW || Orien == Ori::CW, "Orien must be CW or CCW");
	if constexpr (Orien == Ori::CCW) {
		return is_strict_between_ccw<Ang>(angle, pivot, turn);
	} else {
		return is_strict_between_cw<Ang>(angle, pivot, turn);
	}
}

struct alignas(uint64_t) Region : std::array<Point, 2>
{
	using super_ = std::array<Point, 2>;
	using super_::super_;
	constexpr Region() noexcept : super_{}
	{ }
	constexpr Region(Point ccw, Point cw) noexcept : super_{ccw, cw}
	{ }

	using super_::operator=;
	constexpr Region& operator=(const Region& r) noexcept = default;

	template <Ori D>
	constexpr static int ori_id_v = is_ori_cw(D) ? 1 : 0;
	constexpr static int ori_id(Ori D) noexcept { return is_ori_cw(D) ? 1 : 0; }
};

} // namespace rayscan::geo

#endif // RAYSCAN_GEO_ANGLE_HXX
