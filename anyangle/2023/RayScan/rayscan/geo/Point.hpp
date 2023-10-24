#ifndef RAYSCAN_GEO_POINT_HPP
#define RAYSCAN_GEO_POINT_HPP

#include <inx/inx.hpp>
#include <cmath>
#include <ostream>

namespace rayscan::geo {

using coord = int16_t;
using coord_word = uint32_t;
using coord_op = int32_t;
using coord_op2 = int64_t;

struct alignas(coord_word) Point
{
	coord x, y;

	Point() noexcept = default;
	constexpr Point(const Point&) noexcept = default;
	constexpr Point(Point&&) noexcept = default;
	constexpr Point(coord l_x, coord l_y) noexcept : x(l_x), y(l_y)
	{
		assert(std::abs(l_x) < (1 << 14) && std::abs(l_y) < (1 << 14));
	}

	constexpr Point& operator=(const Point&) noexcept = default;
	constexpr Point& operator=(Point&&) noexcept = default;

	constexpr coord_word word() const noexcept { return std::bit_cast<coord_word>(*this); }
	constexpr coord operator[](uint32_t i) const noexcept { assert(i < 2); return std::bit_cast<const coord*>(this)[i]; }
	constexpr coord& operator[](uint32_t i) noexcept { assert(i < 2); return std::bit_cast<coord*>(this)[i]; }
	constexpr bool is_zero() const noexcept { return word() == 0; }

	constexpr static Point zero() noexcept { return Point(0, 0); }

	constexpr Point transpose() const noexcept { return Point(y, x); }
	constexpr Point rotate90cw() const noexcept { return Point(-y, x); }
	constexpr Point rotate90ccw() const noexcept { return Point(y, -x); }

	constexpr coord_op square() const noexcept { return static_cast<coord_op>(x)*static_cast<coord_op>(x) + static_cast<coord_op>(y)*static_cast<coord_op>(y); }
	constexpr coord_op square(Point o) const noexcept;
	constexpr double length() const noexcept { return std::sqrt(square()); }
	constexpr double length(Point o) const noexcept;
};

constexpr bool operator==(Point l, Point r) noexcept { return l.word() == r.word(); }

constexpr Point operator+(Point l, Point r) noexcept { return Point(l.x + r.x, l.y + r.y); }
constexpr Point operator-(Point l, Point r) noexcept { return Point(l.x - r.x, l.y - r.y); }
constexpr Point operator-(Point l) noexcept { return Point(-l.x, -l.y); }

constexpr coord_op Point::square(Point o) const noexcept { return (*this - o).square(); }
constexpr double Point::length(Point o) const noexcept { return (*this - o).length(); }

inline std::ostream& operator<<(std::ostream& os, const Point& p)
{
	os << p.x << ' ' << p.y;
	return os;
}
struct bracket_point
{
	Point p;
	constexpr bracket_point(Point lp) noexcept : p(lp) { }
};
inline std::ostream& operator<<(std::ostream& os, const bracket_point& bp)
{
	os << '(' << bp.p.x << ' ' << bp.p.y << ')';
	return os;
}

} // namespace rayscan::geo

#endif // RAYSCAN_GEO_POINT_HPP
