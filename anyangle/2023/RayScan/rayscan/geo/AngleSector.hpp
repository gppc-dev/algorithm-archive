#ifndef RAYSCAN_GEO_ANGLESECTOR_HPP
#define RAYSCAN_GEO_ANGLESECTOR_HPP

#include "Angle.hxx"

namespace rayscan::geo {

struct BasicAngleSector : Region
{
	constexpr BasicAngleSector()
	{ }
	constexpr BasicAngleSector(Region r) : Region(r)
	{
		assert(!is_ccw(r[0], r[1]));
	}
	constexpr BasicAngleSector(Point cw, Point ccw) : Region(cw, ccw)
	{
		assert(!is_ccw(cw, ccw));
	}

	template <Ori D>
	constexpr static BasicAngleSector create_sector(Point pivot, Point turn) noexcept
	{
		static_assert(D == Ori::CW || D == Ori::CCW, "D must be CW or CCW");
		if constexpr (D == Ori::CW) return BasicAngleSector(pivot, turn);
		else return BasicAngleSector(turn, pivot);
	}

	template <Ori D>
	constexpr BasicAngleSector split_sector(Point split) noexcept
	{
		static_assert(D == Ori::CW || D == Ori::CCW, "D must be CW or CCW");
		assert(within(split));
		if constexpr (D == Ori::CW) return BasicAngleSector(split, (*this)[1]);
		else return BasicAngleSector((*this)[0], split);
	}

	template <Ori D>
	constexpr bool turns_within(Point q) const noexcept
	{
		static_assert(D == Ori::CW || D == Ori::CCW, "D must be CW or CCW");
		assert(is_ori<D>(!(*this)[ori_id_v<D>], q) || is_strict_between_ccw_any(q, (*this)[0], (*this)[1]));
		return !is_ori<D>(!(*this)[ori_id_v<D>], q);
	}

	constexpr bool within(Point q) const noexcept
	{
		return !is_strict_between_ccw_ge(q, (*this)[0], (*this)[1]);
	}
	constexpr bool strictly_within(Point q) const noexcept
	{
		return is_strict_between_cw_le(q, (*this)[0], (*this)[1]);
	}
};

} // namespace rayscan::geo

#endif // RAYSCAN_GEO_ANGLESECTOR_HPP
