#ifndef RAYSCAN_ENV_GRID_HPP
#define RAYSCAN_ENV_GRID_HPP

#include <geo/Point.hpp>
#include <geo/Angle.hxx>
#include <inx/bit_table.hpp>
#include <inx/factory.hpp>
#include <inx/functions.hpp>
#include <vector>
#include <unordered_map>
#include <memory_resource>
#include <any>

namespace rayscan::env {

struct GridVertex
{
	enum class Ambig : uint8_t { NONE, AMBIG_XY_SIGN, AMBIG_XY_NOSIGN };
	enum Corners : uint8_t {
		// outbound forward for CW shape
		F_NORTH = 0,
		F_EAST = 1,
		F_SOUTH = 2,
		F_WEST = 3,
		// outbound backward for CW shape
		B_NORTH = F_NORTH << 2,
		B_EAST = F_EAST << 2,
		B_SOUTH = F_SOUTH << 2,
		B_WEST = F_WEST << 2,
		// abmig points
		A_XY_SIGN = (F_SOUTH | B_EAST) | ((F_NORTH | B_WEST) << 4),
		A_XY_NOSIGN = (F_EAST | B_NORTH) | ((F_WEST | B_SOUTH) << 4),
	};
	geo::Point p;
	Ambig ambig;
	geo::Ori angle;
	Corners corner;

	constexpr static geo::Point corner_pt(Corners c) {
		assert(c < 4);
		switch (c) {
		case F_NORTH: return geo::Point(0, -1);
		case F_EAST: return geo::Point(1, 0);
		case F_SOUTH: return geo::Point(0, 1);
		case F_WEST: return geo::Point(-1, 0);
		default:
		assert(false);
		return geo::Point::zero();
		}
	}
	template <bool Backward, int i = 0>
	constexpr static geo::Point corner_pt_x(Corners c) {
		return corner_pt(static_cast<Corners>( (c >> 2 * (2*i+static_cast<int>(Backward))) & 0b11 ));
	}
	template <int i = 0>
	constexpr static geo::Point corner_pt_x(Corners c, bool backward) {
		assert(backward == false || backward == true);
		return corner_pt(static_cast<Corners>( (c >> 2 * (2*i+static_cast<int>(backward))) & 0b11 ));
	}

	constexpr GridVertex() noexcept : p(geo::Point::zero()), angle(geo::CO), ambig(Ambig::NONE), corner(Corners::F_NORTH)
	{ }
	constexpr GridVertex(const GridVertex&) = default;
	constexpr GridVertex(GridVertex&&) = default;

	constexpr GridVertex& operator=(const GridVertex&) = default;
	constexpr GridVertex& operator=(GridVertex&&) = default;

	constexpr bool fill_mask(uint32_t mask) noexcept {
		// 01
		// 23
		switch (~mask & 0b1111) {
			case 0b0000: ambig = Ambig::NONE; angle = geo::CO; corner = F_NORTH; return false;
			case 0b0001: ambig = Ambig::NONE; angle = geo::CW; corner = static_cast<Corners>(F_WEST | B_NORTH); return true;
			case 0b0010: ambig = Ambig::NONE; angle = geo::CW; corner = static_cast<Corners>(F_NORTH | B_EAST); return true;
			case 0b0011: ambig = Ambig::NONE; angle = geo::CO; corner = static_cast<Corners>(F_WEST | B_EAST); return false;
			case 0b0100: ambig = Ambig::NONE; angle = geo::CW; corner = static_cast<Corners>(F_SOUTH | B_WEST); return true;
			case 0b0101: ambig = Ambig::NONE; angle = geo::CO; corner = static_cast<Corners>(F_SOUTH | B_NORTH); return false;
			case 0b0110: ambig = Ambig::AMBIG_XY_SIGN; angle = geo::CCW; corner = A_XY_SIGN; return true;
			case 0b0111: ambig = Ambig::NONE; angle = geo::CCW; corner = static_cast<Corners>(F_SOUTH | B_EAST); return true;
			case 0b1000: ambig = Ambig::NONE; angle = geo::CW; corner = static_cast<Corners>(F_EAST | B_SOUTH); return true;
			case 0b1001: ambig = Ambig::AMBIG_XY_NOSIGN; angle = geo::CCW; corner = A_XY_NOSIGN; return true;
			case 0b1010: ambig = Ambig::NONE; angle = geo::CO; corner = static_cast<Corners>(F_NORTH | B_SOUTH); return false;
			case 0b1011: ambig = Ambig::NONE; angle = geo::CCW; corner = static_cast<Corners>(F_WEST | B_SOUTH); return true;
			case 0b1100: ambig = Ambig::NONE; angle = geo::CO; corner = static_cast<Corners>(F_EAST | B_WEST); return false;
			case 0b1101: ambig = Ambig::NONE; angle = geo::CCW; corner = static_cast<Corners>(F_EAST | B_NORTH); return true;
			case 0b1110: ambig = Ambig::NONE; angle = geo::CCW; corner = static_cast<Corners>(F_NORTH | B_WEST); return true;
			case 0b1111: ambig = Ambig::NONE; angle = geo::CO; corner = F_NORTH; return false;
			default: return false;
		}
	}
};

class Grid
{
public:
	constexpr static size_t PADDING = 4;
	constexpr static geo::Point POINT_PAD = geo::Point(PADDING, PADDING);

	constexpr static geo::Point pad(geo::Point p) noexcept
	{
		return geo::Point(p.x, p.y) + POINT_PAD;
	}
	constexpr static geo::Point unpad(geo::Point p) noexcept
	{
		return geo::Point(p.x, p.y) - POINT_PAD;
	}

	using table = inx::bit_table<>;

	Grid();

	void setup(const std::vector<bool>& bits, int width, int height);
	template <typename VertexType>
	void construct_vertices() requires std::is_base_of_v<GridVertex, VertexType>
	{
		auto* factory = new inx::Factory<VertexType>(&m_memoryBuffer);
		m_factory = inx::any_ptr(factory);
		std::vector<VertexType*> vertexRef;
		vertexRef.reserve(16*1024);
		construct_vertices_aux([&](GridVertex v) {
			VertexType* p = factory->construct(v);
			vertexRef.push_back(p);
		}, 32);
		m_vertexMap.max_load_factor(0.75);
		m_vertexMap.reserve(vertexRef.size() + 16);
		for (VertexType* v : vertexRef) {
			m_vertexMap.try_emplace(v->p.word(), v);
		}
	}

	uint32_t getWidth() const noexcept { return m_width; }
	uint32_t getHeight() const noexcept { return m_height; }

	const auto& getTable() const noexcept { return m_table; }
	GridVertex* getVertex(geo::coord_word w) { return m_vertexMap.at(w); }
	const GridVertex* getVertex(geo::coord_word w) const { return m_vertexMap.at(w); }
	GridVertex* getVertex(geo::Point p) { return m_vertexMap.at(p.word()); }
	const GridVertex* getVertex(geo::Point p) const { return m_vertexMap.at(p.word()); }
	const auto& getVertexMap() const noexcept { return m_vertexMap; }

protected:
	// construct turning vertices, grouping by regionSize squares
	void construct_vertices_aux(std::function<void(GridVertex)> vertexConstruct, uint32_t regionSize = std::numeric_limits<uint32_t>::max() / 2);

private:
	uint32_t m_width, m_height;
	std::array<table, 2> m_table;
	std::pmr::monotonic_buffer_resource m_memoryBuffer;
	inx::any_ptr m_factory; /// used to auto-delete a typed Factory
	std::pmr::unordered_map<geo::coord_word, GridVertex*> m_vertexMap;
};

} // namespace rayscan::env

#endif // RAYSCAN_ENV_GRID_HPP
