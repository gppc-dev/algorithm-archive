#include "Grid.hpp"

namespace rayscan::env {

Grid::Grid() : m_memoryBuffer(1024 * 1024), m_vertexMap(&m_memoryBuffer)
{ }

void Grid::setup(const std::vector<bool> &bits, int width, int height)
{
	// TODO: optimise load
	m_width = static_cast<uint32_t>(width) + 2*PADDING;
	m_height = static_cast<uint32_t>(height) + 2*PADDING;
	m_table[0].setup(m_width, m_height);
	m_table[1].setup(m_height, m_width);
	width += static_cast<int>(PADDING);
	height += static_cast<int>(PADDING);
	for (int i = 0, y = static_cast<int>(PADDING); y < height; ++y)
	for (int x = static_cast<int>(PADDING); x < width; ++x) {
		if (bits[i++]) {
			m_table[0].bit_or(x, y, 1);
			m_table[1].bit_or(y, x, 1);
		}
	}
}

void Grid::construct_vertices_aux(std::function<void(GridVertex)> vertexConstruct, uint32_t regionSize)
{
	// +1 width and height for lattice points
	const uint32_t height = m_height - 3, width = m_width - 3;
	GridVertex vdata;
	for (uint32_t y = PADDING; y < height; y += regionSize) {
		uint32_t y_min = std::min(y+regionSize, height);
		for (uint32_t x = PADDING; x < width; x += regionSize) {
			uint32_t x_min = std::min(x+regionSize, width);
			for (uint32_t y2 = y; y2 < y_min; ++y2)
			for (uint32_t x2 = x; x2 < x_min; ++x2) {
				if (vdata.fill_mask(m_table[0].region<1, 1, 2, 2>(static_cast<int32_t>(x2), static_cast<int32_t>(y2)))) [[unlikely]] {
					vdata.p = geo::Point(static_cast<geo::coord>(x2), static_cast<geo::coord>(y2));
					vertexConstruct(vdata);
				}
			}
		}
	}
}

} // namespace rayscan::env
