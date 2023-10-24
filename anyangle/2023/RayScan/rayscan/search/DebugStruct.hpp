#ifndef RAYSCAN_SEARCH_DEBUGSTRUCT_HPP
#define RAYSCAN_SEARCH_DEBUGSTRUCT_HPP

#include "fwd.hpp"

#if DEBUG_RAYSHOOT != 0 || DEBUG_SEARCH >= 0
#include <fstream>

#if DEBUG_RAYSHOOT != 0
#if DEBUG_RAYSHOOT < 0
#define DEBUG_RAYSHOOT_FNAME "rayshoot.txt"
#else
#define DEBUG_RAYSHOOT_STR_1(s) #s
#define DEBUG_RAYSHOOT_STR(s) DEBUG_RAYSHOOT_STR_1(s)
#define DEBUG_RAYSHOOT_FNAME "rayshoot-" DEBUG_RAYSHOOT_STR(DEBUG_RAYSHOOT) ".txt"
#endif
#endif

namespace rayscan::search {

#if DEBUG_RAYSHOOT != 0
struct DebugRayShoot
{
	DebugRayShoot() : rayid(0), width(0), height(0), transpose(false) { }
	void setup(const env::Grid::table& table)
	{
		auto m = std::max(table.getHeight(), table.getWidth());
		map.assign(m, std::vector<char>(m));
	}
	void init_ray(const env::Grid::table& table, bool l_transpose)
	{
		width = static_cast<int32_t>(table.getWidth());
		height = static_cast<int32_t>(table.getHeight());
		transpose = l_transpose;
		rayid += 1;
		if (DEBUG_RAYSHOOT < 0 || rayid == DEBUG_RAYSHOOT) {
			for (int y = 0; y < height; ++y) {
				for (int x = 0; x < width; ++x) {
					map[y][x] = table.bit_test(x, y) ? '.' : '#';
				}
			}
		}
	}
	void fill_point(int x, int y, char c)
	{
		if (DEBUG_RAYSHOOT < 0 || rayid == DEBUG_RAYSHOOT) {
			char& pc = map.at(y).at(x);
			if (pc == '#')
				pc = 'x';
			else if (pc == '.')
				pc = c;
		}
	}
	void print() const
	{
		static std::unique_ptr<std::ofstream> out;
		if (DEBUG_RAYSHOOT < 0 || rayid == DEBUG_RAYSHOOT) {
			if (out == nullptr) {
				out = std::make_unique<std::ofstream>(DEBUG_RAYSHOOT_FNAME);
			}
			if (!transpose) {
				for (int32_t y = 0; y < height; ++y) {
					out->write(map[y].data(), width);
					out->put('\n');
				}
			} else {
				std::vector<char> line(height);
				for (int32_t x = 0; x < width; ++x) {
					for (int32_t y = 0; y < height; ++y) {
						line[y] = map[y][x];
					}
					out->write(line.data(), height);
					out->put('\n');
				}
			}
			*out << std::endl;
		}
	}
	size_t rayid;
	int32_t width, height;
	bool transpose;
	std::vector<std::vector<char>> map;
};
#endif

#if DEBUG_SEARCH >= 0
struct DebugSearch
{
	void setup(const env::Grid::table& table)
	{
		expid = 0;
		map.assign(table.getHeight(), std::vector<char>(table.getWidth()));
	}
	void init_expansion(const env::Grid::table& table, size_t expid, geo::Point p)
	{
		this->expid = expid;
		this->p = p;
		for (int y = 0; y < table.getHeight(); ++y) {
			for (int x = 0; x < table.getWidth(); ++x) {
				map[y][x] = table.bit_test(x, y) ? '.' : '#';
			}
		}
	}
	void fill_point(int x, int y, char c)
	{
		char& pc = map.at(y).at(x);
		if (pc == '#' || std::isupper(pc))
			pc = std::toupper(c);
		else
			pc = c;
	}
	void print() const
	{
		std::string fname;
		if (expid >= 0) {
			fname = std::string("expansion-") + std::to_string(p.x) + "-" + std::to_string(p.y) + "-" + std::to_string(expid) + ".txt";
		} else {
			fname = "expansion-path.txt";
		}
		std::ofstream out(fname);
		for (auto& row : map) {
			out.write(row.data(), row.size());
			out.put('\n');
		}
	}
	ssize_t expid;
	geo::Point p;
	std::vector<std::vector<char>> map;
};
#endif

} // namespace rayscan::search

#endif

#endif // RAYSCAN_SEARCH_DEBUGSTRUCT_HPP