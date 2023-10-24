#ifndef RAYSCAN_SEARCH_SEARCH_HPP
#define RAYSCAN_SEARCH_SEARCH_HPP

#include "fwd.hpp"
#include <env/Grid.hpp>
#include "Expansion.hpp"
#include "BasicQueue.hpp"

namespace rayscan::search {

class Search
{
public:
	Search();

	void setup(env::Grid& l_grid);
	void search(geo::Point s, geo::Point t);
	const std::vector<geo::Point>& get_path() const noexcept { return m_path; }

	env::Grid* get_grid() noexcept { return m_grid; }
	const env::Grid* get_grid() const noexcept { return m_grid; }
	Expansion& get_expansion() noexcept { return m_expansion; }
	const Expansion& get_expansion() const noexcept { return m_expansion; }
	BasicQueue& get_queue() noexcept { return m_queue; }
	const BasicQueue& get_queue() const noexcept { return m_queue; }

protected:
	env::Grid* m_grid;
	Expansion m_expansion;
	BasicQueue m_queue;
	std::vector<geo::Point> m_path;
};

} // namespace rayscan::search

#endif // RAYSCAN_SEARCH_SEARCH_HPP
