#include "Search.hpp"
#include "Scan.hpp"

namespace rayscan::search {

Search::Search() : m_grid(nullptr)
{

}

void Search::setup(env::Grid& l_grid)
{
	m_grid = &l_grid;
	m_expansion.setup(*this);
	m_queue.base().reserve(2048);
}

void Search::search(geo::Point s, geo::Point t)
{
	s = env::Grid::pad(s);
	t = env::Grid::pad(t);
	m_queue.clear();
	// node s
	auto* node = m_expansion.search_setup(s, t);
	for (auto* v : m_expansion.expand_s(*node)) {
		v->open = true;
		m_queue.base().push_back(BasicQueue::QueueValue{v->f, v});
	}
	m_queue.heapify();
	node->close = true;
	while (!m_expansion.done() && !m_queue.empty()) {
		node = m_queue.pop();
		if (node->close)
			continue;
		node->close = true;
		// node u
		for (auto* v : m_expansion.expand(*node)) {
			v->open = true;
			m_queue.push(*v);
		}
	}
	m_expansion.get_path(m_path);
	for (auto& p : m_path) {
		p = env::Grid::unpad(p);
	}
}

} // namespace rayscan::search
