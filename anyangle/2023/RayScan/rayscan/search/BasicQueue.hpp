#ifndef RAYSCAN_SEARCH_BASICQUEUE_HPP
#define RAYSCAN_SEARCH_BASICQUEUE_HPP

#include "fwd.hpp"
#include "Vertex.hpp"
#include <vector>
#include <algorithm>

namespace rayscan::search {

class BasicQueue
{
public:
	struct QueueValue
	{
		double f;
		Vertex* v;
		bool operator==(const QueueValue& r) const noexcept { return f == r.f; }
		bool operator!=(const QueueValue& r) const noexcept { return f != r.f; }
		bool operator<(const QueueValue& r) const noexcept { return f > r.f; }
		bool operator>(const QueueValue& r) const noexcept { return r > *this; }
	};

	const auto& base() const noexcept { return m_queue; }
	auto& base() noexcept { return m_queue; }

	bool empty() const noexcept { return m_queue.empty(); }
	const Vertex* top() const noexcept { assert(!m_queue.empty()); return m_queue.front().v; }
	Vertex* top() noexcept { assert(!m_queue.empty()); return m_queue.front().v; }

	void heapify()
	{
		std::make_heap(m_queue.begin(), m_queue.end());
	}

	void push(Vertex& v)
	{
		m_queue.push_back({v.f, &v});
		std::push_heap(m_queue.begin(), m_queue.end());
	}

	Vertex* pop()
	{
		assert(!m_queue.empty());
		Vertex* t = m_queue.front().v;
		std::pop_heap(m_queue.begin(), m_queue.end());
		m_queue.pop_back();
		return t;
	}

	void clear()
	{
		m_queue.clear();
	}

private:
	std::vector<QueueValue> m_queue;
};

} // namespace rayscan::search

#endif // RAYSCAN_SEARCH_BASICQUEUE_HPP
