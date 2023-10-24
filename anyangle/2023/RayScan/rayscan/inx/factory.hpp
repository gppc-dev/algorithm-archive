#ifndef GMLIB_FACTORY_HPP_INCLUDED
#define GMLIB_FACTORY_HPP_INCLUDED

#include "inx.hpp"
#include <memory_resource>
#include <forward_list>

namespace inx
{

namespace details
{
template <typename T, bool IsTrivial>
struct FactoryPoolDelete
{
	using obj_type = std::array<T*, 512>;
	using obj_list = std::forward_list<obj_type>;
	static constexpr uint32 size() noexcept { return std::tuple_size<obj_type>::value; }
	obj_list objects;
	typename obj_list::iterator obj_it;
	uint32 obj_i;
	uint32 obj_size;

	FactoryPoolDelete() : obj_it(objects.before_begin()), obj_i(size()), obj_size(0)
	{ }

	static bool getDel(const T* obj) noexcept
	{
		return reinterpret_cast<const bool*>(obj)[-1];
	}
	static void setDel(T* obj, bool del) noexcept
	{
		reinterpret_cast<bool*>(obj)[-1] = del;
	}

	void push(T* obj)
	{
		if (obj_i >= size()) {
			obj_i = 0;
			if (auto it = std::next(obj_it); it == objects.end())
				obj_it = objects.emplace_after(obj_it);
			else
				obj_it = it;
		}
		(*obj_it)[obj_i++] = obj;
		obj_size++;
	}
};
template <typename T>
struct FactoryPoolDelete<T, true>
{ };
}

template <typename T>
class Factory
{
public:
	static constexpr bool is_trivial() noexcept { return std::is_trivially_destructible_v<T>; }
	static constexpr size_t size() noexcept { return sizeof(T); }
	static constexpr size_t align() noexcept { return alignof(T); }
	static constexpr size_t size_del() noexcept { return is_trivial() ? size() : size() + align(); }
	using auto_delete = details::FactoryPoolDelete<T, is_trivial()>;
	Factory() { }
	Factory(std::pmr::memory_resource* upstream) : m_pool(upstream) { }
	Factory(std::size_t initial_size) : m_pool(initial_size * size()) { }
	Factory(std::size_t initial_size, std::pmr::memory_resource* upstream) : m_pool(initial_size * size_del(), upstream) { }
	~Factory()
	{
		release();
	}

	[[nodiscard]] void* allocate()
	{
		if constexpr (is_trivial()) {
			return m_pool.allocate(size_del(), align());
		} else {
			return static_cast<void*>( static_cast<std::byte*>(m_pool.allocate(size_del(), align())) + align() );
		}
	}
	void deallocate(void* data)
	{
		if constexpr (is_trivial()) {
			m_pool.deallocate(data, size_del(), align());
		} else {
			m_pool.deallocate( static_cast<void*>(static_cast<std::byte*>(data) - align()), size_del(), align() );
		}
	}

	template <typename... Args>
	[[nodiscard]] T* construct(Args&&... args)
	{
		T* data = ::new (allocate()) T(std::forward<Args>(args)...);
		construct_(data);
		return data;
	}
	void destruct(T* data)
	{
		destruct_(data);
		deallocate(data);
	}

	void release() noexcept
	{
		if constexpr (!is_trivial()) {
			constexpr uint32 objsize = auto_delete::size();
			auto it = m_autoDelete.objects.before_begin();
			for (uint32 i = 0, ie = m_autoDelete.obj_size; i < ie; ) {
				++it;
				assert(it != m_autoDelete.objects.end());
				for (uint32 j = 0, je = std::min(ie-i, objsize); j < je; ++i, ++j) {
					T* data = (*it)[j];
					if (auto_delete::getDel(data)) {
						auto_delete::setDel(data, false);
						std::destroy_at(data);
					}
				}
			}
			m_autoDelete.obj_it = m_autoDelete.objects.before_begin();
			m_autoDelete.obj_i = objsize;
			m_autoDelete.obj_size = 0;
		}
		m_pool.release();
	}

protected:
	void construct_(T* data)
	{
		if constexpr (!is_trivial()) {
			auto_delete::setDel(data, true);
			m_autoDelete.push(data);
		}
	}
	void destruct_(T* data)
	{
		if constexpr (!is_trivial()) {
			assert(auto_delete::getDel(data));
			auto_delete::setDel(data, false);
			std::destroy_at(data);
		}
	}

protected:
	std::pmr::monotonic_buffer_resource m_pool;
	auto_delete m_autoDelete;
};

template <typename T>
class ReuseFactory : Factory<T>
{
public:
	using self = ReuseFactory<T>;
	using super = Factory<T>;

	using super::super;
	~ReuseFactory()
	{
		release();
	}

	[[nodiscard]] void* allocate()
	{
		if (m_reuse.empty())
			return super::allocate();
		else {
			void* m = m_reuse.back();
			m_reuse.pop_back();
			return m;
		}
	}
	void deallocate(void* data)
	{
		m_reuse.push_back(data);
		super::deallocate(data);
	}

	template <typename... Args>
	[[nodiscard]] T* construct(Args&&... args)
	{
		T* data = ::new (allocate()) T(std::forward<Args>(args)...);
		this->construct_(data);
		return data;
	}
	void destruct(T* data)
	{
		this->destruct_(data);
		deallocate(data);
	}

	void release() noexcept
	{
		m_reuse = std::vector<void*>();
		super::release();
	}

protected:
	std::vector<void*> m_reuse;
};

} // namespace inx

#endif // GMLIB_FACTORY_HPP_INCLUDED
