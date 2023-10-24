#ifndef GMLIB_BIT_TABLE_HPP_INCLUDED
#define GMLIB_BIT_TABLE_HPP_INCLUDED

#include "inx.hpp"
#include "bits.hpp"
#include <cstring>
#ifndef NDEBUG
#include <vector>
#endif

namespace inx {

template <size_t BitCount = 1, size_t BufferSize = 0, typename PackType = size_t>
class bit_table
{
public:
	static_assert(std::is_integral_v<PackType> && std::is_unsigned_v<PackType>, "PackType must be an unsigned integer type");
	static_assert(0 < BitCount && BitCount <= 8, "BitCount must fall within [1,8]");
	static constexpr size_t bit_count = BitCount;
	static constexpr size_t buffer_size = BufferSize;
	using pack_type = PackType;
	// struct index_t { uint32 word, bit; };
	struct index_t
	{
		uint64_t id;
		index_t() noexcept = default;
		constexpr index_t(uint64_t l_id) noexcept : id(l_id) { }
		constexpr index_t(uint32_t l_word, uint32_t l_bit) noexcept : id((static_cast<uint64_t>(l_word) << pack_bits_size) | static_cast<uint64_t>(l_bit))
		{ assert((l_bit & pack_bits_mask) == l_bit); }

		constexpr uint32_t word() const noexcept { return static_cast<uint32_t>(id >> pack_bits_size); }
		constexpr uint32_t bit() const noexcept { return static_cast<uint32_t>(id & pack_bits_mask); }

		constexpr void adj_col(int64_t i) noexcept
		{
			id = static_cast<uint64_t>(static_cast<int64_t>(id) + (i << bit_adj));
		}
		constexpr void adj_word(int64_t i) noexcept
		{
			id = static_cast<uint64_t>(static_cast<int64_t>(id) + (i << pack_bits_size));
		}
	};

	struct adj_index : index_t
	{
		int64_t row;
		adj_index() noexcept = default;
		constexpr adj_index(index_t l_idx, int64_t l_row) noexcept : index_t(l_idx), row(l_row << pack_bits_size)
		{ }

		using index_t::word;
		constexpr uint32_t word(int64_t rows) const noexcept { return static_cast<uint32_t>(static_cast<uint64_t>(static_cast<int64_t>(this->id) + (rows * row)) >> pack_bits_size); }
		
		constexpr void adj_row(int64_t i) noexcept
		{
			this->id = static_cast<uint64_t>(static_cast<int64_t>(this->id) + (i * row));
		}
		constexpr uint32_t row_word() const noexcept { return static_cast<uint32_t>(row >> pack_bits_size); }
	};

public:
	static constexpr pack_type bit_mask = make_mask<pack_type,bit_count>();
	static constexpr size_t bit_adj = std::bit_width(bit_count - 1);
	static constexpr size_t char_adj = std::bit_width(static_cast<uint32_t>(CHAR_BIT - 1));
	static constexpr size_t pack_bits = sizeof(pack_type) * CHAR_BIT;
	static_assert(8 <= pack_bits && pack_bits <= 64, "pack_bits must be between 8 and 64");
	static constexpr size_t pack_bits_size = std::bit_width(pack_bits - 1);
	static constexpr pack_type pack_bits_mask = make_mask<pack_type,pack_bits_size>();
	static constexpr size_t pack_size = pack_bits_size - bit_adj;
	static constexpr pack_type pack_mask = make_mask<pack_type,pack_size>();
	static constexpr size_t item_count = (1 << pack_size);

public:
	bit_table() noexcept : mWidth(0), mHeight(0), mRowWords(0) { }
	bit_table(uint32 width, uint32 height)
	{
		setup(width, height);
	}

	void setup(uint32 width, uint32 height)
	{
		assert(width > 0);
		assert(height > 0);
		mWidth = width;
		mHeight = height;
		mRowWords = static_cast<uint32>(-(-static_cast<int32>(static_cast<pack_type>(width + 2 * buffer_size)) >> pack_size) + 1);
#ifdef NDEBUG
		mCells = std::make_unique<pack_type[]>((mHeight + 2 * buffer_size) * mRowWords);
#else
		mCells.assign((mHeight + 2 * buffer_size) * mRowWords, 0);
#endif
	}

	pack_type bit_get(int32 x, int32 y) const noexcept
	{
		return bit_get(bit_index(x, y));
	}
	template <size_t I = 0>
	bool bit_test(int32 x, int32 y) const noexcept
	{
		return bit_test<I>(bit_index(x, y));
	}
	void bit_set(int32 x, int32 y, pack_type value) noexcept
	{
		return bit_set(bit_index(x, y), value);
	}
	void bit_clear(int32 x, int32 y) noexcept
	{
		return bit_clear(bit_index(x, y));
	}
	void bit_and(int32 x, int32 y, pack_type value) noexcept
	{
		return bit_and(bit_index(x, y), value);
	}
	void bit_or(int32 x, int32 y, pack_type value) noexcept
	{
		return bit_or(bit_index(x, y), value);
	}
	void bit_xor(int32 x, int32 y, pack_type value) noexcept
	{
		return bit_xor(bit_index(x, y), value);
	}

	void set_buffer(pack_type value) noexcept
	{
		if constexpr (BufferSize != 0) {
			for (int32 i = -static_cast<int32>(BufferSize), j = static_cast<int32>(mHeight); i < 0; i++, j++)
			for (int32 x  = -static_cast<int32>(BufferSize),
					   xe = x + static_cast<int32>(mWidth) + static_cast<int32>(2*BufferSize);
					x < xe; ++x) {
				bit_set(x, i, value);
				bit_set(x, j, value);
			}

			for (int32 i = -static_cast<int32>(BufferSize), j = static_cast<int32>(mWidth); i < 0; i++, j++)
			for (int32 y  = 0,
					   ye = static_cast<int32>(mHeight);
					y < ye; ++y) {
				bit_set(i, y, value);
				bit_set(j, y, value);
			}
		}
	}

	template <int32 W, int32 H>
	pack_type region(index_t id) const noexcept
	{
		static_assert(W > 0 && H > 0, "region must be positive");
		static_assert(static_cast<int64>(W) * static_cast<int64>(H) <= static_cast<int64>(1 << pack_size), "must be packable in a single pack_type");
		constexpr int32 WH = W * H;

#if 0
		if constexpr (bit_count == 1 || bit_count == 2 || bit_count == 4 || bit_count == 8) { // tight packing of bits
			auto id = bit_index(x - X, y - Y);
			uint32_t word = id.word(), bit = id.bit();
			if (bit + (W<<bit_adj) > pack_bits) { // split bits
				uint32 w1count = pack_bits - bit;
				pack_type w2mask = make_mask<pack_type>((W << bit_adj) - w1count);
				pack_type ans = bit_right_shift<pack_type>(mCells[word], bit) | bit_left_shift<pack_type>(mCells[word+1] & w2mask, w1count);
				for (uint32 i = W << bit_adj; i < static_cast<uint32>(H * (W<<bit_adj)); i += W << bit_adj) {
					word += mRowWords;
					ans |= bit_left_shift<pack_type>(bit_right_shift<pack_type>(mCells[word], bit) | bit_left_shift<pack_type>(mCells[word+1] & w2mask, w1count), i);
				}

				return ans;
			} else {
				pack_type w1mask = make_mask<pack_type>(W << bit_adj);
				pack_type ans = bit_right_shift<pack_type>(mCells[word], bit) & w1mask;
				for (uint32 i = W << bit_adj; i < static_cast<uint32>(H * (W<<bit_adj)); i += W << bit_adj) {
					word += mRowWords;
					ans |= bit_left_shift<pack_type>(bit_right_shift<pack_type>(mCells[word], bit) & w1mask, i);
				}

				return ans;
			}
		} else {
			pack_type ans = 0;
			for (int32 i = 0, j = -Y; j < H-Y; j++)
			for (int32 k = -X; k < W-X; k++, i+=bit_count)
				ans |= bit_get(x+k, y+j) << i;
			return ans;
		}
#else
	if constexpr (W == 1 && H == 1) {
		return bit_get(id);
	} else {
		auto word = id.word();
		auto bit = id.bit();
		uint32_t row = mRowWords;
		if constexpr (W == 1) {
			const auto* cell = data() + word;
			pack_type ans = bit_right_shift<pack_type>(*cell, bit) & bit_mask;
			for (size_t i = bit_count; i < bit_count*static_cast<size_t>(H); i += bit_count) {
				cell += row;
				ans |= bit_shift_to<i>(*cell, bit) & bit_left_shift<i>(bit_mask);
			}
			return ans;
		} else if constexpr (std::endian::native == std::endian::little && static_cast<size_t>(W) * bit_count <= (sizeof(size_t) - 1) * CHAR_BIT) {
			// readable in a single non-aligned read
			row *= sizeof(size_t);
			const auto* cell = std::bit_cast<const std::byte*>(data() + word) + (bit >> char_adj);
			bit &= inx::make_mask_v<decltype(bit), char_adj>;
			constexpr size_t bit_row = bit_count * static_cast<size_t>(W);
			size_t ans;
			{
				size_t tmp;
				std::memcpy(&tmp, cell, sizeof(size_t));
				ans = (tmp >> bit) & make_mask_v<pack_type, bit_row>;
			}
			for (size_t i = bit_row; i < bit_row*static_cast<size_t>(H); i += bit_row) {
				cell += row;
				size_t tmp;
				std::memcpy(&tmp, cell, sizeof(size_t));
				ans |= bit_shift_from_to(tmp, bit, i) & make_mask<pack_type>(bit_row, i);
			}
			return static_cast<pack_type>(ans);
		} else {
			// TODO: not done
			assert(false);
			// const auto* cell = data() + word;
			// pack_type ans = bit_right_shift<pack_type>(*cell, bit) & bit_mask;
			// for (size_t i = bit_count; i < bit_count*static_cast<size_t>(H); i += bit_count) {
			// 	cell += row;
			// 	ans |= bit_shift_to<i>(*cell, bit) & bit_left_shift<i>(bit_mask);
			// }
			return {};
		}
	}
#endif
	}

	template <int32 X, int32 Y, int32 W, int32 H>
	pack_type region(int32 x, int32 y) const noexcept
	{
		static_assert(X >= 0 && W > 0 && X < W, "x must lie within region");
		static_assert(Y >= 0 && H > 0 && Y < H, "y must lie within region");
		static_assert(static_cast<int64>(W) * static_cast<int64>(H) <= static_cast<int64>(1 << pack_size), "must be packable in a single pack_type");
		assert(-static_cast<int32>(buffer_size) <= x-X && x-X+W <= static_cast<int32>(mWidth + buffer_size));
		assert(-static_cast<int32>(buffer_size) <= y-Y && y-Y+H <= static_cast<int32>(mHeight + buffer_size));
#ifdef NDEBUG
		return region<W, H>(bit_index(x-X, y-Y));
#else
		auto r = region<W, H>(bit_index(x-X, y-Y));
		auto t = r;
		for (int32 i = -Y; i < H-Y; ++i)
		for (int32 j = -X; j < W-X; ++j) {
			auto q = bit_get(x+j, y+i);
			assert((t & bit_mask) == q);
			t >>= bit_count;
		}
		return r;
#endif
	}

	uint32 getWidth() const noexcept { return mWidth; }
	uint32 getHeight() const noexcept { return mHeight; }
	uint32 getRowWords() const noexcept { return mRowWords; }

	bool empty() const noexcept { return mWidth == 0; }
	void clear()
	{
		mWidth = 0;
		mHeight = 0;
		mRowWords = 0;
		mCells = nullptr;
	}

	std::pair<uint32_t, uint32_t> bit_pair_index(int32 x, int32 y) const noexcept /// returns pair[word,bit]
	{
		assert(-static_cast<int32>(buffer_size) <= x && x < static_cast<int32>(mWidth + buffer_size));
		assert(-static_cast<int32>(buffer_size) <= y && y < static_cast<int32>(mHeight + buffer_size));
		x += buffer_size;
		return {
			static_cast<uint32>( (y + buffer_size) * mRowWords + (x >> pack_size) ),
			static_cast<uint32>( (x & pack_mask) << bit_adj )
		};
	}
	index_t bit_index(int32 x, int32 y) const noexcept
	{
		assert(-static_cast<int32>(buffer_size) <= x && x < static_cast<int32>(mWidth + buffer_size));
		assert(-static_cast<int32>(buffer_size) <= y && y < static_cast<int32>(mHeight + buffer_size));
		x += buffer_size;
		return index_t(
			static_cast<uint32>( (y + buffer_size) * mRowWords + (x >> pack_size) ),
			static_cast<uint32>( (x & pack_mask) << bit_adj )
		);
	}
	adj_index bit_adj_index(int32 x, int32 y) const noexcept
	{
		return adj_index(bit_index(x, y), mRowWords);
	}
	pack_type bit_get(index_t id) const noexcept
	{
		return bit_right_shift<pack_type>(mCells[id.word()], id.bit()) & bit_mask;
	}
	template <size_t I = 0>
	bool bit_test(index_t id) const noexcept
	{
		return static_cast<bool>(bit_right_shift<pack_type>(mCells[id.word()], id.bit()+I) & 1);
	}
	void bit_set(index_t id, pack_type value) noexcept
	{
		assert(value <= bit_mask);
		mCells[id.word()] = (mCells[id.word()] & ~bit_left_shift<pack_type>(bit_mask, id.bit()))
			| bit_left_shift<pack_type>(value & bit_mask, id.bit());
	}
	void bit_clear(index_t id) noexcept
	{
		mCells[id.word()] &= ~bit_left_shift<pack_type>(bit_mask, id.bit());
	}
	void bit_or(index_t id, pack_type value) noexcept
	{
		assert(value <= bit_mask);
		mCells[id.word()] |= bit_left_shift<pack_type>(value & bit_mask, id.bit());
	}
	void bit_and(index_t id, pack_type value) noexcept
	{
		assert(value <= bit_mask);
		mCells[id.word()] &= ~bit_left_shift<pack_type>((~value) & bit_mask, id.bit());
	}
	void bit_xor(index_t id, pack_type value) noexcept
	{
		assert(value <= bit_mask);
		mCells[id.word()] ^= bit_left_shift<pack_type>(value & bit_mask, id.bit());
	}
	void bit_not(index_t id) noexcept
	{
		mCells[id.word()] ^= bit_left_shift<pack_type>(bit_mask, id.bit());
	}

	pack_type word_get(index_t id) const noexcept
	{
		return mCells[id.word()];
	}
	void word_set(index_t id, pack_type value) noexcept
	{
		mCells[id.word()] = value;
	}

	const pack_type* data() const noexcept
	{
#ifdef NDEBUG
		return mCells.get();
#else
		return mCells.data();
#endif
	}
	pack_type* data() noexcept
	{
#ifdef NDEBUG
		return mCells.get();
#else
		return mCells.data();
#endif
	}
	
private:
	uint32 mWidth, mHeight, mRowWords;
#ifdef NDEBUG
	std::unique_ptr<pack_type[]> mCells;
#else
	std::vector<pack_type> mCells;
#endif

};

} // namespace inx::alg

#endif