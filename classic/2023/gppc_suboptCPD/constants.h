#ifndef WARTHOG_CONSTANTS_H
#define WARTHOG_CONSTANTS_H

// constants.h
//
// @author: dharabor
// @created: 01/08/2012
//

#include <cfloat>
#include <cmath>
#include <climits>
#include <unordered_map>
#include <stdint.h>

namespace warthog
{
  // each node in a weighted grid map uses sizeof(dbword) memory.
  // in a uniform-cost grid map each dbword is a contiguous set
  // of nodes s.t. every bit represents a node.
  typedef unsigned char dbword;
  typedef uint32_t cost_t;

  // gridmap constants
  static const uint32_t DBWORD_BITS = sizeof(warthog::dbword)*8;
  static const uint32_t DBWORD_BITS_MASK = (warthog::DBWORD_BITS-1);
  static const uint32_t LOG2_DBWORD_BITS = ceil(log10(warthog::DBWORD_BITS) / log10(2));

  // search and sort constants
  static const double DBL_ONE = 1.0f;
  static const double DBL_TWO = 2.0f;
  static const double DBL_ROOT_TWO = 1.414213562f;
  static const double DBL_ONE_OVER_TWO = 0.5;
  static const double DBL_ONE_OVER_ROOT_TWO = 1.0/DBL_ROOT_TWO;//0.707106781f;
  static const double DBL_ROOT_TWO_OVER_FOUR = DBL_ROOT_TWO*0.25;
  static const double EPS = 1e-1;
  static const warthog::cost_t ONE = 10000;
  static const warthog::cost_t TWO = 20000;
  static const warthog::cost_t ROOT_TWO = DBL_ROOT_TWO * ONE;
  static const warthog::cost_t ONE_OVER_TWO = DBL_ONE_OVER_TWO * ONE;
  static const warthog::cost_t ONE_OVER_ROOT_TWO = DBL_ONE_OVER_ROOT_TWO * ONE;
  static const warthog::cost_t ROOT_TWO_OVER_FOUR = DBL_ROOT_TWO * ONE;
  static const warthog::cost_t INF = 0xffffffff;

  // hashing constants
  static const uint32_t FNV32_offset_basis = 2166136261;
  static const uint32_t FNV32_prime = 16777619;

  // cpd mask
  static const uint16_t DIAGs = 0x00F0; //16 + 32 + 64 + 128;
  static const uint16_t STRAIGHTs = 0x000F;
  static const uint16_t ALLMOVE = 0x7FFF;
  static const uint16_t NOMOVE = 0x8000;
  static const uint16_t HMASK = 0x0100;
  static const uint16_t CENTMASK = 0x0200;
  static const uint16_t OCTILE = 0xFF;

  // mask to int
  static const std::unordered_map<int, int> m2i = {
    {1<<0, 0}, {1<<1, 1}, {1<<2, 2}, {1<<3, 3},
    {1<<4, 4}, {1<<5, 5}, {1<<6, 6}, {1<<7, 7},
    {1<<8, 8}, {1<<9, 9}, {1<<10, 10}, {1<<11, 11},
    {1<<12, 12}, {1<<13, 13}, {1<<14, 14}, {1<<15, 15},
  };

  // graph mapper              0, 1, 2, 3, 4,  5, 6, 7
  static const int16_t dx[] = {0, 0, 1, -1, 1, -1, 1, -1};
  static const int16_t dy[] = {-1, 1, 0, 0, -1, -1, 1, 1};
  static const int dw[] = {1000, 1000, 1000, 1000, 1414, 1414, 1414, 1414};
  static const double doublew[] = {1.0, 1.0, 1.0, 1.0, DBL_ROOT_TWO, DBL_ROOT_TWO, DBL_ROOT_TWO, DBL_ROOT_TWO};
  // Clockwise start with north
  static const int CW[] = {0, 4, 2, 6, 1, 7, 3, 5};
  // Counter Clockwise start with north
  static const int CCW[] = {0, 5, 3, 7, 1, 6, 2, 4};
  static const int INV_MOVE[] = {1, 0, 3, 2, 7, 6, 5, 4};
  static const int INVALID_MOVE = 15;
  static const int v2i[3][3] = {
    // 0~7 represent valid octile moves direction and 
    // 0~14 represent valid move id in road network
    // 15 represent an invalid move
    {5, 3, 7},
    {0, INVALID_MOVE, 1},
    {4, 2, 6}
  };

  static inline int lowb(const int& mask) {
    return mask & (-mask);
  };
}

#endif

