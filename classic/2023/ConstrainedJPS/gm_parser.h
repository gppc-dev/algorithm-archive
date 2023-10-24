#ifndef WARTHOG_GRIDMAP_PARSER_H
#define WARTHOG_GRIDMAP_PARSER_H

// gm_parser.h
//
// A parser for gridmap files written in Nathan Sturtevant's HOG format.
//
// @author: dharabor
// @created: 08/08/2012
//

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <vector>

namespace warthog
{
	class gm_header
	{
		public:
			gm_header(unsigned int height, unsigned int width, const char* type)
				: height_(height), width_(width), type_(type)
			{
			}

			gm_header() : height_(0), width_(0), type_("") 
			{ 
			}

			gm_header(const warthog::gm_header& other) 
			{
				(*this) = other;
			}

			virtual ~gm_header() 
			{ 
			}

			gm_header& operator=(const warthog::gm_header& other)
			{
				this->height_ = other.height_;
				this->width_ = other.width_;
				this->type_ = other.type_;
				return *this;
			}

			unsigned int height_;
			unsigned int width_;
			std::string type_;
	};
}

#endif

