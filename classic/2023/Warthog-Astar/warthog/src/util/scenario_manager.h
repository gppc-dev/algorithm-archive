#ifndef WARTHOG_SCENARIO_MANAGER_H
#define WARTHOG_SCENARIO_MANAGER_H

// scenario_manager.h
//
// Read and generate/write scenario files
//
//	Supported formats for read:
//	    - GPPC 1.0 format (as at 2012 Grid-based Path Planning Competition)
//		  (fields: bucket,map,mapwidth,mapheight,sx,sy,gx,gy,distance)
//	    - DIMACS format (as at the 9th DIMACS Implementation Challenge)
//	      (fields: q [source-id] [target-id])
//
//	Supported formats for generate/write:
//	    - GPPC 1.0 format (as at 2012 Grid-based Path Planning Competition)
//
// @author: dharabor
// @created: 21/08/2012
//

#include "experiment.h"
#include "gridmap.h"
#include "gridmap_expansion_policy.h"
#include "octile_heuristic.h"

#include <vector>
#include <fstream>
#include <iostream>

namespace warthog
{

class scenario_manager 
{
	public:
		scenario_manager();
		~scenario_manager();

		virtual warthog::experiment* 
		get_experiment(unsigned int which) 
		{ 
			if(which < experiments_.size())
			{
				return experiments_[which]; 
		   	}
			return 0;
		}

		virtual void 
		add_experiment(warthog::experiment* newexp) 
		{ 
			experiments_.push_back(newexp); 
		}

		virtual uint32_t 
		num_experiments() 
		{ 
			return (uint32_t)experiments_.size(); 
		}

		virtual size_t 
		mem()
		{
			return sizeof(*this) + 
				sizeof(warthog::experiment) * experiments_.size();
		}

		inline std::string last_file_loaded() { return sfile_; }
		inline void clear() { experiments_.clear(); }
		
		void generate_experiments(warthog::gridmap*, int num);
		void load_scenario(const char* filelocation);
		virtual void load_scenario(std::istream& infile, const char* filelocation = nullptr);
		virtual void write_scenario(std::ostream& out);
		virtual void sort(); // organise by increasing solution length


	protected:
		experiment* generate_single_experiment(warthog::gridmap*);
		void load_gppc_scenario(std::istream& infile);

		std::vector<warthog::experiment*> experiments_;
		std::string sfile_;
};

}

#endif
