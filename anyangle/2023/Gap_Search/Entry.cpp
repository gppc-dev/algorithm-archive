#include <algorithm>
#include <map>
#include "Entry.h"
#include "gap_search.hpp"

/**
 * User code used during preprocessing of a map.  Can be left blank if no pre-processing is required.
 * It will not be called in the same program execution as `PrepareForSearch` is called,
 * all data must be shared through file.
 * 
 * Called with command below:
 * ./run -pre file.map
 * 
 * @param[in] bits Array of 2D table.  (0,0) is located at top-left corner.  bits.size() = height * width
 *                 Packed as 1D array, row-by-ray, i.e. first width bool's give row y=0, next width y=1
 *                 bits[i] returns `true` if (x,y) is traversable, `false` otherwise
 * @param[in] width Give the map's width
 * @param[in] height Give the map's height
 * @param[in] filename The filename you write the preprocessing data to.  Open in write mode.
 */
void PreprocessMap(const std::vector<bool> &bits, int width, int height, const std::string &filename) {}

/**
 * User code used to setup search before queries.  Can also load pre-processing data from file to speed load.
 * It will not be called in the same program execution as `PreprocessMap` is called,
 * all data must be shared through file.
 * 
 * Called with any commands below:
 * ./run -run file.map file.map.scen
 * ./run -check file.map file.map.scen
 * 
 * @param[in] bits Array of 2D table.  (0,0) is located at top-left corner.  bits.size() = height * width
 *                 Packed as 1D array, row-by-ray, i.e. first width bool's give row y=0, next width y=1
 *                 bits[i] returns `true` if (x,y) is traversable, `false` otherwise
 * @param[in] width Give the map's width
 * @param[in] height Give the map's height
 * @param[in] filename The filename you write the preprocessing data to.  Open in write mode.
 * @returns Pointer to data-structure used for search.  Memory should be stored on heap, not stack.
 */
void *PrepareForSearch( const std::vector<bool> &bits, int width, int height, const std::string &filename ) 
{
    GapSolver* solver = new GapSolver( bits, width, height );
    return solver;
}

/**
 * User code used to setup search before queries.  Can also load pre-processing data from file to speed load.
 * It will not be called in the same program execution as `PreprocessMap` is called,
 * all data must be shared through file.
 * 
 * Called with any commands below:
 * ./run -run file.map file.map.scen
 * ./run -check file.map file.map.scen
 * 
 * @param[in,out] data Pointer to data returned from `PrepareForSearch`.  Can static_cast to correct data type.
 * @param[in] s The start (x,y) coordinate of search query
 * @param[in] g The goal (x,y) coordinate of search query
 * @param[out] path The points that forms the shortest path from `s` to `g` computed by search algorithm.
 *                  Shortest path length will calculated by summation of Euclidean distance
 *                  between consecutive pairs path[i]--path[i+1].  Collinear points are allowed.
 *                  Return an empty path if no shortest path exists.
 * @returns `true` if search is complete, including if no-path-exists.  `false` if search only partially completed.
 *          if `false` then `GetPath` will be called again until search is complete.
 */
bool GetPath( void *solver, xyLoc s, xyLoc g, std::vector<xyLoc> &path ) 
{
    GapSolver* gap_solver = ( GapSolver* ) ( solver );

    gap_solver->reset(); 

    Vec2 start( s.x, s.y ); 
    Vec2 goal( g.x, g.y ); 

    bool success = gap_solver->find_path( start, goal );

    if ( ! success )
        return false;   

    std::vector< Vec2 > solution_path;
    gap_solver->solution_path( solution_path ); 

    if ( solution_path.size() <= 1 )
        return true; 

    path.reserve( solution_path.size() );
    
    for( auto it = solution_path.rbegin(); it != solution_path.rend(); ++it )
        path.push_back( { static_cast< double >( it->x ), static_cast< double >( it->y ) } );
    
    return true;
}

/**
 * The algorithm name.  Please update std::string and ensure name is immutable.
 * 
 * @returns the name of the algorithm
 */
std::string GetName() { return "Gap_Solver"; }
