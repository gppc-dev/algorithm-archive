#ifndef HEURISTICS_HPP 
#define HEURISTICS_HPP

#include <cmath>

/**
 * A weighted A* heuristics 
*/

const double H_FACTOR = 1.0 + pow( 0.5, 16 );   
inline double heuristics( const double& dist_, const double& target_dist_ )
{
    // a simple weighted A* heuristics ... 
    return dist_ + H_FACTOR * target_dist_; 
}

// if EXTRA_OPTIMALITY_CHECK is defined the algorithm will take a few extra steps
// after the first solution is found, which can improve optimality of the solution 
// in reality a very small improvement happens maybe once in a hundred or so runs
// the time on these extra steps can be saved if path optimality is not important 
// #define EXTRA_OPTIMALITY_CHECK  

#ifdef EXTRA_OPTIMALITY_CHECK
const double ONE_OVER_H_FACTOR = 1.0 / ( 1 + pow( 0.5, 16 ) );   
inline double estimate_dist( const double& dist_, const double& heurist_ )
{
    // estimation of the solution distance from heuristics 
    // in case of LoS visibility 
    return dist_ + ( heurist_ - dist_ ) * ONE_OVER_H_FACTOR; 
}
#endif //EXTRA_OPTIMALITY_CHECK

#endif // HEURISTICS_HPP