#ifndef SCENE_HPP
#define SCENE_HPP

#include <cassert>
#include <vector>
#include "vec2.hpp"

/**
 * A "scene-matrix" class used in search to store local temporay solition values  
 * It is generated according to dimensions of the occupancy matrix  
*/

class Scene
{
    private:
        std::size_t x_size;   
        std::size_t y_size;   
        std::vector< double > data;

    public: 
        Scene( const std::size_t x_max, const std::size_t y_max ); 

        double& operator [] ( const Vec2& );
        bool out_of_bounds( const int, const int ); 
        bool out_of_bounds( const Vec2& ); 

        void reset(); 
};

Scene::Scene( const std::size_t x_max_, const std::size_t y_max_ ) : 
    x_size( x_max_ + 1 ), 
    y_size( y_max_ + 1 ), 
    data( x_size * y_size, INFINITY ) 
{}

double& Scene::operator [] ( const Vec2& pos_ ) 
{
    assert( ! out_of_bounds( pos_ ) ); 
    return data[ pos_.y * x_size + pos_.x ]; 
}

bool Scene::out_of_bounds( const int x_, const int y_ )
{
    return ( x_ < 0 || x_ >= x_size ) || ( y_ < 0 || y_ >= y_size ); 
}

bool Scene::out_of_bounds( const Vec2& pos_ )
{
    return out_of_bounds( pos_.x, pos_.y ); 
}

void Scene::reset()
{
    std::fill( data.begin(), data.begin() + x_size * y_size, INFINITY ); 
}

#endif // SCENE_HPP
