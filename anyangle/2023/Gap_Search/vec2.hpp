#ifndef VEC2_HPP
#define VEC2_HPP

#include <cmath> 

/**
 * A basic 2D integer vector class  
 * 
*/

class Vec2
{
    public:

        int x;
        int y; 

        // constructors 
        Vec2();
        Vec2( const int, const int );
        Vec2( const Vec2& );

        // assignment 
        Vec2& operator =  ( const Vec2& );

        // logic 
        bool operator == ( const Vec2& ) const;  
        bool operator != ( const Vec2& ) const;  

        // addition / substraction   
        Vec2  operator +  ( const Vec2& ) const;
        Vec2  operator -  ( const Vec2& ) const;
        Vec2& operator += ( const Vec2& );
        Vec2& operator -= ( const Vec2& );

        // scaling 
        Vec2  operator *  ( const int ) const;
        Vec2& operator *= ( const int );

        // products 
        int dot( const Vec2& ) const; 
        int cross( const Vec2& ) const; 

        double norm() const;  

        // turns 
        Vec2 cw() const;  
        Vec2 ccw() const;
};

Vec2::Vec2() : x( 0 ), y( 0 ) {}

Vec2::Vec2( const int x_, const int y_ ) : x( x_ ), y( y_ ) {}

Vec2::Vec2( const Vec2& copy_ ) : x( copy_.x ), y( copy_.y ) {}

Vec2& Vec2::operator = ( const Vec2& rhs_ ) 
{
    x = rhs_.x; 
    y = rhs_.y;

    return *this; 
}

bool Vec2::operator == ( const Vec2& rhs_ ) const
{
    return x == rhs_.x && y == rhs_.y;
}
  
bool Vec2::operator != ( const Vec2& rhs_ ) const
{
    return ! ( x == rhs_.x && y == rhs_.y );
}  

Vec2  Vec2::operator + ( const Vec2& rhs_ ) const
{
    return Vec2( x + rhs_.x, y + rhs_.y );
}

Vec2  Vec2::operator - ( const Vec2& rhs_ ) const
{
    return Vec2( x - rhs_.x, y - rhs_.y );
}

Vec2& Vec2::operator += ( const Vec2& rhs_ )
{
    x += rhs_.x; 
    y += rhs_.y;

    return *this;
}

Vec2& Vec2::operator -= ( const Vec2& rhs_ )
{
    x -= rhs_.x; 
    y -= rhs_.y;

    return *this;
}

Vec2 Vec2::operator * ( const int scale_ ) const
{
    return Vec2( x * scale_, y * scale_ );
}

Vec2& Vec2::operator *= ( const int scale_ )
{
    x -= scale_; 
    y -= scale_;

    return *this;
}

int Vec2::dot( const Vec2& rhs_ ) const
{
    return x * rhs_.x + y * rhs_.y; 
}

int Vec2::cross( const Vec2& rhs_ ) const
{
    return ( x * rhs_.y - y * rhs_.x );
}

double Vec2::norm() const  
{
    return std::sqrt( (double) x * x + y * y );
}

Vec2 Vec2::ccw() const
{
    return Vec2( -y, x );
}

Vec2 Vec2::cw() const  
{
    return Vec2( y, -x );
}

double dist( const Vec2& p0_, const Vec2& p1_ )
{
    return ( p1_ - p0_ ).norm(); 
} 

#endif // VEC2_HPP
