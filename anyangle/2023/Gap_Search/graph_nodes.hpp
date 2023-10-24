#ifndef GRAPH_NODES_HPP
#define GRAPH_NODES_HPP

#include <cstdint>
#include <cassert>
#include "vec2.hpp"

class GraphNode  
{
    public: 
        Vec2 position; 
        GraphNode* next_node;
        uint32_t branch_count;

        bool free;    

        GraphNode(); 
        GraphNode( Vec2, GraphNode* );

        GraphNode* set( const Vec2&, GraphNode* ); 

        void connect_to_parent( GraphNode* ); 
        void disconnect_from_parent(); 

        void store() { free = true; next_node = nullptr; }; 
        void release() { free = false; next_node = nullptr; };
        bool is_free() { return free; } 
}; 

GraphNode::GraphNode() : 
    position(), 
    next_node( nullptr ),
    branch_count( 0 ),
    free( true )
{}

GraphNode::GraphNode( Vec2 pos_, GraphNode* prnt_ ) : 
    position( pos_ ),
    next_node( prnt_ ),
    branch_count( 0 ),
    free( true )
{} 

GraphNode* GraphNode::set( const Vec2& pos_, GraphNode* prnt_ )
{
    position = pos_; 
    branch_count = 0;
    if ( prnt_ != nullptr ) 
        connect_to_parent( prnt_ );
    else
        next_node = nullptr;
    return this;  
}

void GraphNode::connect_to_parent( GraphNode* prnt_ )
{
    next_node = prnt_;
    prnt_->branch_count++; 
}

void GraphNode::disconnect_from_parent()
{
    next_node->branch_count--; 
    next_node = nullptr; 
}

struct BorderRay
{
    int32_t dx;
    int32_t dy;
    bool active; 

    BorderRay( int, int, bool );
    BorderRay();
};

BorderRay::BorderRay( int dx_, int dy_, bool active_ ) :
    dx( dx_ ), dy( dy_ ), active( active_ )
{}

BorderRay::BorderRay() :
    dx( 1 ), dy( 0 ), active( true )
{}

class LeafNode 
{
    public:
        Vec2 position; 
        double distance;   

        union{
            LeafNode* next_node; 
            GraphNode* parent;  
        }; 

        bool free; 

        int32_t x_sign; 
        int32_t y_sign;

        BorderRay top;
        BorderRay btm; 

        bool down_row_blocked;  
        int32_t down_row_open_till; 

        int32_t x; 
        int32_t y;

        LeafNode(); 

        LeafNode* set( Vec2, double, GraphNode*, int32_t, int32_t, BorderRay, BorderRay, bool, int32_t, int32_t, int32_t ); 

        void connect_to_parent( GraphNode* ); 
        void disconnect_from_parent();

        void store() { free = true; next_node = nullptr; }; 
        void release() { free = false; parent = nullptr; }; 
        bool is_free() { return free; } 
}; 

LeafNode::LeafNode() : 
    position(), 
    distance( 0.0 ),
    next_node( nullptr ),
    free( true ),    
    x_sign( 1 ), y_sign( 1 ), 
    top( 0, 1, true ), btm( 1, 0, true ),
    down_row_blocked( false ),   
    down_row_open_till( -1 ), 
    x( 0 ), y( 0 )
{}

LeafNode* LeafNode::set( 
    Vec2 position_, 
    double distance_, 
    GraphNode* parent_,
    int32_t x_sign_, 
    int32_t y_sign_, 
    BorderRay top_, 
    BorderRay btm_, 
    bool down_row_blocked_ = false, 
    int32_t down_row_open_till_ = -1, 
    int32_t x_ = 0, 
    int32_t y_ = 0  
) 
{
    position = position_; 
    distance = distance_; 

    assert( parent_ != nullptr );
    connect_to_parent( parent_ ); 

    x_sign = x_sign_; 
    y_sign = y_sign_; 
    
    top = top_; 
    btm = btm_; 

    down_row_blocked = down_row_blocked_; 
    down_row_open_till = down_row_open_till_; 
    
    x = x_; 
    y = y_;

    return this; 
}

void LeafNode::connect_to_parent( GraphNode* parent_ )
{
    parent = parent_;
    parent_->branch_count++; 
}

void LeafNode::disconnect_from_parent()
{
    parent->branch_count--; 
    parent = nullptr; 
}

#endif // GRAPH_NODES_HPP
