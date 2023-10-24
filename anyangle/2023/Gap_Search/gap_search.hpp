#ifndef POOLS_GAP_SEARCH_HPP
#define POOLS_GAP_SEARCH_HPP

#include <iostream>
#include <cstdint>
#include <vector>

#include "vec2.hpp"
#include "solution_graph.hpp"
#include "graph_nodes.hpp"
#include "scene.hpp"
#include "heuristics.hpp"

class GapSolver
{
    public: 
        const std::vector< bool > & occupancy;
        size_t Nx, Ny;  

    private: 
        Scene scene;
        Vec2 p0; 
        int32_t x_sign, y_sign;  
        int32_t x_max, y_max;
        BorderRay top, btm;

        int32_t x, y, x_start; 
        int32_t steps_counter, steps_max;

        std::vector< LeafNode* > gap_seekers; 

    public: 
        SolutionGraph graph; 

    private: 
        GraphNode* new_parent;
        LeafNode* current_seeker; 
        double current_distance; 

        bool up_row_blocked; 
        int32_t up_row_open_till; 
        bool down_row_blocked; 
        int32_t down_row_open_till; 

        bool vertical_down_blocked;  
        bool vertical_up_blocked; 

        int32_t forward, backward; 
        int32_t up, down; 

        bool up_blocked, prev_up_blocked; 
        bool down_blocked, prev_down_blocked;

        bool search_step_finished; 
        bool gap_restart; 

        Vec2 goal;  
        bool solution_found; 
        bool solution_found_this_step; 
        GraphNode* solution_node;

    public: 
        GapSolver( const std::vector< bool > & occupancy_, const size_t nx_, const size_t ny_ );

    private: 
        Vec2 current_position();

        bool frwd_up_blocked( const Vec2& ); 
        bool frwd_dn_blocked( const Vec2& ); 
        bool bkwd_up_blocked( const Vec2& ); 
        bool bkwd_dn_blocked( const Vec2& );  

        void reset_search_state(); 
        void set_xy_signs_and_grid_shifts( const int32_t, const int32_t );  
        void initial_step_horizontal();
        void check_row_at_y_0_tilted_btm(); 
        void check_row_at_y_0_horizontal_passive_btm(); 
        void check_row_at_y_0_horizontal_active_btm(); 
        void check_row();
        void check_row_at_y_max(); 

        GraphNode* get_new_parent(); 
        double goal_distance( const Vec2& );

        void create_BTM_seeker( const int32_t, const double ); 
        void create_TOP_and_GAP_seekers( const double );
        void update_row_blocked_values();
        bool check_LoS_solution();
        bool check_solution_is_gppc_valid();
        bool check_solution_is_gppc_valid( const Vec2& );  
        bool check_solution_in_interval( const int32_t, const int32_t ); 
        void create_solution_node();

        void regular_step();
        void gap_step();
        void search_step(); 
        void initialize_search( const Vec2&, const Vec2& ); 
        int passive_up_border_probe();
        void root_path( LeafNode*, std::vector< Vec2 >& );
        void root_path( GraphNode*, std::vector< Vec2 >& ); 
        bool LoS_OK( const Vec2&, const Vec2& );
        bool LoS_OK_horizontal( const Vec2&, const Vec2& );  
        bool LoS_OK_vertical( const Vec2&, const Vec2& ); 
        bool cornering_OK( const Vec2&, const Vec2&, const Vec2& ); 

    public:
        bool start_goal_gppc_valid( const Vec2& ); 
        bool find_path( const Vec2&, const Vec2& );
        void solution_path( std::vector< Vec2 >& );
        double solution_distance();
        bool path_OK( std::vector< Vec2 >& ); 
        void reset();
};

GapSolver::GapSolver( const std::vector< bool > & occupancy_, const size_t nx_, const size_t ny_ ) :
    occupancy( occupancy_ ), 
    Nx( nx_ ), Ny( ny_ ), 
    scene( nx_, ny_ ), 
    p0(),  
    x_sign( 1 ), y_sign( 1 ),   
    x_max( 0 ), y_max( 0 ),
    top( 0, 1, true ), btm( 1, 0, true ),
    x( 0 ), y( 0 ), x_start( 0 ),
    steps_counter( 0 ), steps_max( nx_ * ny_ ),
    gap_seekers(), 
    graph( ( nx_ + ny_ ) / 2, ( nx_ + ny_ ) / 2  ), // a rough initial estimate
    new_parent( nullptr ), 
    current_seeker(), 
    current_distance( 0.0 ),
    up_row_blocked( false ), 
    up_row_open_till( -1 ), 
    down_row_blocked( false ),  
    down_row_open_till( -1 ), 
    vertical_down_blocked( false ), 
    vertical_up_blocked( false ),
    forward( 0 ), backward( -1 ), 
    up( 0 ), down( -1 ),
    up_blocked( false ), prev_up_blocked( false ),
    down_blocked( false ), prev_down_blocked( false ),
    search_step_finished( false ),
    gap_restart( false ),  
    goal( 0, 0 ),
    solution_found( false ),  
    solution_found_this_step( false ), 
    solution_node()  
{
    gap_seekers.reserve( 64 ); // a rough initial estimate
}

Vec2 GapSolver::current_position()
{
    return p0 + Vec2( x * x_sign, y * y_sign ); 
} 

bool GapSolver::frwd_up_blocked( const Vec2& pos_ )
{
    return ! occupancy[ ( pos_.y + up ) * Nx + ( pos_.x + forward ) ]; 
}

bool GapSolver::frwd_dn_blocked( const Vec2& pos_ ) 
{
    return ! occupancy[ ( pos_.y + down ) * Nx + ( pos_.x + forward ) ]; 
}

bool GapSolver::bkwd_up_blocked( const Vec2& pos_ ) 
{
    return ! occupancy[ ( pos_.y + up ) * Nx + ( pos_.x + backward ) ]; 
}

bool GapSolver::bkwd_dn_blocked( const Vec2& pos_ ) 
{
    return ! occupancy[ ( pos_.y + down ) * Nx + ( pos_.x + backward ) ]; 
}

void GapSolver::reset_search_state()
{
    search_step_finished = false; 

    p0 = current_seeker->position; 

    set_xy_signs_and_grid_shifts( current_seeker->x_sign, current_seeker->y_sign ); 

    x = current_seeker->x; 
    y = current_seeker->y; 
    x_start = x; 

    current_distance = current_seeker->distance;
    new_parent = nullptr; 
    gap_restart = ( x != 0 || y != 0 ); 

    if ( gap_restart || current_seeker->distance == 0.0 )
        new_parent = current_seeker->parent; 

    top = current_seeker->top; 
    btm = current_seeker->btm; 

    down_row_blocked = current_seeker->down_row_blocked; 
    down_row_open_till = current_seeker->down_row_open_till; 

    solution_found_this_step = false; 
} 

void GapSolver::set_xy_signs_and_grid_shifts( const int32_t x_sign_, const int32_t y_sign_ )
{
    x_sign = x_sign_;
    y_sign = y_sign_;

    x_max = ( x_sign < 0 ) ? p0.x : Nx - p0.x;  
    y_max = ( y_sign < 0 ) ? p0.y : Ny - p0.y;  

    forward = ( x_sign > 0 ) ? 0 : -1; 
    backward = ( x_sign > 0 ) ? -1 : 0; 
    up = ( y_sign > 0 ) ? 0 : -1; 
    down = ( y_sign > 0 ) ? -1 : 0; 
}

void GapSolver::initial_step_horizontal()
{
    if ( x_max >= Nx )
    {
        prev_up_blocked = true; 
        prev_down_blocked = true;  
    }
    else 
    { 
        prev_down_blocked = bkwd_dn_blocked( p0 ); 
        prev_up_blocked = bkwd_up_blocked( p0 );  
    }
    up_blocked = frwd_up_blocked( p0 );
    down_blocked = frwd_dn_blocked( p0 );
    
    up_row_blocked = false; 
    up_row_open_till = -1; 

    down_row_blocked = false; 
    down_row_open_till = -1; 
}

void GapSolver::create_BTM_seeker( const int32_t x_sign_, const double dist_ = -1.0 )
{
    double dist = dist_; 
    if ( dist < 0.0 )
        dist = sqrt( x * x + y * y ); 

    dist += current_distance; 

    Vec2 p = current_position(); 

    if ( scene[ p ] > dist ) 
    {    
        scene[ p ] = dist; 

        if ( x >= y )
        {
            Vec2 p_sol = p + Vec2( x_sign, 0 );

            if ( x + 1 >= x_max || frwd_up_blocked( p_sol ) )
            {
                if ( 
                    p_sol == goal 
                    && scene[ p_sol ] > dist + 1.0
                    && check_solution_is_gppc_valid( p ) 
                    ) 
                {
                    scene[ p_sol ] = dist + 1.0;

                    // create solution node
                    // solution_node.position = goal; 
                    // solution_node.connect_to_parent( 
                    //     graph.get_graph_node()->set( p, get_new_parent() ) 
                    //     ); 

                    if ( solution_node != nullptr )
                    {
                        graph.cut_branch( solution_node );
                        solution_node = nullptr; 
                    }

                    solution_node = graph.get_graph_node()->set(
                        goal,
                        graph.get_graph_node()->set( 
                            p, 
                            get_new_parent() 
                            ) 
                        ); 

                    solution_found = true; 
                    solution_found_this_step = true; 
                    search_step_finished = true; 
                }
                
                return; 
            }
        }

        graph.insert_leaf(
            heuristics( dist, goal_distance( p ) ), 
            graph.get_leaf_node()->set(
                p, dist, get_new_parent(), 
                x_sign_, y_sign, 
                BorderRay( x, y, false ),
                BorderRay( 1, 0, true ) 
            )
        );        
    }  
}  

void GapSolver::create_TOP_and_GAP_seekers( const double dist_ = -1.0 )
{
    if ( y != 0 )
    {    
        gap_seekers.push_back(
            graph.get_leaf_node()->set(
                p0, current_distance, get_new_parent(),
                x_sign, y_sign,
                BorderRay( x, y, true ),
                BorderRay( btm.dx, btm.dy, btm.active ),
                down_row_blocked,
                down_row_open_till,
                x, y 
            )
        );
    }

    double dist = dist_; 
    if ( dist < 0.0 )
        dist = sqrt( x * x + y * y );

    dist += current_distance; 

    Vec2 p = current_position(); 

    if ( scene[ p ] > dist )
    {    
        scene[ p ] = dist; 

        if ( y >= x )
        {
            Vec2 p_sol = p + Vec2( 0, y_sign );

            if ( y + 1 >= y_max || frwd_up_blocked( p_sol ) )
            {
               if ( 
                    p_sol == goal 
                    && scene[ p_sol ] > dist + 1.0
                    && check_solution_is_gppc_valid( p ) 
                    ) 
                {
                    scene[ p_sol ] = dist + 1.0;

                    // solution_node.position = goal; 
                    // solution_node.connect_to_parent( 
                    //     graph.get_graph_node()->set( 
                    //         p, get_new_parent() 
                    //     ) 
                    // );

                    if ( solution_node != nullptr )
                    {
                        graph.cut_branch( solution_node );
                        solution_node = nullptr; 
                    }

                    solution_node = graph.get_graph_node()->set(
                        goal,
                        graph.get_graph_node()->set( 
                            p, 
                            get_new_parent() 
                            ) 
                        ); 

                    solution_found = true; 
                    solution_found_this_step = true; 
                    search_step_finished = true; 
                }

                return; 
            }
        }

        graph.insert_leaf( 
            heuristics( dist, goal_distance( p ) ),
            graph.get_leaf_node()->set(
                p, dist, get_new_parent(),
                x_sign, y_sign, 
                BorderRay( 0, 1, true ),
                BorderRay( x, y, false )
            )
        );
    }
}  

void GapSolver::check_row_at_y_0_tilted_btm()
{
    x_start = 0;  

    initial_step_horizontal(); 

    vertical_down_blocked = prev_down_blocked; 
    vertical_up_blocked = prev_up_blocked; 

    up_row_blocked = false; 
    up_row_open_till = 0; 

    int D_btm = btm.dx;  
    Vec2 p;
    while ( ! up_blocked && ( D_btm > 0 || ( btm.active && D_btm == 0 ) ) ) 
    {
        p = current_position();
        up_blocked = ( x >= x_max || frwd_up_blocked( p ) );
        
        if ( ! up_row_blocked )
        { 
            up_row_open_till = x;  
            if ( up_blocked )
                up_row_blocked = true;  
        }

        x += 1; 
        D_btm -= btm.dy; 
    }
}

void GapSolver::check_row_at_y_0_horizontal_passive_btm() 
{
    vertical_down_blocked = ( x_max >= Nx || y_max == 0 || bkwd_up_blocked( p0 ) ); 
}

void GapSolver::check_row_at_y_0_horizontal_active_btm()
{
    x_start = 0; 

    initial_step_horizontal(); 

    vertical_down_blocked = prev_down_blocked; 
    vertical_up_blocked = prev_up_blocked; 

    LeafNode* up_seeker = nullptr; 
    LeafNode* down_seeker = nullptr; 

    up_row_open_till = 0; 
    up_row_blocked = up_blocked; 
    if ( up_row_blocked && prev_up_blocked )
        up_row_open_till = -1; 

    down_row_open_till = 0; 
    down_row_blocked = down_blocked; 
    if ( down_row_blocked && prev_down_blocked )
        down_row_open_till = -1; 

    bool x_blocked = ( up_blocked && down_blocked );  
    Vec2 p; 
    while ( ! x_blocked )
    { 
        x += 1; 
        prev_down_blocked = down_blocked;
        prev_up_blocked = up_blocked; 

        p = current_position(); 
        down_blocked = ( x >= x_max || frwd_dn_blocked( p ) );         
        up_blocked = ( x >= x_max || frwd_up_blocked( p ) );

        x_blocked = ( 
            ( prev_down_blocked && up_blocked ) || 
            ( prev_up_blocked && down_blocked ) || 
            ( up_blocked && down_blocked ) 
            ); 

        if ( ! up_row_blocked )
        {  
            up_row_open_till = x; 
            if ( up_blocked )
                up_row_blocked = true; 
        }         
        else 
        {         
            if ( prev_up_blocked && ! up_blocked ) 
            {
                up_seeker = graph.get_leaf_node()->set(
                    current_position(), 
                    current_distance + x, 
                    get_new_parent(), 
                    x_sign, y_sign, 
                    BorderRay( 0, 1, true ),
                    BorderRay( 1, 0, false )
                );

            }
            else if ( ! prev_up_blocked && up_blocked )
            {    
                up_seeker->down_row_blocked = true;
                up_seeker->down_row_open_till = abs( p0.x + x * x_sign - up_seeker->position.x );
                graph.insert_leaf(
                    heuristics( up_seeker->distance, goal_distance( up_seeker->position ) ),
                    up_seeker
                );

                up_seeker = nullptr; 
            }
        }

        if ( ! down_row_blocked )
        {    
            down_row_open_till = x; 
            if ( down_blocked )
                down_row_blocked = true; 
         }       
         else 
         {   
            if ( prev_down_blocked && ! down_blocked )
            {  
                down_seeker = graph.get_leaf_node()->set(
                    current_position(), 
                    current_distance + x,
                    get_new_parent(),
                    x_sign, -y_sign, 
                    BorderRay( 0, 1, true ),
                    BorderRay( 1, 0, false )
                );
            }
            else if ( ! prev_down_blocked && down_blocked )
            {
                down_seeker->down_row_blocked = true; 
                down_seeker->down_row_open_till = abs( p0.x + x * x_sign - down_seeker->position.x ); 
                graph.insert_leaf(
                    heuristics( down_seeker->distance, goal_distance( down_seeker->position ) ),
                    down_seeker
                );
                
                down_seeker = nullptr; 
            }
        }
    }

    if ( check_solution_in_interval( 0, x ) && check_solution_is_gppc_valid() )
    {
        create_solution_node(); 
    }
}

void GapSolver::check_row() 
{
    Vec2 p; 
    
    if ( ! top.dx == 0 )
    {
        if ( gap_restart )
        { 
            x_start = x; 
            gap_restart = false; 
            prev_up_blocked = false;  
        }
        else  
        {    
            if ( down_row_blocked && down_row_open_till < 0 )
            {
                search_step_finished = true;
                return;
            }
            
            x = x_start;

            int D_top = y * top.dx - x * top.dy;  
            while ( D_top > 0 || ( ! top.active && D_top == 0 ) )
            { 
                if ( down_row_blocked && x >= down_row_open_till )
                {
                    search_step_finished = true; 
                    return; 
                } 

                x += 1;
                D_top -= top.dy; 
            }    

            prev_up_blocked = bkwd_up_blocked( current_position() ); 
            up_row_blocked = prev_up_blocked; 

            x_start = x; 
        }
    }
    else 
    {
        if ( down_row_open_till < 0 )
        {
            search_step_finished = true; 
            return; 
        }

        x = 0; 
        p = current_position(); 

        if ( top.active )
        {        
            x_start = 0;

            up_blocked = ( x_max == 0 || y_max == 0 || frwd_up_blocked( p ) );
            prev_up_blocked = ( x_max >= Nx || y_max == 0 || bkwd_up_blocked( p ) );
        
            vertical_up_blocked = prev_up_blocked;  

            down_blocked = frwd_dn_blocked( p );
            prev_down_blocked = vertical_down_blocked;
            
            bool vertical_blocked = ( 
                ( prev_up_blocked && up_blocked ) || 
                ( prev_down_blocked && up_blocked ) || 
                ( down_blocked && prev_up_blocked ) 
                );
            
            if ( ! vertical_blocked )
            {
                up_row_open_till = 0;

                if ( vertical_down_blocked and ! vertical_up_blocked )
                {
                    create_BTM_seeker( -x_sign, y );
                } 
                
                if ( ! down_blocked && ! btm.dx == 0 )
                {
                    if ( down_row_open_till > 0 )
                    { 
                        x = 1;
                        prev_up_blocked = up_blocked; 
                    }

                    prev_down_blocked = down_blocked; 
                }
                else 
                {
                }

                up_row_blocked = up_blocked; 
            }
            else 
            {                
                up_row_blocked = true; 
                up_row_open_till = -1; 
                
                if ( ! down_blocked )
                { 
                    x_start = 0; 

                    if ( down_row_open_till > 0 )
                    { 
                        x = 1; 
                        prev_up_blocked = up_blocked; 
                    }
                }
                else
                {
                    return;  
                }
            }
        }
        else 
        {    
            up_blocked = frwd_up_blocked( p ); 

            if ( up_blocked )
            {
                up_row_blocked = true; 
                up_row_open_till = -1; 
            } 
            
            down_blocked = frwd_dn_blocked( p ); 

            if ( down_blocked )
            {
                return;  
            }
            else 
            {
                x = 1; 
                prev_up_blocked = up_blocked; 
            }
        }
    }

    p = current_position(); 

    while ( x < down_row_open_till )
    {
        up_blocked = frwd_up_blocked( p );

        if ( ! up_row_blocked )
        {
            up_row_open_till = x; 
            if ( up_blocked )
                up_row_blocked = true;  
        }

        if ( prev_up_blocked && ! up_blocked )
        {
            create_TOP_and_GAP_seekers(); 
            
            if ( check_solution_in_interval( x_start, x ) && check_solution_is_gppc_valid() )
            {
                create_solution_node();
            }

            return;
        }

        x += 1; 
        p = current_position(); 
        
        prev_up_blocked = up_blocked; 
    }

    if ( x == down_row_open_till )
    {
        if ( ! up_row_blocked )
            up_row_open_till = x;  

        if ( check_solution_in_interval( x_start, x ) && check_solution_is_gppc_valid() )
        { 
            create_solution_node(); 
            return;  
        }

        up_blocked = ( x == x_max || frwd_up_blocked( p ) ); 
        if ( up_blocked )
        {
            up_row_blocked = true; 
            return; 
        }         

        if ( down_row_blocked ) 
        {
            if ( up_row_open_till < 0 )
            {
                search_step_finished = true; 
                return; 
            } 
        
            if ( prev_up_blocked || up_blocked ) 
            {
                return; 
            }

            create_BTM_seeker( x_sign );
            btm = BorderRay( x, y, true ); 
        }
        else 
        {    
            if ( x > 0 && ! up_blocked && prev_up_blocked )
            {
                create_TOP_and_GAP_seekers(); 
                return; 
            } 
        }

        x += 1; 
    }
    else 
    { 
    }

    int D_btm = btm.dx * ( y + 1 ) - btm.dy * x;  
    while ( ! up_blocked && ( D_btm > 0 || ( btm.active && D_btm == 0 ) ) ) 
    {
        p = current_position(); 

        up_blocked = ( x >= x_max || frwd_up_blocked( p ) );
        if ( ! up_row_blocked )
        {
            up_row_open_till = x; 
            if ( up_blocked )
                up_row_blocked = true; 
        } 
        
        x += 1; 
        D_btm -= btm.dy; 
    }
}

void GapSolver::check_row_at_y_max() 
{
    if ( y_sign > 0 )
        return; 

    if ( top.dx == 0 )
    {
        x_start = 0; 
    }   
    else 
    { 
        x = x_start;
        int D_top = y * top.dx - x * top.dy; 
        while ( D_top > 0 || ( ! top.active && D_top == 0 ) )
        { 
            x += 1; 
            D_top -= top.dy; 
        }   

        x_start = x;  
    }

    if ( check_solution_in_interval( x_start, down_row_open_till ) )
    {
        create_solution_node(); 
    } 
}

GraphNode* GapSolver::get_new_parent()
{
    if ( new_parent == nullptr )
        new_parent = graph.get_graph_node()->set( p0, current_seeker->parent );

    return new_parent; 
}

double GapSolver::goal_distance( const Vec2& pos_ )
{
    return ( goal - pos_ ).norm();
}

void GapSolver::update_row_blocked_values()
{
    down_row_blocked = up_row_blocked;
    down_row_open_till = up_row_open_till; 

    up_row_blocked = false; 
    up_row_open_till = -1; 

    vertical_down_blocked = vertical_up_blocked; 
    vertical_up_blocked = false; 
}

bool GapSolver::check_LoS_solution()
{
    bool start_OK = false; 

    Vec2 rel_pos = goal - p0; 
    if ( rel_pos.x >= 0 && rel_pos.y >=0 )
    {
        start_OK = true; 
    }
    else 
    { 
        bool bk_up_blocked = ( p0.x == 0 || p0.y == Ny || ! occupancy[ p0.y * Nx + ( p0.x - 1 ) ] );
        bool fw_dn_blocked = ( p0.x == Nx || p0.y == 0 || ! occupancy[ ( p0.y - 1 ) * Nx + p0.x ] );

        start_OK = !( bk_up_blocked && fw_dn_blocked );
    }

    if ( ! start_OK )
        return false;

    return check_solution_is_gppc_valid(); 
}

bool GapSolver::check_solution_is_gppc_valid()
{
    return check_solution_is_gppc_valid( p0 );
}

bool GapSolver::check_solution_is_gppc_valid( const Vec2& from_pos_ )
{
    Vec2 rel_pos = from_pos_ - goal;  

    if ( rel_pos.x >= 0 && rel_pos.y >= 0 )
        return true; 

    bool bk_up_blocked = ( goal.x == 0 || goal.y == Ny || ! occupancy[ goal.y * Nx + ( goal.x - 1 ) ] );
    bool fw_dn_blocked = ( goal.x == Nx || goal.y == 0 || ! occupancy[ ( goal.y - 1 ) * Nx + goal.x ] );

    if ( bk_up_blocked && fw_dn_blocked )
        return false; 

    return true; 
}

bool GapSolver::check_solution_in_interval( const int32_t x_start_, const int32_t x_end_ )
{
    if ( p0.y + y * y_sign != goal.y )
        return false; 

    int _x_start = ( x_sign > 0 ) ? p0.x + x_start_ : p0.x - x_end_;
    int _x_end = ( x_sign > 0 ) ? p0.x + x_end_ : p0.x - x_start_; 

    if ( _x_start <= goal.x && goal.x <= _x_end )
        return true; 

    return false; 
}

void GapSolver::create_solution_node()
{  
    double d = current_distance + goal_distance( p0 );

    if ( scene[ goal ] >= d )
    {
        if ( solution_node != nullptr ) // a solution was alredy found 
        {
            graph.cut_branch( solution_node ); 
            solution_node = nullptr; 
        }

        scene[ goal ] = d; 
        solution_node = graph.get_graph_node()->set( 
            goal, 
            get_new_parent() 
            ); 
        search_step_finished = true;
    }

    solution_found = true;
    solution_found_this_step = true;  
}

void GapSolver::search_step()
{
    reset_search_state(); 

    if ( ! gap_restart )
    {    
        if ( btm.dy == 0 )
        {
            if ( btm.active )
            {
                check_row_at_y_0_horizontal_active_btm(); 
                update_row_blocked_values(); 
            }
            else
            {
                check_row_at_y_0_horizontal_passive_btm();
            }
        }
        else 
        {   
            check_row_at_y_0_tilted_btm();
            update_row_blocked_values(); 
        } 

        y = 1;
    }

    if ( solution_found_this_step )
        return;  
        
    while ( ! search_step_finished && y < y_max )
    {    
        check_row(); 
        update_row_blocked_values(); 

        y += 1; 
    } 
        
    if ( search_step_finished )
        return; 

    if ( y == y_max )
        check_row_at_y_max(); 
} 

int GapSolver::passive_up_border_probe()
{
    bool _up_row_blocked = false; 

    Vec2 p = current_position(); 

    bool bk_up_blocked = ( x_max >= Nx || y_max == 0 || bkwd_up_blocked( p ) );
    bool fw_up_blocked = ( x_max == 0 || y_max == 0 || frwd_up_blocked( p ) ); 

    _up_row_blocked = fw_up_blocked; 

    if ( fw_up_blocked && bk_up_blocked ) 
    {
        return -1;  
    }

    while ( ! _up_row_blocked )
    {        
        x += 1; 
        p = current_position(); 
        
        fw_up_blocked = ( x >= x_max || frwd_up_blocked( p ) );
        if ( fw_up_blocked )
            _up_row_blocked = true; 
    }

    return x; 
}

void GapSolver::initialize_search( const Vec2& start_, const Vec2& goal_ )
{
    scene[ start_ ] = 0.0; 
    goal = goal_; 

    if ( start_ == goal_ ) 
    {
        solution_node = graph.get_graph_node()->set( start_, nullptr ); 
        solution_found = true;
        return; 
    }

    p0 = start_; 
    current_distance = 0.0; 

    GraphNode* root_node = graph.get_graph_node()->set( start_, nullptr );

    if ( LoS_OK( start_, goal_ ) && check_LoS_solution() )
    {
        new_parent = root_node; 
        create_solution_node(); 
        return; 
    }

    set_xy_signs_and_grid_shifts( 1, 1 ); 
    x = 0;
    y = 0;         

    bool fw_up_blocked( start_.x > Nx || start_.y > Ny || frwd_up_blocked( start_ ) );
    bool fw_dn_blocked( start_.x > Nx || start_.y == 0 || frwd_dn_blocked( start_ ) );
    bool bk_up_blocked( start_.x == 0 || start_.y > Ny || bkwd_up_blocked( start_ ) );
    bool bk_dn_blocked( start_.x == 0 || start_.y == 0 || bkwd_dn_blocked( start_ ) );

    double goal_dist = goal_distance( start_ ); 

    if ( ! ( fw_up_blocked && fw_dn_blocked ) ) 
    {
        if ( start_.y > 0 ) 
        { 
            graph.insert_leaf(
                heuristics( 0, goal_dist ),
                graph.get_leaf_node()->set(
                    start_, 0.0, root_node, 
                    1, 1, 
                    { 0, 1, false },
                    { 1, 0, true }
                ) 
            );  
        }        
        else 
        {
            graph.insert_leaf(
                heuristics( 0, goal_dist ),
                graph.get_leaf_node()->set(
                    start_, 0.0, root_node, 
                    1, 1, 
                    { 0, 1, false }, 
                    { 1, 0, false },
                    true, passive_up_border_probe()
                )
            ); 
        }        
    }       

    if ( ! ( fw_up_blocked && bk_up_blocked ) )
    {    
        p0 = start_; 
        x = 0; 
        y = 0; 
        set_xy_signs_and_grid_shifts( -1, 1 ); 
        
        graph.insert_leaf( 
            heuristics( 0, goal_dist ), 
            graph.get_leaf_node()->set(
                start_, 0.0, root_node, 
                -1, 1,
                { 0, 1, true },
                { 1, 0, false },
                true, passive_up_border_probe() 
            )
        );         
    }

    bool gppc_illegal = ( !fw_up_blocked && !bk_dn_blocked && fw_dn_blocked && bk_up_blocked ); 

    if ( start_.y > 0 && !( bk_up_blocked && bk_dn_blocked ) && !gppc_illegal )
    {
        graph.insert_leaf(
            heuristics( 0, goal_dist ), 
            graph.get_leaf_node()->set( 
                start_, 0.0, root_node, 
                -1, -1, 
                { 0, 1, false }, 
                { 1, 0, true }
            )
        );        
    }

    if ( !( fw_dn_blocked && bk_dn_blocked ) && !gppc_illegal )
    {   
        p0 = start_; 
        x = 0; 
        y = 0; 
        set_xy_signs_and_grid_shifts( 1, -1 ); 

        graph.insert_leaf(
            heuristics( 0.0, goal_dist ), 
            graph.get_leaf_node()->set(
                start_, 0.0, root_node, 
                1, -1, 
                { 0, 1, true },
                { 1, 0, false },
                true, passive_up_border_probe()
            )
        );        
    }
}

void GapSolver::regular_step()
{
    assert( current_seeker == nullptr ); 

    current_seeker = graph.pop_top_leaf();  
    search_step();
    graph.release_leaf( current_seeker ); 
    current_seeker = nullptr; 
}

void GapSolver::gap_step()
{
    assert( current_seeker == nullptr ); 

    current_seeker = gap_seekers.back();
    gap_seekers.pop_back();
    search_step(); 
    graph.release_leaf( current_seeker ); 
    current_seeker = nullptr; 
}

bool GapSolver::find_path( const Vec2& start_, const Vec2& goal_ )
{
    initialize_search( start_, goal_ );
    steps_counter = 1; 

    while (
        graph.number_of_leafs() > 0 //open_set.get_size() > 0 
        && ( 
            ! solution_found 
#ifdef EXTRA_OPTIMALITY_CHECK
            || estimate_dist( graph.top_leaf_distance(), graph.top_leaf_priority() ) < solution_distance()
#endif // EXTRA_OPTIMALITY_CHECK
        )
        && steps_counter < steps_max 
        )
    {
        // std::cout << "step:" << steps_counter << std::endl ;

        // if ( steps_counter == 100 )
        // {
        //     int debug_stop_line = 1; 
        // }
        
        regular_step(); 

        while ( gap_seekers.size() > 0 )
            gap_step(); 

        // std::cout << "step:" << steps_counter << "  N of seekers:" << graph.num_leafs() << std::endl;

        steps_counter++; 

    }

    return solution_found; 
}

void GapSolver::reset()
{
    scene.reset();  

    graph.clear(); 

    gap_seekers.clear(); 

    steps_counter = 0;  
    steps_max = Nx * Ny;  
    current_distance = 0.0;
    new_parent = nullptr;  
    current_seeker = nullptr; 

    solution_node = nullptr; 
    solution_found = false;  
    solution_found_this_step = false;  
    search_step_finished = false ;  

    x = 0; 
    y = 0;
    p0 = Vec2( 0, 0 );      

    up_row_blocked = false; 
    up_row_open_till = -1; 
    down_row_blocked = false; 
    down_row_open_till = -1; 

    vertical_down_blocked = false; 
    vertical_up_blocked = false;

    forward = 0; 
    backward = -1; 
    up = 0;
    down = -1; 

    up_blocked = false;
    prev_up_blocked = false; 
    down_blocked = false; 
    prev_down_blocked = false;

    search_step_finished = false; 
    gap_restart = false; 

    x_sign = 1; 
    y_sign = 1; 

    x_max = 0; 
    y_max = 0; 
}

void GapSolver::root_path( LeafNode* seeker_, std::vector< Vec2 >& path_ )
{
    if ( seeker_->x != 0 || seeker_->y != 0 )
        path_.push_back( p0 + Vec2( seeker_->x * seeker_->x_sign, seeker_->y * seeker_->y_sign ) ); 

    path_.push_back( seeker_->position );

    GraphNode* prnt = seeker_->parent; 
    while ( prnt != nullptr )
    {
        path_.push_back( prnt->position );
        prnt = prnt->next_node; 
    }
}

void GapSolver::root_path( GraphNode* graph_node_, std::vector< Vec2 >& path_ )
{
    path_.push_back( graph_node_->position );

    GraphNode* prnt = graph_node_->next_node; 
    while ( prnt != nullptr )
    {
        path_.push_back( prnt->position );
        prnt = prnt->next_node; 
    }
}

void GapSolver::solution_path( std::vector< Vec2 >& path_ )
{
    if ( solution_found )
        root_path( solution_node, path_ ); 
}

double GapSolver::solution_distance()
{
    if ( ! solution_found )
        return INFINITY; 

    return scene[ goal ]; 
} 

bool GapSolver::path_OK( std::vector< Vec2 >& path_ )
{
    uint n_points = path_.size();

    if ( n_points == 1 ) // goal == start 
        return true;   

    if ( n_points > 1 )
    {
        double dists_sum = 0.0;

        for ( uint i = 0; i < n_points - 1; ++i )
        {
            if ( ! LoS_OK( path_[ i + 1 ], path_[ i ] ) )
                return false;

            dists_sum += ( path_[ i + 1 ] - path_[ i ] ).norm(); 
        }   

        if ( abs( scene[ path_[0] ] - dists_sum ) > 1e-10 )
            return false; 
    }
    
    if ( path_.size() > 2 )
    {    
        for ( uint i = 1; i < n_points - 1; ++i )
        {
            if ( ! cornering_OK( path_[ i - 1 ], path_[ i ], path_[ i + 1 ] ) )
                return false; 
        }
    }

    return true; 
} 

bool GapSolver::LoS_OK( const Vec2& p0_, const Vec2& p1_ )
{
    Vec2 p0( p0_ );  
    Vec2 p1( p1_ );  

    int32_t dx = p1.x - p0.x; 
    int32_t dy = p1.y - p0.y; 

    if ( dx == 0 && dy == 0 )
        return true; 

    if ( dx == 0 )
        return LoS_OK_vertical( p0, p1 );   

    if ( dx < 0 )
    {
        std::swap( p0, p1 );
        dx *= -1; 
        dy *= -1;  
    }

    if ( dy == 0 )
        return LoS_OK_horizontal( p0, p1 );   

    int32_t _y_sign = ( dy < 0 ) ? -1 : 1;   
    int32_t y_shift = ( dy < 0 ) ? -1 : 0;
    dy *= _y_sign;  

    int32_t D = 0; 
    x = 0;
    y = 0; 
    Vec2 p = p0;

    while ( y < dy )
    { 
        while ( D < dx - dy && x < dx )
        { 
            if ( D > -dy )
            {
                p = p0 + Vec2( x, y * _y_sign + y_shift );
                if ( ! occupancy[ p.y * Nx + p.x ] )
                    return false; 
            }    

            D += dy; 
            x += 1; 
        }   

        p = p0 + Vec2( x, y * _y_sign + y_shift ); 
        if ( ! occupancy[ p.y * Nx + p.x ] )
            return false; 
        
        D -= dx; 
        y += 1; 
        
        if ( D == -dy && y < dy )
        {
            if ( ! occupancy[ p.y * Nx + ( p.x + 1 ) ] && ! occupancy[ ( p.y + _y_sign ) * Nx + p.x ] )
                return false; 
        }             
    }

    return true; 
}

bool GapSolver::LoS_OK_horizontal( const Vec2& p0_, const Vec2& p1_ )
{
    int32_t dx = p1_.x - p0_.x;  

    if ( p0_.x < 0 || p1_.x >= Nx )
        return false; 

    Vec2 p = p0_;  

    Vec2 f( 1, 0 );  

    bool fu_blocked = ( p.y == Ny || ! occupancy[ p.y * Nx + p.x ] );
    bool fd_blocked = ( p.y == 0  || ! occupancy[ ( p.y - 1 ) * Nx + p.x ] );

    if ( fu_blocked && fd_blocked )
        return false; 

    bool bu_blocked = fu_blocked; 
    bool bd_blocked = fd_blocked; 

    int32_t x = 1; 
    p += f; 
    while ( x < dx )
    {
        fu_blocked = ( p.x == Nx || p.y == Ny || ! occupancy[ p.y * Nx + p.x ] );
        fd_blocked = ( p.x == Nx || p.y == 0  || ! occupancy[ ( p.y - 1 ) * Nx + p.x ] );
        
        if ( ( fu_blocked && fd_blocked ) || ( fu_blocked && bd_blocked ) || ( bu_blocked && fd_blocked ) )
            return false; 

        bu_blocked = fu_blocked; 
        bd_blocked = fd_blocked; 
        
        x += 1; 
        p += f; 
    }        

    return true; 
}

bool GapSolver::LoS_OK_vertical( const Vec2& p0_, const Vec2& p1_ )
{
    int32_t dy = p1_.y - p0_.y; 
    Vec2 _p0( p0_ ), _p1( p1_ ); 

    if ( dy < 0 ) 
    {
        std::swap( _p0, _p1 ); 
        dy *= -1; 
    }

    if ( _p0.y < 0 || _p1.y >= Ny )
        return false; 

    Vec2 u( 0, 1 );

    Vec2 p = _p0; 

    bool bu_blocked = ( p.x == 0  || ! occupancy[ p.y * Nx + ( p.x - 1 ) ] ); 
    bool fu_blocked = ( p.x == Nx || ! occupancy[ p.y * Nx + p.x ] ); 
    if ( bu_blocked && fu_blocked )
        return false; 

    bool bd_blocked = bu_blocked; 
    bool fd_blocked = fu_blocked; 

    int32_t y = 1; 
    p += u; 
    while ( y < dy )
    { 
        bu_blocked = ( p.x == 0  || ! occupancy[ p.y * Nx + ( p.x - 1 ) ] ); 
        fu_blocked = ( p.x == Nx || ! occupancy[ p.y * Nx + p.x ] );
        if ( ( bu_blocked && fu_blocked ) || ( bu_blocked && fd_blocked ) || ( bd_blocked && fu_blocked ) )
            return false; 

        bd_blocked = bu_blocked; 
        fd_blocked = fu_blocked; 

        y += 1;
        p += u;  
    }

    return true; 
}

bool GapSolver::cornering_OK( const Vec2& p0_, const Vec2& p1_, const Vec2& p2_ )
{
    int n_occupied_cells = ( ! occupancy[ p1_.y * Nx + p1_.x ] ) ? 1 : 0;  
    n_occupied_cells += ( ! occupancy[ ( p1_.y - 1 ) * Nx + p1_.x ] ) ? 1 : 0;
    n_occupied_cells += ( ! occupancy[ ( p1_.y - 1 ) * Nx + ( p1_.x - 1 ) ] ) ? 1 : 0;
    n_occupied_cells += ( ! occupancy[ p1_.y * Nx + ( p1_.x - 1 ) ] ) ? 1 : 0;

    if ( n_occupied_cells != 1 )
        return false; 

    // should check the occupancy according to the cross product (p2-p1)x(p1-p0) 

    return true; 
} 

bool GapSolver::start_goal_gppc_valid( const Vec2& pos_ )
{
    if ( pos_.x >= Nx || pos_.x < 0 || pos_.y >= Ny || pos_.y < 0 )
        return false;  

    if ( ! occupancy[ pos_.y * Nx + pos_.y ] )
        return false;         

    return true; 
}

#endif // POOLS_GAP_SEARCH_HPP