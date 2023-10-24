#ifndef NODES_POOL_HPP 
#define NODES_POOL_HPP 

#include <vector>
#include <memory>
#include <cassert>

#include "graph_nodes.hpp" 

#ifndef DEFAULT_CHUNK_SIZE
    #define DEFAULT_CHUNK_SIZE 64 
#endif

template < class T >
class NodesPool
{
    private : 
        T* next_free_node;   
        std::vector< std::unique_ptr< T[] > > chunks; 
        const size_t chunk_size;

    public:
        uint32_t current_size; 
        uint32_t max_size; 

        NodesPool( const size_t, const size_t );
        NodesPool( const size_t chunk_size_ ) : NodesPool( chunk_size_, 1 ) {};
        NodesPool() : NodesPool( DEFAULT_CHUNK_SIZE, 1 ) {};

        T* get_next(); 
        void reclaim( T* );  

        void clear();

    private: 
        void clear_chunk( T*, T* );
};

template < class T >
NodesPool< T >::NodesPool( const size_t chunk_size_, const size_t ini_num_of_chunks_ ) :
    next_free_node( nullptr ),  
    chunk_size( chunk_size_ ), 
    current_size( 0 ),  
    max_size( 0 )  
{
    for ( size_t i = 0; i < ini_num_of_chunks_; ++i )
        chunks.emplace_back( std::make_unique< T[] >( chunk_size ) ); 

    clear();
}

template < class T >
void NodesPool< T >::clear()
{
    T* tmp_ptr = nullptr; 
    for ( auto it = chunks.rbegin(); it != chunks.rend(); ++it )
    {
        clear_chunk( it->get(), tmp_ptr ); 
        tmp_ptr = it->get(); 
    }

    next_free_node = chunks[0].get();
    current_size = 0; 
}

template < class T >
void NodesPool< T >::clear_chunk( T* chunk_head_, T* chunk_tail_ )
{
    for ( size_t i = 0; i < chunk_size - 1; ++i )
    {
        chunk_head_[ i ].free = true; 
        chunk_head_[ i ].next_node = chunk_head_ + i + 1; 
    }

    chunk_head_[ chunk_size - 1 ].free = true; 
    chunk_head_[ chunk_size - 1 ].next_node = chunk_tail_; 
}

template < class T >
T* NodesPool< T >::get_next()
{
    if ( ! next_free_node ) 
    {
        assert( current_size == chunk_size * chunks.size() ); 
        chunks.emplace_back( std::make_unique< T[] >( chunk_size ) );
        next_free_node = chunks.back().get(); 
        clear_chunk( next_free_node, nullptr );
    }

    T* return_ptr = next_free_node; 
    next_free_node = next_free_node->next_node;

    return_ptr->release();

    current_size++; 
    if ( current_size > max_size )
        max_size = current_size; 

    return return_ptr; 
}

template < class T >
void NodesPool< T >::reclaim( T* node_to_store_ )
{
    node_to_store_->store(); 
    node_to_store_->next_node = next_free_node;
    next_free_node = node_to_store_; 

    current_size--; 
}  

#endif // NODES_POOL_HPP 