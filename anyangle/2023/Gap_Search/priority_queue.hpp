#ifndef PRIORITY_QUEUE_HPP
#define PRIORITY_QUEUE_HPP

#include <vector>
#include <utility>

/**
 * A generic binary priority queue to keep open set of the solution graph's LeafNodes 
*/

#ifndef CHUNK_SIZE
    #define CHUNK_SIZE 64 // a power of 2 constant the queue is growing by 
#endif // CHUNK_SIZE

// see: https://www.geeksforgeeks.org/priority-queue-using-binary-heap/

template < class T >
class BinaryPiorityQueue
{
    private:
        std::vector< double > priority_values;
        std::vector< T > data_items;
        std::size_t size;
        std::size_t capacity;

    public:
        std::size_t max_size; 

        BinaryPiorityQueue( std::size_t );
        BinaryPiorityQueue();

        std::size_t get_size() const { return size; };
        double min_priority() const { return priority_values[ 0 ]; }; 
        T top() const { return data_items[ 0 ]; }; 

        void insert( const double&, const T& );
        // void pop( T& );
        void pop();

        void clear();

    private:
        void shiftUp( const std::size_t );
        void shiftDown( const std::size_t );
}; 

template < class T >
BinaryPiorityQueue< T >::BinaryPiorityQueue( std::size_t n_ ) :
    size( 0 ),
    max_size( 0 )
{
    int _ini_n_chunks = (size_t) ceil( (double) n_ / CHUNK_SIZE ); 

    priority_values.resize( _ini_n_chunks * CHUNK_SIZE, INFINITY ); 
    data_items.resize( _ini_n_chunks * CHUNK_SIZE, T() ); 

    capacity = ( _ini_n_chunks * CHUNK_SIZE - 1 ); 
}

template < class T >
BinaryPiorityQueue< T >::BinaryPiorityQueue() :
    BinaryPiorityQueue( CHUNK_SIZE )  
{}

template < class T >
void BinaryPiorityQueue< T >::insert( const double& new_priority_value_, const T& new_data_item_ )
{
    if ( size == capacity )
    {
        capacity += CHUNK_SIZE;  
        priority_values.resize( capacity, INFINITY );
        data_items.resize( capacity, T() );
    }

    priority_values[ size ] = new_priority_value_;
    data_items[ size ] = new_data_item_;

    shiftUp( size ); 

    size++; 

    if ( size > max_size )
        max_size = size; 
}

template < class T >
void BinaryPiorityQueue< T >::pop()
{
    size--;

    priority_values[ 0 ] = priority_values[ size ];
    data_items[ 0 ] = data_items[ size ];

    priority_values[ size ] = INFINITY;
    data_items[ size ] = T();

    shiftDown( 0 );
}

template < class T >
void BinaryPiorityQueue< T >::clear()
{
    std::fill( priority_values.begin(), priority_values.begin() + size, INFINITY );
    std::fill( data_items.begin(), data_items.end(), T() );
    size = 0; 
}

template < class T >
void BinaryPiorityQueue< T >::shiftUp( const std::size_t start_shifting_from_ )
{
    size_t child = start_shifting_from_; 
    size_t parent;

    while ( child > 0 )
    { 
        parent = ( child - 1 ) >> 1;

        if ( priority_values[ parent ] > priority_values[ child ] )
        {
            std::swap( priority_values[ parent ], priority_values[ child ] );
            std::swap( data_items[ parent ], data_items[ child ] );
        }
        else
        {
            return; 
        }

        child = parent; 
    }
}

template < class T >
void BinaryPiorityQueue< T >::shiftDown( const std::size_t start_shifting_from_ )
{
    size_t parent = start_shifting_from_;
    size_t child = ( parent << 1 ) + 1;

    while ( child < size )
    { 
        if ( priority_values[ child ] > priority_values[ child + 1 ] )
        {
            child++;
        }

        if ( priority_values[ child ] < priority_values[ parent ] )
        {
            std::swap( priority_values[ parent ], priority_values[ child ] );
            std::swap( data_items[ parent ], data_items[ child ] );
        }
        else
        {    
            return;
        }

        parent = child;
        child = ( parent << 1 ) + 1;
    }
}

#endif // PRIORITY_QUEUE_HPP
