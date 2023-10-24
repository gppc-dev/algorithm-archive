#ifndef SOLUTION_GRAPH_HPP
#define SOLUTION_GRAPH_HPP

#include <vector>

#include "graph_nodes.hpp"
#include "priority_queue.hpp"
#include "nodes_pool.hpp"

class SolutionGraph
{
    private:
        BinaryPiorityQueue< LeafNode* > open_set; 
        NodesPool< LeafNode > leafs_pool; 
        NodesPool< GraphNode > tree_nodes_pool; 

    public: 
        SolutionGraph( size_t, size_t ); // open_set init size, graph nodes init size   

        GraphNode* get_graph_node(); // position, parent    
        LeafNode* get_leaf_node(); 
        void insert_leaf( double, LeafNode* ); // all parameters to initialize a LeafNode
        LeafNode* pop_top_leaf(); 
        void release_leaf( LeafNode* );

        size_t number_of_leafs();

        double top_leaf_distance() { return open_set.top()->distance; }; 
        double top_leaf_priority() { return open_set.min_priority(); };  
        uint32_t max_num_leafs() { return open_set.max_size; };
        uint32_t num_leafs() { return open_set.get_size(); };

        uint32_t max_num_tree_nodes() { return tree_nodes_pool.max_size; };
        uint32_t num_tree_nodes() { return tree_nodes_pool.current_size; };

        void cut_branch( GraphNode* );    
        void clear(); 
};

SolutionGraph::SolutionGraph( 
    size_t init_n_leafs_, 
    size_t init_n_tree_nodes_
    ) :
    open_set( init_n_leafs_ ), 
    leafs_pool( init_n_leafs_ ),
    tree_nodes_pool( init_n_tree_nodes_ )
{}   

GraphNode* SolutionGraph::get_graph_node()
{
    return tree_nodes_pool.get_next(); 
}    

LeafNode* SolutionGraph::get_leaf_node()
{
    return leafs_pool.get_next(); 
} 

void SolutionGraph::SolutionGraph::insert_leaf( double priority_, LeafNode* leaf_ )
{
    open_set.insert( priority_, leaf_ );
}

LeafNode* SolutionGraph::pop_top_leaf()
{
    LeafNode* top_leaf = open_set.top(); 
    open_set.pop();
    return top_leaf; 
}

void SolutionGraph::release_leaf( LeafNode* leaf_ )
{
    GraphNode* branch_top = leaf_->parent;
    leaf_->disconnect_from_parent(); 
    leafs_pool.reclaim( leaf_ ); 
    cut_branch( branch_top ); 
}

void SolutionGraph::cut_branch( GraphNode* branch_top_ )
{
    GraphNode* tmp_ptr; 
    while ( branch_top_ != nullptr && branch_top_->branch_count == 0 )
    {
        tmp_ptr = branch_top_->next_node;
        branch_top_->disconnect_from_parent();  
        tree_nodes_pool.reclaim( branch_top_ );
        branch_top_ = tmp_ptr; 
    }
}

size_t SolutionGraph::number_of_leafs()
{
    return open_set.get_size(); 
}

void SolutionGraph::clear()
{
    open_set.clear(); 
    leafs_pool.clear(); 
    tree_nodes_pool.clear(); 
} 

#endif // SOLUTION_GRAPH_HPP
