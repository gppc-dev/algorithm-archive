/*
Copyright (c) 2023 Grid-based Path Planning Competition and Contributors <https://gppc.search-conference.org/>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <algorithm>
#include <map>
#include "ThetaStar.h"
#include "Entry.h"
#include "grid2poly.h"
#include "poly2mesh.h"
#include "mesh2merged.h"
#include "grid2rect.h"

#include <stdio.h>
#include <iostream>
#include <expansion.h>
#include <searchinstance.h>
#include <eps.h>
#include "point.h"
#include "cpd.h"
#include <iomanip>
#include "consts.h"
#include <omp.h>
#include "visibleSearchInstance.h"
#include "dijkstra.h"
#include "polymap2.h"

namespace pl = polyanya;
pl::MeshPtr mp;
pl::CPDPtr cpd;


void output_graph(const std::vector<vector<int>>& visibility_graph, const std::vector<vector<double>>& graph_weight,
                  const std::vector<pl::Point>& coordinate,const std::vector<int>& vertice_mapper,string output_file)
{
    std::cout<<"Saving graph to"<<output_file<<std::endl;
    std::ofstream outputFile(output_file);
    outputFile <<coordinate.size()<<"\n";
    int number_of_edges = 0 ;
    for(const vector<int>& vertices : visibility_graph){
        for(const int & edge : vertices){
            number_of_edges ++;
        }
    }
    outputFile <<number_of_edges<<"\n";
    for(int v_id = 0; v_id < visibility_graph.size(); v_id ++){
        for(int e_id= 0; e_id < visibility_graph[v_id].size(); e_id ++){
            outputFile<<std::fixed<<setprecision(8) <<v_id<<" "<<visibility_graph[v_id][e_id]<<" "<<graph_weight[v_id][e_id]<<"\n";
        }
    }
    outputFile.close();

    size_t lastindex = output_file.find_last_of(".");
    string rawname = output_file.substr(0, lastindex);

    std::ofstream coordinateFile(rawname+".cor");
    int id = 0;
    for(pl::Point cor:coordinate){
        coordinateFile<<std::fixed<<setprecision(8)<<id<<" "<<cor.x<<" "<<cor.y<<"\n";
        id++;
    }

    save_vector(rawname+".vMapper",vertice_mapper);
    std::cout<<"done"<<std::endl;
}



void build_visiblity_graph(string mesh_path, string output_path, const vector<bool> &map, int input_width, int input_height){
    std::cout<<"Building visibility graph ..."<<std::endl;
    ifstream meshfile(mesh_path);
    mp = new pl::Mesh(meshfile);
    // mark obstacle edge;
    mp->pre_compute_obstacle_edge_on_vertex();
    // mark valid turning point;

    mp->mark_turning_point(map, input_width, input_height);
    vector<pl::Point> turning_point;
    vector<int> turning_vertices;
    vector<pl::PointLocation> turning_vertices_location;

    int id = 0;
    int poly = 0;
    for( pl::Vertex& v : mp->mesh_vertices){
        if(v.is_turning_vertex && !v.is_ambig) {
            for (int polygon: v.polygons) {
                if (polygon != -1) {
//                    p.polygons.insert(polygon);
                    poly = polygon;
                }
            }
            // there is some issue here, old implementation assume that these vertex always inside one polygon only. it doesnt
            // use the correct type actually, I fix it here manually assign a random adjacent polygon
            pl::PointLocation location = {pl::PointLocation::ON_CORNER_VERTEX_UNAMBIG, poly, -1, id, -1};
            turning_vertices_location.push_back(location);
            turning_vertices.push_back(id);
            turning_point.push_back(mp->mesh_vertices[id].p);
        }
        id ++;
    }

    const vector<pl::Vertex>& mesh_vertices = mp->mesh_vertices;

    std::vector<pl::visibleSearchInstance*> visibility_search;
    for (int i = 0; i < omp_get_max_threads(); i ++){
        visibility_search.push_back(new pl::visibleSearchInstance(mp));
    }
    std::vector<vector<int>> visibility_graph;
    int number_of_vertices = turning_vertices.size();
    {
        printf("Using %d threads\n", omp_get_max_threads());
        std::vector<std::vector<vector<int>>>thread_graph(omp_get_max_threads());

        int progress = 0;

#pragma omp parallel
        {
            const int thread_count = omp_get_num_threads();
            const int thread_id = omp_get_thread_num();
//            const int begin_int  = 0;
            const int node_count =number_of_vertices;

            int node_begin = (node_count*thread_id) / thread_count ;
            int node_end = (node_count*(thread_id+1)) / thread_count ;
            pl::visibleSearchInstance& thread_dij = *visibility_search[thread_id];

            for(int source_node=node_begin; source_node < node_end; ++source_node){

                int source = turning_vertices[source_node];
                pl::Point source_point = mesh_vertices[source].p;
                auto comp = [&](int v1,int v2)-> bool {
                    double d1 =source_point.distance(mesh_vertices[v1].p);
                    double d2 = source_point.distance(mesh_vertices[v2].p);
                    if(d1<d2){
                        return true;
                    }else if(d1 == d2){
                        return v1 < v2;
                    }else{
                        return false;
                    }
                };
                std::vector<int> visible_vertices;
                thread_dij.search_visible_vertices(
                        turning_vertices[source_node],turning_vertices_location[source_node],visible_vertices);
                if(!visible_vertices.empty()) {
                    // it's possible to be empty;
                    sort(visible_vertices.begin(), visible_vertices.end(), comp);
                    for (int cur = 0; cur < visible_vertices.size() - 1; cur++) {
                        vector<int> remove_list;
                        const pl::Point &cur_point = mesh_vertices[visible_vertices[cur]].p;
                        for (int next = cur + 1; next < visible_vertices.size(); next++) {
                            //remove the collinear edges;
                            const pl::Point &next_point = mesh_vertices[visible_vertices[next]].p;
                            if (get_orientation(source_point, cur_point, next_point) == pl::Orientation::COLLINEAR &&
                                source_point.distance(next_point) > cur_point.distance(next_point)) {
                                remove_list.push_back(next);
                            }
                        }
                        int index = 0;
                        for (int rm: remove_list) {
                            visible_vertices.erase(visible_vertices.begin() + rm - index);
                            index++;
                        }
                    }
                }
                thread_graph[thread_id].push_back(visible_vertices);


#pragma omp critical
                {
                    ++progress;
                    if(progress % 100 == 0) {
                        double ratio = (double)progress / number_of_vertices * 100.0;
                        std::cout << "Progress: [" << progress << "/" << number_of_vertices  << "] "
                                  << std::setprecision(3) << ratio << "% \r";
                        std::cout.flush();
                    }
                }
            }
        }

        for(auto&x:thread_graph){
            for(auto&y:x){
                visibility_graph.push_back(y);
            }
        }

    }
    for (auto i: visibility_search) delete i;

    vector<int> vertice_mapper(mesh_vertices.size());
    std::fill(vertice_mapper.begin(),vertice_mapper.end(),-1);
    for(int i = 0; i < turning_vertices.size(); i ++){
        vertice_mapper[turning_vertices[i]] = i;
    }
    std::vector<vector<double>> graph_weight(visibility_graph.size());
    for(int i = 0; i < visibility_graph.size(); i++){
        graph_weight[i].resize(visibility_graph[i].size());
        for(int j = 0; j < visibility_graph[i].size(); j++){
            double distance = mesh_vertices[turning_vertices[i]].p.distance(mesh_vertices[visibility_graph[i][j]].p);
            if(distance == 0){
                std::cout<<"distance should not be 0 "<< std::endl;
            }
            graph_weight[i][j]= distance;
            visibility_graph[i][j] = vertice_mapper[visibility_graph[i][j]];
        }
    }
    std::cout<<"done"<<std::endl;
    output_graph(visibility_graph,graph_weight,turning_point, vertice_mapper,output_path);
    std::cout<<std::endl;
}


void construct_cpd(const string& input_file, const string& output_file) {


    pl::Graph g = pl::Graph();
    g.load_graph(input_file);
    // only need free flow cost;
    std::cout<<"Loading visibility graph ...."<<std::endl;
    vector<int> dfs_ordering =g.generate_DFS_ordering();
    g.resort_graph(dfs_ordering);

    cout << "Building CPD ... " << flush;
    std::vector<pl::Dijkstra*> dijkstra;
    for (int i = 0; i < omp_get_max_threads(); i ++){
        dijkstra.push_back(new pl::Dijkstra(&g));
    }


    unsigned number_of_nodes = g.number_of_vertices;
    pl::cpd* cpd = new pl::cpd();
    {

        printf("Using %d threads\n", omp_get_max_threads());
        std::vector<pl::cpd>thread_cpd(omp_get_max_threads());

        int progress = 0;

#pragma omp parallel
        {
            const int thread_count = omp_get_num_threads();
            const int thread_id = omp_get_thread_num();
//            const int begin_int  = 0;
            const int node_count = number_of_nodes;

            int node_begin = (node_count*thread_id) / thread_count ;
            int node_end = (node_count*(thread_id+1)) / thread_count ;
            pl::Dijkstra& thread_dij = *dijkstra[thread_id];

            for(int source_node=node_begin; source_node < node_end; ++source_node){
//                thread_dij.full_search(source_node,weight);
//                const std::vector<unsigned>& result =thread_dij.get_first_move_table(source_node);
//                thread_cpd[thread_id].append_row(source_node,result);

                thread_cpd[thread_id].append_row(source_node,thread_dij.run_single_source_dijkstra(source_node));
#pragma omp critical
                {
                    ++progress;
                    if(progress % 100 == 0) {
                        double ratio = (double)progress / number_of_nodes * 100.0;
                        std::cout << "Progress: [" << progress << "/" << number_of_nodes << "] "
                                  << std::setprecision(3) << ratio << "% \r";
                        std::cout.flush();
                    }
                }
            }
        }

        for(auto&x:thread_cpd)
            cpd->append_rows(x);
    }
    for (auto i: dijkstra) delete i;


    printf("Saving data to cpd.txt \n");
    printf("begin size: %d, entry size: %d\n", cpd->entry_count(), cpd->get_entry_size());
    std::string fname = output_file+ ".cpd";
    FILE*f = fopen(fname.c_str(), "wb");
    cpd->save(f);
    fclose(f);
    std::cout.flush();
    std::cout<<"done"<<std::endl;

    string mapper_file = output_file +".mapper";
    cout<<"Saving mapper:"<< mapper_file<< endl;

    vector<int> mapper = pl::invert_permutation(dfs_ordering);
    save_vector(mapper_file,mapper);
    std::cout<<std::endl;
//    test_cpd(g,cpd);



}
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
void PreprocessMap(const std::vector<bool> &bits, int width, int height, const std::string &filename) {
//        grid2poly::convertGrid2Poly(bits, width, height, filename+".poly");
//        poly2mesh::convertPoly2Mesh(filename+".poly",filename+".mesh");
//        mesh2merged::convertMesh2MergedMesh(filename+".mesh",filename+".merged-mesh");
    grid2poly::convertGrid2Poly(bits, width, height, filename+".poly");
    cdtutils::convertPoly2Mesh(filename+".poly",filename+".mesh",width);
    mesh2merged::convertMesh2MergedMesh(filename+".mesh",filename+".merged-mesh");


////        convertgrid2rect(bits, width, height,filename+".merged-mesh");
//    build_visiblity_graph(filename+".merged-mesh", filename+".vis",bits, width, height);
//    construct_cpd(filename+".vis",filename);
//        grid2poly::convertGrid2Poly(bits, width, height,filename+".merged-mesh");
//        mesh2merged::convertMesh2MergedMesh(filename+".mesh",filename+".merged-mesh");
//        convertMesh2MergedMesh(filename+".mesh",filename+".merged-mesh");
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
 * @param[in] bits Array of 2D table.  (0,0) is located at top-left corner.  bits.size() = height * width
 *                 Packed as 1D array, row-by-ray, i.e. first width bool's give row y=0, next width y=1
 *                 bits[i] returns `true` if (x,y) is traversable, `false` otherwise
 * @param[in] width Give the map's width
 * @param[in] height Give the map's height
 * @param[in] filename The filename you write the preprocessing data to.  Open in write mode.
 * @returns Pointer to data-structure used for search.  Memory should be stored on heap, not stack.
 */
void *PrepareForSearch(const vector<bool> &bits, int width, int height, const string &filename) {
    string mesh_path = filename+".merged-mesh";
    ifstream meshfile(mesh_path);
    mp = new pl::Mesh(meshfile);
    pl::SearchInstance* si  = new pl::SearchInstance(mp);
    return si;
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
bool GetPath(void *data, xyLoc s, xyLoc g, std::vector<xyLoc> &path) {
    pl::SearchInstance* si =  (pl::SearchInstance*)(data);
    pl::Point start = pl::Point{(double)s.x,(double)s.y};
    pl::Point goal = pl::Point{(double)g.x,(double)g.y};

    if( start == goal){
        return true;
    }
    si->set_start_goal(start,goal);
    si->search();
    vector<pl::Point> out;
    si->get_path_points(out);
    for(auto p : out){
        path.push_back(xyLoc{(double)(p.x),(double)p.y});
    }

    return true;
}

/**
 * The algorithm name.  Please update std::string and ensure name is immutable.
 *
 * @returns the name of the algorithm
 */
std::string GetName() { return "Poly-CDT-"; }