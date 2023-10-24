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

namespace pl = polyanya;
pl::MeshPtr mp;

void PreprocessMap(const std::vector<bool> &bits, int width, int height, const std::string &filename) {
//    grid2poly::convertGrid2Poly(bits, width, height, filename+".poly");
//    poly2mesh::convertPoly2Mesh(filename+".poly",filename+".mesh");
//    mesh2merged::convertMesh2MergedMesh(filename+".mesh",filename+".merged-mesh");
//    mesh2merged::convertMesh2MergedMesh(filename+".mesh",filename+".merged-mesh");
    convertgrid2rect(bits, width, height,filename+".merged-mesh");
}

void *PrepareForSearch(const std::vector<bool> &bits, int width, int height, const std::string &filename) {

    string mesh_path = filename+".merged-mesh";
    ifstream meshfile(mesh_path);
    mp = new pl::Mesh(meshfile);
    pl::SearchInstance* si  = new pl::SearchInstance(mp);
    return si;
}

bool GetPath(void *data, xyLoc s, xyLoc g, std::vector<xyLoc> &path) {
    pl::SearchInstance* si =  (pl::SearchInstance*)(data);
    pl::Point start = pl::Point{(double)s.x,(double)s.y};
    pl::Point goal = pl::Point{(double)g.x,(double)g.y};

    si->set_start_goal(start,goal);
    si->search();
    vector<pl::Point> out;
    si->get_path_points(out);
    for(auto p : out){
        path.push_back(xyLoc{(double)(p.x),(double)p.y});
    }
    return true;
}

std::string GetName() { return "Polyanya-"; }
