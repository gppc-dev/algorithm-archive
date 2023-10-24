//
// Created by Bojie Shen on 13/5/2023.
//
#pragma once
#include "CDT.h"
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <Fade_2D.h>
#include <iomanip>
namespace cdtutils
{

using namespace std;

struct CustomPoint2D
{
    CustomPoint2D(double i1, double i2) {
        x = i1;
        y = i2;
    }

    bool operator==(const CustomPoint2D & other) const {
        return (x == other.x) && (y == other.y);
    }

    bool operator!=(const CustomPoint2D& other) const {
        return !(*this == other);
    }

    double x;
    double y;
    int id = -1;
    int tri_id = -1;
    int center_id =-1;
};

struct CustomEdge
{
    CustomEdge(size_t i1, size_t i2) {
        vertices.first = i1;
        vertices.second = i2;
    }
    std::pair<std::size_t, std::size_t> vertices;
};

struct CustomPoly
{
    std::vector<CustomPoint2D> vertices;
    std::vector<CustomEdge> edges;
};


double polarAngle(const CustomPoint2D& p, const CustomPoint2D& center);
bool compare(const CustomPoint2D& p1, const CustomPoint2D& p2, const CustomPoint2D& center);
void fail(const string& message);
vector<CustomPoly> *read_polys(istream& infile);
void convertPoly2Mesh(string input_file, string output_file, int width);

}

