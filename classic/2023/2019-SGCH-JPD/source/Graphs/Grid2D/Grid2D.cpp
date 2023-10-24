#include "Grid2D.h"
#include <math.h>
using namespace std;

#ifndef NO_HOG
void Grid2D::GenerateGrid(const MapEnvironment *env, int padding) {
  // Read the height and width of the map.
  Map* map = env->GetMap();
  width_ = map->GetMapWidth();
  height_ = map->GetMapHeight();
  padding_ = padding;
  padded_width_ = width_ + 2 * padding_;
  padded_height_ = height_ + 2 * padding_;
  y_mult_ = padded_width_;
  padding_offset_ = y_mult_ * padding_ + padding_;

  traversable_.resize(padded_width_ * padded_height_, false);

  num_blocked_ = 0;
  num_unblocked_ = 0;

  // Read the nodes of the original graph.
  for (int x = 0; x < width_; x++) {
    for (int y = 0; y < height_; y++) {
      if (map->GetTerrainType(x, y) == kGround
          || map->GetTerrainType(x, y) == kSwamp) {
        num_unblocked_++;
        traversable_[ToXYLin(x, y)] = true;
      } else {
        num_blocked_++;
      }
    }
  }
  PrintGridInfo();
}
#endif

void Grid2D::GenerateGrid(std::vector<bool> cells, int width, int height, int padding) {
  string s;

  width_ = width;
  height_ = height;

#ifndef SG_QUIET
  cerr << "Grid dimensions: " << width_ << " x " << height_ << endl;
#endif

  padding_ = padding;
  padded_width_ = width_ + 2 * padding_;
  padded_height_ = height_ + 2 * padding_;
  y_mult_ = padded_width_;
  padding_offset_ = y_mult_ * padding_ + padding_;

  traversable_.resize(padded_width_ * padded_height_, false);

  num_blocked_ = 0;
  num_unblocked_ = 0;

  int i = 0;
  for (int y = 0; y < height_; y++) {
    for (int x = 0; x < width_; x++) {
      char cell = cells[i] ? '.' : '@';
      if (cell == '.' || cell == 'G' || cell == 'S') {
        num_unblocked_++;
        traversable_[ToXYLin(x, y)] = true;
      } else {
        num_blocked_++;
      }

      i++;
    }
  }

#ifndef SG_QUIET
  PrintGridInfo();
#endif
}

void Grid2D::GetNeighbors(SgchXyLoc l, std::vector<SgchXyLoc> & neighbors,
                          bool eight_neighbor, bool diagonal_rule) {
  neighbors.clear();

  int x = l.x;
  int y = l.y;

  if (!IsTraversable(x, y))
    return;

  bool b_n = y == 0 || !IsTraversable(x, y - 1);
  bool b_s = y == height_ - 1 || !IsTraversable(x, y + 1);
  bool b_w = x == 0 || !IsTraversable(x - 1, y);
  bool b_e = x == width_ - 1 || !IsTraversable(x + 1, y);

  bool b_nw = y == 0 || x == 0 || !IsTraversable(x - 1, y - 1);
  bool b_ne = y == 0 || x == width_ - 1 || !IsTraversable(x + 1, y - 1);
  bool b_sw = y == height_ - 1 || x == 0 || !IsTraversable(x - 1, y + 1);
  bool b_se = y == height_ - 1 || x == width_ - 1
      || !IsTraversable(x + 1, y + 1);

  // Cardinal neighbors.
  neighbors.clear();
  if (!b_n)
    neighbors.push_back(SgchXyLoc(x, y - 1));
  if (!b_s)
    neighbors.push_back(SgchXyLoc(x, y + 1));
  if (!b_w)
    neighbors.push_back(SgchXyLoc(x - 1, y));
  if (!b_e)
    neighbors.push_back(SgchXyLoc(x + 1, y));

  // Diagonal neighbors.
  if (!eight_neighbor)
    return;

  if (!b_nw)
    if (!diagonal_rule || !(b_n || b_w))
      neighbors.push_back(SgchXyLoc(x - 1, y - 1));
  if (!b_ne)
    if (!diagonal_rule || !(b_n || b_e))
      neighbors.push_back(SgchXyLoc(x + 1, y - 1));
  if (!b_sw)
    if (!diagonal_rule || !(b_s || b_w))
      neighbors.push_back(SgchXyLoc(x - 1, y + 1));
  if (!b_se)
    if (!diagonal_rule || !(b_s || b_e))
      neighbors.push_back(SgchXyLoc(x + 1, y + 1));
}


#ifndef NO_HOG
void Grid2D::DrawNode(const MapEnvironment *env, int x, int y, int priority) {
  env->OpenGLPriorityDraw(SgchXyLoc(x,y), priority);
}
void Grid2D::DrawDirectedNode(const MapEnvironment *env, int x, int y, double r,
                      double length) {
  env->GLDrawColoredLine(fmax((double)x, 0.001), fmax((double)y, 0.001),
                         fmax((double)x + cos(r) * length, 0.001),
                         fmax((double)y + sin(r) * length, 0.001));
}
void Grid2D::DrawArrowNode(const MapEnvironment *env, int x, int y, double r,
                   double length, int priority) {
  double angle_to_radian = 0.0174533;
  double p1d = 0.2*length; double p1a = 270 * angle_to_radian;
  double p2d = 0.2*length; double p2a = 90 * angle_to_radian;
  double p3d = 0.8*length; double p3a = 0 * angle_to_radian;

  auto calculate_coordinates = [&](double d, double a, double x, double y, double r) -> xyPosCont {
    double x_new = cos(a + r) * d + x;
    double y_new = sin(a + r) * d + y;
    return xyPosCont(x_new,y_new);
  };

  xyPosCont p1 = calculate_coordinates(p1d, p1a, x, y, r);
  xyPosCont p2 = calculate_coordinates(p2d, p2a, x, y, r);
  xyPosCont p3 = calculate_coordinates(p3d, p3a, x, y, r);

  env->GLDrawColoredLine(p1.x, p1.y, p2.x, p2.y, priority);
  env->GLDrawColoredLine(p2.x, p2.y, p3.x, p3.y, priority);
  env->GLDrawColoredLine(p3.x, p3.y, p1.x, p1.y, priority);
}

#endif



