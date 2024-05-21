#ifndef SPACE_POLYGON_HPP
#define SPACE_POLYGON_HPP

#include "Vec2.hpp"
#include <vector>

struct PolygonCollision {
  bool isCollide;
  double depth;
  Vec2 normal;
};

PolygonCollision calculatePolygonCollision(std::vector<Vec2> &polygonA, std::vector<Vec2> &polygonB);

#endif // SPACE_POLYGON_HPP
