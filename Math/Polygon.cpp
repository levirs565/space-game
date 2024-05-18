#include "Polygon.hpp"

std::pair<double, double>
findPolygonProjectionMinMax(std::vector<Vec2> &polygon, Vec2 target) {
  double minProjection = std::numeric_limits<double>::max();
  double maxProjection = std::numeric_limits<double>::min();

  for (const Vec2 &vertex : polygon) {
    const double projection = vertex.dot(target);
    minProjection = std::min(minProjection, projection);
    maxProjection = std::max(maxProjection, projection);
  }

  return {minProjection, maxProjection};
}

bool isPolygonCollideInternal(std::vector<Vec2> &polygonA,
                              std::vector<Vec2> &polygonB) {
  for (size_t i = 0; i < polygonA.size(); i++) {
    const Vec2 &currentVertex = polygonA.at(i);
    const Vec2 &nextVertex = polygonA.at((i + 1) % polygonA.size());

    Vec2 normal(nextVertex);
    normal.substract(currentVertex);
    normal.makePerpendicular();
    normal.normalize();

    auto [minProjectionA, maxProjectionA] =
        findPolygonProjectionMinMax(polygonA, normal);
    auto [minProjectionB, maxProjectionB] =
        findPolygonProjectionMinMax(polygonB, normal);

    if (maxProjectionA < minProjectionB || maxProjectionB < minProjectionA)
      return false;
  }

  return true;
}

bool isPolygonCollide(std::vector<Vec2> &polygonA,
                      std::vector<Vec2> &polygonB) {
  return isPolygonCollideInternal(polygonA, polygonB) &&
         isPolygonCollideInternal(polygonB, polygonA);
}