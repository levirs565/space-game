#include "Polygon.hpp"

Vec2 findPolygonCenter(const std::vector<Vec2> &polygon) {
  Vec2 sum;
  for (const Vec2 &vertex : polygon) {
    sum.add(vertex, 1);
  }
  sum.scale(1.0 / double(polygon.size()));
  return sum;
}

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
                              std::vector<Vec2> &polygonB, Vec2 &minDepthNormal,
                              double &minDepth) {
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

    double axisDepth = std::min(maxProjectionB - minProjectionA,
                                maxProjectionA - minProjectionB);
    if (axisDepth < minDepth) {
      minDepth = axisDepth;
      minDepthNormal = normal;
    }
  }

  return true;
}

PolygonCollision calculatePolygonCollision(std::vector<Vec2> &polygonA,
                                           std::vector<Vec2> &polygonB) {
  Vec2 minDepthNormal;
  double minDepth = std::numeric_limits<double>::max();
  PolygonCollision result;
  result.isCollide =
      isPolygonCollideInternal(polygonA, polygonB, minDepthNormal, minDepth) &&
      isPolygonCollideInternal(polygonB, polygonA, minDepthNormal, minDepth);

  Vec2 centerA = findPolygonCenter(polygonA);
  Vec2 centerB = findPolygonCenter(polygonB);

  Vec2 direction{centerB};
  direction.substract(centerA);

  if (direction.dot(minDepthNormal) < 0) {
    minDepthNormal.scale(-1);
  }

  //  minDepth /= minDepthNormal.length();
  //  minDepthNormal.normalize();
  result.depth = minDepth;
  result.normal = minDepthNormal;
  return result;
}