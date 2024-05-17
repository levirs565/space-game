#include "MathUtils.hpp"

std::optional<Vec2> rayCircleIntersection(const Vec2 &point, const Vec2 &ray,
                                          const Vec2 &circle, double radius) {
  Vec2 u{circle};
  u.substract(point);

  double dot = u.dot(ray);

  if (dot < 0)
    return std::nullopt;

  Vec2 u1 = u.projectInto(ray, false);

  Vec2 u2{u};
  u2.substract(u1);

  double d = u2.length();

  if (d > radius)
    return std::nullopt;
  if (std::abs(d - radius) < 0.1)
    return std::nullopt;

  double m = std::sqrt(radius * radius - d * d);

  Vec2 p{u1};
  p.add(ray, -m);

  return std::optional(p);
}
