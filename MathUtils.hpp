#ifndef SPACE_MATHUTILS_HPP
#define SPACE_MATHUTILS_HPP

#include "Vec2.hpp"
#include <optional>

inline double mapValue(double value, double minValue, double maxValue,
                double minOutput, double maxOutput) {
  return minOutput +
         (maxOutput - minOutput) * ((value - minValue) / (maxValue - minValue));
}

std::optional<Vec2> rayCircleIntersection(const Vec2 &point, const Vec2 &ray,
                                          const Vec2 &circle, double radius);

#endif // SPACE_MATHUTILS_HPP
