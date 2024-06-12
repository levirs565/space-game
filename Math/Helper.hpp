#ifndef SPACE_HELPER_HPP
#define SPACE_HELPER_HPP

#include "Vec2.hpp"
#include <optional>
#include <numbers>

inline double mapValue(double value, double minValue, double maxValue,
                double minOutput, double maxOutput) {
  return minOutput +
         (maxOutput - minOutput) * ((value - minValue) / (maxValue - minValue));
}

std::optional<Vec2> rayCircleIntersection(const Vec2 &point, const Vec2 &ray,
                                          const Vec2 &circle, double radius);

inline double deg2Rad(double deg) {
  return deg * std::numbers::pi / 180.0;
}

inline double rad2Deg(double rad) {
  return rad * 180 * std::numbers::inv_pi;
}

#endif // SPACE_HELPER_HPP
