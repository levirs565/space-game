#ifndef VEC2_HPP_
#define VEC2_HPP_

#include <cmath>

struct Vec2 {
  double x;
  double y;

  Vec2(double x, double y) : x(x), y(y) {}

  void rotate(double radian) {
    double cos = std::cos(radian);
    double sin = std::sin(radian);
    double nextX = x * cos - y * sin;
    double nextY = x * sin + y * cos;

    x = nextX;
    y = nextY;
  }

  void rotateAround(double radian, const Vec2 &center) {
    substract(center);
    rotate(radian);
    add(center, 1);
  }

  void add(const Vec2 &other, double scale) {
    x += scale * other.x;
    y += scale * other.y;
  }

  void substract(const Vec2 &other) {
    x -= other.x;
    y -= other.y;
  }

  void scale(double factor) {
    x *= factor;
    y *= factor;
  }

  double length() const { return std::sqrt(x * x + y * y); }

  double getRotation() { return std::atan2(y, x); }

  void normalize() {
    double l = length();
    if (l == 0)
      return;
    x /= l;
    y /= l;
  }

  void makePerpendicular() {
    std::swap(x, y);
    x *= -1;
  }

  double dot(const Vec2 &other) const { return x * other.x + y * other.y; }

  Vec2 projectInto(const Vec2 &other, bool clamp) const {
    Vec2 axis = other;
    axis.normalize();
    double projectionLength = dot(axis);
    if (clamp && projectionLength < 0)
      projectionLength = 0;
    else if (clamp && projectionLength > other.length())
      projectionLength = other.length();
    axis.scale(projectionLength);
    return axis;
  }

  inline Vec2 parallelComponent(const Vec2 &basis) const {
    return projectInto(basis, false);
  }

  Vec2 perpendicularComponent(const Vec2 &basis) const {
    Vec2 copy{*this};
    copy.substract(parallelComponent(basis));
    return copy;
  }

  double angleBetween(const Vec2 &other) const {
    return std::acos(dot(other) / length() / other.length());
  }

  double orientedAngleTo(const Vec2 &other) const {
    return std::atan2(x * other.y - y * other.x, dot(other));
  }

  /**
   * basis must be vector with length 1
   */
  Vec2 limitMaxDeviationCos(const Vec2 &basis, const double maxCos) {
    Vec2 currentDirection{*this};
    currentDirection.normalize();

    double currentCosAngle = currentDirection.dot(basis);

    if (currentCosAngle >= maxCos)
      return *this;

    Vec2 perpendicular = perpendicularComponent(basis);
    perpendicular.normalize();
    perpendicular.scale(std::sqrt(1 - maxCos * maxCos));

    Vec2 result{basis};
    result.scale(currentCosAngle);
    result.add(perpendicular, 1);
    result.scale(length());

    return result;
  }
};

#endif // VEC2_HPP_