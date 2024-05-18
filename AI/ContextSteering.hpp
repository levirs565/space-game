#ifndef SPACE_CONTEXTSTEERING_HPP
#define SPACE_CONTEXTSTEERING_HPP

#include "../Math/Vec2.hpp"
#include <SDL.h>
#include <cmath>
#include <ranges>

struct ContextSteeringMap {
  static constexpr int angleCount = 12;
  static constexpr double deltaAngle = 2.0 * M_PI / angleCount;
  double data[angleCount] = {};

  static Vec2 directionBy(int index) {
    Vec2 vector{1, 0};
    vector.rotate(deltaAngle * index);
    return vector;
  }

  static int shiftIndex(int index, int delta) {
    return (index + delta + angleCount) % angleCount;
  }

  void addVector(const Vec2 &vector, double minCos = 0);

  int indexOf(const double *ptr) { return ptr - begin(); }

  void clear() { std::fill(begin(), end(), 0); }

  double *begin() { return &data[0]; }

  double *end() { return &data[angleCount]; }

  auto withIndex() {
    return *this | std::views::transform([this](double &value) {
      return std::make_tuple(std::ref(value), indexOf(&value));
    });
  }

  void draw(SDL_Renderer *renderer, const Vec2 &position, double radius,
            double angleDeviation, int index);
};

struct ContextSteering {
  ContextSteeringMap interestMap, dangerMap, resultMap;

  void clear();
  Vec2 getResult();
  void draw(SDL_Renderer *renderer, const Vec2 &position, double radius);
};

#endif // SPACE_CONTEXTSTEERING_HPP
