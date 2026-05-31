#include "ContextSteering.hpp"
#include <numbers>
#include <algorithm>

void ContextSteeringMap::addVector(const Vec2 &vector, double minCos) {
  Vec2 normalizedVector{vector};
  normalizedVector.normalize();
  for (double &value : *this) {
    Vec2 currentDirection = directionBy(indexOf(&value));
    const double cos = currentDirection.dot(normalizedVector);
    if (cos < minCos) {
      // SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "Dot rejected %f", cos);
      continue;
    }

    if (cos != 0) {
      // SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "Dot %f", cos);
    }
    value = std::max(currentDirection.dot(vector), value);
  }
}

void ContextSteeringMap::draw(SDL_Renderer *renderer, const Mat3 &matrix, double radius,
          double angleDeviation, int index) {
  double value = data[index];
  if (value == 0)
    return;

  Vec2 start = (matrix * Vec2(0, 0)).toCartesian();

  Vec2 end{directionBy(index)};
  end.rotate(angleDeviation);
  end.scale(value * radius);
  end = (matrix * end).toCartesian();

  SDL_RenderLine(renderer, start.x, start.y, end.x, end.y);
}

void ContextSteering::clear() {
  interestMap.clear();
  dangerMap.clear();
}

Vec2 ContextSteering::getResult() {
  for (auto [value, index] : resultMap.withIndex()) {
    value =
        std::clamp(interestMap.data[index] - dangerMap.data[index], 0.0, 1.0);
  }

  Vec2 result{0, 0};
  for (auto [value, index] : resultMap.withIndex()) {
    result.add(ContextSteeringMap::directionBy(index), value);
  }

  result.normalize();
  return result;
}

void ContextSteering::draw(SDL_Renderer *renderer, const Mat3 &matrix, double radius) {
  static const double angleDeviation = 2.0 / 180.0 * std::numbers::pi;
  for (const auto [_, index] : interestMap.withIndex()) {
    SDL_SetRenderDrawColor(renderer, 255, 255, 0, 255);
    resultMap.draw(renderer, matrix, radius, -angleDeviation, index);
  }
  SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
  for (const auto [_, index] : dangerMap.withIndex())
    dangerMap.draw(renderer, matrix, radius, angleDeviation, index);
}