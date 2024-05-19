#ifndef SPACE_MAP_HPP
#define SPACE_MAP_HPP

#include "Entity/GameEntity.hpp"
#include <vector>

struct Map {
  std::vector<std::unique_ptr<GameEntity>> entityList;
  Vec2 size;

  static Map fromAsset();
};

#endif // SPACE_MAP_HPP
