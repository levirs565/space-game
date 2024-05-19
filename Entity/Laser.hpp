#ifndef SPACE_LASER_HPP
#define SPACE_LASER_HPP

#include "../AssetManager.hpp"
#include "../Math/Vec2.hpp"
#include "GameEntity.hpp"

class Laser : public GameEntity {
public:
  Vec2 directionVector{0, -1};

  Laser(const Vec2 &position, double angle, const std::string & textureName);
  void onTick(IGameStage *stage) override;
  void onHit(GameEntity *other) override;
};

#endif // SPACE_LASER_HPP
