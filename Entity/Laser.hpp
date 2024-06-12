#ifndef SPACE_LASER_HPP
#define SPACE_LASER_HPP

#include "../AssetManager.hpp"
#include "../Math/Vec2.hpp"
#include "GameEntity.hpp"

class Laser : public GameEntity {
public:
  Laser(const Vec2 &position, const Vec2& direction, const std::string & textureName);
  void onTick(IGameStage *stage) override;
  void onHit(IGameStage *stage, GameEntity *other) override;
};

#endif // SPACE_LASER_HPP
