#ifndef SPACE_LASER_HPP
#define SPACE_LASER_HPP

#include "GameEntity.hpp"
#include "../Vec2.hpp"
#include "../AssetManager.hpp"

class Laser : public GameEntity {
public:
  Vec2 directionVector{0, -1};

  Laser(TextureLoader *textureLoader, const Vec2 &position, double angle);
  void onTick(IGameStage *stage) override;
  void onHit(GameEntity *other) override;
};

#endif // SPACE_LASER_HPP
