#ifndef SPACE_POWERUPHEALTH_HPP
#define SPACE_POWERUPHEALTH_HPP

#include "GameEntity.hpp"

class PowerUpHealth : public GameEntity {
public:
  PowerUpHealth(const Vec2& position);

  void onTick(IGameStage *stage) override {}

  void onHit(IGameStage *stage, GameEntity *other) override;
};

#endif // SPACE_POWERUPHEALTH_HPP
