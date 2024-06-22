#ifndef SPACE_ENEMY_HPP
#define SPACE_ENEMY_HPP

#include "Ship.hpp"
#include "../AssetManager.hpp"
#include "../AI/ContextSteering.hpp"

class Enemy : public Ship {
public:
  Uint32 lastFire = 0;
  ContextSteering contextSteering;
  Vec2 contextSteeringResult{0, 0};
  std::vector<GameEntity *> nearEntity;
  bool hasExplode = false;

  Enemy(const Vec2 &position);

  void onTick(IGameStage *stage) override;
  void onDraw(SDL_Renderer *renderer, const Vec2 &cameraPosition) override;
  void onHit(IGameStage *stage, GameEntity *other) override;
};

#endif // SPACE_ENEMY_HPP
