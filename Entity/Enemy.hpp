#ifndef SPACE_ENEMY_HPP
#define SPACE_ENEMY_HPP

#include "GameEntity.hpp"
#include "../AssetManager.hpp"
#include "../AI/ContextSteering.hpp"

class Enemy : public GameEntity {
public:
  double speed;
  Vec2 direction{0, 0};
  Vec2 smoothedDirection{0, 0};
  Vec2 acceleration{0, 0};
  Uint32 lastFire = 0;
  ContextSteering contextSteering;
  Vec2 contextSteeringResult{0, 0};
  std::vector<GameEntity *> nearEntity;

  Enemy(TextureLoader *textureLoader, const Vec2 &position);

  void onTick(IGameStage *stage) override;
  void onDraw(SDL_Renderer *renderer, const Vec2 &cameraPosition) override;
  void onHit(GameEntity *other) override;
};

#endif // SPACE_ENEMY_HPP
