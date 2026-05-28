#include "PowerUpHealth.hpp"
#include "../AssetManager.hpp"
#include "PlayerShip.hpp"

PowerUpHealth::PowerUpHealth(const Vec2 &position) : GameEntity(position, Vec2(0, 0)) {
  collisionResponse = CollisionResponse::RejectBoth;
  texture = TextureManager::getInstance()->load("PNG/Power-ups/pill_blue.png");
  updateBoundingBox();
  opacity = 0;
}

void PowerUpHealth::onTick(IGameStage *stage) {
  Uint32 currentTick = stage->getTick();
  if (mStartTick == SDL_MAX_UINT32) {
    mStartTick = currentTick;
  }
  const float fadeDuration = 250;
  opacity = std::min(float(currentTick - mStartTick) / fadeDuration, 1.0f);
}

void PowerUpHealth::onHit(IGameStage *stage, GameEntity *other) {
  if (dynamic_cast<PlayerShip*>(other) != nullptr)
    mustGone = true;
}
