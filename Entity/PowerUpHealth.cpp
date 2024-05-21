#include "PowerUpHealth.hpp"
#include "../AssetManager.hpp"
#include "PlayerShip.hpp"

PowerUpHealth::PowerUpHealth(const Vec2 &position) : GameEntity(position, 0) {
  collisionResponse = CollisionResponse::RejectBoth;
  texture = TextureManager::getInstance()->load("PNG/Power-ups/pill_blue.png");
  updateBoundingBox();
}
void PowerUpHealth::onHit(GameEntity *other) {
  if (dynamic_cast<PlayerShip*>(other) != nullptr)
    mustGone = true;
}
