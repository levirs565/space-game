#include "Laser.hpp"
#include "PowerUpHealth.hpp"

Laser::Laser(const Vec2 &position, const Vec2& direction, const std::string &textureName)
    : GameEntity(position, direction) {
  collisionResponse = CollisionResponse::RejectBoth;
  texture = TextureManager::getInstance()->load("PNG/Lasers/" + textureName + ".png");
  maxSpeed = speed = 7.5;
  drawRotationShift = - std::numbers::pi / 2;
  updateBoundingBox();
}

void Laser::onTick(IGameStage *stage) {
  Vec2 cameraSize = stage->getCameraSize();
  Vec2 cameraStart{stage->getCameraPosition()};
  Vec2 cameraEnd{cameraStart};
  cameraStart.add(cameraSize, -0.5);
  cameraEnd.add(cameraSize, 1.5);

  if (x1 < cameraStart.x || x0 > cameraEnd.x || y1 < cameraStart.y ||
      y0 > cameraEnd.y)
    mustGone = true;
}

void Laser::onHit(IGameStage *stage, GameEntity *other) {
  if (dynamic_cast<Laser *>(other) != nullptr)
    return;
  if (dynamic_cast<PowerUpHealth *>(other) != nullptr)
    return;
  mustGone = true;
}