#include "Laser.hpp"
#include "PowerUpHealth.hpp"

Laser::Laser(const Vec2 &position, double angle, const std::string &textureName)
    : GameEntity(position, angle) {
  collisionResponse = CollisionResponse::RejectBoth;
  texture =
      TextureManager::getInstance()->load("PNG/Lasers/" + textureName + ".png");
  directionVector.rotate(angle * M_PI / 180.0);
  updateBoundingBox();
}

void Laser::onTick(IGameStage *stage) {
  position.add(directionVector, 7.5);

  updateBoundingBox();

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