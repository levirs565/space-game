#include "Laser.hpp"

Laser::Laser(TextureLoader *textureLoader, const Vec2 &position, double angle)
    : GameEntity(position, angle) {
  texture = textureLoader->load(
      "/home/levirs565/Unduhan/SpaceShooterRedux/PNG/Lasers/laserBlue01.png");
  directionVector.rotate(angle * M_PI / 180.0);
  updateBoundingBox();
}

void Laser::onTick(IGameStage *stage) {
  position.add(directionVector, 7.5);

  updateBoundingBox();
}

void Laser::onHit(GameEntity *other) {
  if (dynamic_cast<Laser *>(other) != nullptr)
    return;
  mustGone = true;
}