#include "PlayerShip.hpp"
#include "../AssetManager.hpp"
#include "../Math/Helper.hpp"
#include "Meteor.hpp"
#include "PowerUpHealth.hpp"

PlayerShip::PlayerShip(const Vec2 &position)
    : GameEntity(position, Vec2(1, 0)) {
  collisionResponse = CollisionResponse::Repel;
  TextureManager *manager = TextureManager::getInstance();
  texture = manager->load("PNG/playerShip3_blue.png");
  damagedTexture.push_back(manager->load("PNG/"
                                         "Damage/playerShip3_damage1.png"));
  damagedTexture.push_back(manager->load("PNG/"
                                         "Damage/playerShip3_damage2.png"));
  damagedTexture.push_back(manager->load("PNG/"
                                         "Damage/playerShip3_damage3.png"));

  maxSpeed = 5;
  maxAccelerationLength = 0.1;
  maxAngularSpeed = deg2Rad(5);
  drawRotationShift = -std::numbers::pi / 2;

  shieldTexture = manager->load("PNG/Effects/shield3.png");
  updateBoundingBox();
}

void PlayerShip::setDirection(Direction direction, Rotation rotation) {
  if (direction == DIRECTION_UP) {
    acceleration = this->direction;
  } else if (direction == DIRECTION_DOWN && speed > 0) {
    acceleration = this->direction;
    acceleration.scale(-speed);
  } else {
    acceleration = {0, 0};
  }

  if (rotation == ROTATION_LEFT)
    angularAcceleration = -maxAngularSpeed;
  else if (rotation == ROTATION_RIGHT)
    angularAcceleration = maxAngularSpeed;
  else
    angularAcceleration = 0;
}

void PlayerShip::onTick(IGameStage *stage) {
  const Vec2 &worldSize = stage->getWorldSize();
  position.x = SDL_clamp(position.x, 0, worldSize.x);
  position.y = SDL_clamp(position.y, 0, worldSize.y);

  updateBoundingBox();

  if (hasShield && stage->getTick() - shieldActivationTime >= 5000) {
    hasShield = false;
    shieldDeactivationTIme = stage->getTick();
  }
}

void PlayerShip::onHit(IGameStage *stage, GameEntity *other) {
  if (dynamic_cast<PowerUpHealth *>(other) != nullptr) {
    healthCount = 4;
    return;
  }
  if (Laser *laser = dynamic_cast<Laser *>(other);
      laser != nullptr && !hasShield) {
    healthCount--;

    if (stage->getTick() - shieldDeactivationTIme >= 1000) {
      hasShield = true;
      shieldActivationTime = stage->getTick();
    }
  }
}

void PlayerShip::onDraw(SDL_Renderer *renderer, const Vec2 &cameraPosition) {
  GameEntity::onDraw(renderer, cameraPosition);
  if (healthCount <= 3) {
    int index = std::min(4 - healthCount, int(damagedTexture.size())) - 1;
    drawTexture(renderer, cameraPosition, damagedTexture[index]);
  }
  if (hasShield)
    drawTexture(renderer, cameraPosition, shieldTexture);
}

void PlayerShip::doFire(IGameStage *stage) {
  if ((stage->getTick() - lastFire >= 500)) {
    SDL_Rect rect = getRect();
    Vec2 laserPos(rect.w, 0);
    laserPos.rotate(direction.getRotation());
    laserPos.add(position, 1);
    stage->addLaser(laserPos, direction.getRotation(), "laserBlue01");
    lastFire = stage->getTick();
  }
}