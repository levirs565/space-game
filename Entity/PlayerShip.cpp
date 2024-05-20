#include "PlayerShip.hpp"
#include "../AssetManager.hpp"
#include "PowerUpHealth.hpp"

PlayerShip::PlayerShip(const Vec2 &position) : GameEntity(position, 25) {
  TextureManager *manager = TextureManager::getInstance();
  texture = manager->load("PNG/playerShip3_blue.png");
  damagedTexture.push_back(manager->load("PNG/"
                                         "Damage/playerShip3_damage1.png"));
  damagedTexture.push_back(manager->load("PNG/"
                                         "Damage/playerShip3_damage2.png"));
  damagedTexture.push_back(manager->load("PNG/"
                                         "Damage/playerShip3_damage3.png"));

  shieldTexture = manager->load("PNG/Effects/shield3.png");
}

void PlayerShip::setDirection(Direction direction, Rotation rotation) {
  if (direction == DIRECTION_UP) {
    directionVector.y = -1;
    directionVector.x = 0;
  } else if (direction == DIRECTION_DOWN) {
    directionVector.x = 0;
    directionVector.y = 1;
  } else {
    directionVector.x = 0;
    directionVector.y = 0;
  }

  if (rotation == ROTATION_LEFT)
    angle -= 5;

  if (rotation == ROTATION_RIGHT)
    angle += 5;

  directionVector.rotate(angle * M_PI / 180.0);
}

void PlayerShip::onTick(IGameStage *stage) {
  position.add(directionVector, 5);

  const Vec2 &worldSize = stage->getWorldSize();
  position.x = SDL_clamp(position.x, 0, worldSize.x);
  position.y = SDL_clamp(position.y, 0, worldSize.y);

  updateBoundingBox();

  if (hasShield && SDL_GetTicks() - shieldActivationTime >= 5000) {
    hasShield = false;
    shieldDeactivationTIme = SDL_GetTicks();
  }
}

void PlayerShip::onHit(GameEntity *other) {
  if (dynamic_cast<PowerUpHealth*>(other) != nullptr) {
    healthCount = 4;
    return;
  }
  if (Laser *laser = dynamic_cast<Laser *>(other); other != nullptr && !hasShield) {
    healthCount--;

    if (!hasShield) {
      if (SDL_GetTicks() - shieldDeactivationTIme >= 1000) {
        hasShield = true;
        shieldActivationTime = SDL_GetTicks();
      }
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
  if ((SDL_GetTicks() - lastFire >= 500)) {
    SDL_Rect rect = getRect();
    Vec2 laserPos(0, -rect.h);
    laserPos.rotate(angle * M_PI / 180.0);
    laserPos.add(position, 1);
    stage->addLaser(laserPos, angle, "laserBlue01");
    lastFire = SDL_GetTicks();
  }
}