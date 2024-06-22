#include "Enemy.hpp"

#include "../AI/FlowField.hpp"
#include "../AssetManager.hpp"
#include "../Math/Helper.hpp"
#include "../SAP.hpp"
#include "Laser.hpp"
#include "Meteor.hpp"
#include "PlayerShip.hpp"
#include "PowerUpHealth.hpp"
#include "../Particle/Particle.hpp"

Enemy::Enemy(const Vec2 &position) : Ship(position, Vec2(1, 0)) {
  collisionResponse = CollisionResponse::Repel;
  maxSpeed = 2;
  maxAngularSpeed = deg2Rad(5);
  maxAccelerationLength = 0.1;
  drawRotationShift = std::numbers::pi / 2;

  texture = TextureManager::getInstance()->load("PNG/Enemies/enemyBlack1.png");
  updateBoundingBox();
}

void Enemy::onTick(IGameStage *stage) {
  nearEntity = stage->getSAP()->queryArea(
      position.x - 3 * boundingRadius, position.y - 3 * boundingRadius,
      position.x + 3 * boundingRadius, position.y + 3 * boundingRadius, false);

  contextSteering.clear();

  bool canAttack = false;
  Vec2 extraRotation{0, 0};

  Vec2 distanceVector{stage->getPlayerEntity()->position};
  distanceVector.substract(position);
  const double distance = distanceVector.length();

  const double clampedRadius = 2 * boundingRadius + 25;
  const double minRayLength = 1 * boundingRadius + 25;
  const double maxRayLength = 2 * boundingRadius;

  for (GameEntity *entity : nearEntity) {
    if (entity == this)
      continue;
    if (dynamic_cast<Laser *>(entity) != nullptr)
      continue;
    if (dynamic_cast<PowerUpHealth *>(entity) != nullptr)
      continue;

    Vec2 distanceVec{entity->position};
    distanceVec.substract(position);

    if (distanceVec.length() < clampedRadius) {
      Vec2 avoid{distanceVec};
      avoid.normalize();
      contextSteering.dangerMap.addVector(avoid);
    }

    for (const auto &[_, index] : contextSteering.dangerMap.withIndex()) {
      Vec2 ray = ContextSteeringMap::directionBy(index);
      auto intersection = rayCircleIntersection(position, ray, entity->position,
                                                entity->boundingRadius);
      if (intersection.has_value()) {
        Vec2 avoid = intersection.value();
        double length = avoid.length();

        avoid.normalize();
        avoid.scale(mapValue(std::clamp(length, minRayLength, maxRayLength),
                             minRayLength, maxRayLength, 1.0, 0.0));
        contextSteering.dangerMap.addVector(avoid, 0);
      }
    }
  }

  bool hasLineOfSight = stage->getFlowField()->hasLineOfSigh(position);
  if (distance > 400 || !hasLineOfSight) {
    bool pathFindSuccess = stage->getFlowField()->addDirectionToSteering(
        position, direction, contextSteering.interestMap,
        hasLineOfSight ? 0.5 : 1);

    if (hasLineOfSight || !pathFindSuccess) {
      Vec2 seekDirection{distanceVector};
      seekDirection.normalize();

      contextSteering.interestMap.addVector(seekDirection);
    }
  }

  if (distance < 600) {
    Vec2 distanceNormalized{distanceVector};
    distanceNormalized.normalize();

    auto intersection = rayCircleIntersection(
        position, direction, stage->getPlayerEntity()->position,
        stage->getPlayerEntity()->boundingRadius);
    if (intersection.has_value()) {
      Vec2 playerPosition = stage->getPlayerEntity()->position;
      canAttack = true;

      Vec2 perpendicularDistance{distanceNormalized};
      perpendicularDistance.makePerpendicular();

      for (int perpendicularShift : {-5, 0, 5}) {
        Vec2 from{position};
        from.add(perpendicularDistance, perpendicularShift);

        Vec2 to{playerPosition};
        to.add(perpendicularDistance, perpendicularShift);

        for (SAPRay ray = stage->getSAP()->queryRay(from.x, from.y, to.x, to.y);
             ray.next();) {
          if (ray.currentEntity == this) {
            continue;
          }
          if (dynamic_cast<PlayerShip *>(ray.currentEntity) != nullptr) {
            break;
          }
          if (dynamic_cast<Laser *>(ray.currentEntity) != nullptr) {
            continue;
          }
          if (dynamic_cast<PowerUpHealth *>(ray.currentEntity) != nullptr) {
            continue;
          }
          canAttack = false;
          break;
        }

        if (!canAttack)
          break;
      }
    }

    extraRotation = distanceNormalized;
  }

  if (stage->getTick() - lastFire >= 1000 && canAttack) {
    SDL_Rect enemyRect = getRect();
    Vec2 laserPos(enemyRect.w, 0);
    double laserAngle = direction.getRotation();
    laserPos.rotate(laserAngle);
    laserPos.add(position, 1);
    stage->addLaser(laserPos, laserAngle, "laserRed01");
    lastFire = stage->getTick();
  }

  contextSteeringResult = contextSteering.getResult();

  Vec2 desiredVelocity{contextSteeringResult};
  desiredVelocity.normalize();
  desiredVelocity.scale(maxSpeed);

  if (contextSteeringResult.length() != 0) {
    Vec2 desiredDirection(contextSteeringResult);
    desiredDirection.normalize();
    applyAngularSteering(desiredDirection);
  } else if (extraRotation.length() != 0) {
    applyAngularSteering(extraRotation);
  }

  applyLinearSteering(desiredVelocity);
}

void Enemy::onDraw(SDL_Renderer *renderer, const Vec2 &cameraPosition) {
  GameEntity::onDraw(renderer, cameraPosition);

  Vec2 onCameraPosition{position};
  onCameraPosition.substract(cameraPosition);
  contextSteering.draw(renderer, onCameraPosition, boundingRadius);

  Vec2 steeringLine{contextSteeringResult};
  steeringLine.normalize();
  steeringLine.scale(boundingRadius * 1.5);
  steeringLine.add(onCameraPosition, 1);
  SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255);
  SDL_RenderDrawLine(renderer, onCameraPosition.x, onCameraPosition.y,
                     steeringLine.x, steeringLine.y);
}

void Enemy::onHit(IGameStage *stage, GameEntity *other) {
  if (Laser *laser = dynamic_cast<Laser *>(other); laser != nullptr) {
    if (!hasExplode) {
      std::unique_ptr<Particle> particle = std::make_unique<Particle>();
      particle->texture = TextureManager::getInstance()->load("Explosion/explosion00.png");
      particle->position = position;
      particle->scale = 0.5;
      stage->addParticle(std::move(particle));
      hasExplode = true;
    }
    mustGone = true;
  } else if (dynamic_cast<Meteor *>(other) != nullptr) {
    // mustGone = true;
  }
}