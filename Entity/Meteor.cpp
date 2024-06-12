#include "Meteor.hpp"
#include "../AssetManager.hpp"

Meteor::Meteor(const Vec2 &position,
                const std::string &type)
    : GameEntity(position, Vec2(0, 0)) {
  collisionResponse = CollisionResponse::Repel;
  texture = TextureManager::getInstance()->load(
      "PNG/Meteors/meteor" + type +
      ".png");
  updateBoundingBox();
}