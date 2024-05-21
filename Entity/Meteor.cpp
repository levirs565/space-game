#include "Meteor.hpp"
#include "../AssetManager.hpp"

Meteor::Meteor(const Vec2 &position,
                const std::string &type)
    : GameEntity(position, 0) {
  collisionResponse = CollisionResponse::Repel;
  texture = TextureManager::getInstance()->load(
      "PNG/Meteors/meteor" + type +
      ".png");
  updateBoundingBox();
}