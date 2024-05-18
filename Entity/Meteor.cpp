#include "Meteor.hpp"
#include "../AssetManager.hpp"

Meteor::Meteor(const Vec2 &position,
                const std::string &type)
    : GameEntity(position, 0) {
  texture = TextureManager::getInstance()->load(
      "PNG/Meteors/meteor" + type +
      ".png");
  updateBoundingBox();
}