#include "Meteor.hpp"

Meteor::Meteor(TextureLoader *textureLoader, const Vec2 &position,
                const std::string &type)
    : GameEntity(position, 0) {
  texture = textureLoader->load(
      "/home/levirs565/Unduhan/SpaceShooterRedux/PNG/Meteors/meteor" + type +
      ".png");
  updateBoundingBox();
}