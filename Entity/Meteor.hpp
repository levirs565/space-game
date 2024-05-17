#ifndef SPACE_METEOR_HPP
#define SPACE_METEOR_HPP

#include "../AssetManager.hpp"
#include "GameEntity.hpp"

class Meteor : public GameEntity {
public:
  Meteor(TextureLoader *textureLoader, const Vec2 &position,
         const std::string &type);

  void onTick(IGameStage *stage) override {}
};

#endif // SPACE_METEOR_HPP
