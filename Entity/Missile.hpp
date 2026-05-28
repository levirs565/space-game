#ifndef SPACE_MISSILE_HPP
#define SPACE_MISSILE_HPP
#include "GameEntity.hpp"

class Missile : public GameEntity {
  Vec2 startDirection;
  std::vector<Vec2> bezier;
  float bezierPosition;
  Vec2 getBezierPosition(double t);
public:
  Missile(const Vec2 &position, const Vec2& direction, const std::string & textureName);
  void onTick(IGameStage *stage) override;
  void onDraw(SDL_Renderer *renderer, const Mat3 &viewMatrix) override;
  void onHit(IGameStage *stage, GameEntity *other) override;
};

#endif //SPACE_MISSILE_HPP
