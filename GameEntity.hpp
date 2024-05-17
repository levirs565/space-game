#ifndef GAMEENTITY_HPP_
#define GAMEENTITY_HPP_

#include <SDL.h>
#include <vector>

#include "IGameStage.hpp"
#include "Vec2.hpp"

class GameEntity {
public:
  SDL_Texture *texture;
  Vec2 position;
  double angle;
  bool mustGone = false;
  std::vector<Vec2> boundingBox;
  double boundingRadius = 0;
  double x0, y0, x1, y1;

  GameEntity(const Vec2 &position, double angle)
      : position(position), angle(angle) {}

  SDL_Rect getRect() const;

  virtual void onPreTick() {}
  virtual void onTick(IGameStage *stage) = 0;

  void drawTexture(SDL_Renderer *renderer, const Vec2 &cameraPosition,
                   SDL_Texture *texture);

  virtual void onDraw(SDL_Renderer *renderer, const Vec2 &cameraPosition);

  virtual void onHit(GameEntity *other) {}

  void updateBoundingBox();
};

#endif // GAMEENTITY_HPP_`