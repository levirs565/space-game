#ifndef SPACE_GAMEENTITY_HPP
#define SPACE_GAMEENTITY_HPP

#include <SDL.h>
#include <vector>

#include "../IGameStage.hpp"
#include "../Math/Vec2.hpp"

class GameEntity {
public:
  enum class CollisionResponse {
    Repel, RejectBoth };

  CollisionResponse collisionResponse;
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

#endif // SPACE_GAMEENTITY_HPP
