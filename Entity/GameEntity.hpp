#ifndef SPACE_GAMEENTITY_HPP
#define SPACE_GAMEENTITY_HPP

#include <SDL.h>
#include <vector>

#include "../IGameStage.hpp"
#include "../Math/Vec2.hpp"

class GameEntity {
private:
  size_t mId;
  static size_t sNextId;

public:
  enum class CollisionResponse { Repel, RejectBoth };

  CollisionResponse collisionResponse;
  SDL_Texture *texture;
  Vec2 position;
  Vec2 direction;
  Vec2 smoothedDirection;
  double speed = 0;
  double maxSpeed = 0;
  double maxAngularSpeed = 0;

  Vec2 acceleration;
  double maxAccelerationLength = 0;
  double angularAcceleration = 0;
  double mass = 1;

  bool mustGone = false;
  std::vector<Vec2> boundingBox;
  double boundingRadius = 0;
  double x0, y0, x1, y1;

  double drawRotationShift = 0;

  GameEntity(const Vec2 &position, const Vec2 &direction)
      : position(position), direction(direction),
        smoothedDirection(direction), mId(sNextId++) {}
  virtual ~GameEntity() = default;

  SDL_Rect getRect() const;

  virtual void onPreTick() {}
  virtual void onTick(IGameStage *stage) = 0;
  void drawTexture(SDL_Renderer *renderer, const Vec2 &cameraPosition,
                   SDL_Texture *texture);
  virtual void onDraw(SDL_Renderer *renderer, const Vec2 &cameraPosition);
  virtual void onHit(IGameStage *stage, GameEntity *other) {}

  inline Vec2 getVelocity() {
    Vec2 velocity{direction};
    velocity.scale(speed);
    return velocity;
  }

  Vec2 addVelocity(Vec2 extraVelocity, double extraAngleVelocity);

  void onUpdatePhysic();
  void updateBoundingBox();

  size_t getId() { return mId; };
};

#endif // SPACE_GAMEENTITY_HPP
