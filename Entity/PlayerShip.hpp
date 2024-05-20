#ifndef SPACE_PLAYERSHIP_HPP
#define SPACE_PLAYERSHIP_HPP

#include <SDL.h>

#include "../AssetManager.hpp"
#include "GameEntity.hpp"
#include "Laser.hpp"

class PlayerShip : public GameEntity {
public:
  enum Direction { DIRECTION_UP, DIRECTION_DOWN, DIRECTION_NONE };

  enum Rotation { ROTATION_LEFT, ROTATION_RIGHT, ROTATION_NONE };

  Vec2 directionVector{0, 0};
  Uint32 lastFire = 0;
  int healthCount = 4;
  Uint32 shieldActivationTime = 0;
  Uint32 shieldDeactivationTIme = 0;
  bool hasShield = false;
  std::vector<SDL_Texture *> damagedTexture;
  SDL_Texture * shieldTexture;

  explicit PlayerShip(const Vec2 &position);
  void setDirection(Direction direction, Rotation rotation);
  void onTick(IGameStage *stage) override;
  void onHit(GameEntity *other) override;
  void onDraw(SDL_Renderer *renderer, const Vec2 &cameraPosition) override;
  void doFire(IGameStage *stage);
};

#endif // SPACE_PLAYERSHIP_HPP
