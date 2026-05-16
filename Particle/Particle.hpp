#ifndef SPACE_PARTICLE_HPP
#define SPACE_PARTICLE_HPP

#include "../Math/Mat3.hpp"
#include "../Math/Vec2.hpp"
#include <SDL3/SDL.h>

class Particle {
protected:
  void drawTexture(SDL_Renderer* renderer, const Mat3& viewMatrix, SDL_Texture* texture);
public:
  Vec2 position;
  Vec2 velocity;
  Uint8 alpha = 255;
  bool isActive = true;
  SDL_Texture* texture = nullptr;
  double scale = 1;

  void onDraw(SDL_Renderer* renderer, const Mat3& viewMatrix);
  void onUpdate();
};

#endif // SPACE_PARTICLE_HPP
