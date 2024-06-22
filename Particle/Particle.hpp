#ifndef SPACE_PARTICLE_HPP
#define SPACE_PARTICLE_HPP

#include "../Math/Vec2.hpp"
#include <SDL.h>

class Particle {
protected:
  void drawTexture(SDL_Renderer* renderer, const Vec2& cameraPosition, SDL_Texture* texture);
public:
  Vec2 position;
  Vec2 velocity;
  Uint8 alpha = 255;
  bool isActive = true;
  SDL_Texture* texture = nullptr;
  double scale = 1;

  void onDraw(SDL_Renderer* renderer, const Vec2& cameraPosition);
  void onUpdate();
};

#endif // SPACE_PARTICLE_HPP
