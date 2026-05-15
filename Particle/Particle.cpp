#include "Particle.hpp"
#include <cassert>

void Particle::drawTexture(SDL_Renderer *renderer, const Vec2 &cameraPosition,
                           SDL_Texture *texture) {
  SDL_FRect rect;

  SDL_GetTextureSize(texture, &rect.w, &rect.h);
  rect.w = floor(rect.w * scale);
  rect.h = floor(rect.h * scale);
  rect.x = position.x - cameraPosition.x - double(rect.w) / 2;
  rect.y = position.y - cameraPosition.y - double(rect.h) / 2;
  SDL_SetTextureBlendMode(texture, SDL_BLENDMODE_BLEND);
  SDL_SetTextureAlphaMod(texture, alpha);
  SDL_RenderTextureRotated(renderer, texture, nullptr, &rect,
                   0,
                   nullptr, SDL_FLIP_NONE);
}

void Particle::onUpdate() {
  position.add(velocity, 1);
  if (alpha < 5)
    alpha = 0;
  else alpha -= 5;
  if (alpha == 0) {
    isActive = false;
  }
}

void Particle::onDraw(SDL_Renderer *renderer, const Vec2 &cameraPosition) {
  drawTexture(renderer, cameraPosition, texture);
}
