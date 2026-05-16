#include "Particle.hpp"
#include <cassert>

void Particle::drawTexture(SDL_Renderer *renderer, const Mat3 &viewMatrix,
                           SDL_Texture *texture) {
  SDL_FRect rect;

  SDL_GetTextureSize(texture, &rect.w, &rect.h);

  Vec3 screenPosition = viewMatrix * position;
  Vec2 viewScale = viewMatrix.getScale();

  rect.w = rect.w * scale * viewScale.x;
  rect.h = rect.h * scale * viewScale.y;
  rect.x = screenPosition.x - double(rect.w) / 2;
  rect.y = screenPosition.y - double(rect.h) / 2;
  SDL_SetTextureBlendMode(texture, SDL_BLENDMODE_BLEND);
  SDL_SetTextureAlphaMod(texture, alpha);
  SDL_RenderTextureRotated(renderer, texture, nullptr, &rect, 0, nullptr,
                           SDL_FLIP_NONE);
}

void Particle::onUpdate() {
  position.add(velocity, 1);
  if (alpha < 5)
    alpha = 0;
  else
    alpha -= 5;
  if (alpha == 0) {
    isActive = false;
  }
}

void Particle::onDraw(SDL_Renderer *renderer, const Mat3 &viewMatrix) {
  drawTexture(renderer, viewMatrix, texture);
}
