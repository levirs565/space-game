#include "SDLHelper.hpp"
SDL_FRect SDLHelper::calculateTextureRectByCenter(SDL_Texture *texture,
                                                 const Vec2 &center,
                                                 double scale) {
  float textureWidth, textureHeight;
  SDL_GetTextureSize(texture, &textureWidth, &textureHeight);

  Vec2 size{double(textureWidth), double(textureHeight)};
  size.scale(scale);

  return calculateRect(center, size);
}
SDL_FRect SDLHelper::calculateRect(const Vec2 &center, const Vec2 &size) {
  Vec2 halfSize{size};
  halfSize.scale(0.5);

  Vec2 topLeft{center};
  topLeft.substract(halfSize);

  return {.x = float(topLeft.x),
          .y = float(topLeft.y),
          .w = float(size.x),
          .h = float(size.y)};
}
