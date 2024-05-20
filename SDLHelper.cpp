#include "SDLHelper.hpp"
SDL_Rect SDLHelper::calculateTextureRectByCenter(SDL_Texture *texture,
                                                 const Vec2 &center,
                                                 double scale) {
  int textureWidth, textureHeight;
  SDL_QueryTexture(texture, nullptr, nullptr, &textureWidth, &textureHeight);

  Vec2 size{double(textureWidth), double(textureHeight)};
  size.scale(scale);

  return calculateRect(center, size);
}
SDL_Rect SDLHelper::calculateRect(const Vec2 &center, const Vec2 &size) {
  Vec2 halfSize{size};
  halfSize.scale(0.5);

  Vec2 topLeft{center};
  topLeft.substract(halfSize);

  return {.x = int(std::floor(topLeft.x)),
          .y = int(std::floor(topLeft.y)),
          .w = int(std::ceil(size.x)),
          .h = int(std::ceil(size.y))};
}
