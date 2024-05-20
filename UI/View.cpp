#include "View.hpp"
#include "../SDLHelper.hpp"

Vec2 View::getTextureSize(SDL_Texture *texture) {
  int width, height;
  SDL_QueryTexture(texture, nullptr, nullptr, &width, &height);
  return { double(width), double(height) };
}
SDL_Rect View::calculateTextureRect(SDL_Texture *texture, double scale) {
  return SDLHelper::calculateTextureRectByCenter(texture, mCenterPosition, scale);
}
SDL_Rect View::calculateRect(const Vec2 &size) {
  return SDLHelper::calculateRect(mCenterPosition, size);
}
bool View::isPointInside(SDL_Point point) {
  SDL_Rect rect = getRect();
  return SDL_PointInRect(&point, &rect) == SDL_TRUE;
}
