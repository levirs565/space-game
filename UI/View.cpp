#include "View.hpp"
#include "../SDLHelper.hpp"

Vec2 View::getTextureSize(SDL_Texture *texture) {
  float width, height;
  SDL_GetTextureSize(texture, &width, &height);
  return {double(width), double(height)};
}
SDL_FRect View::calculateTextureRect(SDL_Texture *texture, double scale) {
  return SDLHelper::calculateTextureRectByCenter(texture, mCenterPosition,
                                                 scale);
}
SDL_FRect View::calculateRect(const Vec2 &size) {
  return SDLHelper::calculateRect(mCenterPosition, size);
}
bool View::handleSDLEvent(const SDL_Event& event) {
  if (event.type == SDL_EVENT_MOUSE_BUTTON_DOWN &&
      event.button.button == SDL_BUTTON_LEFT) {
    SDL_FPoint point = {event.button.x, event.button.y};
    if (isPointInside(point)) {
      return onClick(point);
    }
  }
  return false;
}

bool View::isPointInside(SDL_FPoint point) {
  SDL_FRect rect = getRect();
  return SDL_PointInRectFloat(&point, &rect);
}
