#ifndef SPACE_VIEW_HPP
#define SPACE_VIEW_HPP

#include "../Math/Vec2.hpp"

#include <SDL3/SDL.h>
#include <functional>

class View {
  Vec2 mCenterPosition;

protected:
  Vec2 getTextureSize(SDL_Texture *texture);
  SDL_FRect calculateRect(const Vec2 &size);
  SDL_FRect calculateTextureRect(SDL_Texture *texture, double scale);

public:
  virtual ~View() = default;

  virtual void setCenterPosition(const Vec2 &centerPosition) {
    mCenterPosition = centerPosition;
  }

  Vec2 getCenterPosition() { return mCenterPosition; }

  bool handleSDLEvent(const SDL_Event &event);
  virtual bool onClick(SDL_FPoint point) { return false; };

  bool isPointInside(SDL_FPoint point);

  virtual Vec2 getLayoutSize() = 0;
  virtual SDL_FRect getRect() = 0;

  virtual void update() = 0;
  virtual void draw(SDL_Renderer *renderer) = 0;
};

#endif // SPACE_VIEW_HPP
