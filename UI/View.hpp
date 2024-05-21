#ifndef SPACE_VIEW_HPP
#define SPACE_VIEW_HPP

#include "../Math/Vec2.hpp"
#include <SDL.h>

class View {
  Vec2 mCenterPosition;
protected:
  Vec2 getTextureSize(SDL_Texture* texture);
  SDL_Rect calculateRect(const Vec2& size);
  SDL_Rect calculateTextureRect(SDL_Texture *texture, double scale);
public:
  virtual ~View() = default;

  void setCenterPosition(const Vec2& centerPosition) {
    mCenterPosition = centerPosition;
  }

  Vec2 getCenterPosition() {
    return mCenterPosition;
  }

  bool isPointInside(SDL_Point point);

  virtual Vec2 getLayoutSize() = 0;
  virtual SDL_Rect getRect() = 0;

  virtual void update() = 0;
  virtual void draw(SDL_Renderer* renderer) = 0;
};

#endif // SPACE_VIEW_HPP
