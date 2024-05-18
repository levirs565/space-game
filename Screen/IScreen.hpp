#ifndef SPACE_ISCREEN_HPP
#define SPACE_ISCREEN_HPP

#include "SDL.h"
#include "../Math/Vec2.hpp"

class IScreen {
public:
  virtual void onSizeChanged(const Vec2& size) = 0;
  virtual void onSDLEvent(const SDL_Event& event) = 0;
  virtual void onUpdate() = 0;
  virtual void onDraw(SDL_Renderer* renderer) = 0;
  virtual void onPostDraw() = 0;
};

#endif // SPACE_ISCREEN_HPP
