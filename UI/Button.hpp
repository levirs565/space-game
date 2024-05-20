#ifndef SPACE_BUTTON_HPP
#define SPACE_BUTTON_HPP

#include "../TextRenderer.hpp"
#include "View.hpp"

class Button : public View {
  double mScale = 1;
  bool mFocus = false;

  TextRenderer mTextRenderer;
  SDL_Texture *mButtonTexture;

  static constexpr double mFocusScale = 1.05;

public:
  explicit Button(std::string text);

  void setFocus(bool focus) { mFocus = focus; };
  Vec2 getLayoutSize() override;
  SDL_Rect getRect() override;

  void update() override;
  void draw(SDL_Renderer *renderer) override;
};

#endif // SPACE_BUTTON_HPP
