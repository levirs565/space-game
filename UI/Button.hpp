#ifndef SPACE_BUTTON_HPP
#define SPACE_BUTTON_HPP

#include "../SDLHelper.hpp"
#include "../TextRenderer.hpp"
#include "View.hpp"

class Button : public View {
  uint32_t mBaseColor, mOutlineColor;
  double mScale = 1;
  double mHoverOpacity = 0;
  bool mFocus = false;

  TextRenderer mTextRenderer;
  SDL_Texture *mButtonTexture = nullptr;
  SDL_Texture *mButtonOutlineTexture = nullptr;
  SDL_Texture *mButtonHoverTexture = nullptr;
  SDL_Renderer *mRenderer = nullptr;
  SDLHelper::Radius mRadius;

  static constexpr double mFocusScale = 1.05;

public:
  explicit Button(std::string text, SDLHelper::Radius radius = {},
                  uint32_t baseColor = 0x131B2EFF,
                  uint32_t outlineColor = 0x35B7EFFF);
  ~Button() override;

  bool isSelectable = false;
  bool isSelected = false;
  bool scaleWhenHovered = true;
  double width = 222;
  std::function<bool(Button *)> onClickHandler = [](Button *button) {
    return false;
  };

  Vec2 getLayoutSize() override;
  SDL_FRect getRect() override;

  void update() override;
  void draw(SDL_Renderer *renderer) override;
  bool onClick(SDL_FPoint point) override;
};

#endif // SPACE_BUTTON_HPP
