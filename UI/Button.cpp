#include "Button.hpp"
#include "../AssetManager.hpp"
#include "../SDLHelper.hpp"
#include <algorithm>
#include <cassert>

Button::Button(std::string text, SDLHelper::Radius radius, uint32_t baseColor,
               uint32_t outlineColor)
    : mTextRenderer(
          FontManager::getInstance()->load("Bonus/kenvector_future.ttf", 16),
          {.r = 255, .g = 255, .b = 255, .a = 255}),
      mRadius(radius), mBaseColor(baseColor), mOutlineColor(outlineColor) {
  mTextRenderer.setText(std::move(text));
}

Button::~Button() {
  if (mButtonTexture != nullptr) {
    SDL_DestroyTexture(mButtonTexture);
    SDL_DestroyTexture(mButtonOutlineTexture);
    SDL_DestroyTexture(mButtonHoverTexture);
  }
}

Vec2 Button::getLayoutSize() { return {222, 39}; }
void Button::update() {
  if (mRenderer == nullptr)
    return;

  SDL_FPoint mouse;
  SDL_GetMouseState(&mouse.x, &mouse.y);
  SDL_RenderCoordinatesFromWindow(mRenderer, mouse.x, mouse.y, &mouse.x,
                                  &mouse.y);
  setFocus(isPointInside(mouse));

  double targetScale = mFocus ? mFocusScale : 1;
  mScale += (targetScale - mScale) * (1.0 - std::exp(-0.5));
  mScale = std::clamp(mScale, 1.0, mFocusScale);

  double targetOpacity = mFocus ? 1.0 : 0.0;
  mHoverOpacity += (targetOpacity - mHoverOpacity) * (1.0 - std::exp(-0.5));
  mHoverOpacity = std::clamp(mHoverOpacity, 0.0, 1.0);
}
void Button::draw(SDL_Renderer *renderer) {
  mRenderer = renderer;

  if (mButtonTexture == nullptr) {
    Vec2 size = getLayoutSize();
    mButtonTexture = SDLHelper::createBeveledRectTexture(
        renderer, size.x, size.y, mRadius,
        [&](int x, int y) { return mBaseColor; });
    int border = 2;
    mButtonOutlineTexture = SDLHelper::createBeveledRectTextureOutline(
        renderer, size.x, size.y, border, mRadius,
        [&](int x, int y) { return mOutlineColor; });
    mButtonHoverTexture = SDLHelper::createBeveledRectTexture(
        renderer, size.x, size.y, mRadius,
        [&](int x, int y) { return mOutlineColor; });
  }

  SDL_FRect rect = calculateTextureRect(mButtonTexture, mScale);
  SDL_RenderTexture(renderer, mButtonTexture, nullptr, &rect);

  SDL_FRect innerRect = calculateTextureRect(mButtonOutlineTexture, mScale);
  SDL_RenderTexture(renderer, mButtonOutlineTexture, nullptr, &innerRect);

  SDL_SetTextureBlendMode(mButtonHoverTexture, SDL_BLENDMODE_BLEND);
  SDL_SetTextureAlphaModFloat(mButtonHoverTexture, mHoverOpacity);

  SDL_FRect hoverRect = calculateTextureRect(mButtonHoverTexture, mScale);
  SDL_RenderTexture(renderer, mButtonHoverTexture, nullptr, &hoverRect);

  SDL_Texture *textTexture = mTextRenderer.getTexture(renderer);
  rect = calculateTextureRect(textTexture, mScale);
  SDL_RenderTexture(renderer, textTexture, nullptr, &rect);
}
SDL_FRect Button::getRect() { return calculateRect(mScale * getLayoutSize()); }
