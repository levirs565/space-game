#include "Button.hpp"

#include "../AppSettings.hpp"
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
  SDL_DestroyTexture(mButtonTexture);
  SDL_DestroyTexture(mButtonOutlineTexture);
  SDL_DestroyTexture(mButtonHoverTexture);
}

Vec2 Button::getLayoutSize() { return {width, height}; }
void Button::update() {
  if (mRenderer == nullptr)
    return;

  SDL_FPoint mouse;
  SDL_GetMouseState(&mouse.x, &mouse.y);
  SDL_RenderCoordinatesFromWindow(mRenderer, mouse.x, mouse.y, &mouse.x,
                                  &mouse.y);
  mFocus = isPointInside(mouse);

  double targetScale = scaleWhenHovered ? mFocus ? mFocusScale : 1 : 1;
  mScale += (targetScale - mScale) * (1.0 - std::exp(-0.5));
  mScale = std::clamp(mScale, 1.0, mFocusScale);

  double targetOpacity = mFocus ? isSelectable ? 0.25 : 1.0 : 0.0;
  if (isSelectable && isSelected)
    targetOpacity = 1.0;
  mHoverOpacity += (targetOpacity - mHoverOpacity) * (1.0 - std::exp(-0.5));
  mHoverOpacity = std::clamp(mHoverOpacity, 0.0, 1.0);
}
void Button::draw(SDL_Renderer *renderer) {
  mRenderer = renderer;

  AppSettings *settings = getAppSettings();
  const int border = 2;
  if (mButtonTexture == nullptr && !settings->uiHardwareRendering) {
    Vec2 size = getLayoutSize();
    mButtonTexture = SDLHelper::createBeveledRectTexture(
        renderer, size.x, size.y, mRadius, mBaseColor);

    mButtonOutlineTexture = SDLHelper::createBeveledRectTextureOutline(
        renderer, size.x, size.y, border, mRadius, mOutlineColor);
    mButtonHoverTexture = SDLHelper::createBeveledRectTexture(
        renderer, size.x, size.y, mRadius, mOutlineColor);
  }

  if (!settings->uiHardwareRendering) {
    SDL_FRect rect = calculateTextureRect(mButtonTexture, mScale);
    SDL_RenderTexture(renderer, mButtonTexture, nullptr, &rect);

    SDL_FRect innerRect = calculateTextureRect(mButtonOutlineTexture, mScale);
    SDL_RenderTexture(renderer, mButtonOutlineTexture, nullptr, &innerRect);

    SDL_SetTextureBlendMode(mButtonHoverTexture, SDL_BLENDMODE_BLEND);
    SDL_SetTextureAlphaModFloat(mButtonHoverTexture, mHoverOpacity);

    SDL_FRect hoverRect = calculateTextureRect(mButtonHoverTexture, mScale);
    SDL_RenderTexture(renderer, mButtonHoverTexture, nullptr, &hoverRect);
  } else {
    SDL_FRect rect = getRect();
    SDLHelper::drawBeveledRect(renderer, rect, mRadius,
                               SDLHelper::hexToFColor(mBaseColor));
    auto color = SDLHelper::hexToFColor(mOutlineColor);
    SDLHelper::drawBeveledRectOutline(renderer, rect, border, mRadius, color);
    color.a = mHoverOpacity;
    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
    SDLHelper::drawBeveledRect(renderer, rect, mRadius, color);
  }

  SDL_Texture *textTexture = textureOverride != nullptr
                                 ? textureOverride
                                 : mTextRenderer.getTexture(renderer);
  SDL_FRect rect = calculateTextureRect(textTexture, mScale);
  SDL_RenderTexture(renderer, textTexture, nullptr, &rect);
}
bool Button::onClick(SDL_FPoint point) { return onClickHandler(this); }
SDL_FRect Button::getRect() { return calculateRect(mScale * getLayoutSize()); }
