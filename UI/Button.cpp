#include "Button.hpp"
#include "../AssetManager.hpp"
#include "../SDLHelper.hpp"

Button::Button(std::string text)
    : mTextRenderer(
          FontManager::getInstance()->load("Bonus/kenvector_future.ttf", 16),
          {.r = 0, .g = 0, .b = 0, .a = 255}) {
  mButtonTexture = TextureManager::getInstance()->load("PNG/UI/buttonBlue.png");
  mTextRenderer.setText(std::move(text));
}
Vec2 Button::getLayoutSize() {
  return getTextureSize(mButtonTexture);
}
void Button::update() {
  SDL_Point mouse;
  SDL_GetMouseState(&mouse.x, &mouse.y);
  setFocus(isPointInside(mouse));

  double targetScale = mFocus ? mFocusScale : 1;
  mScale += (targetScale - mScale) * (1.0 - std::exp(-0.5));
  mScale = std::clamp(mScale, 1.0, mFocusScale);
}
void Button::draw(SDL_Renderer *renderer) {
  SDL_Rect rect = calculateTextureRect(mButtonTexture, mScale);
  SDL_RenderCopy(renderer, mButtonTexture, nullptr, &rect);

  SDL_Texture *textTexture = mTextRenderer.getTexture(renderer);
  rect = calculateTextureRect(textTexture, mScale);
  SDL_RenderCopy(renderer, textTexture, nullptr, &rect);
}
SDL_Rect Button::getRect() {
  return calculateTextureRect(mButtonTexture, mScale);
}
