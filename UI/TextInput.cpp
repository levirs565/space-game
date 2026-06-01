#include "TextInput.hpp"
#include "../AssetManager.hpp"
#include "../SDLHelper.hpp"

Vec2 TextInput::getLayoutSize() { return {300, 40}; }
SDL_FRect TextInput::getRect() { return calculateRect(getLayoutSize()); }
void TextInput::update() {}

void TextInput::setText(const std::string &text) {
  if (text.length() <= 15)
    mTextRenderer.setText(text);
  else
    mTextRenderer.setText(text.substr(0, 15));
}
void TextInput::draw(SDL_Renderer *renderer) {
  if (mOutlineTexture == nullptr) {
    Vec2 size = getLayoutSize();
    SDLHelper::Radius radius = {.topLeft = 10, .bottomRight = 10};
    mOutlineTexture = SDLHelper::createBeveledRectTextureOutline(
        renderer, size.x, size.y, 2, radius, 0x2b85b1FF);
    mBackgroundTexture = SDLHelper::createBeveledRectTexture(
        renderer, size.x, size.y, radius, 0x0b1326ff);
  }

  SDL_FRect backgroundRect = calculateTextureRect(mBackgroundTexture, 1.0);
  SDL_RenderTexture(renderer, mBackgroundTexture, nullptr, &backgroundRect);

  SDL_FRect outlineRect = calculateTextureRect(mOutlineTexture, 1.0);
  SDL_RenderTexture(renderer, mOutlineTexture, nullptr, &outlineRect);

  SDL_FRect outer = getRect();

  constexpr int borderWidth = 2;
  SDL_FRect inner = outer;
  inner.x += borderWidth;
  inner.y += borderWidth;
  inner.w -= 2 * borderWidth;
  inner.h -= 2 * borderWidth;

  SDL_FRect textRect = inner;
  textRect.x += 4;

  SDL_Texture *textTexture = mTextRenderer.getTexture(renderer);
  SDL_GetTextureSize(textTexture, &textRect.w, &textRect.h);

  textRect.y += (inner.h - textRect.h) / 2;
  SDL_RenderTexture(renderer, textTexture, nullptr, &textRect);

  constexpr int tickPadding = 8;
  SDL_FRect tickRect{.x = textRect.x +
                          (!mTextRenderer.getText().empty() ? textRect.w : 0),
                     .y = inner.y + tickPadding,
                     .w = 2,
                     .h = inner.h - 2 * tickPadding};
  SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
  SDL_RenderFillRect(renderer, &tickRect);
}
TextInput::TextInput()
    : mTextRenderer(
          FontManager::getInstance()->load("Bonus/kenvector_future.ttf", 16),
          {.r = 255, .g = 255, .b = 255, .a = 255}) {}
