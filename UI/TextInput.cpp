#include "TextInput.hpp"
#include "../AssetManager.hpp"

Vec2 TextInput::getLayoutSize() { return {300, 40}; }
SDL_Rect TextInput::getRect() { return calculateRect(getLayoutSize()); }
void TextInput::update() {}

void TextInput::setText(const std::string &text) {
  mTextRenderer.setText(text);
}
void TextInput::draw(SDL_Renderer *renderer) {
  SDL_Rect outer = getRect();

  SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
  SDL_RenderFillRect(renderer, &outer);

  constexpr int borderWidth = 2;
  SDL_Rect inner = outer;
  inner.x += borderWidth;
  inner.y += borderWidth;
  inner.w -= 2 * borderWidth;
  inner.h -= 2 * borderWidth;
  SDL_SetRenderDrawColor(renderer, 7, 6, 7, 255);
  SDL_RenderFillRect(renderer, &inner);

  SDL_Rect textRect = inner;
  textRect.x += 4;

  SDL_Texture *textTexture = mTextRenderer.getTexture(renderer);
  SDL_QueryTexture(textTexture, nullptr, nullptr, &textRect.w, &textRect.h);

  textRect.y += (inner.h - textRect.h) / 2;
  SDL_RenderCopy(renderer, textTexture, nullptr, &textRect);

  constexpr int tickPadding = 8;
  SDL_Rect tickRect{
      .x = textRect.x + (!mTextRenderer.getText().empty()  ? textRect.w : 0),
      .y = inner.y + tickPadding,
      .w = 2,
      .h = inner.h - 2 * tickPadding
  };
  SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
  SDL_RenderFillRect(renderer, &tickRect);
}
TextInput::TextInput()
    : mTextRenderer(
          FontManager::getInstance()->load("Bonus/kenvector_future.ttf", 16),
          {.r = 255, .g = 255, .b = 255, .a = 255}) {}
