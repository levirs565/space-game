#include "Label.hpp"
#include "../AssetManager.hpp"

Vec2 Label::getLayoutSize() { return mTextRenderer.getSize(); }

void Label::update() {}

SDL_Rect Label::getRect() { return calculateRect(getLayoutSize()); }
void Label::draw(SDL_Renderer *renderer) {
  SDL_Texture *texture = mTextRenderer.getTexture(renderer);
  SDL_Rect rect = calculateTextureRect(texture, 1);
  SDL_RenderCopy(renderer, texture, nullptr, &rect);
}

Label::Label(std::string text)
    : mTextRenderer(
          FontManager::getInstance()->load("Bonus/kenvector_future.ttf", 16),
          {.r = 255, .g = 255, .b = 255, .a = 255}) {
  mTextRenderer.setText(std::move(text));
}
void Label::setText(std::string text) {
  mTextRenderer.setText(text);
}
