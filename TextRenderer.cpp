#include "TextRenderer.hpp"

void TextRenderer::setText(const std::string &text) {
  if (text == mText)
    return;

  mText = text;

  clear();
  createSurface();
}

TextRenderer::~TextRenderer() { clear(); }

void TextRenderer::clear() {
  if (mTextTexture != nullptr) {
    SDL_DestroyTexture(mTextTexture);
    mTextTexture = nullptr;
  }
  if (mTextSurface != nullptr) {
    SDL_FreeSurface(mTextSurface);
    mTextSurface = nullptr;
  }
}
SDL_Texture *TextRenderer::getTexture(SDL_Renderer *renderer) {
  if (mTextTexture == nullptr) {
    mTextTexture = SDL_CreateTextureFromSurface(renderer, mTextSurface);
  }
  return mTextTexture;
}
void TextRenderer::createSurface() {
  mTextSurface = TTF_RenderUTF8_Solid(mFont, mText.c_str(), mColor);
}
TextRenderer::TextRenderer(TTF_Font *font, SDL_Color color)
    : mFont(font), mColor(color) {
  createSurface();
}
Vec2 TextRenderer::getSize() {
  int width, height;
  TTF_SizeUTF8(mFont, mText.c_str(), &width, &height);
  return { double(width), double(height) };
}
