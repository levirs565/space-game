#include "TextRenderer.hpp"

void TextRenderer::setText(std::string text) {
  if (text == mText)
    return;

  mText = std::move(text);

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
