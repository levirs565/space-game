#ifndef SPACE_TEXTRENDERER_HPP
#define SPACE_TEXTRENDERER_HPP

#include <SDL_ttf.h>
#include <string>
#include "Math/Vec2.hpp"

class TextRenderer {
  TTF_Font *mFont;
  SDL_Surface *mTextSurface = nullptr;
  SDL_Texture *mTextTexture = nullptr;
  std::string mText;
  SDL_Color mColor;

  void clear();
  void createSurface();
public:
  TextRenderer(TTF_Font* font, SDL_Color color);
  ~TextRenderer();

  void setText(const std::string& text);
  const std::string& getText() {return mText;}
  SDL_Texture *getTexture(SDL_Renderer *renderer);
  Vec2 getSize();
};

#endif // SPACE_TEXTRENDERER_HPP
