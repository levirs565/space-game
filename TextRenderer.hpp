#ifndef SPACE_TEXTRENDERER_HPP
#define SPACE_TEXTRENDERER_HPP

#include <SDL_ttf.h>
#include <string>

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

  void setText(std::string text);
  SDL_Texture *getTexture(SDL_Renderer *renderer);
};

#endif // SPACE_TEXTRENDERER_HPP
