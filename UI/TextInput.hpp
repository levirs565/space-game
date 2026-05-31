#ifndef SPACE_TEXTINPUT_HPP
#define SPACE_TEXTINPUT_HPP

#include "../TextRenderer.hpp"
#include "View.hpp"

class TextInput : public View {
  TextRenderer mTextRenderer;
  SDL_Texture *mOutlineTexture = nullptr;
  SDL_Texture *mBackgroundTexture = nullptr;

public:
  TextInput();

  void setText(const std::string &text);
  std::string getText() { return mTextRenderer.getText(); }

  Vec2 getLayoutSize() override;
  SDL_FRect getRect() override;
  void update() override;
  void draw(SDL_Renderer *renderer) override;
};

#endif // SPACE_TEXTINPUT_HPP
