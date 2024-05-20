#ifndef SPACE_TEXTINPUT_HPP
#define SPACE_TEXTINPUT_HPP

#include "View.hpp"
#include "../TextRenderer.hpp"

class TextInput : public View {
  TextRenderer mTextRenderer;
public:
  TextInput();

  void setText(const std::string& text);
  std::string getText() {return mTextRenderer.getText();}

  Vec2 getLayoutSize() override;
  SDL_Rect getRect() override;
  void update() override;
  void draw(SDL_Renderer *renderer) override;
};

#endif // SPACE_TEXTINPUT_HPP
