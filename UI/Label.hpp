#ifndef SPACE_LABEL_HPP
#define SPACE_LABEL_HPP

#include "View.hpp"
#include "../TextRenderer.hpp"

class Label : public View {
  TextRenderer mTextRenderer;
public:
  explicit Label(std::string text);

  Vec2 getLayoutSize() override;
  SDL_Rect getRect() override;
  void update() override;
  void draw(SDL_Renderer *renderer) override;
};

#endif // SPACE_LABEL_HPP
