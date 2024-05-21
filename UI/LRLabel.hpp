#ifndef SPACE_LRLABEL_HPP
#define SPACE_LRLABEL_HPP

#include "Label.hpp"

class LRLabel : public View {
private:
  Label mLeft{""};
  Label mRight{""};
public:
  void layout();
  void setText(const std::string &left, const std::string &right);

  Vec2 getLayoutSize() override;
  SDL_Rect getRect() override;
  void update() override;
  void draw(SDL_Renderer *renderer) override;
};

#endif // SPACE_LRLABEL_HPP
