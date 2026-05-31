#ifndef SPACE_HORIZONTALSLIDE_HPP
#define SPACE_HORIZONTALSLIDE_HPP
#include "View.hpp"

class HorizontalSlide : public View {
  bool mFirstUpdate = true;
public:
  double layoutWidth = 0;
  std::vector<View*> viewList;
  int currentIndex = 0;

  Vec2 getLayoutSize() override;
  SDL_FRect getRect() override;
  void update() override;
  void draw(SDL_Renderer *renderer) override;
  bool onClick(SDL_FPoint point) override;
};

#endif //SPACE_HORIZONTALSLIDE_HPP
