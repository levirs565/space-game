#ifndef SPACE_COLUMN_HPP
#define SPACE_COLUMN_HPP

#include <vector>
#include "View.hpp"

class Column : View {
  static constexpr double mGap = 20;
public:
  std::vector<View*> viewList;

  View* findByPoint(SDL_FPoint point);

  Vec2 getLayoutSize() override;
  SDL_FRect getRect() override;
  void layout(Vec2 size);
  void update() override;
  void draw(SDL_Renderer *renderer) override;
};

#endif // SPACE_COLUMN_HPP
