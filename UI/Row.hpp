#ifndef SPACE_ROW_HPP
#define SPACE_ROW_HPP
#include "View.hpp"
#include <vector>

class Row : public View {
  double mGap;

  double calculateGap() const;
public:
  Row(double gap = 20);

  double spaceBetweenWidth = 0;

  std::vector<View *> viewList;

  Vec2 getLayoutSize() override;
  SDL_FRect getRect() override;
  void layout(Vec2 size);
  void update() override;
  void draw(SDL_Renderer *renderer) override;
  void setCenterPosition(const Vec2 &centerPosition) override;
  bool onClick(SDL_FPoint point) override;
};

#endif // SPACE_ROW_HPP
