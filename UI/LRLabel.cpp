#include "LRLabel.hpp"

Vec2 LRLabel::getLayoutSize() {
  return {500, std::max(mLeft.getLayoutSize().y, mRight.getLayoutSize().y)};
}
SDL_Rect LRLabel::getRect() { return calculateRect(getLayoutSize()); }
void LRLabel::update() {
  mLeft.update();
  mRight.update();
}
void LRLabel::draw(SDL_Renderer *renderer) {
  mLeft.draw(renderer);
  mRight.draw(renderer);
}
void LRLabel::layout() {
  Vec2 layoutSize = getLayoutSize();
  Vec2 leftSize = mLeft.getLayoutSize();
  Vec2 rightSize = mRight.getLayoutSize();
  Vec2 centerPosition = getCenterPosition();

  double startX = centerPosition.x - layoutSize.x / 2;
  double endX = centerPosition.x + layoutSize.x / 2;

  mLeft.setCenterPosition({startX + leftSize.x / 2, centerPosition.y});
  mRight.setCenterPosition({endX - rightSize.x / 2, centerPosition.y});
}
void LRLabel::setText(const std::string &left, const std::string &right) {
  mLeft.setText(left);
  mRight.setText(right);
};
