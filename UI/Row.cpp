#include "Row.hpp"

void Row::layout(Vec2 size) { setCenterPosition(0.5 * size); }
void Row::draw(SDL_Renderer *renderer) {
  for (View *view : viewList)
    view->draw(renderer);
}

void Row::setCenterPosition(const Vec2 &centerPosition) {
  View::setCenterPosition(centerPosition);

  Vec2 layoutSize = getLayoutSize();
  double centerY = centerPosition.y;
  double currentX = centerPosition.x - layoutSize.x / 2;
  double gap = calculateGap();

  for (View *view : viewList) {
    Vec2 viewSize = view->getLayoutSize();
    Vec2 childPosition{currentX + viewSize.x / 2, centerY};
    view->setCenterPosition(childPosition);
    currentX += viewSize.x + gap;
  }
}
bool Row::onClick(SDL_FPoint point) {
  for (View *view : viewList) {
    if (view->isPointInside(point)) {
      return view->onClick(point);
    }
  }
  return false;
}

Vec2 Row::getLayoutSize() {
  double maxHeight = 0;
  double width = 0;

  for (View *view : viewList) {
    Vec2 viewSize = view->getLayoutSize();
    if (viewSize.y > maxHeight)
      maxHeight = viewSize.y;

    width += viewSize.x;
  }

  width += (double(viewList.size()) - 1) * calculateGap();
  return {width, maxHeight};
}

SDL_FRect Row::getRect() { return calculateRect(getLayoutSize()); }
void Row::update() {
  for (View *view : viewList)
    view->update();
}
double Row::calculateGap() const {
  if (spaceBetweenWidth == 0)
    return mGap;

  double width = 0;
  for (View *view : viewList) {
    width += view->getLayoutSize().x;
  }

  double availableWidth = std::max(0.0, spaceBetweenWidth - width);
  return availableWidth / (double(viewList.size()) - 1);
}
Row::Row(double gap) : mGap(gap) {}