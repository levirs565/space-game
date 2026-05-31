#include "Column.hpp"

void Column::layout(Vec2 size) { setCenterPosition(0.5 * size); }
void Column::draw(SDL_Renderer *renderer) {
  for (View *view : viewList)
    view->draw(renderer);
}

void Column::setCenterPosition(const Vec2 &centerPosition) {
  View::setCenterPosition(centerPosition);

  Vec2 layoutSize = getLayoutSize();
  double centerX = centerPosition.x;
  double currentY = centerPosition.y - layoutSize.y / 2;

  for (View *view : viewList) {
    Vec2 viewSize = view->getLayoutSize();
    Vec2 childPosition{centerX, currentY + viewSize.y / 2};
    view->setCenterPosition(childPosition);
    currentY += viewSize.y + mGap;
  }
}
bool Column::onClick(SDL_FPoint point) {
  for (View *view : viewList) {
    if (view->isPointInside(point)) {
      return view->onClick(point);
    }
  }
  return false;
}

Vec2 Column::getLayoutSize() {
  double maxWidth = 0;
  double height = 0;

  for (View *view : viewList) {
    Vec2 viewSize = view->getLayoutSize();
    if (viewSize.x > maxWidth)
      maxWidth = viewSize.x;

    height += viewSize.y;
  }

  height += (double(viewList.size()) - 1) * mGap;
  return {maxWidth, height};
}

SDL_FRect Column::getRect() { return calculateRect(getLayoutSize()); }
void Column::update() {
  for (View *view : viewList)
    view->update();
}
View *Column::findByPoint(SDL_FPoint point) {
  for (View *view : viewList) {
    if (view->isPointInside(point)) {
      return view;
    }
  }
  return nullptr;
}
