#include "Column.hpp"

void Column::layout(Vec2 size) {
  Vec2 layoutSize = getLayoutSize();
  double centerX = size.x / 2;
  double currentY = size.y / 2 - layoutSize.y / 2;

  for (View *view : viewList) {
    Vec2 viewSize = view->getLayoutSize();
    Vec2 centerPosition{centerX,
                        currentY + viewSize.y / 2};
    view->setCenterPosition(centerPosition);
    currentY += viewSize.y + mGap;
  }
}
void Column::draw(SDL_Renderer *renderer) {
  for (View *view : viewList)
    view->draw(renderer);
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

SDL_Rect Column::getRect() { return calculateRect(getLayoutSize()); }
void Column::update() {
  for (View *view : viewList)
    view->update();
}
