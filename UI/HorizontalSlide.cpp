#include "HorizontalSlide.hpp"
Vec2 HorizontalSlide::getLayoutSize() {
  double maxHeight = 0;
  for (auto view : viewList) {
    maxHeight = std::max(maxHeight, view->getLayoutSize().y);
  }
  return {layoutWidth, maxHeight};
}
SDL_FRect HorizontalSlide::getRect() { return calculateRect(getLayoutSize()); }
void HorizontalSlide::update() {
  bool firstUpdate = mFirstUpdate;
  mFirstUpdate = false;
  Vec2 currentCenter = getCenterPosition();
  for (int index = 0; auto view : viewList) {
    double targetX = currentCenter.x + (index - currentIndex) * layoutWidth;
    double x = view->getCenterPosition().x;
    if (firstUpdate) {
      x = targetX;
    } else {
      x += (targetX - x) * (1.0 - std::exp(-0.1));
    }

    view->setCenterPosition(Vec2{x, currentCenter.y});

    index++;
  }

  for (auto view : viewList)
    view->update();
}
void HorizontalSlide::draw(SDL_Renderer *renderer) {
  for (auto view : viewList)
    view->draw(renderer);
}
bool HorizontalSlide::onClick(SDL_FPoint point) {
  for (View *view : viewList) {
    if (view->isPointInside(point)) {
      return view->onClick(point);
    }
  }
  return false;
}