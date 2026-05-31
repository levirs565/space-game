#include "Panel.hpp"

#include "../SDLHelper.hpp"
Panel::Panel(View *child) : mChild(child) {}
Vec2 Panel::getLayoutSize() {
  Vec2 size = mChild->getLayoutSize();
  return size + 2 * mPadding;
}
SDL_FRect Panel::getRect() {
  SDL_FRect rect = mChild->getRect();
  rect.x -= mPadding.x;
  rect.y -= mPadding.y;
  rect.w += 2.0 * mPadding.x;
  rect.h += 2.0 * mPadding.y;
  return rect;
}
void Panel::update() { mChild->update(); }
void Panel::draw(SDL_Renderer *renderer) {
  if (mOutlineTexture == nullptr) {
    Vec2 size = getLayoutSize();
    SDLHelper::Radius radius = {.topLeft = 20, .bottomRight = 20};
    mOutlineTexture = SDLHelper::createBeveledRectTextureOutline(
        renderer, size.x, size.y, 2, radius,
        [&](int x, int y) { return 0x36bbf5FF; });
    mBackgroundTexture = SDLHelper::createBeveledRectTexture(
        renderer, size.x, size.y, radius,
        [](int x, int y) { return 0x171f33FF; });
  }

  SDL_FRect backgroundRect = calculateTextureRect(mBackgroundTexture, 1.0);
  SDL_RenderTexture(renderer, mBackgroundTexture, nullptr, &backgroundRect);

  SDL_FRect rect = calculateTextureRect(mOutlineTexture, 1.0);
  SDL_RenderTexture(renderer, mOutlineTexture, nullptr, &rect);

  mChild->draw(renderer);
}
void Panel::setCenterPosition(const Vec2 &centerPosition) {
  View::setCenterPosition(centerPosition);
  mChild->setCenterPosition(centerPosition);
}
bool Panel::onClick(SDL_FPoint point) {
  if (mChild->isPointInside(point))
    return mChild->onClick(point);
  return false;
}