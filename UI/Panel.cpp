#include "Panel.hpp"

#include "../AppSettings.hpp"
#include "../SDLHelper.hpp"
Panel::Panel(View *child) : mChild(child) {}
Panel::~Panel() {
  SDL_DestroyTexture(mOutlineTexture);
  SDL_DestroyTexture(mBackgroundTexture);
}

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
  AppSettings *settings = getAppSettings();
  SDLHelper::Radius radius = {.topLeft = 20, .bottomRight = 20};
  int thickness = 2;
  Uint32 outlineColor = 0x36bbf5FF, backgroundColor = 0x171f33FF;
  if (mOutlineTexture == nullptr && !settings->uiHardwareRendering) {
    Vec2 size = getLayoutSize();
    mOutlineTexture = SDLHelper::createBeveledRectTextureOutline(
        renderer, size.x, size.y, thickness, radius, outlineColor);
    mBackgroundTexture = SDLHelper::createBeveledRectTexture(
        renderer, size.x, size.y, radius, backgroundColor);
  }

  if (!settings->uiHardwareRendering) {
    SDL_FRect backgroundRect = calculateTextureRect(mBackgroundTexture, 1.0);
    SDL_RenderTexture(renderer, mBackgroundTexture, nullptr, &backgroundRect);

    SDL_FRect rect = calculateTextureRect(mOutlineTexture, 1.0);
    SDL_RenderTexture(renderer, mOutlineTexture, nullptr, &rect);
  } else {
    SDL_FRect rect = getRect();
    SDLHelper::drawBeveledRect(renderer, rect, radius,
                               SDLHelper::hexToFColor(backgroundColor));
    SDLHelper::drawBeveledRectOutline(renderer, rect, thickness, radius,
                                      SDLHelper::hexToFColor(outlineColor));
  }

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