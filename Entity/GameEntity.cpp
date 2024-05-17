#include "GameEntity.hpp"

void GameEntity::drawTexture(SDL_Renderer *renderer, const Vec2 &cameraPosition,
                             SDL_Texture *texture) {
  SDL_Rect rect;

  SDL_QueryTexture(texture, nullptr, nullptr, &rect.w, &rect.h);
  rect.x = int(position.x - cameraPosition.x - double(rect.w) / 2);
  rect.y = int(position.y - cameraPosition.y - double(rect.h) / 2);

  SDL_RenderCopyEx(renderer, texture, nullptr, &rect, angle, nullptr,
                   SDL_FLIP_NONE);
}

SDL_Rect GameEntity::getRect() const {
  SDL_Rect r;
  SDL_QueryTexture(texture, nullptr, nullptr, &r.w, &r.h);
  r.x = int(position.x - r.w / 2);
  r.y = int(position.y - r.h / 2);
  return r;
}

void GameEntity::onDraw(SDL_Renderer *renderer, const Vec2 &cameraPosition) {
  drawTexture(renderer, cameraPosition, texture);
}

void GameEntity::updateBoundingBox() {
  boundingBox.clear();

  int width, height;
  SDL_QueryTexture(texture, nullptr, nullptr, &width, &height);
  double halfWidth = double(width) / 2;
  double halfHeight = double(height) / 2;

  boundingRadius = hypot(halfWidth, halfHeight);

  Vec2 topRight(position.x + halfWidth, position.y - halfHeight);
  Vec2 bottomRight{position.x + halfWidth, position.y + halfHeight};
  Vec2 bottomLeft{position.x - halfWidth, position.y + halfHeight};
  Vec2 topLeft{position.x - halfWidth, position.y - halfHeight};

  double radianAngle = angle * M_PI / 180.0;
  topRight.rotateAround(radianAngle, position);
  bottomRight.rotateAround(radianAngle, position);
  bottomLeft.rotateAround(radianAngle, position);
  topLeft.rotateAround(radianAngle, position);

  boundingBox.push_back(topRight);
  boundingBox.push_back(bottomRight);
  boundingBox.push_back(bottomLeft);
  boundingBox.push_back(topLeft);

  auto [minX, maxX] = std::minmax_element(
      boundingBox.begin(), boundingBox.end(),
      [](const Vec2 &a, const Vec2 &b) { return a.x < b.x; });
  auto [minY, maxY] = std::minmax_element(
      boundingBox.begin(), boundingBox.end(),
      [](const Vec2 &a, const Vec2 &b) { return a.y < b.y; });
  x0 = minX->x;
  x1 = maxX->x;
  y0 = minY->y;
  y1 = maxY->y;
}