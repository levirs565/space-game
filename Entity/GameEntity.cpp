#include "GameEntity.hpp"
#include "../Math/Helper.hpp"
#include <algorithm>

size_t GameEntity::sNextId = 0;

void GameEntity::drawTexture(SDL_Renderer *renderer, const Vec2 &cameraPosition,
                             SDL_Texture *texture) {
  SDL_FRect rect;

  SDL_GetTextureSize(texture, &rect.w, &rect.h);
  rect.x = position.x - cameraPosition.x - double(rect.w) / 2;
  rect.y = position.y - cameraPosition.y - double(rect.h) / 2;

  SDL_RenderTextureRotated(renderer, texture, nullptr, &rect,
                   rad2Deg(smoothedDirection.getRotation() - drawRotationShift),
                   nullptr, SDL_FLIP_NONE);
}

SDL_FRect GameEntity::getRect() const {
  SDL_FRect r;
  SDL_GetTextureSize(texture, &r.w, &r.h);
  r.x = position.x - r.w / 2;
  r.y = position.y - r.h / 2;
  return r;
}

void GameEntity::onDraw(SDL_Renderer *renderer, const Vec2 &cameraPosition) {
  drawTexture(renderer, cameraPosition, texture);
}

void GameEntity::updateBoundingBox() {
  boundingBox.clear();

  float width, height;
  SDL_GetTextureSize(texture, &width, &height);
  double halfWidth = double(width) / 2;
  double halfHeight = double(height) / 2;

  boundingRadius = hypot(halfWidth, halfHeight);

  Vec2 topRight(position.x + halfWidth, position.y - halfHeight);
  Vec2 bottomRight{position.x + halfWidth, position.y + halfHeight};
  Vec2 bottomLeft{position.x - halfWidth, position.y + halfHeight};
  Vec2 topLeft{position.x - halfWidth, position.y - halfHeight};

  double radianAngle = smoothedDirection.getRotation() - drawRotationShift;
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
void GameEntity::onUpdatePhysic() {
  position.add(velocity, 1);

  Vec2 deltaDirection{direction};
  deltaDirection.substract(smoothedDirection);
  smoothedDirection.add(deltaDirection, 0.15);

  updateBoundingBox();
}