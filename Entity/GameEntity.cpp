#include "GameEntity.hpp"
#include "../Math/Helper.hpp"
#include <algorithm>

size_t GameEntity::sNextId = 0;

void GameEntity::drawTexture(SDL_Renderer *renderer, const Mat3 &viewMatrix,
                             SDL_Texture *texture, float alpha) {
  SDL_FRect rect;

  SDL_GetTextureSize(texture, &rect.w, &rect.h);

  Vec3 screenPosition = viewMatrix * position;
  Vec2 scale = viewMatrix.getScale();

  rect.w *= scale.x;
  rect.h *= scale.y;

  rect.x = screenPosition.x - rect.w / 2;
  rect.y = screenPosition.y - rect.h / 2;

  // Default blend mode is blend
  SDL_SetTextureBlendMode(texture, SDL_BLENDMODE_BLEND);
  SDL_SetTextureAlphaModFloat(texture, alpha);

  SDL_RenderTextureRotated(
      renderer, texture, nullptr, &rect,
      rad2Deg(smoothedDirection.getRotation() - drawRotationShift), nullptr,
      SDL_FLIP_NONE);
}

SDL_FRect GameEntity::getRect() const {
  SDL_FRect r;
  SDL_GetTextureSize(texture, &r.w, &r.h);
  r.x = position.x - r.w / 2;
  r.y = position.y - r.h / 2;
  return r;
}

void GameEntity::onDraw(SDL_Renderer *renderer, const Mat3 &viewMatrix) {
  drawTexture(renderer, viewMatrix, texture, opacity);
}

void GameEntity::updateBoundingBox() {
  boundingBox.clear();

  float width, height;
  SDL_GetTextureSize(texture, &width, &height);
  double halfWidth = double(width) / 2;
  double halfHeight = double(height) / 2;

  boundingRadius = hypot(halfWidth, halfHeight);

  double radianAngle = smoothedDirection.getRotation() - drawRotationShift;
  Mat3 modelMatrix = Mat3::translation(position) * Mat3::rotation(radianAngle);

  Vec2 topRight(halfWidth, -halfHeight);
  Vec2 bottomRight{halfWidth, halfHeight};
  Vec2 bottomLeft{-halfWidth, halfHeight};
  Vec2 topLeft{-halfWidth, -halfHeight};

  boundingBox.push_back((modelMatrix * topRight).toCartesian());
  boundingBox.push_back((modelMatrix * bottomRight).toCartesian());
  boundingBox.push_back((modelMatrix * bottomLeft).toCartesian());
  boundingBox.push_back((modelMatrix * topLeft).toCartesian());

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