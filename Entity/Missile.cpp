#include "Missile.hpp"

#include "../AssetManager.hpp"
#include "Laser.hpp"
#include "PowerUpHealth.hpp"

#include <numbers>
#include <ranges>

Vec2 Missile::getBezierPosition(double t) {
  const Vec2 &P0 = bezier[0], &P1 = bezier[1], &P2 = bezier[2], &P3 = bezier[3];
  double u = 1 - t;
  Vec2 result = u * u * u * P0 + 3 * u * u * t * P1 + 3 * u * t * t * P2 +
             t * t * t * P3;
  return result;
}

Missile::Missile(const Vec2 &position, const Vec2 &direction,
                 const std::string &textureName)
    : GameEntity(position, direction) {
  collisionResponse = CollisionResponse::RejectBoth;
  texture = TextureManager::getInstance()->load("PNG/Missiles/" + textureName +
                                                ".png");
  startDirection = direction;
  maxSpeed = 7.5;
  // velocity = direction;
  // velocity.scale(maxSpeed);
  drawRotationShift = -std::numbers::pi / 2;
  updateBoundingBox();
}

void Missile::onTick(IGameStage *stage) {
  Vec2 cameraSize = stage->getCameraSize();
  Vec2 cameraStart{stage->getCameraPosition()};
  Vec2 cameraEnd{cameraStart};
  cameraStart.add(cameraSize, -0.5);
  cameraEnd.add(cameraSize, 1.5);

  if (x1 < cameraStart.x || x0 > cameraEnd.x || y1 < cameraStart.y ||
      y0 > cameraEnd.y)
    mustGone = true;

  if (bezier.empty() || bezierPosition >= 1.0) {
    Mat3 matrix =
        Mat3::translation(position) * Mat3::rotation(startDirection.getRotation());

    bezier.clear();
    bezier.push_back(Vec2(0, 0));
    bezier.push_back(Vec2(50, 50));
    bezier.push_back(Vec2(100, -50));
    bezier.push_back(Vec2(150, 0));

    for (auto &point : bezier) {
      point = (matrix * point.toHomogenous()).toCartesian();
    }

    bezierPosition = 0;
  } else {
    double t = bezierPosition;
    double L = maxSpeed;
    const Vec2 &A = bezier[0], &B = bezier[1], &C = bezier[2], &D = bezier[3];
    Vec2 v1 = -3 * A + 9 * B - 9 * C + 3 * D;
    Vec2 v2 = 6 * A - 12 * B + 6 * C;
    Vec2 v3 = -3 * A + 3 * B;
    Vec2 divider = t * t * v1 + t * v2 + v3;
    t = std::min(t + L / divider.length(), 1.0);
    bezierPosition = t;

    double u = 1 - t;
    position = getBezierPosition(t);

    Vec2 derivation =
        3 * u * u * (B - A) + 6 * u * t * (C - B) + 3 * t * t * (D - C);
    direction = derivation;
  }
}
void Missile::onDraw(SDL_Renderer *renderer, const Mat3 &viewMatrix) {
  GameEntity::onDraw(renderer, viewMatrix);

  if (bezier.empty())
    return;

  const bool debug = false;

  if (!debug)
    return;

  std::array<SDL_FPoint, 100> rendered{};

  for (size_t index = 0; auto &point : rendered) {
    double t = double(index) / rendered.size();

    Vec2 p = (viewMatrix * getBezierPosition(t)).toCartesian();
    point.x = p.x;
    point.y = p.y;

    index++;
  }

  SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
  SDL_RenderLines(renderer, rendered.data(), rendered.size());
}

void Missile::onHit(IGameStage *stage, GameEntity *other) {
  if (dynamic_cast<Laser *>(other) != nullptr)
    return;
  if (dynamic_cast<Missile *>(other) != nullptr)
    return;
  if (dynamic_cast<PowerUpHealth *>(other) != nullptr)
    return;
  mustGone = true;
}