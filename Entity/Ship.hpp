#ifndef SPACE_SHIP_HPP
#define SPACE_SHIP_HPP

#include "./GameEntity.hpp"

class Ship : public GameEntity {
public:
  double maxAccelerationLength = 0.1;

  Ship(const Vec2& position, const Vec2& direction) : GameEntity(position, direction) {}
protected:
  void applyAngularSteering(const Vec2& desiredDirection);
  void applyLinearSteering(const Vec2&desiredVelocity);
};

#endif // SPACE_SHIP_HPP
