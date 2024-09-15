#include "Ship.hpp"
#include "../Math/Helper.hpp"
#include <algorithm>

void Ship::applyAngularSteering(const Vec2 &desiredDirection) {
  const double deltaAngle =
      std::clamp(direction.orientedAngleTo(desiredDirection), -maxAngularSpeed,
                 maxAngularSpeed);

  direction.rotate(deltaAngle);
}

void Ship::applyLinearSteering(const Vec2 &desiredVelocity) {
  Vec2 acceleration{desiredVelocity};
  acceleration.substract(velocity);

  if (acceleration.length() > maxAccelerationLength) {
    acceleration.normalize();
    acceleration.scale(maxAccelerationLength);
  }

  velocity.add(acceleration, 1);
}
