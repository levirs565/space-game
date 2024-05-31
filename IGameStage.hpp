#ifndef SPACE_IGAMESTAGE_HPP
#define SPACE_IGAMESTAGE_HPP

#include <memory>
#include <vector>

#include "Math/Vec2.hpp"

class FlowField;
class GameEntity;
class SAP;

class IGameStage {
public:
  virtual void addLaser(const Vec2 &position, double angle, const std::string&textureName) = 0;

  virtual const Vec2 &getWorldSize() = 0;

  virtual FlowField *getFlowField() = 0;

  virtual SAP *getSAP() = 0;

  virtual GameEntity *getPlayerEntity() = 0;

  virtual const Vec2& getCameraPosition() = 0;

  virtual const Vec2& getCameraSize() = 0;

  virtual Uint32 getTick() const = 0;
};

#endif // SPACE_IGAMESTAGE_HPP
