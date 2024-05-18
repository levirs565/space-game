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
  virtual void addLaser(const Vec2 &position, double angle) = 0;

  virtual const Vec2 &getWorldSize() = 0;

  virtual FlowField *getFlowField() = 0;

  virtual SAP *getSAP() = 0;

  virtual GameEntity *getPlayerEntity() = 0;
};

#endif // SPACE_IGAMESTAGE_HPP
