#ifndef IGAMESTAGE_HPP_
#define IGAMESTAGE_HPP_

#include <vector>
#include <memory>

#include "Vec2.hpp"

class APathFinder;
class GameEntity;
class SAP;

class IGameStage {
public:
  virtual const Vec2 &getPlayerPosition() = 0;

  virtual void addLaser(const Vec2 &position, double angle) = 0;

  virtual const Vec2 &getWorldSize() = 0;

  virtual std::vector<std::unique_ptr<GameEntity>> &getEntities() = 0;

  virtual Vec2 getFlowDirection(const Vec2 &position,
                                const Vec2 &direction) = 0;

  virtual std::vector<Vec2> findNeighbourObstacle(const Vec2 &position) = 0;

  virtual APathFinder *getPathFinder() = 0;

  virtual SAP *getSAP() = 0;

  virtual GameEntity *getPlayerEntity() = 0;
};


#endif // IGAMESTAGE_HPP_