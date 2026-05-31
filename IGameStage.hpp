#ifndef SPACE_IGAMESTAGE_HPP
#define SPACE_IGAMESTAGE_HPP

#include "Math/Mat3.hpp"

#include <memory>
#include <string>

struct GameParams {
  int ship;
  std::string color;
  bool enemyWithMissile;
  bool enemyWithLaser;
};

class FlowField;
class GameEntity;
class SAP;
class Particle;

class IGameStage {
public:
  virtual ~IGameStage() = default;
  virtual void addLaser(const Vec2 &position, double angle,
                        const std::string &textureName) = 0;
  virtual void addMissile(const Vec2 &position, double angle,
                          const std::string &textureName) = 0;

  virtual const Vec2 &getWorldSize() = 0;

  virtual FlowField *getFlowField() = 0;

  virtual SAP *getSAP() = 0;

  virtual GameEntity *getPlayerEntity() = 0;

  virtual const Vec2 &getCameraPosition() = 0;

  virtual const Vec2 &getCameraSize() = 0;

  [[nodiscard]] virtual Uint32 getTick() const = 0;

  virtual void addParticle(std::unique_ptr<Particle> &&particle) = 0;
};

#endif // SPACE_IGAMESTAGE_HPP
