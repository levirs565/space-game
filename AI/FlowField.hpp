#ifndef SPACE_FLOWFIELD_HPP
#define SPACE_FLOWFIELD_HPP

#include "../Entity/GameEntity.hpp"
#include "../Math/Vec2.hpp"
#include "ContextSteering.hpp"
#include <SDL.h>
#include <SDL_ttf.h>
#include <set>
#include <unordered_map>
#include <vector>

class FlowField {
public:
  using NodePosition = std::pair<int, int>;

private:
  class Node {
  public:
    int cost;
    Vec2 direction{0, 0};
    bool lineOfSight;
    int obstacleCount = 0;

    bool isWalkable() const {
      return obstacleCount == 0;
    }
  };

  struct ObstacleData {
    int left, right, top, bottom;
  };

  std::vector<std::vector<Node>> mGrid;
  std::unordered_map<GameEntity *, ObstacleData> mObstacleMap;
  int mRowCount = 0;
  int mColumnCount = 0;
  int mEntitySize = 0;

public:
  void init(const Vec2 &worldSize, int entitySize);

  void addObstacle(GameEntity *entity);
  void moveObstacle(GameEntity *entity);

  void drawGrid(SDL_Renderer *renderer, const Vec2 &cameraPosition,
                Vec2 &cameraSize);

  inline NodePosition getNodePositionFromWorldPosition(const Vec2 &position) {
    return {std::clamp(int(position.y / mEntitySize), 0, mRowCount - 1),
            std::clamp(int(position.x / mEntitySize), 0, mColumnCount - 1)};
  }

  std::vector<NodePosition> getNeighbours(const NodePosition &position);

  std::vector<NodePosition> getEdges(const NodePosition &position);

  inline Vec2 getDirection(const Vec2 &position, const Vec2 &direction) {
    return getDirection(getNodePositionFromWorldPosition(position), direction);
  }

  Vec2 getDirection(const NodePosition &nodePosition, const Vec2 &direction);

  inline bool addDirectionToSteering(const Vec2 &position,
                                     const Vec2 &direction,
                                     ContextSteeringMap &map, double scale) {
    return addDirectionToSteering(getNodePositionFromWorldPosition(position),
                                  direction, map, scale);
  }

  bool addDirectionToSteering(const NodePosition &nodePosition,
                              const Vec2 &direction, ContextSteeringMap &map,
                              double scale);

  bool canWalk(const NodePosition &from, const NodePosition &to);
  void calculateLineOfSight(const NodePosition &from, const NodePosition &to);
  void generateHeatmap(const Vec2 &target);

  inline bool hasLineOfSigh(const Vec2 &position) {
    const NodePosition node = getNodePositionFromWorldPosition(position);

    return mGrid[node.first][node.second].lineOfSight;
  }
};

#endif // SPACE_FLOWFIELD_HPP
