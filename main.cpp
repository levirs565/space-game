#include <SDL.h>
#include <SDL_image.h>
#include <SDL_mixer.h>
#include <SDL_ttf.h>
#include <cmath>
#include <exception>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <queue>
#include <random>
#include <ranges>
#include <set>
#include <unordered_map>
#include <vector>

#include "DataFormat.hpp"
#include "GameEntity.hpp"
#include "IGameStage.hpp"
#include "SAP.hpp"

class TextureLoader {
public:
  explicit TextureLoader(SDL_Renderer *renderer) : mRenderer(renderer) {}

  TextureLoader(const TextureLoader &other) = delete;

  SDL_Texture *load(const std::string &name) {
    if (mCache.count(name) > 0)
      return mCache[name];

    SDL_LogMessage(SDL_LOG_CATEGORY_APPLICATION, SDL_LOG_PRIORITY_INFO,
                   "Loading texture %s", name.c_str());
    SDL_Texture *texture = IMG_LoadTexture(mRenderer, name.c_str());
    mCache[name] = texture;
    return texture;
  }

private:
  SDL_Renderer *mRenderer;
  std::unordered_map<std::string, SDL_Texture *> mCache;
};

class Laser : public GameEntity {
public:
  Vec2 directionVector{0, -1};

  Laser(TextureLoader *textureLoader, const Vec2 &position, double angle)
      : GameEntity(position, angle) {
    texture = textureLoader->load(
        "/home/levirs565/Unduhan/SpaceShooterRedux/PNG/Lasers/laserBlue01.png");
    directionVector.rotate(angle * M_PI / 180.0);
    updateBoundingBox();
  }

  void onTick(IGameStage *stage) override {
    position.add(directionVector, 7.5);

    updateBoundingBox();
  }

  void onHit(GameEntity *other) override {
    if (dynamic_cast<Laser *>(other) != nullptr)
      return;
    mustGone = true;
  }
};

class Meteor : public GameEntity {
public:
  explicit Meteor(TextureLoader *textureLoader, const Vec2 &position,
                  const std::string &type)
      : GameEntity(position, 0) {
    texture = textureLoader->load(
        "/home/levirs565/Unduhan/SpaceShooterRedux/PNG/Meteors/meteor" + type +
        ".png");
    updateBoundingBox();
  }

  void onTick(IGameStage *stage) override {}
};

class PlayerShip : public GameEntity {
public:
  enum Direction { DIRECTION_UP, DIRECTION_DOWN, DIRECTION_NONE };

  enum Rotation { ROTATION_LEFT, ROTATION_RIGHT, ROTATION_NONE };

  Vec2 directionVector{0, 0};
  Uint32 lastFire = 0;
  int healthCount = 4;
  std::vector<SDL_Texture *> damagedTexture;

  explicit PlayerShip(TextureLoader *textureLoader, const Vec2 &position)
      : GameEntity(position, 25) {
    texture = textureLoader->load(
        "/home/levirs565/Unduhan/SpaceShooterRedux/PNG/playerShip3_blue.png");
    damagedTexture.push_back(
        textureLoader->load("/home/levirs565/Unduhan/SpaceShooterRedux/PNG/"
                            "Damage/playerShip3_damage1.png"));
    damagedTexture.push_back(
        textureLoader->load("/home/levirs565/Unduhan/SpaceShooterRedux/PNG/"
                            "Damage/playerShip3_damage2.png"));
    damagedTexture.push_back(
        textureLoader->load("/home/levirs565/Unduhan/SpaceShooterRedux/PNG/"
                            "Damage/playerShip3_damage3.png"));
  }

  void setDirection(Direction direction, Rotation rotation) {
    if (direction == DIRECTION_UP) {
      directionVector.y = -1;
      directionVector.x = 0;
    } else if (direction == DIRECTION_DOWN) {
      directionVector.x = 0;
      directionVector.y = 1;
    } else {
      directionVector.x = 0;
      directionVector.y = 0;
    }

    if (rotation == ROTATION_LEFT)
      angle -= 5;

    if (rotation == ROTATION_RIGHT)
      angle += 5;

    directionVector.rotate(angle * M_PI / 180.0);
  }

  void onTick(IGameStage *stage) override {
    position.add(directionVector, 5);

    const Vec2 &worldSize = stage->getWorldSize();
    position.x = SDL_clamp(position.x, 0, worldSize.x);
    position.y = SDL_clamp(position.y, 0, worldSize.y);

    updateBoundingBox();
  }

  void onHit(GameEntity *other) override {
    if (Laser *laser = dynamic_cast<Laser *>(other); other != nullptr) {
      healthCount--;
    }
  }

  void onDraw(SDL_Renderer *renderer, const Vec2 &cameraPosition) override {
    GameEntity::onDraw(renderer, cameraPosition);
    if (healthCount <= 3) {
      int index = std::min(4 - healthCount, int(damagedTexture.size())) - 1;
      drawTexture(renderer, cameraPosition, damagedTexture[index]);
    }
  }

  void doFire(IGameStage *stage) {
    if ((SDL_GetTicks() - lastFire >= 500)) {
      SDL_Rect rect = getRect();
      Vec2 laserPos(0, -rect.h);
      laserPos.rotate(angle * M_PI / 180.0);
      laserPos.add(position, 1);
      stage->addLaser(laserPos, angle);
      lastFire = SDL_GetTicks();
    }
  }
};

struct ContextSteeringMap {
  static constexpr int angleCount = 12;
  static constexpr double deltaAngle = 2.0 * M_PI / angleCount;
  double data[angleCount] = {};

  static Vec2 directionBy(int index) {
    Vec2 vector{1, 0};
    vector.rotate(deltaAngle * index);
    return vector;
  }

  static int shiftIndex(int index, int delta) {
    return (index + delta + angleCount) % angleCount;
  }

  void addVector(const Vec2 &vector, double minCos = 0) {
    Vec2 normalizedVector{vector};
    normalizedVector.normalize();
    for (double &value : *this) {
      Vec2 currentDirection = directionBy(indexOf(&value));
      const double cos = currentDirection.dot(normalizedVector);
      if (cos < minCos) {
        // SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "Dot rejected %f", cos);
        continue;
      }

      if (cos != 0) {
        // SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "Dot %f", cos);
      }
      value = std::max(currentDirection.dot(vector), value);
    }
  }

  int indexOf(const double *ptr) { return ptr - begin(); }

  void clear() { std::fill(begin(), end(), 0); }

  double *begin() { return &data[0]; }

  double *end() { return &data[angleCount]; }

  auto withIndex() {
    return *this | std::views::transform([this](double &value) {
      return std::make_tuple(std::ref(value), indexOf(&value));
    });
  }

  void draw(SDL_Renderer *renderer, const Vec2 &position, double radius,
            double angleDeviation, int index) {
    double value = data[index];
    if (value == 0)
      return;
    Vec2 end{directionBy(index)};
    end.rotate(angleDeviation);
    end.scale(value * radius);
    end.add(position, 1);

    SDL_RenderDrawLine(renderer, position.x, position.y, end.x, end.y);
  }
};

struct ContextSteering {
  ContextSteeringMap interestMap, dangerMap, resultMap;
  int selectedIndex = 0;

  void clear() {
    interestMap.clear();
    dangerMap.clear();
  }

  Vec2 getResult() {
    for (auto [value, index] : resultMap.withIndex()) {
      value =
          std::clamp(interestMap.data[index] - dangerMap.data[index], 0.0, 1.0);
    }

    Vec2 result{0, 0};
    for (auto [value, index] : resultMap.withIndex()) {
      result.add(ContextSteeringMap::directionBy(index), value);
    }

    result.normalize();
    return result;
  }

  void draw(SDL_Renderer *renderer, const Vec2 &position, double radius) {
    static const double angleDeviation = 2.0 / 180.0 * M_PI;
    for (const auto [_, index] : interestMap.withIndex()) {
      SDL_SetRenderDrawColor(renderer, 255, 255, 0, 255);
      resultMap.draw(renderer, position, radius, -angleDeviation, index);
    }
    SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
    for (const auto [_, index] : dangerMap.withIndex())
      dangerMap.draw(renderer, position, radius, angleDeviation, index);
  }
};

class APathFinder {
public:
  using NodePosition = std::pair<int, int>;

private:
  class Node {
  public:
    NodePosition parentPosition;
    bool isWalkable = true;
    int cost;
    Vec2 direction{0, 0};
    bool lineOfSight;
  };

  std::vector<std::vector<Node>> mGrid;
  int mRowCount = 0;
  int mColumnCount = 0;
  int mEntitySize = 0;

public:
  void init(const Vec2 &worldSize, int entitySize) {
    mRowCount = ceil(worldSize.y / entitySize);
    mColumnCount = ceil(worldSize.x / entitySize);
    mEntitySize = entitySize;
    mGrid.clear();
    mGrid.resize(mRowCount, std::vector<Node>(mColumnCount));
  }

  void clearState() {
    for (std::vector<Node> &row : mGrid) {
      for (Node &cell : row) {
        cell.isWalkable = true;
      }
    }
  }

  void addObstacle(const Vec2 &centerPosition, int radius) {
    int left =
        floor(std::max((centerPosition.x - radius) / mEntitySize - 0.5, 0.0));
    int top =
        floor(std::max((centerPosition.y - radius) / mEntitySize - 0.5, 0.0));
    int right = int(
        round(std::max((centerPosition.x + radius) / mEntitySize + 0.5, 0.0)));
    int bottom = int(
        round(std::max((centerPosition.y + radius) / mEntitySize + 0.5, 0.0)));

    if (left > mColumnCount || right > mColumnCount)
      return;
    if (top > mRowCount || bottom > mRowCount)
      return;

    for (int row = top; row < bottom; row++) {
      for (int column = left; column < right; column++) {
        mGrid[row][column].isWalkable = false;
      }
    }
  }

  void drawGrid(SDL_Renderer *renderer, const Vec2 &cameraPosition,
                Vec2 &cameraSize) {
    static TTF_Font *font =
        TTF_OpenFont("/home/levirs565/Unduhan/kenney_space-shooter-redux/Bonus/"
                     "kenvector_future.ttf",
                     16);

    int left = floor(cameraPosition.x / mEntitySize);
    int top = floor(cameraPosition.y / mEntitySize);
    int right =
        std::min(int(ceil((cameraPosition.x + cameraSize.x) / mEntitySize)),
                 mColumnCount - 1);
    int bottom =
        std::min(int(ceil((cameraPosition.y + cameraSize.y) / mEntitySize)),
                 mRowCount - 1);

    int startX = -int(fmod(cameraPosition.x, mEntitySize));
    int startY = -int(fmod(cameraPosition.y, mEntitySize));

    SDL_Rect r;
    r.w = mEntitySize;
    r.h = mEntitySize;

    SDL_Color color;
    color.r = 255;
    color.g = 0;
    color.b = 0;
    color.a = 192;

    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
    for (int row = top; row <= bottom; row++) {
      for (int column = left; column <= right; column++) {
        const Node &node = mGrid[row][column];
        r.x = startX + (column - left) * mEntitySize;
        r.y = startY + (row - top) * mEntitySize;
        if (node.lineOfSight)
          SDL_SetRenderDrawColor(renderer, 255, 255, 0, 64);
        else if (node.isWalkable)
          SDL_SetRenderDrawColor(renderer, 255, 255, 255, 64);
        else
          SDL_SetRenderDrawColor(renderer, 255, 0, 0, 64);
        SDL_RenderFillRect(renderer, &r);
        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 192);
        SDL_RenderDrawRect(renderer, &r);

        Vec2 direction = getDirection(NodePosition{row, column}, {0, 0});
        Vec2 arrowFrom = {r.x + mEntitySize / 2.0, r.y + mEntitySize / 2.0};
        direction.scale(mEntitySize / 3.0);
        Vec2 arrowTo{arrowFrom};
        arrowTo.add(direction, 1);

        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
        SDL_RenderDrawLine(renderer, arrowFrom.x, arrowFrom.y, arrowTo.x,
                           arrowTo.y);

        std::string text = node.cost != std::numeric_limits<int>::max()
                               ? std::to_string(node.cost)
                               : "INFTY";
        SDL_Surface *sf = TTF_RenderText_Solid(font, text.data(), color);
        SDL_Texture *texture = SDL_CreateTextureFromSurface(renderer, sf);
        SDL_Rect textRect;
        SDL_QueryTexture(texture, nullptr, nullptr, &textRect.w, &textRect.h);
        textRect.x = r.x;
        textRect.y = r.y;

        SDL_RenderCopy(renderer, texture, nullptr, &textRect);

        SDL_DestroyTexture(texture);
        SDL_FreeSurface(sf);
      }
    }
  }

  NodePosition getNodePositionFromWorldPosition(const Vec2 &position) {
    return {std::clamp(int(position.y / mEntitySize), 0, mRowCount - 1),
            std::clamp(int(position.x / mEntitySize), 0, mColumnCount - 1)};
  }

  std::vector<NodePosition> getNeighbours(const NodePosition &position) {
    std::vector<NodePosition> neighbours;

    for (int rowShift = -1; rowShift <= 1; rowShift++) {
      for (int columnShift = -1; columnShift <= 1; columnShift++) {
        if (rowShift == 0 && columnShift == 0)
          continue;

        NodePosition newPos{position.first + rowShift,
                            position.second + columnShift};

        if (newPos.first < 0 || newPos.first >= mRowCount)
          continue;
        if (newPos.second < 0 || newPos.second >= mColumnCount)
          continue;

        neighbours.push_back(newPos);
      }
    }

    return neighbours;
  }

  std::vector<NodePosition> edgeOffsets = {{0, -1}, {-1, 0}, {1, 0}, {0, 1}};

  std::vector<NodePosition> getEdges(const NodePosition &position) {
    std::vector<NodePosition> neighbours;

    for (auto [offsetFirst, offsetSecond] : edgeOffsets) {
      NodePosition newPos{position.first + offsetFirst,
                          position.second + offsetSecond};

      if (newPos.first < 0 || newPos.first >= mRowCount)
        continue;
      if (newPos.second < 0 || newPos.second >= mColumnCount)
        continue;

      neighbours.push_back(newPos);
    }

    return neighbours;
  }

  std::vector<Vec2> getNeighbourObstacle(const Vec2 &position) {
    NodePosition nodePosition = getNodePositionFromWorldPosition(position);

    std::vector<Vec2> obstacle;

    if (!mGrid[nodePosition.first][nodePosition.second].isWalkable)
      obstacle.emplace_back(nodePosition.second * mEntitySize + mEntitySize / 2,
                            nodePosition.first * mEntitySize + mEntitySize / 2);

    for (const NodePosition &neighbour : getNeighbours(nodePosition)) {
      if (mGrid[neighbour.first][neighbour.second].isWalkable)
        continue;

      obstacle.emplace_back(neighbour.second * mEntitySize + mEntitySize / 2,
                            neighbour.first * mEntitySize + mEntitySize / 2);
    }

    return obstacle;
  }

  Vec2 getDirection(const Vec2 &position, const Vec2 &direction) {
    return getDirection(getNodePositionFromWorldPosition(position), direction);
  }

  Vec2 getDirection(const NodePosition &nodePosition, const Vec2 &direction) {
    int currentCost = mGrid[nodePosition.first][nodePosition.second].cost;
    bool considerDirection = direction.length() > 0;
    if (currentCost == 0)
      return {0, 0};

    int minCost = std::numeric_limits<int>::max();
    Vec2 result{0, 0};
    double maxDot = -1;

    for (const NodePosition &neighbour : getNeighbours(nodePosition)) {
      if (!canWalk(nodePosition, neighbour))
        continue;

      const Node &neighbourNode = mGrid[neighbour.first][neighbour.second];

      int cost = neighbourNode.cost;

      if (cost >= currentCost)
        continue;

      Vec2 force{double(neighbour.second - nodePosition.second),
                 double(neighbour.first - nodePosition.first)};
      force.normalize();
      double dot = force.dot(direction);

      if ((!considerDirection && cost < minCost) ||
          (considerDirection &&
           (dot > maxDot || (dot == maxDot && cost < minCost)))) {
        result = force;
        minCost = cost;
        maxDot = dot;
      }
    }

    return result;
  }

  bool addDirectionToSteering(const Vec2 &position, const Vec2 &direction,
                              ContextSteeringMap &map, double scale) {
    return addDirectionToSteering(getNodePositionFromWorldPosition(position),
                                  direction, map, scale);
  }

  bool addDirectionToSteering(const NodePosition &nodePosition,
                              const Vec2 &direction, ContextSteeringMap &map,
                              double scale) {
    int currentCost = mGrid[nodePosition.first][nodePosition.second].cost;

    if (currentCost == std::numeric_limits<int>::max())
      return false;

    Vec2 result{0, 0};
    bool success = false;

    for (const NodePosition &neighbour : getNeighbours(nodePosition)) {
      if (!canWalk(nodePosition, neighbour))
        continue;

      const Node &neighbourNode = mGrid[neighbour.first][neighbour.second];

      int cost = neighbourNode.cost;

      if (cost >= currentCost)
        continue;

      success = true;

      Vec2 force{double(neighbour.second - nodePosition.second),
                 double(neighbour.first - nodePosition.first)};
      force.normalize();
      double dot = force.dot(direction);
      dot = (dot + 1.5) / 2.5;
      force.scale(dot * scale);
      map.addVector(force);
  GAMEENTITY_HPP_  }

    return success;
  }

  int getDistance(const NodePosition &from, const NodePosition &to) {
    int deltaRow = std::abs(from.first - to.first);
    int deltaColumn = std::abs(from.second - to.second);

    if (deltaColumn > deltaRow)
      return 14 * deltaRow + 10 * (deltaColumn - deltaRow);

    return 14 * deltaColumn + 10 * (deltaRow - deltaColumn);
  }

  bool canWalk(const NodePosition &from, const NodePosition &to) {
    if (!mGrid[to.first][to.second].isWalkable)
      return false;

    int deltaRow = to.first - from.first;
    int deltaColumn = to.second - from.second;

    if (deltaRow == 0 || deltaColumn == 0)
      return true;

    if (abs(deltaRow) > 1 || abs(deltaColumn) > 1)
      throw std::invalid_argument("node is too far");

    return mGrid[from.first][to.second].isWalkable &&
           mGrid[to.first][from.second].isWalkable;
  }

  void calculateLineOfSight(const NodePosition &from, const NodePosition &to) {
    const double deltaFirst = to.first - from.first;
    const double deltaSecond = to.second - from.second;

    const double deltaFirstAbs = std::abs(deltaFirst);
    const double deltaSecondAbs = std::abs(deltaSecond);

    const double deltaFirstSign = std::copysign(1, deltaFirst);
    const double deltaSecondSign = std::copysign(1, deltaSecond);

    bool hasLineOfSight = false;

    if (deltaFirstAbs >= deltaSecondAbs) {
      if (mGrid[from.first + deltaFirstSign][from.second].lineOfSight)
        hasLineOfSight = true;
    }

    if (deltaSecondAbs >= deltaFirstAbs) {
      if (mGrid[from.first][from.second + deltaSecondSign].lineOfSight)
        hasLineOfSight = true;
    }

    if (deltaFirstAbs > 0 && deltaSecondAbs > 0) {
      if (!mGrid[from.first + deltaFirstSign][from.second + deltaSecondSign]
               .lineOfSight)
        hasLineOfSight = false;
      else if (deltaFirstAbs == deltaSecondAbs) {
        if (!mGrid[from.first + deltaFirstSign][from.second].isWalkable ||
            !mGrid[from.first][from.second + deltaSecondSign].isWalkable ==
                std::numeric_limits<int>::max()) {
          hasLineOfSight = false;
        }
      }
    }

    mGrid[from.first][from.second].lineOfSight = hasLineOfSight;
    if (!mGrid[to.first][to.second].lineOfSight) {
      SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "Aneh");
    }
  }

  void generateHeatmap(const Vec2 &target) {
    const NodePosition targetNodePos = getNodePositionFromWorldPosition(target);

    for (std::vector<Node> &row : mGrid) {
      for (Node &cell : row) {
        cell.cost = std::numeric_limits<int>::max();
        cell.lineOfSight = false;
      }
    }

    if (!mGrid[targetNodePos.first][targetNodePos.second].isWalkable)
      return;

    std::vector<NodePosition> openSet;
    std::set<NodePosition> closedSet;

    mGrid[targetNodePos.first][targetNodePos.second].cost = 0;
    mGrid[targetNodePos.first][targetNodePos.second].lineOfSight = true;
    openSet.push_back(targetNodePos);

    while (!openSet.empty()) {
      NodePosition currentPos = openSet.front();
      openSet.erase(openSet.begin());
      closedSet.insert(currentPos);

      Node &currentNode = mGrid[currentPos.first][currentPos.second];

      if (currentPos != targetNodePos) {
        calculateLineOfSight(currentPos, targetNodePos);
      }

      if (currentNode.cost >= 10)
        continue;

      for (const NodePosition &neighbour : getEdges(currentPos)) {
        Node &node = mGrid[neighbour.first][neighbour.second];

        if (!canWalk(currentPos, neighbour) || closedSet.count(neighbour) > 0)
          continue;

        int newCost = currentNode.cost + 1;

        bool isQueued =
            std::count(openSet.begin(), openSet.end(), neighbour) > 0;
        if (newCost < node.cost || !isQueued) {
          node.cost = newCost;
          node.parentPosition = currentPos;
          if (!isQueued)
            openSet.push_back(neighbour);
        }
      }
    }
  }

  bool hasLineOfSigh(const Vec2 &position) {
    const NodePosition node = getNodePositionFromWorldPosition(position);

    return mGrid[node.first][node.second].lineOfSight;
  }
};

std::optional<Vec2> rayCircleIntersection(const Vec2 &point, const Vec2 &ray,
                                          const Vec2 &circle, double radius) {
  Vec2 u{circle};
  u.substract(point);

  double dot = u.dot(ray);

  if (dot < 0)
    return std::nullopt;

  Vec2 u1 = u.projectInto(ray, false);

  Vec2 u2{u};
  u2.substract(u1);

  double d = u2.length();

  if (d > radius)
    return std::nullopt;
  if (std::abs(d - radius) < 0.1)
    return std::nullopt;

  double m = std::sqrt(radius * radius - d * d);

  Vec2 p{u1};
  p.add(ray, -m);

  return std::optional(p);
}

double mapValue(double value, double minValue, double maxValue,
                double minOutput, double maxOutput) {
  return minOutput +
         (maxOutput - minOutput) * ((value - minValue) / (maxValue - minValue));
}

class Enemy : public GameEntity {
public:
  double speed;
  Vec2 direction{0, 0};
  Vec2 smoothedDirection{0, 0};
  Vec2 acceleration{0, 0};
  Uint32 lastFire = 0;
  ContextSteering contextSteering;
  Vec2 contextSteeringResult{0, 0};
  std::vector<GameEntity *> nearEntity;

  Enemy(TextureLoader *textureLoader, const Vec2 &position)
      : GameEntity(position, 0) {
    texture = textureLoader->load("/home/levirs565/Unduhan/SpaceShooterRedux/"
                                  "PNG/Enemies/enemyBlack1.png");
  }

  void onPreTick() override {}

  void onTick(IGameStage *stage) override {
    nearEntity = stage->getSAP()->queryArea(
        position.x - 3 * boundingRadius, position.y - 3 * boundingRadius,
        position.x + 3 * boundingRadius, position.y + 3 * boundingRadius,
        false);

    contextSteering.clear();

    Vec2 velocity = direction;
    velocity.scale(speed);

    bool canAttack = false;
    Vec2 extraRotation{0, 0};

    Vec2 distanceVector{stage->getPlayerPosition()};
    distanceVector.substract(position);
    const double distance = distanceVector.length();

    const double clampedRadius = 2 * boundingRadius + 25;
    const double minRayLength = 1 * boundingRadius + 25;
    const double maxRayLength = 2 * boundingRadius;
    const double minCos = std::cos(45.0 * M_PI / 180.0);

    for (GameEntity *entity : nearEntity) {
      if (entity == this)
        continue;
      if (dynamic_cast<Laser *>(entity) != nullptr)
        continue;

      Vec2 distanceVec{entity->position};
      distanceVec.substract(position);

      if (distanceVec.length() < clampedRadius) {
        Vec2 avoid{distanceVec};
        avoid.normalize();
        contextSteering.dangerMap.addVector(avoid);
      }

      for (const auto &[_, index] : contextSteering.dangerMap.withIndex()) {
        Vec2 ray = ContextSteeringMap::directionBy(index);
        auto intersection = rayCircleIntersection(
            position, ray, entity->position, entity->boundingRadius);
        if (intersection.has_value()) {
          Vec2 avoid = intersection.value();
          double length = avoid.length();

          avoid.normalize();
          avoid.scale(mapValue(std::clamp(length, minRayLength, maxRayLength),
                               minRayLength, maxRayLength, 1.0, 0.0));
          contextSteering.dangerMap.addVector(avoid, 0);
        }
      }
    }

    bool hasLineOfSight = stage->getPathFinder()->hasLineOfSigh(position);
    if (distance > 400 || !hasLineOfSight) {
      bool pathFindSuccess = stage->getPathFinder()->addDirectionToSteering(
          position, direction, contextSteering.interestMap,
          hasLineOfSight ? 0.5 : 1);

      if (hasLineOfSight || !pathFindSuccess) {
        Vec2 seekDirection{distanceVector};
        seekDirection.normalize();

        contextSteering.interestMap.addVector(seekDirection);
      }
    }

    if (distance < 600) {
      Vec2 distanceNormalized{distanceVector};
      distanceNormalized.normalize();

      auto intersection =
          rayCircleIntersection(position, direction, stage->getPlayerPosition(),
                                stage->getPlayerEntity()->boundingRadius);
      if (intersection.has_value()) {
        Vec2 playerPosition = stage->getPlayerPosition();
        canAttack = true;

        Vec2 perpendicularDistance{distanceNormalized};
        perpendicularDistance.makePerpendicular();

        for (int perpendicularShift : {-5, 0, 5}) {
          Vec2 from{position};
          from.add(perpendicularDistance, perpendicularShift);

          Vec2 to{playerPosition};
          to.add(perpendicularDistance, perpendicularShift);

          for (SAPRay ray =
                   stage->getSAP()->queryRay(from.x, from.y, to.x, to.y);
               ray.next();) {
            if (ray.currentEntity == this) {
              continue;
            }
            if (dynamic_cast<PlayerShip *>(ray.currentEntity) != nullptr) {
              break;
            }
            if (dynamic_cast<Laser *>(ray.currentEntity) != nullptr) {
              break;
            }
            canAttack = false;
            break;
          }

          if (!canAttack)
            break;
        }
      }

      extraRotation = distanceNormalized;
    }

    contextSteeringResult = contextSteering.getResult();

    Vec2 desiredVelocity{contextSteeringResult};
    desiredVelocity.normalize();
    desiredVelocity.scale(2);
    desiredVelocity.substract(velocity);
    if (desiredVelocity.length() > 0.1) {
      desiredVelocity.normalize();
      desiredVelocity.scale(0.1);
    }

    acceleration = desiredVelocity;

    Vec2 newVelocity{velocity};
    newVelocity.add(acceleration, 1);

    double newSpeed = std::min(newVelocity.length(), 2.0);

    Vec2 newDirection = newSpeed != 0 ? newVelocity : direction;

    if (extraRotation.length() > 0 && contextSteeringResult.length() == 0) {
      newDirection.rotate(newDirection.orientedAngleTo(extraRotation));
    }

    newDirection.normalize();

    const double maxDeltaAngle = 5.0 / 180.0 * M_PI;
    const double deltaAngle = direction.orientedAngleTo(newDirection);
    const double absDeltaAngle = abs(deltaAngle);

    if (absDeltaAngle > maxDeltaAngle) {
      newDirection = direction;
      newDirection.rotate(std::copysign(maxDeltaAngle, deltaAngle));

      newSpeed = std::copysign(abs(newVelocity.dot(newDirection)), newSpeed);
    }

    newVelocity = newDirection;
    newVelocity.scale(newSpeed);

    position.add(newVelocity, 1);

    speed = newSpeed;
    direction = newDirection;

    Vec2 last = smoothedDirection;
    Vec2 deltaDirection{direction};
    deltaDirection.substract(smoothedDirection);
    smoothedDirection.add(deltaDirection, 0.15);

    angle = smoothedDirection.getRotation() * 180.0 / M_PI - 90;

    if (SDL_GetTicks() - lastFire >= 1000 && canAttack) {
      SDL_Rect enemyRect = getRect();
      Vec2 laserPos(0, enemyRect.h);
      double laserAngle = direction.getRotation() * 180.0 / M_PI - 90;
      laserPos.rotate((laserAngle)*M_PI / 180.0);
      laserPos.add(position, 1);
      stage->addLaser(laserPos, laserAngle - 180);
      lastFire = SDL_GetTicks();
    }

    updateBoundingBox();
  }

  void onDraw(SDL_Renderer *renderer, const Vec2 &cameraPosition) override {
    GameEntity::onDraw(renderer, cameraPosition);

    Vec2 onCameraPosition{position};
    onCameraPosition.substract(cameraPosition);
    contextSteering.draw(renderer, onCameraPosition, boundingRadius);

    Vec2 steeringLine{contextSteeringResult};
    steeringLine.normalize();
    steeringLine.scale(boundingRadius * 1.5);
    steeringLine.add(onCameraPosition, 1);
    SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255);
    SDL_RenderDrawLine(renderer, onCameraPosition.x, onCameraPosition.y,
                       steeringLine.x, steeringLine.y);
  }

  void onHit(GameEntity *other) override {
    if (Laser *laser = dynamic_cast<Laser *>(other); laser != nullptr) {
      mustGone = true;
    } else if (dynamic_cast<Meteor *>(other) != nullptr) {
      // mustGone = true;
    }
  }
};

class App : public IGameStage {
public:
  App() {
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
      std::cout << "Initializing SDL failed" << std::endl;
      exit(1);
    }
    IMG_Init(IMG_INIT_JPG | IMG_INIT_PNG);
    if (Mix_OpenAudio(44100, MIX_DEFAULT_FORMAT, 2, 1024) == 1) {
      std::cout << "Initializing mixer failed" << std::endl;
      exit(1);
    }

    mWindow = SDL_CreateWindow("Space", SDL_WINDOWPOS_UNDEFINED,
                               SDL_WINDOWPOS_UNDEFINED, 800, 600, 0);

    if (!mWindow) {
      std::cout << "Initializing window failed" << std::endl;
      exit(1);
    }

    SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "linear");

    mRenderer = SDL_CreateRenderer(mWindow, -1, SDL_RENDERER_ACCELERATED);

    if (!mRenderer) {
      std::cout << "Initializing renderer failed" << std::endl;
      exit(1);
    }

    if (TTF_Init() < 0) {
      std::cout << "TTF Init failed" << std::endl;
      exit(1);
    }

    mTextureLoader = std::make_unique<TextureLoader>(mRenderer);
    mBackgroundTexture = mTextureLoader->load(
        "/home/levirs565/Unduhan/SpaceShooterRedux/Backgrounds/black.png");

    mLaserSound = Mix_LoadWAV(
        "/home/levirs565/Unduhan/SpaceShooterRedux/Bonus/sfx_laser1.ogg");

    std::unique_ptr<PlayerShip> playerShip =
        std::make_unique<PlayerShip>(mTextureLoader.get(), Vec2(400, 700));
    mPlayerShip = playerShip.get();
    addEntity(std::move(playerShip));

    addEntity(
        std::move(std::make_unique<Enemy>(mTextureLoader.get(), Vec2(100, 0))));
    addEntity(
        std::move(std::make_unique<Enemy>(mTextureLoader.get(), Vec2(300, 0))));
    addEntity(
        std::move(std::make_unique<Enemy>(mTextureLoader.get(), Vec2(800, 0))));

    addEntity(std::move(std::make_unique<Meteor>(
        mTextureLoader.get(), Vec2(100, 500), "Brown_big1")));
    addEntity(std::move(std::make_unique<Meteor>(
        mTextureLoader.get(), Vec2(300, 500), "Brown_big2")));
    addEntity(std::move(std::make_unique<Meteor>(
        mTextureLoader.get(), Vec2(600, 500), "Brown_big3")));
    addEntity(std::move(std::make_unique<Meteor>(
        mTextureLoader.get(), Vec2(615, 1000), "Brown_big3")));
    mPathFinder.init(mWordSize, 110);

    mPathFinder.clearState();
    for (std::vector<std::unique_ptr<GameEntity>>::iterator it =
             mEntityList.begin();
         it != mEntityList.end(); it++) {
      std::unique_ptr<GameEntity> &entity = *it;
      if (dynamic_cast<Meteor *>(entity.get()) == nullptr)
        continue;
      mPathFinder.addObstacle(entity->position, entity->boundingRadius);
    }

    mPathFinder.generateHeatmap(mPlayerShip->position);
  }

  void addEntity(std::unique_ptr<GameEntity> &&entity) {
    GameEntity *ptr = entity.get();
    mEntityList.push_back(std::move(entity));
    mSAP.add(ptr);
  }

  void processKeyDown(const SDL_KeyboardEvent &key) {
    if (key.repeat != 0)
      return;

    if (key.keysym.scancode == SDL_SCANCODE_UP)
      mIsUp = true;

    if (key.keysym.scancode == SDL_SCANCODE_DOWN)
      mIsDown = true;

    if (key.keysym.scancode == SDL_SCANCODE_LEFT)
      mIsLeft = true;

    if (key.keysym.scancode == SDL_SCANCODE_RIGHT)
      mIsRight = true;

    if (key.keysym.scancode == SDL_SCANCODE_LCTRL)
      mIsFire = true;
  }

  void processKeyUp(const SDL_KeyboardEvent &key) {
    if (key.repeat != 0)
      return;

    if (key.keysym.scancode == SDL_SCANCODE_UP)
      mIsUp = false;

    if (key.keysym.scancode == SDL_SCANCODE_DOWN)
      mIsDown = false;

    if (key.keysym.scancode == SDL_SCANCODE_LEFT)
      mIsLeft = false;

    if (key.keysym.scancode == SDL_SCANCODE_RIGHT)
      mIsRight = false;

    if (key.keysym.scancode == SDL_SCANCODE_LCTRL)
      mIsFire = false;
  }

  void processInput() {
    SDL_Event event;

    while (SDL_PollEvent(&event)) {
      switch (event.type) {
      case SDL_QUIT:
        exit(0);
        break;
      case SDL_KEYDOWN:
        processKeyDown(event.key);
        break;
      case SDL_KEYUP:
        processKeyUp(event.key);
        break;
      default:
        break;
      }
    }
  }

  void prepareScene() {
    SDL_SetRenderDrawColor(mRenderer, 96, 128, 255, 255);
    SDL_RenderClear(mRenderer);
  }

  void presentScene() { SDL_RenderPresent(mRenderer); }

  void calculateCamera() {
    mCameraPosition.x = SDL_clamp(mPlayerShip->position.x - mCameraSize.x / 2,
                                  0, mWordSize.x - mCameraSize.x);
    mCameraPosition.y = SDL_clamp(mPlayerShip->position.y - mCameraSize.y / 2,
                                  0, mWordSize.y - mCameraSize.y);
  }

  void drawBackground() {
    SDL_Rect rect;
    SDL_QueryTexture(mBackgroundTexture, nullptr, nullptr, &rect.w, &rect.h);

    double backgroundStartY = -fmod(mCameraPosition.y, double(rect.h));
    double backgroundStartX = -fmod(mCameraPosition.x, double(rect.w));
    int backgroundCountY =
        int(ceil((mCameraSize.y - backgroundStartY) / double(rect.h)));
    int backgroundCountX =
        int(ceil((mCameraSize.x - backgroundStartX) / double(rect.w)));

    for (int backgroundRow = 0; backgroundRow < backgroundCountY;
         backgroundRow++) {
      for (int backgroundColumn = 0; backgroundColumn < backgroundCountX;
           backgroundColumn++) {
        rect.x = int(backgroundStartX + backgroundColumn * rect.w);
        rect.y = int(backgroundStartY + backgroundRow * rect.h);
        SDL_RenderCopy(mRenderer, mBackgroundTexture, nullptr, &rect);
      }
    }
  }

  void run() {
    while (true) {
      prepareScene();
      processInput();

      mPlayerShip->setDirection(mIsUp     ? PlayerShip::DIRECTION_UP
                                : mIsDown ? PlayerShip::DIRECTION_DOWN
                                          : PlayerShip::DIRECTION_NONE,
                                mIsLeft    ? PlayerShip::ROTATION_LEFT
                                : mIsRight ? PlayerShip::ROTATION_RIGHT
                                           : PlayerShip::ROTATION_NONE);

      for (auto &entity : mEntityList) {
        entity->onPreTick();
      }

      size_t entityCount = mEntityList.size();
      for (size_t i = 0; i < entityCount; i++) {
        std::unique_ptr<GameEntity> &entity = mEntityList[i];
        entity->onTick(this);
        // setelah onTick, jangan gunakan entity kembali karena ada kemungkinan
        // penambahan elemen ke mEntityList yang menyebabkan operasi std::move
        // terhadap entity sehingga entity berada dalam keadaan invalid
      }

      for (std::unique_ptr<GameEntity> &entity : mEntityList) {
        mSAP.move(entity.get());
      }

      for (std::vector<std::unique_ptr<GameEntity>>::iterator it =
               mEntityList.begin();
           it != mEntityList.end(); it++) {
        std::unique_ptr<GameEntity> &entity = *it;

        if (entity->mustGone) {
          mSAP.remove(entity.get());
          it = mEntityList.erase(it) - 1;
          continue;
        }
      }

      mSAP.update();

      calculateCamera();
      drawBackground();

      if (mIsFire)
        mPlayerShip->doFire(this);

      for (auto &[entity, collisionSet] : mSAP.getCollisionMap()) {
        for (auto otherEntity : collisionSet) {
          if (isPolygonCollide(entity->boundingBox, otherEntity->boundingBox)) {
            entity->onHit(otherEntity);
          }
        }
      }

      SDL_SetRenderDrawColor(mRenderer, 0, 255, 0, 255);

      for (GameEntity *entity :
           mSAP.queryArea(mCameraPosition.x, mCameraPosition.y,
                          mCameraPosition.x + mCameraSize.x,
                          mCameraPosition.y + mCameraSize.y, false)) {
        entity->onDraw(mRenderer, mCameraPosition);

        for (size_t i = 0; i < entity->boundingBox.size(); i++) {
          Vec2 current = entity->boundingBox[i];
          Vec2 next = entity->boundingBox[(i + 1) % entity->boundingBox.size()];

          current.substract(mCameraPosition);
          next.substract(mCameraPosition);
          SDL_RenderDrawLine(mRenderer, int(current.x), int(current.y),
                             int(next.x), int(next.y));
        }
      }

      // mPathFinder.drawGrid(mRenderer, mCameraPosition, mCameraSize);
      mPathFinder.generateHeatmap(mPlayerShip->position);

      presentScene();
      SDL_Delay(16);
    }
  }

  const Vec2 &getPlayerPosition() override { return mPlayerShip->position; }

  GameEntity *getPlayerEntity() override { return mPlayerShip; }

  void addLaser(const Vec2 &position, double angle) override {
    std::unique_ptr<Laser> laser =
        std::make_unique<Laser>(mTextureLoader.get(), position, angle);
    addEntity(std::move(laser));
    Mix_PlayChannel(1, mLaserSound, 0);
  }

  const Vec2 &getWorldSize() override { return mWordSize; }

  std::vector<std::unique_ptr<GameEntity>> &getEntities() override {
    return mEntityList;
  }

  Vec2 getFlowDirection(const Vec2 &position, const Vec2 &direction) override {
    return mPathFinder.getDirection(position, direction);
  }

  std::vector<Vec2> findNeighbourObstacle(const Vec2 &position) override {
    return mPathFinder.getNeighbourObstacle(position);
  }

  APathFinder *getPathFinder() { return &mPathFinder; }

  SAP *getSAP() { return &mSAP; }

private:
  SDL_Renderer *mRenderer;
  SDL_Window *mWindow;
  SDL_Texture *mBackgroundTexture;
  PlayerShip *mPlayerShip;
  std::unique_ptr<TextureLoader> mTextureLoader;
  std::vector<std::unique_ptr<GameEntity>> mEntityList;
  APathFinder mPathFinder;
  Vec2 mCameraSize{800, 600};
  Vec2 mCameraPosition{0, 0};
  Vec2 mWordSize{5000, 5000};
  Mix_Chunk *mLaserSound;
  bool mIsUp = false;
  bool mIsLeft = false;
  bool mIsDown = false;
  bool mIsRight = false;
  bool mIsFire = false;
  SAP mSAP;
};

int main() {
  App app;
  app.run();
  return 0;
}