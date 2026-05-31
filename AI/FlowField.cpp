#include "FlowField.hpp"
#include "../AssetManager.hpp"
#include "../TextRenderer.hpp"
#include <algorithm>
#include <map>

void FlowField::init(const Vec2 &worldSize, int entitySize) {
  mRowCount = ceil(worldSize.y / entitySize);
  mColumnCount = ceil(worldSize.x / entitySize);
  mEntitySize = entitySize;
  mGrid.clear();
  mGrid.resize(mRowCount, std::vector<Node>(mColumnCount));
}

void FlowField::addObstacle(GameEntity *entity) {
  mObstacleMap[entity] = {.left = 0, .right = 0, .top = 0, .bottom = 0};
  moveObstacle(entity);
}

void FlowField::moveObstacle(GameEntity *entity) {
  ObstacleData &data = mObstacleMap[entity];

  for (int row = data.top; row < data.bottom; row++) {
    for (int column = data.left; column < data.right; column++) {
      mGrid[row][column].obstacleCount -= 1;
    }
  }

  data.left = floor(
      std::clamp(entity->x0 / mEntitySize, 0.0, double(mColumnCount) - 1));
  data.top =
      floor(std::clamp(entity->y0 / mEntitySize, 0.0, double(mRowCount) - 1));
  data.right = int(
      ceil(std::clamp(entity->x1 / mEntitySize, 0.0, double(mColumnCount))));
  data.bottom =
      int(ceil(std::clamp(entity->y1 / mEntitySize, 0.0, double(mRowCount))));

  for (int row = data.top; row < data.bottom; row++) {
    for (int column = data.left; column < data.right; column++) {
      mGrid[row][column].obstacleCount += 1;
    }
  }
}

void FlowField::drawGrid(SDL_Renderer *renderer, const Mat3 &viewMatrix,
                         Vec2 &viewSize) {
  static TTF_Font *font =
      FontManager::getInstance()->load("Bonus/kenvector_future.ttf", 16);

  Mat3 inverse = viewMatrix.affineInverse();
  Vec3 topLeft = inverse * Vec2(0, 0);
  Vec3 bottomRight = inverse * Vec2(viewSize);

  int left = floor(topLeft.x / mEntitySize);
  int top = floor(topLeft.y / mEntitySize);
  int right =
      std::min(int(ceil(bottomRight.x / mEntitySize)), mColumnCount - 1);
  int bottom = std::min(int(ceil(bottomRight.y / mEntitySize)), mRowCount - 1);

  Vec2 scaling = viewMatrix.getScale();
  SDL_FRect r;
  r.w = mEntitySize * scaling.x;
  r.h = mEntitySize * scaling.y;

  SDL_Color color;
  color.r = 255;
  color.g = 0;
  color.b = 0;
  color.a = 192;

  SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);

  std::map<std::string, std::unique_ptr<TextRenderer>> rendererMap;

  for (int row = top; row <= bottom; row++) {
    for (int column = left; column <= right; column++) {
      const Node &node = mGrid[row][column];

      Vec2 worldPosition = Vec2(column * mEntitySize, row * mEntitySize);
      Vec2 position = (viewMatrix * worldPosition).toCartesian();

      r.x = position.x;
      r.y = position.y;
      if (node.lineOfSight)
        SDL_SetRenderDrawColor(renderer, 255, 255, 0, 64);
      else if (node.isWalkable())
        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 64);
      else
        SDL_SetRenderDrawColor(renderer, 255, 0, 0, 64);
      SDL_RenderRect(renderer, &r);
      SDL_SetRenderDrawColor(renderer, 255, 255, 255, 192);
      SDL_RenderRect(renderer, &r);

      Vec2 direction = getDirection(NodePosition{row, column}, {0, 0});
      Vec2 arrowFromWorld =
          worldPosition + 0.5 * Vec2(mEntitySize, mEntitySize);
      Vec2 arrowToWorld = arrowFromWorld + mEntitySize / 3 * direction;

      Vec2 arrowFrom = (viewMatrix * arrowFromWorld).toCartesian();
      Vec2 arrowTo = (viewMatrix * arrowToWorld).toCartesian();

      SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
      SDL_RenderLine(renderer, arrowFrom.x, arrowFrom.y, arrowTo.x, arrowTo.y);

      std::string text = node.cost != std::numeric_limits<int>::max()
                             ? std::to_string(node.cost)
                             : "INFTY";

      if (!rendererMap.contains(text)) {
        auto textRenderer = std::make_unique<TextRenderer>(font, SDL_Color{.r = 255, .g = 255, .b = 255, .a = 255});
        textRenderer->setText(text);
        rendererMap[text] = std::move(textRenderer);
      }

      SDL_Texture *texture = rendererMap[text]->getTexture(renderer);
      SDL_FRect textRect;
      SDL_GetTextureSize(texture, &textRect.w, &textRect.h);
      textRect.w *= scaling.x;
      textRect.h *= scaling.y;
      textRect.x = r.x;
      textRect.y = r.y;

      SDL_RenderTexture(renderer, texture, nullptr, &textRect);
    }
  }
}

std::vector<FlowField::NodePosition>
FlowField::getNeighbours(const NodePosition &position) {
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

std::vector<FlowField::NodePosition>
FlowField::getEdges(const NodePosition &position) {
  static std::vector<NodePosition> edgeOffsets = {
      {0, -1}, {-1, 0}, {1, 0}, {0, 1}};

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

Vec2 FlowField::getDirection(const NodePosition &nodePosition,
                             const Vec2 &direction) {
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

bool FlowField::addDirectionToSteering(const NodePosition &nodePosition,
                                       const Vec2 &direction,
                                       ContextSteeringMap &map, double scale) {
  int currentCost = mGrid[nodePosition.first][nodePosition.second].cost;

  if (currentCost == std::numeric_limits<int>::max())
    return false;

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
  }

  return success;
}

bool FlowField::canWalk(const NodePosition &from, const NodePosition &to) {
  if (!mGrid[to.first][to.second].isWalkable())
    return false;

  int deltaRow = to.first - from.first;
  int deltaColumn = to.second - from.second;

  if (deltaRow == 0 || deltaColumn == 0)
    return true;

  if (abs(deltaRow) > 1 || abs(deltaColumn) > 1)
    throw std::invalid_argument("node is too far");

  return mGrid[from.first][to.second].isWalkable() &&
         mGrid[to.first][from.second].isWalkable();
}

void FlowField::calculateLineOfSight(const NodePosition &from,
                                     const NodePosition &to) {
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
      if (!mGrid[from.first + deltaFirstSign][from.second].isWalkable() ||
          !mGrid[from.first][from.second + deltaSecondSign].isWalkable() ==
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

void FlowField::generateHeatmap(const Vec2 &target) {
  const NodePosition targetNodePos = getNodePositionFromWorldPosition(target);

  for (std::vector<Node> &row : mGrid) {
    for (Node &cell : row) {
      cell.cost = std::numeric_limits<int>::max();
      cell.lineOfSight = false;
    }
  }

  if (!mGrid[targetNodePos.first][targetNodePos.second].isWalkable())
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

      bool isQueued = std::count(openSet.begin(), openSet.end(), neighbour) > 0;
      if (newCost < node.cost || !isQueued) {
        node.cost = newCost;
        if (!isQueued)
          openSet.push_back(neighbour);
      }
    }
  }
}
