#include "FlowField.hpp"

void APathFinder::init(const Vec2 &worldSize, int entitySize) {
  mRowCount = ceil(worldSize.y / entitySize);
  mColumnCount = ceil(worldSize.x / entitySize);
  mEntitySize = entitySize;
  mGrid.clear();
  mGrid.resize(mRowCount, std::vector<Node>(mColumnCount));
}

void APathFinder::clearState() {
  for (std::vector<Node> &row : mGrid) {
    for (Node &cell : row) {
      cell.isWalkable = true;
    }
  }
}

void APathFinder::addObstacle(const Vec2 &centerPosition, int radius) {
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

void APathFinder::drawGrid(SDL_Renderer *renderer, const Vec2 &cameraPosition,
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

std::vector<APathFinder::NodePosition> APathFinder::getNeighbours(const NodePosition &position) {
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

std::vector<APathFinder::NodePosition> APathFinder::getEdges(const NodePosition &position) {
  static std::vector<NodePosition> edgeOffsets = {{0, -1}, {-1, 0}, {1, 0}, {0, 1}};

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

std::vector<Vec2> APathFinder::getNeighbourObstacle(const Vec2 &position) {
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

Vec2 APathFinder::getDirection(const NodePosition &nodePosition, const Vec2 &direction) {
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

bool APathFinder::addDirectionToSteering(const NodePosition &nodePosition,
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
  }

  return success;
}

int APathFinder::getDistance(const NodePosition &from, const NodePosition &to) {
  int deltaRow = std::abs(from.first - to.first);
  int deltaColumn = std::abs(from.second - to.second);

  if (deltaColumn > deltaRow)
    return 14 * deltaRow + 10 * (deltaColumn - deltaRow);

  return 14 * deltaColumn + 10 * (deltaRow - deltaColumn);
}

bool APathFinder::canWalk(const NodePosition &from, const NodePosition &to) {
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

void APathFinder::calculateLineOfSight(const NodePosition &from, const NodePosition &to) {
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

void APathFinder::generateHeatmap(const Vec2 &target) {
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