#include <iostream>
#include <SDL.h>
#include <SDL_image.h>
#include <SDL_mixer.h>
#include <SDL_ttf.h>
#include <cmath>
#include <vector>
#include <unordered_map>
#include <memory>
#include <functional>
#include <set>
#include <exception>
#include <queue>

class TextureLoader {
public:
    explicit TextureLoader(SDL_Renderer *renderer) : mRenderer(renderer) {
    }

    TextureLoader(const TextureLoader &other) = delete;

    SDL_Texture *load(const std::string &name) {
        if (mCache.count(name) > 0)
            return mCache[name];

        SDL_LogMessage(SDL_LOG_CATEGORY_APPLICATION, SDL_LOG_PRIORITY_INFO, "Loading texture %s", name.c_str());
        SDL_Texture *texture = IMG_LoadTexture(mRenderer, name.c_str());
        mCache[name] = texture;
        return texture;
    }

private:
    SDL_Renderer *mRenderer;
    std::unordered_map<std::string, SDL_Texture *> mCache;
};

class Vec2 {
public:
    double x;
    double y;

    Vec2(double x, double y) : x(x), y(y) {

    }

    void rotate(double radian) {
        double cos = SDL_cos(radian);
        double sin = SDL_sin(radian);
        double nextX = x * cos - y * sin;
        double nextY = x * sin + y * cos;

        x = nextX;
        y = nextY;
    }

    void rotateAround(double radian, const Vec2 &center) {
        substract(center);
        rotate(radian);
        add(center, 1);
    }

    void add(const Vec2 &other, double scale) {
        x += scale * other.x;
        y += scale * other.y;
    }

    void substract(const Vec2 &other) {
        x -= other.x;
        y -= other.y;
    }

    void scale(double factor) {
        x *= factor;
        y *= factor;
    }

    double length() const {
        return SDL_sqrt(x * x + y * y);
    }

    double getRotation() {
        return SDL_atan2(y, x);
    }

    void normalize() {
        double l = length();
        if (l == 0) return;
        x /= l;
        y /= l;
    }

    void makePerpendicular() {
        std::swap(x, y);
        x *= -1;
    }

    double dot(const Vec2 &other) const {
        return x * other.x + y * other.y;
    }

    Vec2 projectInto(const Vec2 &other, bool clamp) const {
        Vec2 axis = other;
        axis.normalize();
        double projectionLength = dot(axis);
        if (clamp && projectionLength < 0) projectionLength = 0;
        else if (clamp && projectionLength > other.length()) projectionLength = other.length();
        axis.scale(projectionLength);
        return axis;
    }

    inline Vec2 parallelComponent(const Vec2 &basis) const {
        return projectInto(basis, false);
    }

    Vec2 perpendicularComponent(const Vec2 &basis) const {
        Vec2 copy{*this};
        copy.substract(parallelComponent(basis));
        return copy;
    }

    double angleBetween(const Vec2 &other) const {
        return std::acos(dot(other) / length() / other.length());
    }

    double orientedAngleTo(const Vec2 &other) const {
        return std::atan2(x * other.y - y * other.x, dot(other));
    }

    /**
     * basis must be vector with length 1
    */
    Vec2 limitMaxDeviationCos(const Vec2 &basis, const double maxCos) {
        Vec2 currentDirection{*this};
        currentDirection.normalize();

        double currentCosAngle = currentDirection.dot(basis);

        if (currentCosAngle >= maxCos) return *this;


        Vec2 perpendicular = perpendicularComponent(basis);
        perpendicular.normalize();
        perpendicular.scale(std::sqrt(1 - maxCos*maxCos));

        Vec2 result{basis};
        result.scale(currentCosAngle);
        result.add(perpendicular, 1);
        result.scale(length());

        return result;
    }
};

std::pair<double, double> findPolygonProjectionMinMax(std::vector<Vec2> &polygon, Vec2 target) {
    double minProjection = std::numeric_limits<double>::max();
    double maxProjection = std::numeric_limits<double>::min();

    for (const Vec2 &vertex: polygon) {
        const double projection = vertex.dot(target);
        minProjection = std::min(minProjection, projection);
        maxProjection = std::max(maxProjection, projection);
    }

    return {minProjection, maxProjection};
}

bool isPolygonCollideInternal(std::vector<Vec2> &polygonA, std::vector<Vec2> &polygonB) {
    for (size_t i = 0; i < polygonA.size(); i++) {
        const Vec2 &currentVertex = polygonA.at(i);
        const Vec2 &nextVertex = polygonA.at((i + 1) % polygonA.size());

        Vec2 normal(nextVertex);
        normal.substract(currentVertex);
        normal.makePerpendicular();
        normal.normalize();

        auto [minProjectionA, maxProjectionA] = findPolygonProjectionMinMax(polygonA, normal);
        auto [minProjectionB, maxProjectionB] = findPolygonProjectionMinMax(polygonB, normal);

        if (maxProjectionA < minProjectionB || maxProjectionB < minProjectionA) return false;
    }

    return true;
}

bool isPolygonCollide(std::vector<Vec2> &polygonA, std::vector<Vec2> &polygonB) {
    return isPolygonCollideInternal(polygonA, polygonB) && isPolygonCollideInternal(polygonB, polygonA);
}

bool lineIntersectCircle(const Vec2 &nonAhead, const Vec2 &ahead, const Vec2 &ahead2, double myRadius,
                         const Vec2 &circlePosition,
                         double circleRadius) {
    Vec2 pureDistance(circlePosition);
    pureDistance.substract(nonAhead);

    Vec2 distanceAhead(circlePosition);
    distanceAhead.substract(ahead);

    Vec2 distanceAhead2(circlePosition);
    distanceAhead2.substract(ahead2);

    return pureDistance.length() <= circleRadius + myRadius || distanceAhead.length() <= circleRadius + myRadius ||
           distanceAhead2.length() <= circleRadius + myRadius;
}

class GameEntity;

class IGameStage {
public:
    virtual const Vec2 &getPlayerPosition() = 0;

    virtual void addLaser(const Vec2 &position, double angle) = 0;

    virtual const Vec2 &getWorldSize() = 0;

    virtual std::vector<std::unique_ptr<GameEntity>> &getEntities() = 0;

    virtual Vec2 getFlowDirection(const Vec2 &position) = 0;

    virtual std::vector<Vec2> findNeighbourObstacle(const Vec2 &position) = 0;
};


class GameEntity {
public:
    SDL_Texture *texture;
    Vec2 position;
    double angle;
    bool mustGone = false;
    std::vector<Vec2> boundingBox;
    double boundingRadius = 0;

    GameEntity(const Vec2 &position, double angle) : position(position), angle(angle) {}

    SDL_Rect getRect() const {
        SDL_Rect r;
        SDL_QueryTexture(texture, nullptr, nullptr, &r.w, &r.h);
        r.x = int(position.x - r.w / 2);
        r.y = int(position.y - r.h / 2);
        return r;
    }

    virtual void onPreTick() {

    }
    virtual void onTick(IGameStage *stage) = 0;

    void drawTexture(SDL_Renderer *renderer, const Vec2 &cameraPosition, SDL_Texture *texture) {
        SDL_Rect rect;

        SDL_QueryTexture(texture, nullptr, nullptr, &rect.w, &rect.h);
        rect.x = int(position.x - cameraPosition.x - double(rect.w) / 2);
        rect.y = int(position.y - cameraPosition.y - double(rect.h) / 2);

        SDL_RenderCopyEx(renderer, texture, nullptr, &rect, angle, nullptr, SDL_FLIP_NONE);
    }

    virtual void onDraw(SDL_Renderer *renderer, const Vec2 &cameraPosition) {
        drawTexture(renderer, cameraPosition, texture);
    };

    virtual void onHit(GameEntity *other) {

    }

    void updateBoundingBox() {
        boundingBox.clear();

        int width, height;
        SDL_QueryTexture(texture, nullptr, nullptr, &width, &height);
        double halfWidth = double(width) / 2;
        double halfHeight = double(height) / 2;

        boundingRadius = hypot(halfWidth, halfHeight);

        Vec2 topRight(position.x + halfWidth, position.y - halfHeight);
        Vec2 bottomRight{position.x + halfWidth, position.y + halfHeight};
        Vec2 bottomLeft{position.x - halfWidth, position.y + halfHeight};
        Vec2 topLeft{position.x - halfWidth, position.y - halfHeight};

        double radianAngle = angle * M_PI / 180.0;
        topRight.rotateAround(radianAngle, position);
        bottomRight.rotateAround(radianAngle, position);
        bottomLeft.rotateAround(radianAngle, position);
        topLeft.rotateAround(radianAngle, position);

        boundingBox.push_back(topRight);
        boundingBox.push_back(bottomRight);
        boundingBox.push_back(bottomLeft);
        boundingBox.push_back(topLeft);
    }
};

class Laser : public GameEntity {
public:
    Vec2 directionVector{0, -1};

    Laser(TextureLoader *textureLoader, const Vec2 &position, double angle) : GameEntity(position, angle) {
        texture = textureLoader->load("/home/levirs565/Unduhan/SpaceShooterRedux/PNG/Lasers/laserBlue01.png");
        directionVector.rotate(angle * M_PI / 180.0);
        updateBoundingBox();
    }

    void onTick(IGameStage *stage) override {
        position.add(directionVector, 7.5);

        updateBoundingBox();
    }

    void onHit(GameEntity *other) override {
        mustGone = true;
    }
};

class Meteor : public GameEntity {
public:
    explicit Meteor(TextureLoader *textureLoader, const Vec2 &position, const std::string &type)
            : GameEntity(position, 0) {
        texture = textureLoader->load("/home/levirs565/Unduhan/SpaceShooterRedux/PNG/Meteors/meteor" + type + ".png");
        updateBoundingBox();
    }

    void onTick(IGameStage *stage) override {

    }
};

class PlayerShip : public GameEntity {
public:
    enum Direction {
        DIRECTION_UP,
        DIRECTION_DOWN,
        DIRECTION_NONE
    };

    enum Rotation {
        ROTATION_LEFT,
        ROTATION_RIGHT,
        ROTATION_NONE
    };

    Vec2 directionVector{0, 0};
    Uint32 lastFire = 0;
    int healthCount = 4;
    std::vector<SDL_Texture *> damagedTexture;

    explicit PlayerShip(TextureLoader *textureLoader, const Vec2 &position) : GameEntity(position, 25) {
        texture = textureLoader->load("/home/levirs565/Unduhan/SpaceShooterRedux/PNG/playerShip3_blue.png");
        damagedTexture.push_back(textureLoader->load(
                "/home/levirs565/Unduhan/SpaceShooterRedux/PNG/Damage/playerShip3_damage1.png"));
        damagedTexture.push_back(textureLoader->load(
                "/home/levirs565/Unduhan/SpaceShooterRedux/PNG/Damage/playerShip3_damage2.png"));
        damagedTexture.push_back(textureLoader->load(
                "/home/levirs565/Unduhan/SpaceShooterRedux/PNG/Damage/playerShip3_damage3.png"));
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
        for (std::vector<Node> &row: mGrid) {
            for (Node &cell: row) {
                cell.isWalkable = true;
            }
        }
    }

    void addObstacle(const Vec2 &centerPosition, int radius) {
        int left = floor(std::max((centerPosition.x - radius) / mEntitySize - 0.5, 0.0));
        int top = floor(std::max((centerPosition.y - radius) / mEntitySize - 0.5, 0.0));
        int right = int(round(std::max((centerPosition.x + radius) / mEntitySize + 0.5, 0.0)));
        int bottom = int(round(std::max((centerPosition.y + radius) / mEntitySize + 0.5, 0.0)));

        if (left > mColumnCount || right > mColumnCount) return;
        if (top > mRowCount || bottom > mRowCount) return;

        for (int row = top; row < bottom; row++) {
            for (int column = left; column < right; column++) {
                mGrid[row][column].isWalkable = false;
            }
        }
    }

    void drawGrid(SDL_Renderer *renderer, const Vec2 &cameraPosition, Vec2 &cameraSize) {
        static TTF_Font *font = TTF_OpenFont(
                "/home/levirs565/Unduhan/kenney_space-shooter-redux/Bonus/kenvector_future.ttf", 16);

        int left = floor(cameraPosition.x / mEntitySize);
        int top = floor(cameraPosition.y / mEntitySize);
        int right = std::min(int(ceil((cameraPosition.x + cameraSize.x) / mEntitySize)), mColumnCount - 1);
        int bottom = std::min(int(ceil((cameraPosition.y + cameraSize.y) / mEntitySize)), mRowCount - 1);

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
                if (node.isWalkable)
                    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 64);
                else
                    SDL_SetRenderDrawColor(renderer, 255, 0, 0, 64);
                SDL_RenderFillRect(renderer, &r);
                SDL_SetRenderDrawColor(renderer, 255, 255, 255, 192);
                SDL_RenderDrawRect(renderer, &r);

                Vec2 direction = getDirection(NodePosition{row, column});
                Vec2 arrowFrom = {r.x + mEntitySize / 2.0, r.y + mEntitySize / 2.0};
                direction.scale(mEntitySize / 3.0);
                Vec2 arrowTo{arrowFrom};
                arrowTo.add(direction, 1);

                SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
                SDL_RenderDrawLine(renderer, arrowFrom.x, arrowFrom.y, arrowTo.x, arrowTo.y);

                std::string text = node.cost != std::numeric_limits<int>::max() ? std::to_string(node.cost) : "INFTY";
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
        return {
                std::clamp(int(position.y / mEntitySize), 0, mRowCount - 1),
                std::clamp(int(position.x / mEntitySize), 0, mColumnCount - 1)
        };
    }

    std::vector<NodePosition> getNeighbours(const NodePosition &position) {
        std::vector<NodePosition> neighbours;

        for (int rowShift = -1; rowShift <= 1; rowShift++) {
            for (int columnShift = -1; columnShift <= 1; columnShift++) {
                if (rowShift == 0 && columnShift == 0) continue;

                NodePosition newPos{
                        position.first + rowShift,
                        position.second + columnShift
                };

                if (newPos.first < 0 || newPos.first >= mRowCount) continue;
                if (newPos.second < 0 || newPos.second >= mColumnCount) continue;

                neighbours.push_back(newPos);
            }
        }

        return neighbours;
    }

    std::vector<NodePosition> edgeOffsets = {
            {0,  -1},
            {-1, 0},
            {1,  0},
            {0,  1}
    };

    std::vector<NodePosition> getEdges(const NodePosition &position) {
        std::vector<NodePosition> neighbours;

        for (auto [offsetFirst, offsetSecond]: edgeOffsets) {
            NodePosition newPos{
                    position.first + offsetFirst,
                    position.second + offsetSecond
            };

            if (newPos.first < 0 || newPos.first >= mRowCount) continue;
            if (newPos.second < 0 || newPos.second >= mColumnCount) continue;

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

        for (const NodePosition &neighbour: getNeighbours(nodePosition)) {
            if (mGrid[neighbour.first][neighbour.second].isWalkable) continue;

            obstacle.emplace_back(neighbour.second * mEntitySize + mEntitySize / 2,
                                  neighbour.first * mEntitySize + mEntitySize / 2);
        }

        return obstacle;
    }

    Vec2 getDirection(const Vec2 &position) {
        return getDirection(getNodePositionFromWorldPosition(position));
    }

    Vec2 getDirection(const NodePosition &nodePosition) {
        int currentCost = mGrid[nodePosition.first][nodePosition.second].cost;

        if (currentCost == 0) return {0, 0};

        NodePosition min = nodePosition;
        int minCost = std::numeric_limits<int>::max();

        for (const NodePosition &neighbour: getNeighbours(nodePosition)) {
            if (!canWalk(nodePosition, neighbour)) continue;

            const Node &neighbourNode = mGrid[neighbour.first][neighbour.second];

            int cost = neighbourNode.cost - currentCost;

            if (cost < minCost) {
                min = neighbour;
                minCost = cost;
            }
        }

        if (minCost != std::numeric_limits<int>::max()) {
            Vec2 force{
                    double(min.second - nodePosition.second),
                    double(min.first - nodePosition.first)
            };
            force.normalize();
            return force;
        }

        return {0, 0};
    }

    int getDistance(const NodePosition &from, const NodePosition &to) {
        int deltaRow = std::abs(from.first - to.first);
        int deltaColumn = std::abs(from.second - to.second);

        if (deltaColumn > deltaRow)
            return 14 * deltaRow + 10 * (deltaColumn - deltaRow);

        return 14 * deltaColumn + 10 * (deltaRow - deltaColumn);
    }

    bool canWalk(const NodePosition &from, const NodePosition &to) {
        if (!mGrid[to.first][to.second].isWalkable) return false;

        int deltaRow = to.first - from.first;
        int deltaColumn = to.second - from.second;

        if (deltaRow == 0 || deltaColumn == 0) return true;

        if (abs(deltaRow) > 1 || abs(deltaColumn) > 1)
            throw std::invalid_argument("node is too far");

        return mGrid[from.first][to.second].isWalkable &&
               mGrid[to.first][from.second].isWalkable;
    }

    void generateHeatmap(const Vec2 &target) {
        const NodePosition targetNodePos = getNodePositionFromWorldPosition(target);

        if (!mGrid[targetNodePos.first][targetNodePos.second].isWalkable) return;

        for (std::vector<Node> &row: mGrid) {
            for (Node &cell: row) {
                cell.cost = std::numeric_limits<int>::max();
            }
        }

        std::vector<NodePosition> openSet;
        std::set<NodePosition> closedSet;

        mGrid[targetNodePos.first][targetNodePos.second].cost = 0;
        openSet.push_back(targetNodePos);

        while (!openSet.empty()) {
            NodePosition currentPos = openSet.front();
            openSet.erase(openSet.begin());
            closedSet.insert(currentPos);

            Node &currentNode = mGrid[currentPos.first][currentPos.second];

            if (currentNode.cost >= 10) continue;

            for (const NodePosition &neighbour: getEdges(currentPos)) {
                Node &node = mGrid[neighbour.first][neighbour.second];

                if (!canWalk(currentPos, neighbour) || closedSet.count(neighbour) > 0)
                    continue;

                int newCost = currentNode.cost + 1;

                bool isQueued = std::count(openSet.begin(), openSet.end(), neighbour) > 0;
                if (newCost < node.cost || !isQueued) {
                    node.cost = newCost;
                    node.parentPosition = currentPos;
                    if (!isQueued)
                        openSet.push_back(neighbour);
                }
            }
        }
    }
};

namespace SteeringBehaviour {
    Vec2
    makeArrival(const Vec2 &from, const Vec2 &to, const Vec2 &steering, const Vec2 &velocity, double slowingDistance,
                double stopRadius) {
        Vec2 delta = Vec2(to);
        delta.substract(from);
        double distance = delta.length();

        if (distance < slowingDistance + stopRadius) {
            Vec2 desiredVelocity = velocity;
            desiredVelocity.add(steering, 1);
            desiredVelocity.scale((std::max(distance - stopRadius, 0.0)) / slowingDistance);

            Vec2 newSteering = desiredVelocity;
            newSteering.substract(velocity);
            return newSteering;
        }

        return steering;
    }

    Vec2 seek(const Vec2 &from, const Vec2 &to, const Vec2 &currentVelocity, double maxVelocity, double maxSteering) {
        Vec2 desiredVelocity = Vec2(to);
        desiredVelocity.substract(from);

        desiredVelocity.normalize();
        desiredVelocity.scale(maxVelocity);

        Vec2 steering = Vec2(desiredVelocity);
        steering.substract(currentVelocity);

        if (steering.length() > maxSteering) {
            steering.normalize();
            steering.scale(maxSteering);
        }

        return steering;
    }

    Vec2 flee(const Vec2 &from, const Vec2 &to, const Vec2 &currentVelocity, double maxVelocity) {
        Vec2 desiredVelocity = Vec2(from);
        desiredVelocity.substract(to);

        desiredVelocity.normalize();
        desiredVelocity.scale(maxVelocity);

        Vec2 steering = Vec2(desiredVelocity);
        steering.substract(currentVelocity);
        return steering;
    }

    Vec2 applyOnlyNear(const Vec2 &from, const Vec2 &to, const Vec2 &steering, double maxDistance) {
        Vec2 delta = Vec2(to);
        delta.substract(from);
        double distance = delta.length();

        if (distance >= maxDistance) return {0, 0};

        return steering;
    }

    Vec2
    separation(GameEntity *currentEntity, const std::vector<GameEntity *> &othersEntity, const Vec2 &currentVelocity,
               const Vec2 &currentDirection,
               double separationDistance, double maxSpeed, double maxForce) {
        Vec2 desiredVelocity{0, 0};
        int total = 0;
        for (GameEntity *other: othersEntity) {
            Vec2 distanceVec(other->position);
            distanceVec.substract(currentEntity->position);
            double distance = distanceVec.length();
            double angle = distanceVec.angleBetween(currentDirection);

            if (distance < separationDistance && angle < 3 * M_PI_4) {
                distanceVec.scale(-1);
                distanceVec.normalize();
                distanceVec.scale(1 / distance);

                desiredVelocity.add(distanceVec, 1);
                total++;
            }
        }

        if (total > 0) {
            desiredVelocity.scale(1.0 / total);
            desiredVelocity.normalize();
            desiredVelocity.scale(maxSpeed);

            Vec2 steering = Vec2(desiredVelocity);
            steering.substract(currentVelocity);

            if (steering.length() > maxForce) {
                steering.normalize();
                steering.scale(maxForce);
            }

            return steering;
        }
        return {0, 0};
    }

    Vec2
    separation(const Vec2 &currentPosition, const std::vector<Vec2> &othersPosition, const Vec2 &currentVelocity,
               double separationDistance, double maxSpeed, double maxForce) {
        Vec2 desiredVelocity{0, 0};
        int total = 0;
        for (const Vec2 &other: othersPosition) {
            Vec2 distanceVec(other);
            distanceVec.substract(currentPosition);
            double distance = distanceVec.length();

            if (distance < separationDistance) {
                distanceVec.scale(-1);
                distanceVec.normalize();
                distanceVec.scale(1 / distance);

                desiredVelocity.add(distanceVec, 1);
                total++;
            }
        }

        if (total > 0) {
            desiredVelocity.scale(1.0 / total);
            desiredVelocity.normalize();
            desiredVelocity.scale(maxSpeed);

            Vec2 steering = Vec2(desiredVelocity);
            steering.substract(currentVelocity);

            if (steering.length() > maxForce) {
                steering.normalize();
                steering.scale(maxForce);
            }

            return steering;
        }
        return {0, 0};
    }

    Vec2
    singleSeparation(const Vec2 &from, double fromRadius, const Vec2 &to, double toRadius, double separationDistance) {
        Vec2 distanceVec(to);
        distanceVec.substract(from);
        double distance = distanceVec.length() - fromRadius - toRadius;

        if (distance < separationDistance && separationDistance - distance > 0.1) {
            distanceVec.scale(-1);
            distanceVec.normalize();
            distanceVec.scale(separationDistance / distance);

            return distanceVec;
        }

        return {0, 0};
    }

    Vec2 pathFollowing(const Vec2 &currentPosition, const std::vector<Vec2> &path, int &currentNode,
                       const Vec2 &currentVelocity, double maxVelocity, double maxSteering) {
        if (!path.empty()) {
            const double nextTime = 25;
            Vec2 future = currentVelocity;
            future.normalize();
            future.scale(nextTime);
            future.add(currentPosition, 1);

            Vec2 projection{0, 0};
            Vec2 target{0, 0};
            double minDistance = std::numeric_limits<double>::max();

            for (size_t i = 0; i < path.size() - 1; i++) {
                const Vec2 &currentPoint = path[i];
                const Vec2 &nextPoint = path[i + 1];

                Vec2 futureFromCurrent{future};
                futureFromCurrent.substract(currentPoint);
                Vec2 nextFromCurrent{nextPoint};
                nextFromCurrent.substract(currentPoint);

                Vec2 futureProjection = futureFromCurrent.projectInto(nextFromCurrent, true);
                futureProjection.add(currentPoint, 1);

                Vec2 distanceVec = future;
                distanceVec.substract(futureProjection);
                double distance = distanceVec.length();

                if (distance < minDistance) {
                    currentNode = i;
                    minDistance = distance;
                    projection = futureProjection;

                    Vec2 direction{nextPoint};
                    direction.substract(currentPoint);
                    direction.normalize();
                    direction.scale(10);

                    target = futureProjection;
                    target.add(direction, 1);
                }
            }

            if (minDistance > 55 && minDistance != std::numeric_limits<double>::max()) {
                return seek(currentPosition, target, currentVelocity, maxVelocity, maxSteering);
            }
        }

        return {0, 0};
    }

    Vec2 followField(const Vec2 &direction, const Vec2 &currentVelocity, double maxVelocity, double maxSteering) {
        Vec2 desiredVelocity = direction;

        desiredVelocity.normalize();
        desiredVelocity.scale(maxVelocity);

        Vec2 steering = Vec2(desiredVelocity);
        steering.substract(currentVelocity);

        if (steering.length() > maxSteering) {
            steering.normalize();
            steering.scale(maxSteering);
        }

        return steering;
    }
}

class Enemy : public GameEntity {
public:
    double speed;
    Vec2 direction{0, 0};
    Vec2 acceleration{0, 0};
    Uint32 lastFire = 0;
    Uint32 lastUpdatePath = 0;
    std::vector<Enemy *> othersEnemy;
    std::vector<Vec2> steeringList;
    double lastSide = 0;
    const Enemy *lastThreat = nullptr;
    const Enemy *avoidedEnemy = nullptr;

    Enemy(TextureLoader *textureLoader, const Vec2 &position) : GameEntity(position, 0) {
        texture = textureLoader->load("/home/levirs565/Unduhan/SpaceShooterRedux/PNG/Enemies/enemyBlack1.png");
    }

    Vec2 steerAvoidCloseNeighbors(const double minSeparationDistance, const std::vector<Enemy *> &enemyList) {
        for (const Enemy *enemy: enemyList) {
            const double radiusSum = boundingRadius + enemy->boundingRadius;
            const double minCenterDistance = radiusSum + minSeparationDistance;

            Vec2 distance{enemy->position};
            distance.substract(position);

            if (distance.length() < minCenterDistance) {
                distance.scale(-1);
                return distance.perpendicularComponent(direction);
            }
        }

        return {0, 0};
    }

    double predictNearestApproachTime(const Enemy *enemy) {
        Vec2 enemyVelocity{enemy->direction};
        enemyVelocity.scale(enemy->speed);

        Vec2 velocity{direction};
        velocity.scale(speed);

        Vec2 relativeVelocity{enemyVelocity};
        relativeVelocity.substract(velocity);
        const double relativeSpeed = relativeVelocity.length();

        if (relativeSpeed == 0) return 0;

        relativeVelocity.normalize();

        Vec2 relativePosition{position};
        relativePosition.substract(enemy->position);
        const double projection = relativeVelocity.dot(relativePosition);

        return projection / relativeSpeed;
    }

    double
    computeNearestApproachPosition(const Enemy *enemy, double time, Vec2 &myFinalPosition, Vec2 &otherFinalPosition) {
        myFinalPosition = direction;
        myFinalPosition.scale(speed * time);
        myFinalPosition.add(position, 1);

        otherFinalPosition = enemy->direction;
        otherFinalPosition.scale(enemy->speed * time);
        otherFinalPosition.add(enemy->position, 1);

        Vec2 distance(myFinalPosition);
        distance.substract(otherFinalPosition);

        return distance.length();
    }

    Vec2 steerAvoidNeighbors(const double minTimeToCollision, const std::vector<Enemy *> &enemyList) {
        const Vec2 closeNeighborsSteer = steerAvoidCloseNeighbors(0, enemyList);
        if (closeNeighborsSteer.length() > 0) return closeNeighborsSteer;

        const double collisionDangerThreshold = boundingRadius * 2;
        double minTime = minTimeToCollision;
        const Enemy *threat = nullptr;
        Vec2 myNearestPosition{0, 0};
        Vec2 threatNearestPosition{0, 0};

        for (const Enemy *enemy: enemyList) {
            const double time = predictNearestApproachTime(enemy);

            if (time >= 0 && time < minTime) {
                Vec2 tempMyNearestPosition{0, 0};
                Vec2 tempThreatNearestPosition{0, 0};
                if (computeNearestApproachPosition(enemy, time, tempMyNearestPosition, tempThreatNearestPosition) <
                    collisionDangerThreshold) {
                    threat = enemy;
                    minTime = time;
                    myNearestPosition = tempMyNearestPosition;
                    threatNearestPosition = tempThreatNearestPosition;
                }
            }
        }

        Vec2 sideVector = direction;
        sideVector.makePerpendicular();

        double steer = 0;
        if (threat != nullptr && lastSide != 0) {
            steer = lastSide;
        } else if (threat != nullptr) {
            double paralellness = direction.dot(threat->direction);
            double angle = 0.707;

            if (paralellness < -angle) {
                Vec2 offset{threatNearestPosition};
                offset.substract(position);
                double sideDot = offset.dot(sideVector);
                steer = sideDot > 0 ? -1 : 1;
            } else if (paralellness > angle) {
                Vec2 offset{threat->position};
                offset.substract(position);
                double sideDot = offset.dot(sideVector);
                steer = sideDot > 0 ? -1 : 1;
            } else {
                if (threat->speed <= speed) {
                    Vec2 velocity{threat->direction};
                    if (threat->speed > 0)
                        velocity.scale(threat->speed);
                    double sideDot = sideVector.dot(velocity);
                    steer = sideDot > 0 ? -1 : 1;
                }
            }
        }
        if (steer != 0)
            SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "Steer %f", steer);
//        lastSide = steer;
        lastThreat = threat;

        sideVector.scale(steer);
        return sideVector;
    }

    void onPreTick() override {
        avoidedEnemy = nullptr;
    }

    void onTick(IGameStage *stage) override {
        Vec2 velocity = direction;
        velocity.scale(speed);

        bool canAttack = false;

        Vec2 distanceVector{stage->getPlayerPosition()};
        distanceVector.substract(position);
        const double distance = distanceVector.length();

        Vec2 steering{0, 0};
        Vec2 directionLock{0, 0};
        bool cannotAvoidance = distance < 250;
        bool canFollowField = true;

        if (distance < 250) {
            directionLock = distanceVector;
            directionLock.normalize();
        }

        steeringList.clear();

        othersEnemy.clear();
        for (const std::unique_ptr<GameEntity> &entity: stage->getEntities()) {
            if (entity.get() == this) continue;

            if (Enemy *otherEnemy = dynamic_cast<Enemy *>(entity.get()); otherEnemy != nullptr) {
                othersEnemy.push_back(otherEnemy);
            }
        }

        if (distance > 225 && distance < 250) {
            if (speed != 0) {
                steering = direction;
                steering.scale(-1 * std::clamp(speed, -0.1, 0.1));
                steeringList.push_back(steering);
            }
        } else {
            Vec2 field = stage->getFlowDirection(position);
            if (distance < 225)
                field.scale(-1);


            steering = SteeringBehaviour::followField(field, velocity, 2, 0.1);
            if (field.length() == 0) {
                steering = distanceVector;
                steering.normalize();
                steering.scale(-0.1);
            }

            const double minNeighbourDistance = boundingRadius + 10;
            const double maxNeighbourDistance = 4 * boundingRadius;
            const double maxNeigbourAngle = 135.0 * M_PI / 180.0;
            const double maxRejectAngle = M_PI_2;
            double minDistance = std::numeric_limits<double>::max();

            for (const auto enemy : othersEnemy) {
                Vec2 distanceVec = enemy->position;
                distanceVec.substract(position);
                const double distance = distanceVec.length();

                if (enemy->avoidedEnemy == this) continue;

                if (distance > maxNeighbourDistance)
                    continue;

                if (distance > minNeighbourDistance && 
                    direction.angleBetween(distanceVec) > maxNeigbourAngle)
                    continue;

                if (distanceVec.angleBetween(field) < maxRejectAngle) {
                    minDistance = distance;
                    avoidedEnemy = enemy;
                    canFollowField = false;
                }
            }

            if (canFollowField)
                steeringList.push_back(steering);
            else steering = {0, 0};
        }

        //steering = SteeringBehaviour::makeArrival(position, stage->getPlayerPosition(), steering, velocity, 100, 200);

        if (!cannotAvoidance) {
            const Enemy *oldThreat = lastThreat;
            Vec2 steering2 = steerAvoidNeighbors(120, othersEnemy);
            if (steering2.length() > 0.5) {
                steering2.normalize();
                steering2.scale(0.5);
            }
            steeringList.push_back(steering2);
            if (steering2.length() > 0) {
                steering = steering2;
                directionLock = {0, 0};
            }

            if (lastThreat == nullptr) lastThreat = oldThreat;
        }
//        steering.add(steering2, 1.5);

//        Vec2 steering3 = SteeringBehaviour::separation(position, stage->findNeighbourObstacle(position), velocity, 55, 2, 0.1);
//        steeringList.push_back(steering3);
//        steering.add(steering3,1.5);


        acceleration = steering;

        Vec2 newVelocity{velocity};
        newVelocity.add(acceleration, 1);

        double newSpeed = std::min(newVelocity.length(), 2.0);

        Vec2 newDirection = newSpeed != 0 ? newVelocity : direction;
        newDirection.normalize();

//        if (directionLock.length() > 0) {
//            const double oldNewSpeed = newSpeed;
//            newSpeed = directionLock.dot(newDirection) * abs(newSpeed);
//            newDirection = directionLock;
////            SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "New speed %f, old %f", newSpeed, oldNewSpeed);
//        }

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

//        if (abs(direction.orientedAngleTo(newDirection) * 180 / M_PI) > 5.5)
//            SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "Delta rotation exceed limit %f",
//                        direction.orientedAngleTo(newDirection) * 180 / M_PI);

        speed = newSpeed;
        direction = newDirection;

        angle = direction.getRotation() * 180.0 / M_PI - 90;

        if (SDL_GetTicks() - lastFire >= 1000 && canAttack) {
            SDL_Rect enemyRect = getRect();
            Vec2 laserPos(0, enemyRect.h);
            laserPos.rotate((angle) * M_PI / 180.0);
            laserPos.add(position, 1);
            stage->addLaser(laserPos, angle - 180);
            lastFire = SDL_GetTicks();
        }

        updateBoundingBox();
    }

    void onDraw(SDL_Renderer *renderer, const Vec2 &cameraPosition) override {
        GameEntity::onDraw(renderer, cameraPosition);

        Vec2 lineStart{position};
        lineStart.substract(cameraPosition);

        SDL_SetRenderDrawColor(renderer, 0, 255, 255, 255);

        for (const Vec2 &steering: steeringList) {
            Vec2 lineEnd{steering};
            lineEnd.normalize();
            lineEnd.scale(40);
            lineEnd.add(position, 1);
            lineEnd.substract(cameraPosition);

            SDL_RenderDrawLine(renderer, lineStart.x, lineStart.y, lineEnd.x, lineEnd.y);
        }

       static TTF_Font * font = TTF_OpenFont("/home/levirs565/Unduhan/kenney_space-shooter-redux/Bonus/kenvector_future.ttf", 16);
       SDL_Color color;
       color.r = 255;
       color.g = 255;
       color.b = 255;
       color.a = 255;
       Vec2 posInCamera{position};
       posInCamera.substract(cameraPosition);


        const double minNeighbourDistance = boundingRadius + 10;
        const double maxNeighbourDistance = 4 * boundingRadius;
        const double maxNeigbourAngle = 135.0 * M_PI / 180.0;

       for (GameEntity * other : othersEnemy) {
           Vec2 distanceVec(other->position);
           distanceVec.substract(position);
           double distance = distanceVec.length();
           double angle = distanceVec.angleBetween(direction);

            if (distance > maxNeighbourDistance)
                continue;

            if (distance > minNeighbourDistance && 
               angle > maxNeigbourAngle)
                continue;

            Vec2 projection = distanceVec.projectInto(direction, false);
            projection.add(position, 1);
            projection.substract(cameraPosition);

            distanceVec.scale(-1);
            distanceVec.normalize();
            distanceVec.scale(1 / distance);


            Vec2 otherInCamera{other->position};
            otherInCamera.substract(cameraPosition);

            SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
            SDL_RenderDrawLine(renderer, posInCamera.x, posInCamera.y, otherInCamera.x, otherInCamera.y);
            SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255);
            SDL_RenderDrawLine(renderer, posInCamera.x, posInCamera.y, projection.x, projection.y);
            SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255);
            SDL_RenderDrawLine(renderer, otherInCamera.x, otherInCamera.y, projection.x, projection.y);

            std::string text = std::to_string(angle);
            SDL_Surface* surface = TTF_RenderText_Solid(font, text.c_str(), color);
            SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer, surface);
            SDL_Rect textRect;
            SDL_QueryTexture(texture, nullptr, nullptr, &textRect.w, &textRect.h);
            textRect.x = (otherInCamera.x + projection.x) / 2.0 - textRect.w;
            textRect.y = (otherInCamera.y + projection.y) / 2.0 - textRect.h;
            SDL_RenderCopy(renderer, texture, nullptr, &textRect);

            SDL_DestroyTexture(texture);
            SDL_FreeSurface(surface);
       }
    }

    void onHit(GameEntity *other) override {
        if (Laser *laser = dynamic_cast<Laser *>(other); laser != nullptr) {
            mustGone = true;
        } else if (dynamic_cast<Meteor *>(other) != nullptr) {
            //mustGone = true;
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

        mWindow = SDL_CreateWindow(
                "Space",
                SDL_WINDOWPOS_UNDEFINED,
                SDL_WINDOWPOS_UNDEFINED,
                800,
                600,
                0);

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
        mBackgroundTexture = mTextureLoader->load("/home/levirs565/Unduhan/SpaceShooterRedux/Backgrounds/black.png");

        mLaserSound = Mix_LoadWAV("/home/levirs565/Unduhan/SpaceShooterRedux/Bonus/sfx_laser1.ogg");

        std::unique_ptr<PlayerShip> playerShip = std::make_unique<PlayerShip>(mTextureLoader.get(), Vec2(400, 700));
        mPlayerShip = playerShip.get();
        mEntityList.push_back(std::move(playerShip));

        mEntityList.push_back(std::move(std::make_unique<Enemy>(mTextureLoader.get(), Vec2(100, 0))));
        mEntityList.push_back(std::move(std::make_unique<Enemy>(mTextureLoader.get(), Vec2(300, 0))));
        mEntityList.push_back(std::move(std::make_unique<Enemy>(mTextureLoader.get(), Vec2(600, 0))));

        mEntityList.push_back(std::move(std::make_unique<Meteor>(mTextureLoader.get(), Vec2(100, 500), "Brown_big1")));
        mEntityList.push_back(std::move(std::make_unique<Meteor>(mTextureLoader.get(), Vec2(300, 500), "Brown_big2")));
        mEntityList.push_back(std::move(std::make_unique<Meteor>(mTextureLoader.get(), Vec2(600, 500), "Brown_big3")));
        mEntityList.push_back(std::move(std::make_unique<Meteor>(mTextureLoader.get(), Vec2(615, 1000), "Brown_big3")));
        mPathFinder.init(mWordSize, 110);

        mPathFinder.clearState();
        for (std::vector<std::unique_ptr<GameEntity>>::iterator it = mEntityList.begin();
             it != mEntityList.end(); it++) {
            std::unique_ptr<GameEntity> &entity = *it;
            if (dynamic_cast<Meteor *>(entity.get()) == nullptr) continue;
            mPathFinder.addObstacle(entity->position, entity->boundingRadius);
        }

        mPathFinder.generateHeatmap(mPlayerShip->position);
    }

    void processKeyDown(const SDL_KeyboardEvent &key) {
        if (key.repeat != 0) return;

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
        if (key.repeat != 0) return;

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

    void presentScene() {
        SDL_RenderPresent(mRenderer);
    }

    void calculateCamera() {
        mCameraPosition.x = SDL_clamp(mPlayerShip->position.x - mCameraSize.x / 2, 0, mWordSize.x - mCameraSize.x);
        mCameraPosition.y = SDL_clamp(mPlayerShip->position.y - mCameraSize.y / 2, 0, mWordSize.y - mCameraSize.y);
    }

    void drawBackground() {
        SDL_Rect rect;
        SDL_QueryTexture(mBackgroundTexture, nullptr, nullptr, &rect.w, &rect.h);

        double backgroundStartY = -fmod(mCameraPosition.y, double(rect.h));
        double backgroundStartX = -fmod(mCameraPosition.x, double(rect.w));
        int backgroundCountY = int(
                ceil((mCameraSize.y - backgroundStartY) / double(rect.h)));
        int backgroundCountX = int(
                ceil((mCameraSize.x - backgroundStartX) / double(rect.w)));

        for (int backgroundRow = 0; backgroundRow < backgroundCountY; backgroundRow++) {
            for (int backgroundColumn = 0; backgroundColumn < backgroundCountX; backgroundColumn++) {
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

            mPlayerShip->setDirection(
                    mIsUp ? PlayerShip::DIRECTION_UP
                          : mIsDown ? PlayerShip::DIRECTION_DOWN
                                    : PlayerShip::DIRECTION_NONE,
                    mIsLeft ? PlayerShip::ROTATION_LEFT
                            : mIsRight ? PlayerShip::ROTATION_RIGHT
                                       : PlayerShip::ROTATION_NONE);

            for (auto& entity : mEntityList) {
                entity->onPreTick();
            }

            size_t entityCount = mEntityList.size();
            for (size_t i = 0; i < entityCount; i++) {
                const std::unique_ptr<GameEntity> &entity = mEntityList[i];
                entity->onTick(this);
                // setelah onTick, jangan gunakan entity kembali karena ada kemungkinan penambahan elemen ke mEntityList
                // yang menyebabkan operasi std::move terhadap entity sehingga entity berada dalam keadaan invalid
            }

            calculateCamera();
            drawBackground();

            if (mIsFire)
                mPlayerShip->doFire(this);

            for (std::vector<std::unique_ptr<GameEntity>>::iterator it = mEntityList.begin();
                 it != mEntityList.end(); it++) {
                std::unique_ptr<GameEntity> &entity = *it;

                for (std::vector<std::unique_ptr<GameEntity>>::iterator otherIt = it + 1;
                     otherIt != mEntityList.end(); otherIt++) {
                    std::unique_ptr<GameEntity> &otherEntity = *otherIt;

                    if (isPolygonCollide(entity->boundingBox, otherEntity->boundingBox)) {
                        entity->onHit(otherEntity.get());
                        otherEntity->onHit(entity.get());
                    }
                }
            }

            SDL_SetRenderDrawColor(mRenderer, 0, 255, 0, 255);
            for (std::vector<std::unique_ptr<GameEntity>>::iterator it = mEntityList.begin();
                 it != mEntityList.end(); it++) {
                std::unique_ptr<GameEntity> &entity = *it;

                if (entity->mustGone) {
                    it = mEntityList.erase(it) - 1;
                    continue;
                }

                entity->onDraw(mRenderer, mCameraPosition);

                for (size_t i = 0; i < entity->boundingBox.size(); i++) {
                    Vec2 current = entity->boundingBox[i];
                    Vec2 next = entity->boundingBox[(i + 1) % entity->boundingBox.size()];

                    current.substract(mCameraPosition);
                    next.substract(mCameraPosition);
                    SDL_RenderDrawLine(mRenderer, int(current.x), int(current.y), int(next.x), int(next.y));
                }
            }

            mPathFinder.drawGrid(mRenderer, mCameraPosition, mCameraSize);
            mPathFinder.generateHeatmap(mPlayerShip->position);

            presentScene();
            SDL_Delay(16);
        }
    }

    const Vec2 &getPlayerPosition() override {
        return mPlayerShip->position;
    }

    void addLaser(const Vec2 &position, double angle) override {
        std::unique_ptr<Laser> laser = std::make_unique<Laser>(mTextureLoader.get(), position,
                                                               angle);
        mEntityList.push_back(std::move(laser));
        Mix_PlayChannel(1, mLaserSound, 0);
    }

    const Vec2 &getWorldSize() override {
        return mWordSize;
    }

    std::vector<std::unique_ptr<GameEntity>> &getEntities() override {
        return mEntityList;
    }

    Vec2 getFlowDirection(const Vec2 &position) override {
        return mPathFinder.getDirection(position);
    }

    std::vector<Vec2> findNeighbourObstacle(const Vec2 &position) override {
        return mPathFinder.getNeighbourObstacle(position);
    }

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
};

int main() {
    App app;
    app.run();
    return 0;
}