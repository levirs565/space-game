#include <iostream>
#include <SDL.h>
#include <SDL_image.h>
#include <SDL_mixer.h>
#include <cmath>
#include <vector>
#include <unordered_map>
#include <memory>
#include <functional>
#include <set>

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

    virtual void findPath(const Vec2 &from, const Vec2 &to) = 0;
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
        int gCost;
        int hCost;
        bool isSelected = false;

        int getFCost() const {
            return gCost + hCost;
        }
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

    void clearSelection() {
        for (std::vector<Node> &row: mGrid) {
            for (Node &cell: row) {
                cell.isSelected = false;
            }
        }
    }

    void addObstacle(const Vec2 &centerPosition, int radius) {
        int left = floor(std::max((centerPosition.x - radius) / mEntitySize, 0.0));
        int top = floor(std::max((centerPosition.y - radius) / mEntitySize, 0.0));
        int right = ceil(std::max((centerPosition.x + radius) / mEntitySize, 0.0));
        int bottom = ceil(std::max((centerPosition.y + radius) / mEntitySize, 0.0));

        if (left >= mColumnCount || right >= mColumnCount) return;
        if (top >= mRowCount || bottom >= mRowCount) return;

        for (int row = top; row < bottom; row++) {
            for (int column = left; column < right; column++) {
                mGrid[row][column].isWalkable = false;
            }
        }
    }

    void drawGrid(SDL_Renderer *renderer, const Vec2 &cameraPosition, Vec2 &cameraSize) {
        int left = floor(cameraPosition.x / mEntitySize);
        int top = floor(cameraPosition.y / mEntitySize);
        int right = ceil((cameraPosition.x + cameraSize.x) / mEntitySize);
        int bottom = ceil((cameraPosition.y + cameraSize.y) / mEntitySize);

        int startX = -int(fmod(cameraPosition.x, mEntitySize));
        int startY = -int(fmod(cameraPosition.y, mEntitySize));

        SDL_Rect r;
        r.w = mEntitySize;
        r.h = mEntitySize;

        SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
        for (int row = top; row <= bottom; row++) {
            for (int column = left; column <= right; column++) {
                r.x = startX + (column - left) * mEntitySize;
                r.y = startY + (row - top) * mEntitySize;
                if (mGrid[row][column].isSelected)
                    SDL_SetRenderDrawColor(renderer, 255, 255, 0, 64);
                else if (mGrid[row][column].isWalkable)
                    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 64);
                else
                    SDL_SetRenderDrawColor(renderer, 255, 0, 0, 64);
                SDL_RenderFillRect(renderer, &r);
                SDL_SetRenderDrawColor(renderer, 255, 255, 255, 192);
                SDL_RenderDrawRect(renderer, &r);
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

    int getDistance(const NodePosition &from, const NodePosition &to) {
        int deltaRow = std::abs(from.first - to.first);
        int deltaColumn = std::abs(from.second - to.second);

        if (deltaColumn > deltaRow)
            return 14 * deltaRow + 10 * (deltaColumn - deltaRow);

        return 14 * deltaColumn + 10 * (deltaRow - deltaColumn);
    }

    void retracePath(const NodePosition &from, const NodePosition &to) {
        NodePosition pos = to;

        while (pos != from) {
            Node &node = mGrid[pos.first][pos.second];
            node.isSelected = true;
            pos = node.parentPosition;
        }
    }

    void findPath(const Vec2 &from, const Vec2 &to) {
        const NodePosition fromNodePos = getNodePositionFromWorldPosition(from);
        const NodePosition toNodePos = getNodePositionFromWorldPosition(to);

        std::vector<NodePosition> openSet;
        std::set<NodePosition> closedSet;

        openSet.push_back(fromNodePos);

        while (!openSet.empty()) {
            int minCostIndex = 0;
            NodePosition minPos = openSet[0];
            const Node *minCostNode = &mGrid[minPos.first][minPos.second];

            for (int i = 1; i < openSet.size(); i++) {
                const Node *currentNode = &mGrid[openSet[i].first][openSet[i].second];
                if (currentNode->getFCost() < minCostNode->getFCost() ||
                    currentNode->getFCost() == currentNode->getFCost()) {
                    if (currentNode->hCost < minCostNode->hCost) {
                        minCostNode = currentNode;
                        minCostIndex = i;
                        minPos = openSet[i];
                    }
                }
            }

            openSet.erase(openSet.begin() + minCostIndex);
            closedSet.insert(minPos);

            if (minPos == toNodePos) {
                retracePath(fromNodePos, toNodePos);
                return;
            }

            for (const NodePosition &neighbour: getNeighbours(minPos)) {
                Node &node = mGrid[neighbour.first][neighbour.second];

                if (!node.isWalkable || closedSet.count(neighbour) > 0)
                    continue;

                int newGCost = minCostNode->gCost + getDistance(neighbour, minPos);

                if (newGCost < node.gCost || std::count(openSet.begin(), openSet.end(), neighbour) == 0) {
                    node.gCost = newGCost;
                    node.hCost = getDistance(neighbour, toNodePos);
                    node.parentPosition = minPos;

                    if (std::count(openSet.begin(), openSet.end(), neighbour) == 0)
                        openSet.push_back(neighbour);
                }
            }
        }
    }
};

namespace SteeringBehaviour {
    Vec2 seek(const Vec2 &from, const Vec2 &to, const Vec2 &currentVelocity, double maxVelocity, double slowingDistance,
              double stopRadius) {
        Vec2 desiredVelocity = Vec2(to);
        desiredVelocity.substract(from);

        double distance = desiredVelocity.length();

        desiredVelocity.normalize();
        desiredVelocity.scale(maxVelocity);

        Vec2 direction(desiredVelocity);

        if (distance < slowingDistance + stopRadius) {
            desiredVelocity.scale((std::max(distance - stopRadius, 0.0)) / slowingDistance);
        }

        Vec2 steering = Vec2(desiredVelocity);
        steering.substract(currentVelocity);
        return steering;
    }

    Vec2
    separation(GameEntity *currentEntity, const std::vector<GameEntity *> othersEntity, double separationDistance) {
        Vec2 steering{0, 0};
        for (GameEntity *other: othersEntity) {
            Vec2 distanceVec(other->position);
            distanceVec.substract(currentEntity->position);
            double distance = distanceVec.length();

            if (distance < separationDistance) {
                distanceVec.scale(-1);
                distanceVec.normalize();
                distanceVec.scale(1.0 / (distance / separationDistance));

                steering.add(distanceVec, 1);
            }
        }

        return steering;
    }
}

class Enemy : public GameEntity {
public:
    Vec2 velocity{0, 0};
    Uint32 lastFire = 0;
    int num;

    Enemy(TextureLoader *textureLoader, const Vec2 &position) : GameEntity(position, 0) {
        texture = textureLoader->load("/home/levirs565/Unduhan/SpaceShooterRedux/PNG/Enemies/enemyBlack1.png");
    }

    void onTick(IGameStage *stage) override {
        bool canAttack = false;
        Vec2 steering = SteeringBehaviour::seek(position, stage->getPlayerPosition(), velocity, 2, 100, 200);
        Vec2 direction = stage->getPlayerPosition();
        direction.substract(position);

        std::vector<GameEntity *> othersEnemy;
        for (const std::unique_ptr<GameEntity> &entity: stage->getEntities()) {
            if (entity.get() == this) continue;

            if (Enemy *otherEnemy = dynamic_cast<Enemy *>(entity.get()); otherEnemy != nullptr) {
                othersEnemy.push_back(otherEnemy);
            }
        }

        steering.add(SteeringBehaviour::separation(this, othersEnemy, 250), 1);

        velocity.add(steering, 1);

        if (velocity.length() > 2) {
            velocity.normalize();
            velocity.scale(2);
        }

        position.add(velocity, 1);

        angle = direction.getRotation() * 180.0 / M_PI - 90;

        if (SDL_GetTicks() - lastFire >= 1000 && canAttack) {
            SDL_Rect enemyRect = getRect();
            Vec2 laserPos(0, enemyRect.h);
            laserPos.rotate((angle) * M_PI / 180.0);
            laserPos.add(position, 1);
            stage->addLaser(laserPos, angle - 180);
            lastFire = SDL_GetTicks();
        }

        stage->findPath(position, stage->getPlayerPosition());

        updateBoundingBox();
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

        mTextureLoader = std::make_unique<TextureLoader>(mRenderer);
        mBackgroundTexture = mTextureLoader->load("/home/levirs565/Unduhan/SpaceShooterRedux/Backgrounds/black.png");

        mLaserSound = Mix_LoadWAV("/home/levirs565/Unduhan/SpaceShooterRedux/Bonus/sfx_laser1.ogg");

        std::unique_ptr<PlayerShip> playerShip = std::make_unique<PlayerShip>(mTextureLoader.get(), Vec2(400, 400));
        mPlayerShip = playerShip.get();
        mEntityList.push_back(std::move(playerShip));

        mEntityList.push_back(std::move(std::make_unique<Enemy>(mTextureLoader.get(), Vec2(100, -100))));
        mEntityList.push_back(std::move(std::make_unique<Enemy>(mTextureLoader.get(), Vec2(300, -100))));
        mEntityList.push_back(std::move(std::make_unique<Enemy>(mTextureLoader.get(), Vec2(600, -100))));

        mEntityList.push_back(std::move(std::make_unique<Meteor>(mTextureLoader.get(), Vec2(100, 250), "Brown_big1")));
        mEntityList.push_back(std::move(std::make_unique<Meteor>(mTextureLoader.get(), Vec2(300, 250), "Brown_big2")));
        mEntityList.push_back(std::move(std::make_unique<Meteor>(mTextureLoader.get(), Vec2(600, 250), "Brown_big3")));
        mPathFinder.init(mWordSize, 110);

        mPathFinder.clearState();
        for (std::vector<std::unique_ptr<GameEntity>>::iterator it = mEntityList.begin();
             it != mEntityList.end(); it++) {
            std::unique_ptr<GameEntity> &entity = *it;
            if (dynamic_cast<Meteor *>(entity.get()) == nullptr) continue;
            mPathFinder.addObstacle(entity->position, entity->boundingRadius);
        }
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

            mPathFinder.clearSelection();
            mPlayerShip->setDirection(
                    mIsUp ? PlayerShip::DIRECTION_UP
                          : mIsDown ? PlayerShip::DIRECTION_DOWN
                                    : PlayerShip::DIRECTION_NONE,
                    mIsLeft ? PlayerShip::ROTATION_LEFT
                            : mIsRight ? PlayerShip::ROTATION_RIGHT
                                       : PlayerShip::ROTATION_NONE);

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

    void findPath(const Vec2 &from, const Vec2 &to) override {
        mPathFinder.findPath(from, to);
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