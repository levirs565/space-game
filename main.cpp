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
#include <ranges>
#include <map>
#include <random>

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

class APathFinder;
class GameEntity;
class SAP;

class IGameStage {
public:
    virtual const Vec2 &getPlayerPosition() = 0;

    virtual void addLaser(const Vec2 &position, double angle) = 0;

    virtual const Vec2 &getWorldSize() = 0;

    virtual std::vector<std::unique_ptr<GameEntity>> &getEntities() = 0;

    virtual Vec2 getFlowDirection(const Vec2 &position, const Vec2 &direction) = 0;

    virtual std::vector<Vec2> findNeighbourObstacle(const Vec2 &position) = 0;

    virtual APathFinder* getPathFinder() = 0;

    virtual SAP* getSAP() = 0;
};


class GameEntity {
public:
    SDL_Texture *texture;
    Vec2 position;
    double angle;
    bool mustGone = false;
    std::vector<Vec2> boundingBox;
    double boundingRadius = 0;
    double x0, y0, x1, y1;

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

        auto [minX, maxX] = std::minmax_element(boundingBox.begin(), boundingBox.end(), [] (const Vec2& a, const Vec2& b) { return a.x < b.x; });
        auto [minY, maxY] = std::minmax_element(boundingBox.begin(), boundingBox.end(), [] (const Vec2& a, const Vec2& b) { return a.y < b.y; });
        x0 = minX->x;
        x1 = maxX->x;
        y0 = minY->y;
        y1 = maxY->y;
    }
};

using SAPCollisionMap = std::unordered_map<GameEntity*, std::set<GameEntity*>>;

class SAPDimension {
public:

    struct Item {
        double value;
        int interval;
        GameEntity* entity;
        int stabs = 0;
        bool isPendingDelete = false;

        Item(double value, int interval, GameEntity* entity): value{value}, interval{interval}, entity{entity} {

        }

        bool operator < (const Item& other) {
            return value < other.value || (value == other.value && (
                ( entity == other.entity && interval < other.interval ) ||
                (interval > other.interval) 
            ));
        }
    };

    bool isY;
    std::vector<std::shared_ptr<Item>> intervalList;
    std::vector<std::shared_ptr<Item>> bufferList;
    SAPCollisionMap* mCollisionMap;

    void swapCallback(std::shared_ptr<Item>& item2, std::shared_ptr<Item>& item1) {
        GameEntity* entity1 = item1->entity;
        GameEntity* entity2 = item2->entity;

        if (item1->interval == 0 && item2->interval == 1) {
            setPair(entity1, entity2);

            item2->stabs++;
            item1->stabs++;
        } else if (item1->interval == 1 && item2->interval == 0) {
            (*mCollisionMap)[entity1].erase(entity2);
            (*mCollisionMap)[entity2].erase(entity1);

            item2->stabs--;
            item1->stabs--;
        } else {
            std::swap(item1->stabs, item2->stabs);
        }
    }

    void insertSort(size_t i) {
        std::shared_ptr<Item> v = std::move(intervalList[i]);
        int k = i - 1;
        while (k >= 0 && *(v.get()) < *(intervalList[k].get())) {
            swapCallback(intervalList[k], v);
            intervalList[k+1] = std::move(intervalList[k]);
            k--;
        }
        intervalList[k+1] = std::move(v);
    }

    void setStabs(size_t i) {
        if (i > 0) {
            int leftStab = intervalList[i - 1]->stabs;
            if (intervalList[i - 1]->interval == 1) {
                intervalList[i]->stabs = leftStab - 1;
            } else {
                intervalList[i]->stabs = leftStab + 1;
            }
        } else {
            intervalList[i]->stabs = 0;
        }
    }

    void setPair(GameEntity* entity1, GameEntity* entity2) {
        if (
            entity1->x0 < entity2->x1 &&
            entity1->x1 > entity2->x0 &&
            entity1->y0 < entity2->y1 &&
            entity1->y1 > entity2->y0
        ) {
            (*mCollisionMap)[entity1].insert(entity2);
            (*mCollisionMap)[entity2].insert(entity1);
        }
    }

    void processSets(std::shared_ptr<Item>& item, std::set<GameEntity*>& setMaintain, std::initializer_list<std::set<GameEntity*>*> setCollide) {
        if (item->interval == 0) {
            setMaintain.insert(item->entity);
        } else {
            setMaintain.erase(item->entity);
            for (auto set : setCollide) {
                for (GameEntity* other : *set) {
                    setPair(item->entity, other);
                }
            }
        }
    }

    void run() {
        std::set<GameEntity*> setInsert, setInterval;

        bool checkStab = false;

        size_t i = 0;
        size_t j = 0; 

        std::sort(
            bufferList.begin(), 
            bufferList.end(), 
            [] (const std::shared_ptr<Item>& a,const std::shared_ptr<Item>& b) {
            return *(a.get()) < *(b.get());
        });

        while (i < intervalList.size()) {
            std::shared_ptr<Item>& item = intervalList[i];

            if (item->isPendingDelete) {
                intervalList.erase(intervalList.begin() + i);
                checkStab = true;
                continue;
            } 

            bool hasNew = j < bufferList.size();
            if (hasNew) {
                std::shared_ptr<Item>& newItem = bufferList[j];

                if (newItem->isPendingDelete) {
                    checkStab = true;
                    j++;
                    continue;
                }

                if (*(newItem.get()) < *(item.get())) {
                    if (isY) {
                        processSets(newItem, setInsert, {&setInsert, &setInterval});
                    }

                    intervalList.insert(intervalList.begin() + i, newItem);
                    setStabs(i);
                    i++;
                    j++;

                    checkStab = true;

                    continue;
                }
            }

            if (checkStab) setStabs(i);

            if (j < bufferList.size() && isY) {
                processSets(item, setInterval, {&setInsert});
            }

            insertSort(i);
            i++;
        }

        while (j < bufferList.size()) {
            std::shared_ptr<Item>& newItem = bufferList[j];

            if (!newItem->isPendingDelete) {
                if (isY) {
                    processSets(newItem, setInsert, {&setInsert});
                }

                intervalList.insert(intervalList.begin() + i, newItem);
                setStabs(i);
                i++;
            }

            j++;
        }
        bufferList.clear();

        if (intervalList.size() < i) {
            intervalList.erase(intervalList.begin() + i, intervalList.end());
        }

        bufferList.clear();

        double prev = std::numeric_limits<double>::lowest();
        for (const std::shared_ptr<Item>& item : intervalList) {            
            if (item->value < prev) throw std::domain_error("Sorting faield");
            prev = item->value;
        }
    }

    std::vector<size_t> iterateStabs(size_t index) {
        if (index >= intervalList.size()) return {};

        std::set<GameEntity*> skip;
        int stabs = intervalList[index]->stabs;
        std::vector<size_t> result;

        size_t i = index;
        while (stabs > 0 && i > 0) {
            i--; 
            std::shared_ptr<Item>& item = intervalList[i];
            if (item->interval == 0 && !skip.contains(item->entity)) {
                stabs--;
                result.push_back(i);
                continue;
            }
            skip.insert(item->entity);
        }

        return result;
    }

    void addStabsToSet(size_t index, std::unordered_map<GameEntity*, int>& set) {
        for (size_t index : iterateStabs(index)) {
            GameEntity* entity = intervalList[index]->entity;
            set[entity] = set.contains(entity) ? set[entity] + 1 : 1;
        }
    }

    std::pair<int, int> binarySearch(double value) {
        int start = 0, end = intervalList.size() - 1;
        
        int lower = -1, upper = intervalList.size();

        while (start <= end) {
            size_t mid = (start + end) / 2;
            double currentValue = intervalList[mid]->value;

            if (value == currentValue) {
                lower = mid;
                upper = mid;

                size_t num = mid - 1;
                while (num > 0 && value == intervalList[num]->value) {
                    lower = num;
                    num--;
                }

                num = mid + 1;

                while (num < intervalList.size() && value == intervalList[num]->value) {
                    upper = num;
                    num++;
                }

                return {lower, upper};
            } else if (value < currentValue) {
                upper = mid;
                end = mid - 1;
            } else {
                lower = mid;
                start = mid + 1;
            }
        }

        return {lower, upper};
    }

public:

    SAPDimension(bool isY, SAPCollisionMap* collisionMap): isY{isY}, mCollisionMap{collisionMap} {

    }
};

struct SAPRayDimension {
    double startValue;
    double deltaValue;
    int step;
    int sideCheck;
    double dRatio;
    int startIndex;
    int currentIndex;
    SAPDimension* currentDimension;
    std::unordered_map<GameEntity*, int>* hitMap;

    SAPRayDimension(SAPDimension* dimension, double from, double to, std::unordered_map<GameEntity*, int>* hitMap): startValue{from}, currentDimension{dimension}, hitMap{hitMap} {
        deltaValue = to - from;
        auto [lower, upper] = dimension->binarySearch(from);

        if (deltaValue >= 0) {
            step = 1;
            sideCheck = 0;
            startIndex = upper;
        } else {
            step = -1;
            sideCheck = 1;
            startIndex = lower;
        }

        currentIndex = startIndex;
        recalculateDRatio();
    }

    void recalculateDRatio() {
        double value = currentIndex >= 0 && currentIndex < currentDimension->intervalList.size() ? currentDimension->intervalList[currentIndex]->value : std::copysign(std::numeric_limits<double>::infinity(), step);
        dRatio = (value - startValue) / deltaValue;
        if (dRatio < 0)
            dRatio = std::numeric_limits<double>::infinity();
    }

    inline void addHit(GameEntity* entity) {
        (*hitMap)[entity] = hitMap->contains(entity) ? (*hitMap)[entity] + 1 : 1;
    }

    GameEntity* getCurrentEntity() {
        return currentDimension->intervalList[currentIndex]->entity;
    }

    void incrementRay() {
        std::shared_ptr<SAPDimension::Item>& item = currentDimension->intervalList[currentIndex];
        GameEntity* entity = item->entity;

        if (item->interval == sideCheck) {
            addHit(item->entity);
        } else {
            (*hitMap)[item->entity] = 0;
        }

        currentIndex += step;
        recalculateDRatio();
    }

    void findInternalHit(std::vector<GameEntity*>* list ) {
        for (size_t index : currentDimension->iterateStabs(currentIndex + sideCheck)) {
            GameEntity* entity = currentDimension->intervalList[index]->entity;
            addHit(entity);

            if (list != nullptr && (*hitMap)[entity] > 1) {
                list->push_back(entity);
            }
        }
    } 
};


class SAPRay {
    SAPRayDimension mXRay;
    SAPRayDimension mYRay;
    std::unordered_map<GameEntity*, int> mHitMap;
    bool mCheckInternalHit = true;
    std::vector<GameEntity*> mInternalHit;
public:
    SAPRay(SAPDimension* dimensionX, double x0, double x1, SAPDimension* dimensionY, double y0, double y1) : mXRay{dimensionX, x0, x1, &mHitMap},
        mYRay{dimensionY, y0, y1, &mHitMap} 
         {
    }

    GameEntity* currentEntity = nullptr;

    bool next() {
        if (mCheckInternalHit) {
            mCheckInternalHit = false;
            mXRay.findInternalHit(nullptr);
            mYRay.findInternalHit(&mInternalHit);
        }


        if (mInternalHit.size() > 0) {
            currentEntity = mInternalHit[0];
            mInternalHit.erase(mInternalHit.begin());
            return true;
        }

        double minRatio = std::min(mXRay.dRatio, mYRay.dRatio);
        while (minRatio <= 1) {
            GameEntity* entity = nullptr;
            if (minRatio == mXRay.dRatio) {
                entity = mXRay.getCurrentEntity();
                mXRay.incrementRay();
            } else {
                entity = mYRay.getCurrentEntity();
                mYRay.incrementRay();
            }
            if (mHitMap[entity] > 1) {
                currentEntity = entity;
                return true;
            }

            minRatio = std::min(mXRay.dRatio, mYRay.dRatio);
        }

        currentEntity = nullptr;
        return false;
    }
};

class SAP {
    struct Item {
        std::shared_ptr<SAPDimension::Item> x0, y0, x1, y1;
    };

    std::unordered_map<GameEntity*, Item> entities;
    SAPCollisionMap mCollisionList;
    std::vector<GameEntity*> mPendingRemove;

    SAPDimension dimensionX{false, &mCollisionList};
    SAPDimension dimensionY{true, &mCollisionList};

public:
    void add(GameEntity* entity) {
        Item item{
            .x0 = std::make_shared<SAPDimension::Item>(
                entity->x0,
                0,
                entity
            ),
            .y0 = std::make_shared<SAPDimension::Item>(
                entity->y0,
                0,
                entity
            ),
            .x1 = std::make_shared<SAPDimension::Item>(
                entity->x1,
                1,
                entity
            ),
            .y1 = std::make_shared<SAPDimension::Item>(
                entity->y1,
                1,
                entity
            )
        };

        dimensionX.bufferList.push_back(item.x0);
        dimensionX.bufferList.push_back(item.x1);
        dimensionY.bufferList.push_back(item.y0);
        dimensionY.bufferList.push_back(item.y1);

        entities[entity] = std::move(item);
    }

    void move(GameEntity* entity) {
        if (!entities.contains(entity)) return;

        Item& item = entities[entity];
        item.x0->value = entity->x0;
        item.x1->value = entity->x1;
        item.y0->value = entity->y0;
        item.y1->value = entity->y1;
    }

    void remove(GameEntity* entity) {
        if (!entities.contains(entity)) return;

        Item& item = entities[entity];
        item.x0->isPendingDelete = true;
        item.x1->isPendingDelete = true;
        item.y0->isPendingDelete = true;
        item.y1->isPendingDelete = true;
        mPendingRemove.push_back(entity);
    }

    void update() {
        dimensionX.run();
        dimensionY.run();

        for (GameEntity* entity : mPendingRemove) {
            for (auto& [_, collisionList] : mCollisionList) {
                collisionList.erase(entity);
            }

            mCollisionList.erase(entity);
            entities.erase(entity);
        }

        mPendingRemove.clear();
    }

    std::vector<GameEntity*> queryArea(double x0, double y0, double x1, double y1, bool enclosed) {
        int minScore = enclosed ? 3 : 0;
        auto [_xl, xi] = dimensionX.binarySearch(x0);
        auto [_yl, yi] = dimensionY.binarySearch(y0);
        std::unordered_map<GameEntity*, int> xSet, ySet;

        if (!enclosed) {
            dimensionX.addStabsToSet(xi, xSet);
            dimensionX.addStabsToSet(yi, ySet);
        }

        while (xi < dimensionX.intervalList.size() && dimensionX.intervalList[xi]->value <= x1) {
            GameEntity* entity = dimensionX.intervalList[xi]->entity;
            xSet[entity] = xSet.contains(entity) ? xSet[entity] + 2 : 2;
            xi++;
        }

        while (yi < dimensionY.intervalList.size() && dimensionY.intervalList[yi]->value <= y1) {
            GameEntity* entity = dimensionY.intervalList[yi]->entity;
            ySet[entity] = ySet.contains(entity) ? ySet[entity] + 2 : 2;
            yi++;
        }

        std::vector<GameEntity*> result;
        for (auto [entity, xScore] : xSet) {
            if (ySet.contains(entity) && xScore > minScore && ySet[entity] > minScore) {
                result.push_back(entity);
            }
        }

        return result;
    }

    SAPRay queryRay(double x0, double y0, double x1, double y1) {
        return SAPRay(&dimensionX, x0, x1, &dimensionY, y0, y1);
    }

    SAPCollisionMap& getCollisionMap() {
        return mCollisionList;
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

    void addVector(const Vec2& vector, double minCos = 0) {
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

    int indexOf(const double* ptr) {
        return ptr - begin();
    }

    void clear() {
        std::fill(begin(), end(), 0);
    }

    double* begin() {
        return &data[0];
    }

    double* end() {
        return &data[angleCount];
    }

    auto withIndex() {
        return *this | std::views::transform([this](double& value) {
            return std::make_tuple(std::ref(value), indexOf(&value));
        });
    }

    void draw(SDL_Renderer* renderer, const Vec2& position, double radius, double angleDeviation, int index) {
        double value = data[index];
        if (value == 0) return;
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
            value = std::clamp(interestMap.data[index] - dangerMap.data[index], 0.0, 1.0);
        }

        Vec2 result{0, 0};
        for (auto [value, index] : resultMap.withIndex()) {
            result.add(ContextSteeringMap::directionBy(index), value);
        }
        
        result.normalize();
        return result;
    }

    void draw(SDL_Renderer* renderer, const Vec2& position, double radius) {
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
                if (node.lineOfSight)
                    SDL_SetRenderDrawColor(renderer, 255, 255, 0, 64);
                else if (node.isWalkable)
                    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 64);
                else
                    SDL_SetRenderDrawColor(renderer, 255, 0, 0, 64);
                SDL_RenderFillRect(renderer, &r);
                SDL_SetRenderDrawColor(renderer, 255, 255, 255, 192);
                SDL_RenderDrawRect(renderer, &r);

                Vec2 direction = getDirection(NodePosition{row, column}, {0,0});
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

    Vec2 getDirection(const Vec2 &position, const Vec2& direction) {
        return getDirection(getNodePositionFromWorldPosition(position), direction);
    }


    Vec2 getDirection(const NodePosition &nodePosition, const Vec2& direction) {
        int currentCost = mGrid[nodePosition.first][nodePosition.second].cost;
        bool considerDirection = direction.length() > 0;
        if (currentCost == 0) return {0, 0};

        int minCost = std::numeric_limits<int>::max();
        Vec2 result{0, 0};
        double maxDot = -1;

        for (const NodePosition &neighbour: getNeighbours(nodePosition)) {
            if (!canWalk(nodePosition, neighbour)) continue;

            const Node &neighbourNode = mGrid[neighbour.first][neighbour.second];


            int cost = neighbourNode.cost;

            if (cost >= currentCost) continue;

            Vec2 force{
                    double(neighbour.second - nodePosition.second),
                    double(neighbour.first - nodePosition.first)
            };
            force.normalize();
            double dot = force.dot(direction);

            if (
                (!considerDirection && cost < minCost) ||
                (considerDirection && (
                    dot > maxDot || (dot == maxDot && cost < minCost)
                ))) {
                result = force;
                minCost = cost;
                maxDot = dot;
            }
        }

        return result;
    }

    bool addDirectionToSteering(const Vec2 &position, const Vec2& direction, ContextSteeringMap& map, double scale) {
        return addDirectionToSteering(getNodePositionFromWorldPosition(position), direction, map, scale);
    }

    bool addDirectionToSteering(const NodePosition &nodePosition, const Vec2& direction, ContextSteeringMap& map, double scale) {
        int currentCost = mGrid[nodePosition.first][nodePosition.second].cost;

        if (currentCost == std::numeric_limits<int>::max())
            return false;

        Vec2 result{0, 0};
        bool success = false;

        for (const NodePosition &neighbour: getNeighbours(nodePosition)) {
            if (!canWalk(nodePosition, neighbour)) continue;

            const Node &neighbourNode = mGrid[neighbour.first][neighbour.second];

            int cost = neighbourNode.cost;

            if (cost >= currentCost) continue;
            
            success = true;

            Vec2 force{
                    double(neighbour.second - nodePosition.second),
                    double(neighbour.first - nodePosition.first)
            };
            force.normalize();
            double dot = force.dot(direction);
            dot = (dot + 1.5) / 2.5; 
            force.scale(dot * scale);
            map.addVector(force);
        }

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
        if (!mGrid[to.first][to.second].isWalkable) return false;

        int deltaRow = to.first - from.first;
        int deltaColumn = to.second - from.second;

        if (deltaRow == 0 || deltaColumn == 0) return true;

        if (abs(deltaRow) > 1 || abs(deltaColumn) > 1)
            throw std::invalid_argument("node is too far");

        return mGrid[from.first][to.second].isWalkable &&
               mGrid[to.first][from.second].isWalkable;
    }

    void calculateLineOfSight(const NodePosition& from, const NodePosition& to) {
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
            if (!mGrid[from.first + deltaFirstSign][from.second  + deltaSecondSign].lineOfSight)
                hasLineOfSight = false;
            else if (deltaFirstAbs == deltaSecondAbs) {
                if (!mGrid[from.first + deltaFirstSign][from.second].isWalkable||
                    !mGrid[from.first][from.second + deltaSecondSign].isWalkable == std::numeric_limits<int>::max()
                ) {
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

        for (std::vector<Node> &row: mGrid) {
            for (Node &cell: row) {
                cell.cost = std::numeric_limits<int>::max();
                cell.lineOfSight = false;
            }
        }

        if (!mGrid[targetNodePos.first][targetNodePos.second].isWalkable) return;

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

    bool hasLineOfSigh(const Vec2& position) {
        const NodePosition node = getNodePositionFromWorldPosition(position);

        return mGrid[node.first][node.second].lineOfSight;
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

std::optional<Vec2> rayCircleIntersection(const Vec2& point, const Vec2& ray, const Vec2& circle, double radius) {
    Vec2 u{circle};
    u.substract(point);

    double dot = u.dot(ray);

    if (dot < 0) return std::nullopt;

    Vec2 u1 = u.projectInto(ray, false);
    
    Vec2 u2{u};
    u2.substract(u1);

    double d = u2.length();

    if (d > radius) return std::nullopt;
    if (std::abs(d - radius) < 0.1) return std::nullopt;

    double m = std::sqrt(radius * radius - d * d);

    Vec2 p{u1};
    p.add(ray, -m);

    return std::optional(p);
}

double mapValue(double value, double minValue, double maxValue, double minOutput, double maxOutput) {
    return minOutput + (maxOutput - minOutput) * ((value - minValue) / (maxValue - minValue));
}

class Enemy : public GameEntity {
public:
    double speed;
    Vec2 direction{0, 0};
    Vec2 smoothedDirection{0,0};
    Vec2 acceleration{0, 0};
    Uint32 lastFire = 0;
    Uint32 lastUpdatePath = 0;
    double lastSide = 0;
    const Enemy *lastThreat = nullptr;
    const Enemy *avoidedEnemy = nullptr;
    std::vector<Uint32> avoidanceTime;
    ContextSteering contextSteering;
    Vec2 contextSteeringResult{0, 0};
    bool isSeek = true;
    Vec2 flowPreferedDirection{1, 0};
    std::vector<GameEntity*> nearEntity;
    double wanderAngle = 0;
    std::random_device rd;
    std::mt19937 e2{rd()};
    std::uniform_real_distribution<> dist{0.0, 1.0};

    Enemy(TextureLoader *textureLoader, const Vec2 &position) : GameEntity(position, 0) {
        texture = textureLoader->load("/home/levirs565/Unduhan/SpaceShooterRedux/PNG/Enemies/enemyBlack1.png");
        regenerateFlowPreferedDirection();
    }

    void regenerateFlowPreferedDirection() {
        flowPreferedDirection = {1, 0};
        const double rotation = double(std::rand()) / RAND_MAX * M_PI_2 - M_PI_2 / 2.0;
        if (!isSeek) {
            flowPreferedDirection = direction;
            flowPreferedDirection.rotate(rotation);
        } else {
            flowPreferedDirection.rotate(rotation);
        }
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
        nearEntity = stage->getSAP()->queryArea(
            position.x - 3 * boundingRadius,
            position.y - 3 * boundingRadius,
            position.x + 3 * boundingRadius,
            position.y + 3 * boundingRadius,
            false
        );

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

        for (GameEntity* entity : nearEntity) {
            if (entity == this) continue;
            if (dynamic_cast<Laser*>(entity) != nullptr) continue;

            Vec2 distanceVec{entity->position};
            distanceVec.substract(position);

            if (distanceVec.length() < clampedRadius) {
                Vec2 avoid{distanceVec};
                avoid.normalize();
                contextSteering.dangerMap.addVector(avoid);
            }

            for (const auto& [_, index] : contextSteering.dangerMap.withIndex()) {
                Vec2 ray = ContextSteeringMap::directionBy(index);
                auto intersection = rayCircleIntersection(position, ray, entity->position, entity->boundingRadius);
                if (intersection.has_value()) {
                    Vec2 avoid = intersection.value();
                    double length = avoid.length();

                    avoid.normalize();
                    avoid.scale(
                        mapValue(
                            std::clamp(length, minRayLength, maxRayLength),
                            minRayLength, maxRayLength, 1.0, 0.0)
                    );
                    contextSteering.dangerMap.addVector(avoid, 0);
                } 
            }
        }
        
        bool hasLineOfSight = stage->getPathFinder()->hasLineOfSigh(position);
        if (distance > 400 || !hasLineOfSight) {
            bool pathFindSuccess = stage->getPathFinder()->addDirectionToSteering(position, direction, contextSteering.interestMap, hasLineOfSight ? 0.5 : 1);

            if (hasLineOfSight || !pathFindSuccess) {
                Vec2 seekDirection{distanceVector};
                seekDirection.normalize();

                contextSteering.interestMap.addVector(seekDirection);
            }
        } 
        
        if (distance < 600) {
            Vec2 distanceNormalized{distanceVector};
            distanceNormalized.normalize();

            auto intersection = rayCircleIntersection(position, direction, stage->getPlayerPosition(), boundingRadius);
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

                    for (SAPRay ray = stage->getSAP()->queryRay(from.x, from.y, to.x, to.y); ray.next(); ) {
                        if (ray.currentEntity == this) {
                            continue;
                        }
                        if (dynamic_cast<PlayerShip*>(ray.currentEntity) != nullptr) {
                            break;
                        }
                        if (dynamic_cast<Laser*>(ray.currentEntity) != nullptr) {
                            break;
                        }
                        canAttack = false;
                        break;
                    }

                    if (!canAttack) break;
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
            laserPos.rotate((angle) * M_PI / 180.0);
            laserPos.add(position, 1);
            stage->addLaser(laserPos, angle - 180);
            lastFire = SDL_GetTicks();
        }

        updateBoundingBox();
    }

    void onDraw(SDL_Renderer *renderer, const Vec2 &cameraPosition) override {
        GameEntity::onDraw(renderer, cameraPosition);

        Vec2 onCameraPosition{position};
        onCameraPosition.substract(cameraPosition);
        contextSteering.draw(renderer, onCameraPosition, boundingRadius);
        
        // SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "Steer %f", contextSteeringResult.length());
        Vec2 steeringLine{contextSteeringResult};
        steeringLine.normalize();
        steeringLine.scale(boundingRadius * 1.5);
        steeringLine.add(onCameraPosition, 1); 
        SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255);
        SDL_RenderDrawLine(renderer, onCameraPosition.x, onCameraPosition.y, steeringLine.x, steeringLine.y);

        // Vec2 flowLine{flowPreferedDirection};
        // flowLine.scale(boundingRadius * 2.0);
        // flowLine.add(onCameraPosition, 1);

        // if (isSeek)
        //     SDL_SetRenderDrawColor(renderer, 255, 0, 255, 255);
        // else 
        //     SDL_SetRenderDrawColor(renderer, 0, 255, 255, 255);
        // SDL_RenderDrawLine(renderer, onCameraPosition.x, onCameraPosition.y, flowLine.x, flowLine.y);


    //    static TTF_Font * font = TTF_OpenFont("/home/levirs565/Unduhan/kenney_space-shooter-redux/Bonus/kenvector_future.ttf", 16);
    //    SDL_Color color;
    //    color.r = 255;
    //    color.g = 255;
    //    color.b = 255;
    //    color.a = 255;
    //    Vec2 posInCamera{position};
    //    posInCamera.substract(cameraPosition);


    //     const double minNeighbourDistance = boundingRadius + 10;
    //     const double maxNeighbourDistance = 4 * boundingRadius;
    //     const double maxNeigbourAngle = 135.0 * M_PI / 180.0;

    //    for (GameEntity * other : othersEnemy) {
    //        Vec2 distanceVec(other->position);
    //        distanceVec.substract(position);
    //        double distance = distanceVec.length();
    //        double angle = distanceVec.angleBetween(direction);

    //         if (distance > maxNeighbourDistance)
    //             continue;

    //         if (distance > minNeighbourDistance && 
    //            angle > maxNeigbourAngle)
    //             continue;

    //         Vec2 projection = distanceVec.projectInto(direction, false);
    //         projection.add(position, 1);
    //         projection.substract(cameraPosition);

    //         distanceVec.scale(-1);
    //         distanceVec.normalize();
    //         distanceVec.scale(1 / distance);


    //         Vec2 otherInCamera{other->position};
    //         otherInCamera.substract(cameraPosition);

    //         SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
    //         SDL_RenderDrawLine(renderer, posInCamera.x, posInCamera.y, otherInCamera.x, otherInCamera.y);
    //         SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255);
    //         SDL_RenderDrawLine(renderer, posInCamera.x, posInCamera.y, projection.x, projection.y);
    //         SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255);
    //         SDL_RenderDrawLine(renderer, otherInCamera.x, otherInCamera.y, projection.x, projection.y);

    //         std::string text = std::to_string(angle);
    //         SDL_Surface* surface = TTF_RenderText_Solid(font, text.c_str(), color);
    //         SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer, surface);
    //         SDL_Rect textRect;
    //         SDL_QueryTexture(texture, nullptr, nullptr, &textRect.w, &textRect.h);
    //         textRect.x = (otherInCamera.x + projection.x) / 2.0 - textRect.w;
    //         textRect.y = (otherInCamera.y + projection.y) / 2.0 - textRect.h;
    //         SDL_RenderCopy(renderer, texture, nullptr, &textRect);

    //         SDL_DestroyTexture(texture);
    //         SDL_FreeSurface(surface);
    //    }
    }

    void onHit(GameEntity *other) override {
        if (Laser *laser = dynamic_cast<Laser *>(other); laser != nullptr) {
            // mustGone = true;
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
        addEntity(std::move(playerShip));

        addEntity(std::move(std::make_unique<Enemy>(mTextureLoader.get(), Vec2(100, 0))));
        addEntity(std::move(std::make_unique<Enemy>(mTextureLoader.get(), Vec2(300, 0))));
        addEntity(std::move(std::make_unique<Enemy>(mTextureLoader.get(), Vec2(800, 0))));

        addEntity(std::move(std::make_unique<Meteor>(mTextureLoader.get(), Vec2(100, 500), "Brown_big1")));
        addEntity(std::move(std::make_unique<Meteor>(mTextureLoader.get(), Vec2(300, 500), "Brown_big2")));
        addEntity(std::move(std::make_unique<Meteor>(mTextureLoader.get(), Vec2(600, 500), "Brown_big3")));
        addEntity(std::move(std::make_unique<Meteor>(mTextureLoader.get(), Vec2(615, 1000), "Brown_big3")));
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

    void addEntity(std::unique_ptr<GameEntity>&& entity) {
        GameEntity* ptr = entity.get();
        mEntityList.push_back(std::move(entity));
        mSAP.add(ptr);
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
                std::unique_ptr<GameEntity>& entity = mEntityList[i];
                entity->onTick(this);
                // setelah onTick, jangan gunakan entity kembali karena ada kemungkinan penambahan elemen ke mEntityList
                // yang menyebabkan operasi std::move terhadap entity sehingga entity berada dalam keadaan invalid
            }

            for (std::unique_ptr<GameEntity>& entity : mEntityList) {
                mSAP.move(entity.get());
            }

            for (std::vector<std::unique_ptr<GameEntity>>::iterator it = mEntityList.begin();
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

            for (auto& [entity, collisionSet] : mSAP.getCollisionMap()) {
                for (auto otherEntity : collisionSet) {
                    if (isPolygonCollide(entity->boundingBox, otherEntity->boundingBox)) {
                        entity->onHit(otherEntity);
                    }
                }
            }

            SDL_SetRenderDrawColor(mRenderer, 0, 255, 0, 255);

            for (GameEntity* entity : mSAP.queryArea(mCameraPosition.x, mCameraPosition.y, mCameraPosition.x + mCameraSize.x, mCameraPosition.y + mCameraSize.y, false)) {
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
        addEntity(std::move(laser));
        Mix_PlayChannel(1, mLaserSound, 0);
    }

    const Vec2 &getWorldSize() override {
        return mWordSize;
    }

    std::vector<std::unique_ptr<GameEntity>> &getEntities() override {
        return mEntityList;
    }

    Vec2 getFlowDirection(const Vec2 &position, const Vec2 &direction) override {
        return mPathFinder.getDirection(position, direction);
    }

    std::vector<Vec2> findNeighbourObstacle(const Vec2 &position) override {
        return mPathFinder.getNeighbourObstacle(position);
    }

    APathFinder* getPathFinder() {
        return &mPathFinder;
    }

    SAP* getSAP() {
        return &mSAP;
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
    SAP mSAP;
};

int main() {
    App app;
    app.run();
    return 0;
}