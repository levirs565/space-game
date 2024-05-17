#ifndef SAP_HPP_
#define SAP_HPP_

#include "GameEntity.hpp"
#include <set>
#include <unordered_map>

using SAPCollisionMap =
    std::unordered_map<GameEntity *, std::set<GameEntity *>>;

class SAPDimension {
public:
  struct Item {
    double value;
    int interval;
    GameEntity *entity;
    int stabs = 0;
    bool isPendingDelete = false;

    Item(double value, int interval, GameEntity *entity)
        : value{value}, interval{interval}, entity{entity} {}

    bool operator<(const Item &other) const {
      return value < other.value ||
             (value == other.value &&
              ((entity == other.entity && interval < other.interval) ||
               (interval > other.interval)));
    }
  };

  bool isY;
  std::vector<std::shared_ptr<Item>> intervalList;
  std::vector<std::shared_ptr<Item>> bufferList;
  SAPCollisionMap *mCollisionMap;

  void swapCallback(std::shared_ptr<Item> &item2, std::shared_ptr<Item> &item1);

  void setStabs(size_t i);

  void setPair(GameEntity *entity1, GameEntity *entity2);

  void processSets(std::shared_ptr<Item> &item,
                   std::set<GameEntity *> &setMaintain,
                   std::initializer_list<std::set<GameEntity *> *> setCollide);

  void run();

  std::vector<size_t> iterateStabs(size_t index);

  void addStabsToSet(size_t index, std::unordered_map<GameEntity *, int> &set);

  std::pair<int, int> binarySearch(double value);

public:
  SAPDimension(bool isY, SAPCollisionMap *collisionMap)
      : isY{isY}, mCollisionMap{collisionMap} {}
};

struct SAPRayDimension {
  double startValue;
  double deltaValue;
  int step;
  int sideCheck;
  double dRatio{};
  int startIndex;
  int currentIndex;
  SAPDimension *currentDimension;
  std::unordered_map<GameEntity *, int> *hitMap;

  SAPRayDimension(SAPDimension *dimension, double from, double to,
                  std::unordered_map<GameEntity *, int> *hitMap);

  void recalculateDRatio();

  inline void addHit(GameEntity *entity) {
    (*hitMap)[entity] = hitMap->contains(entity) ? (*hitMap)[entity] + 1 : 1;
  }

  inline GameEntity *getCurrentEntity() {
    return currentDimension->intervalList[currentIndex]->entity;
  }

  void incrementRay();
  void findInternalHit(std::vector<GameEntity *> *list);
};

class SAPRay {
  SAPRayDimension mXRay;
  SAPRayDimension mYRay;
  std::unordered_map<GameEntity *, int> mHitMap;
  bool mCheckInternalHit = true;
  std::vector<GameEntity *> mInternalHit;

public:
  SAPRay(SAPDimension *dimensionX, double x0, double x1,
         SAPDimension *dimensionY, double y0, double y1)
      : mXRay{dimensionX, x0, x1, &mHitMap},
        mYRay{dimensionY, y0, y1, &mHitMap} {}

  GameEntity *currentEntity = nullptr;

  bool next();
};

class SAP {
  struct Item {
    std::shared_ptr<SAPDimension::Item> x0, y0, x1, y1;
  };

  std::unordered_map<GameEntity *, Item> entities;
  SAPCollisionMap mCollisionList;
  std::vector<GameEntity *> mPendingRemove;

  SAPDimension dimensionX{false, &mCollisionList};
  SAPDimension dimensionY{true, &mCollisionList};

public:
  void add(GameEntity *entity);
  void move(GameEntity *entity);
  void remove(GameEntity *entity);
  void update();

  std::vector<GameEntity *> queryArea(double x0, double y0, double x1,
                                      double y1, bool enclosed);
  inline SAPRay queryRay(double x0, double y0, double x1, double y1) {
    return {&dimensionX, x0, x1, &dimensionY, y0, y1};
  }
  inline SAPCollisionMap &getCollisionMap() { return mCollisionList; }
};

#endif // SAP_HPP_