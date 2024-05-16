#ifndef SAP_HPP_
#define SAP_HPP_

#include <unordered_map>
#include <set>
#include "GameEntity.hpp"

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

    bool operator<(const Item &other) {
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

  void swapCallback(std::shared_ptr<Item> &item2,
                    std::shared_ptr<Item> &item1) {
    GameEntity *entity1 = item1->entity;
    GameEntity *entity2 = item2->entity;

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
      intervalList[k + 1] = std::move(intervalList[k]);
      k--;
    }
    intervalList[k + 1] = std::move(v);
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

  void setPair(GameEntity *entity1, GameEntity *entity2) {
    if (entity1->x0 < entity2->x1 && entity1->x1 > entity2->x0 &&
        entity1->y0 < entity2->y1 && entity1->y1 > entity2->y0) {
      (*mCollisionMap)[entity1].insert(entity2);
      (*mCollisionMap)[entity2].insert(entity1);
    }
  }

  void processSets(std::shared_ptr<Item> &item,
                   std::set<GameEntity *> &setMaintain,
                   std::initializer_list<std::set<GameEntity *> *> setCollide) {
    if (item->interval == 0) {
      setMaintain.insert(item->entity);
    } else {
      setMaintain.erase(item->entity);
      for (auto set : setCollide) {
        for (GameEntity *other : *set) {
          setPair(item->entity, other);
        }
      }
    }
  }

  void run() {
    std::set<GameEntity *> setInsert, setInterval;

    bool checkStab = false;

    size_t i = 0;
    size_t j = 0;

    std::sort(
        bufferList.begin(), bufferList.end(),
        [](const std::shared_ptr<Item> &a, const std::shared_ptr<Item> &b) {
          return *(a.get()) < *(b.get());
        });

    while (i < intervalList.size()) {
      std::shared_ptr<Item> &item = intervalList[i];

      if (item->isPendingDelete) {
        intervalList.erase(intervalList.begin() + i);
        checkStab = true;
        continue;
      }

      bool hasNew = j < bufferList.size();
      if (hasNew) {
        std::shared_ptr<Item> &newItem = bufferList[j];

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

      if (checkStab)
        setStabs(i);

      if (j < bufferList.size() && isY) {
        processSets(item, setInterval, {&setInsert});
      }

      insertSort(i);
      i++;
    }

    while (j < bufferList.size()) {
      std::shared_ptr<Item> &newItem = bufferList[j];

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
    for (const std::shared_ptr<Item> &item : intervalList) {
      if (item->value < prev)
        throw std::domain_error("Sorting faield");
      prev = item->value;
    }
  }

  std::vector<size_t> iterateStabs(size_t index) {
    if (index >= intervalList.size())
      return {};

    std::set<GameEntity *> skip;
    int stabs = intervalList[index]->stabs;
    std::vector<size_t> result;

    size_t i = index;
    while (stabs > 0 && i > 0) {
      i--;
      std::shared_ptr<Item> &item = intervalList[i];
      if (item->interval == 0 && !skip.contains(item->entity)) {
        stabs--;
        result.push_back(i);
        continue;
      }
      skip.insert(item->entity);
    }

    return result;
  }

  void addStabsToSet(size_t index, std::unordered_map<GameEntity *, int> &set) {
    for (size_t index : iterateStabs(index)) {
      GameEntity *entity = intervalList[index]->entity;
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
  SAPDimension(bool isY, SAPCollisionMap *collisionMap)
      : isY{isY}, mCollisionMap{collisionMap} {}
};

struct SAPRayDimension {
  double startValue;
  double deltaValue;
  int step;
  int sideCheck;
  double dRatio;
  int startIndex;
  int currentIndex;
  SAPDimension *currentDimension;
  std::unordered_map<GameEntity *, int> *hitMap;

  SAPRayDimension(SAPDimension *dimension, double from, double to,
                  std::unordered_map<GameEntity *, int> *hitMap)
      : startValue{from}, currentDimension{dimension}, hitMap{hitMap} {
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
    double value =
        currentIndex >= 0 &&
                currentIndex < currentDimension->intervalList.size()
            ? currentDimension->intervalList[currentIndex]->value
            : std::copysign(std::numeric_limits<double>::infinity(), step);
    dRatio = (value - startValue) / deltaValue;
    if (dRatio < 0)
      dRatio = std::numeric_limits<double>::infinity();
  }

  inline void addHit(GameEntity *entity) {
    (*hitMap)[entity] = hitMap->contains(entity) ? (*hitMap)[entity] + 1 : 1;
  }

  GameEntity *getCurrentEntity() {
    return currentDimension->intervalList[currentIndex]->entity;
  }

  void incrementRay() {
    std::shared_ptr<SAPDimension::Item> &item =
        currentDimension->intervalList[currentIndex];
    GameEntity *entity = item->entity;

    if (item->interval == sideCheck) {
      addHit(item->entity);
    } else {
      (*hitMap)[item->entity] = 0;
    }

    currentIndex += step;
    recalculateDRatio();
  }

  void findInternalHit(std::vector<GameEntity *> *list) {
    for (size_t index :
         currentDimension->iterateStabs(currentIndex + sideCheck)) {
      GameEntity *entity = currentDimension->intervalList[index]->entity;
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
  std::unordered_map<GameEntity *, int> mHitMap;
  bool mCheckInternalHit = true;
  std::vector<GameEntity *> mInternalHit;

public:
  SAPRay(SAPDimension *dimensionX, double x0, double x1,
         SAPDimension *dimensionY, double y0, double y1)
      : mXRay{dimensionX, x0, x1, &mHitMap},
        mYRay{dimensionY, y0, y1, &mHitMap} {}

  GameEntity *currentEntity = nullptr;

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
      GameEntity *entity = nullptr;
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

  std::unordered_map<GameEntity *, Item> entities;
  SAPCollisionMap mCollisionList;
  std::vector<GameEntity *> mPendingRemove;

  SAPDimension dimensionX{false, &mCollisionList};
  SAPDimension dimensionY{true, &mCollisionList};

public:
  void add(GameEntity *entity) {
    Item item{.x0 = std::make_shared<SAPDimension::Item>(entity->x0, 0, entity),
              .y0 = std::make_shared<SAPDimension::Item>(entity->y0, 0, entity),
              .x1 = std::make_shared<SAPDimension::Item>(entity->x1, 1, entity),
              .y1 =
                  std::make_shared<SAPDimension::Item>(entity->y1, 1, entity)};

    dimensionX.bufferList.push_back(item.x0);
    dimensionX.bufferList.push_back(item.x1);
    dimensionY.bufferList.push_back(item.y0);
    dimensionY.bufferList.push_back(item.y1);

    entities[entity] = std::move(item);
  }

  void move(GameEntity *entity) {
    if (!entities.contains(entity))
      return;

    Item &item = entities[entity];
    item.x0->value = entity->x0;
    item.x1->value = entity->x1;
    item.y0->value = entity->y0;
    item.y1->value = entity->y1;
  }

  void remove(GameEntity *entity) {
    if (!entities.contains(entity))
      return;

    Item &item = entities[entity];
    item.x0->isPendingDelete = true;
    item.x1->isPendingDelete = true;
    item.y0->isPendingDelete = true;
    item.y1->isPendingDelete = true;
    mPendingRemove.push_back(entity);
  }

  void update() {
    dimensionX.run();
    dimensionY.run();

    for (GameEntity *entity : mPendingRemove) {
      for (auto &[_, collisionList] : mCollisionList) {
        collisionList.erase(entity);
      }

      mCollisionList.erase(entity);
      entities.erase(entity);
    }

    mPendingRemove.clear();
  }

  std::vector<GameEntity *> queryArea(double x0, double y0, double x1,
                                      double y1, bool enclosed) {
    int minScore = enclosed ? 3 : 0;
    auto [_xl, xi] = dimensionX.binarySearch(x0);
    auto [_yl, yi] = dimensionY.binarySearch(y0);
    std::unordered_map<GameEntity *, int> xSet, ySet;

    if (!enclosed) {
      dimensionX.addStabsToSet(xi, xSet);
      dimensionX.addStabsToSet(yi, ySet);
    }

    while (xi < dimensionX.intervalList.size() &&
           dimensionX.intervalList[xi]->value <= x1) {
      GameEntity *entity = dimensionX.intervalList[xi]->entity;
      xSet[entity] = xSet.contains(entity) ? xSet[entity] + 2 : 2;
      xi++;
    }

    while (yi < dimensionY.intervalList.size() &&
           dimensionY.intervalList[yi]->value <= y1) {
      GameEntity *entity = dimensionY.intervalList[yi]->entity;
      ySet[entity] = ySet.contains(entity) ? ySet[entity] + 2 : 2;
      yi++;
    }

    std::vector<GameEntity *> result;
    for (auto [entity, xScore] : xSet) {
      if (ySet.contains(entity) && xScore > minScore &&
          ySet[entity] > minScore) {
        result.push_back(entity);
      }
    }

    return result;
  }

  SAPRay queryRay(double x0, double y0, double x1, double y1) {
    return SAPRay(&dimensionX, x0, x1, &dimensionY, y0, y1);
  }

  SAPCollisionMap &getCollisionMap() { return mCollisionList; }
};


#endif // SAP_HPP_