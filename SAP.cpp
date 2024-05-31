#include "SAP.hpp"
#include <iostream>

namespace SAPCollisionMapHelper {
void orderEntityForKey(GameEntity *&entity1, GameEntity *&entity2) {
  if (entity1->getId() > entity2->getId()) {
    std::swap(entity1, entity2);
  }
}

void removeCollision(SAPCollisionMap &map, GameEntity *entity1,
                    GameEntity *entity2) {
  orderEntityForKey(entity1, entity2);
  map[entity1].erase(entity2);
}

void addCollision(SAPCollisionMap &map, GameEntity *entity1,
                  GameEntity *entity2) {
  orderEntityForKey(entity1, entity2);
  map[entity1].insert(entity2);
}
} // namespace SAPCollisionMapHelper

void SAPDimension::swapCallback(std::shared_ptr<Item> &item2,
                                std::shared_ptr<Item> &item1) {
  GameEntity *entity1 = item1->entity;
  GameEntity *entity2 = item2->entity;

  if (item1->interval == 0 && item2->interval == 1) {
    // 0      1      0        1
    // [item2 item2][item1 item1]
    // 0      1      2        1]
    // [item2 [item1 item2] item1]

    setPair(entity1, entity2);

    item2->stabs++;
    item1->stabs++;
  } else if (item1->interval == 1 && item2->interval == 0) {
    // 0      1      2      1
    // [item1 [item2 item1] item2]
    // 0      1      0      1
    // [item1 item1] [item2 item2

    SAPCollisionMapHelper::removeCollision(*mCollisionMap, entity1, entity2);

    item2->stabs--;
    item1->stabs--;
  } else {
    // First possibility
    // 0      1
    // [item2 [item1
    // 0      1
    // [item1 [item2

    // Second possibility
    // n      n - 1
    // item2] item1]
    // n      n - 1
    // item1] item2]

    std::swap(item1->stabs, item2->stabs);
  }
}

void SAPDimension::setStabs(size_t i) {
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

void SAPDimension::setPair(GameEntity *entity1, GameEntity *entity2) {
  if (entity1->x0 < entity2->x1 && entity1->x1 > entity2->x0 &&
      entity1->y0 < entity2->y1 && entity1->y1 > entity2->y0) {
    SAPCollisionMapHelper::addCollision(*mCollisionMap, entity1, entity2);
  }
}

void SAPDimension::processSets(
    std::shared_ptr<Item> &item, std::set<GameEntity *> &setMaintain,
    std::initializer_list<std::set<GameEntity *> *> setCollide) {
  {
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
}

void SAPDimension::run() {
  std::set<GameEntity *> setInsert, setInterval;

  bool checkStab = false;

  size_t i = 0;
  size_t j = 0;

  std::sort(bufferList.begin(), bufferList.end(),
            [](const std::shared_ptr<Item> &a, const std::shared_ptr<Item> &b) {
              return *a < *b;
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

      if (*newItem < *item) {
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

    if (hasNew && isY) {
      processSets(item, setInterval, {&setInsert});
    }

    std::shared_ptr<Item> v = std::move(intervalList[i]);
    int k = int(i) - 1;
    while (k >= 0 && *v < *(intervalList[k].get())) {
      swapCallback(intervalList[k], v);
      intervalList[k + 1] = std::move(intervalList[k]);
      k--;
    }
    intervalList[k + 1] = std::move(v);

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
}

std::vector<size_t> SAPDimension::iterateStabs(size_t index) {
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

void SAPDimension::addStabsToSet(size_t index,
                                 std::unordered_map<GameEntity *, int> &set) {

  for (size_t index : iterateStabs(index)) {
    GameEntity *entity = intervalList[index]->entity;
    set[entity] = set.contains(entity) ? set[entity] + 1 : 1;
  }
}

std::pair<int, int> SAPDimension::binarySearch(double value) {
  int start = 0, end = int(intervalList.size() - 1);

  int lower = -1, upper = int(intervalList.size());

  while (start <= end) {
    int mid = (start + end) / 2;
    double currentValue = intervalList[mid]->value;

    if (value == currentValue) {
      lower = mid;
      upper = mid;

      int num = mid - 1;
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

SAPRayDimension::SAPRayDimension(SAPDimension *dimension, double from,
                                 double to,
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

void SAPRayDimension::recalculateDRatio() {
  double value =
      currentIndex >= 0 && currentIndex < currentDimension->intervalList.size()
          ? currentDimension->intervalList[currentIndex]->value
          : std::copysign(std::numeric_limits<double>::infinity(), step);
  dRatio = (value - startValue) / deltaValue;
  if (dRatio < 0)
    dRatio = std::numeric_limits<double>::infinity();
}

void SAPRayDimension::incrementRay() {
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

void SAPRayDimension::findInternalHit(std::vector<GameEntity *> *list) {
  for (size_t index :
       currentDimension->iterateStabs(currentIndex + sideCheck)) {
    GameEntity *entity = currentDimension->intervalList[index]->entity;
    addHit(entity);

    if (list != nullptr && (*hitMap)[entity] > 1) {
      list->push_back(entity);
    }
  }
}

bool SAPRay::next() {
  if (mCheckInternalHit) {
    mCheckInternalHit = false;
    mXRay.findInternalHit(nullptr);
    mYRay.findInternalHit(&mInternalHit);
  }

  if (!mInternalHit.empty()) {
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

void SAP::add(GameEntity *entity) {
  Item item{.x0 = std::make_shared<SAPDimension::Item>(entity->x0, 0, entity),
            .y0 = std::make_shared<SAPDimension::Item>(entity->y0, 0, entity),
            .x1 = std::make_shared<SAPDimension::Item>(entity->x1, 1, entity),
            .y1 = std::make_shared<SAPDimension::Item>(entity->y1, 1, entity)};

  dimensionX.bufferList.push_back(item.x0);
  dimensionX.bufferList.push_back(item.x1);
  dimensionY.bufferList.push_back(item.y0);
  dimensionY.bufferList.push_back(item.y1);

  entities[entity] = std::move(item);
}

void SAP::move(GameEntity *entity) {
  if (!entities.contains(entity))
    return;

  Item &item = entities[entity];
  item.x0->value = entity->x0;
  item.x1->value = entity->x1;
  item.y0->value = entity->y0;
  item.y1->value = entity->y1;
}

void SAP::remove(GameEntity *entity) {
  if (!entities.contains(entity))
    return;

  Item &item = entities[entity];
  item.x0->isPendingDelete = true;
  item.x1->isPendingDelete = true;
  item.y0->isPendingDelete = true;
  item.y1->isPendingDelete = true;
  mPendingRemove.push_back(entity);
}

void SAP::update() {
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

std::vector<GameEntity *> SAP::queryArea(double x0, double y0, double x1,
                                         double y1, bool enclosed) {
  int minScore = enclosed ? 3 : 0;
  auto [_xl, xi] = dimensionX.binarySearch(x0);
  auto [_yl, yi] = dimensionY.binarySearch(y0);
  std::unordered_map<GameEntity *, int> xSet, ySet;

  if (!enclosed) {
    dimensionX.addStabsToSet(xi, xSet);
    dimensionY.addStabsToSet(yi, ySet);
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
    if (ySet.contains(entity) && xScore > minScore && ySet[entity] > minScore) {
      result.push_back(entity);
    }
  }

  return result;
}