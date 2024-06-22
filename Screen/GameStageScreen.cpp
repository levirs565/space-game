#include "GameStageScreen.hpp"
#include "../Entity/Enemy.hpp"
#include "../Entity/Meteor.hpp"
#include "../Entity/PowerUpHealth.hpp"
#include "../Map.hpp"
#include "../Math/Polygon.hpp"

void GameStageScreen::processKeyDown(const SDL_KeyboardEvent &key) {
  if (key.repeat != 0)
    return;

  if (key.keysym.scancode == SDL_SCANCODE_UP ||
      key.keysym.scancode == SDL_SCANCODE_W)
    mIsUp = true;

  if (key.keysym.scancode == SDL_SCANCODE_DOWN ||
      key.keysym.scancode == SDL_SCANCODE_S)
    mIsDown = true;

  if (key.keysym.scancode == SDL_SCANCODE_LEFT ||
      key.keysym.scancode == SDL_SCANCODE_A)
    mIsLeft = true;

  if (key.keysym.scancode == SDL_SCANCODE_RIGHT ||
      key.keysym.scancode == SDL_SCANCODE_D)
    mIsRight = true;

  if (key.keysym.scancode == SDL_SCANCODE_LCTRL ||
      key.keysym.scancode == SDL_SCANCODE_SPACE)
    mIsFire = true;
}

void GameStageScreen::processKeyUp(const SDL_KeyboardEvent &key) {
  if (key.repeat != 0)
    return;

  if (key.keysym.scancode == SDL_SCANCODE_UP ||
      key.keysym.scancode == SDL_SCANCODE_W)
    mIsUp = false;

  if (key.keysym.scancode == SDL_SCANCODE_DOWN ||
      key.keysym.scancode == SDL_SCANCODE_S)
    mIsDown = false;

  if (key.keysym.scancode == SDL_SCANCODE_LEFT ||
      key.keysym.scancode == SDL_SCANCODE_A)
    mIsLeft = false;

  if (key.keysym.scancode == SDL_SCANCODE_RIGHT ||
      key.keysym.scancode == SDL_SCANCODE_D)
    mIsRight = false;

  if (key.keysym.scancode == SDL_SCANCODE_LCTRL ||
      key.keysym.scancode == SDL_SCANCODE_SPACE)
    mIsFire = false;
}
void GameStageScreen::addLaser(const Vec2 &position, double angle,
                               const std::string &textureName) {
  Vec2 direction{1, 0};
  direction.rotate(angle);
  std::unique_ptr<Laser> laser =
      std::make_unique<Laser>(position, direction, textureName);
  addEntity(std::move(laser));
  Mix_PlayChannel(1, mLaserSound, 0);
}
void GameStageScreen::addEntity(std::unique_ptr<GameEntity> &&entity) {
  GameEntity *ptr = entity.get();
  mEntityList.push_back(std::move(entity));
  mSAP.add(ptr);
}

void GameStageScreen::onSDLEvent(const SDL_Event &event) {
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
void GameStageScreen::drawBackground(SDL_Renderer *renderer) {
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
      SDL_RenderCopy(renderer, mBackgroundTexture, nullptr, &rect);
    }
  }
}
void GameStageScreen::calculateCamera() {
  mCameraPosition.x = SDL_clamp(mPlayerShip->position.x - mCameraSize.x / 2, 0,
                                mWordSize.x - mCameraSize.x);
  mCameraPosition.y = SDL_clamp(mPlayerShip->position.y - mCameraSize.y / 2, 0,
                                mWordSize.y - mCameraSize.y);
}
GameStageScreen::GameStageScreen(std::function<void(Event)> callback)
    : mCallback(std::move(callback)), mRandomAngleEngine(mRandomAngleDevice()),
      mRandomHealthEngine(mRandomHealthDevice()) {
  mBackgroundTexture =
      TextureManager::getInstance()->load("Backgrounds/black.png");
  mPlayerLifeTexture =
      TextureManager::getInstance()->load("PNG/UI/playerLife3_blue.png");

  std::string laserSoundPath =
      AssetManager::getInstance()->getAsset("Bonus/sfx_laser1.ogg").string();
  mLaserSound = Mix_LoadWAV(laserSoundPath.c_str());

  std::unique_ptr<PlayerShip> playerShip =
      std::make_unique<PlayerShip>(Vec2(400, 700));
  mPlayerShip = playerShip.get();
  addEntity(std::move(playerShip));

  Map map = Map::fromAsset();

  mWordSize = map.size;

  for (auto &entity : map.entityList) {
    addEntity(std::move(entity));
  }

  mPathFinder.init(mWordSize, 110);

  for (std::vector<std::unique_ptr<GameEntity>>::iterator it =
           mEntityList.begin();
       it != mEntityList.end(); it++) {
    std::unique_ptr<GameEntity> &entity = *it;
    if (dynamic_cast<Meteor *>(entity.get()) == nullptr)
      continue;
    mPathFinder.addObstacle(entity.get());
  }

  mPathFinder.generateHeatmap(mPlayerShip->position);
}

void GameStageScreen::spawnEnemy() {
  Vec2 enemyPosition;
  do {
    int angle = mRandomAngle(mRandomAngleEngine);
    Vec2 direction(1, 0);
    direction.rotate(double(angle) * M_PI / 180);
    direction.scale(800);
    enemyPosition = mPlayerShip->position;
    enemyPosition.add(direction, 1);
  } while (enemyPosition.x < 0 || enemyPosition.y < 0 ||
           enemyPosition.x > mWordSize.x || enemyPosition.y > mWordSize.y);
  addEntity(std::move(std::make_unique<Enemy>(enemyPosition)));
}

void GameStageScreen::spawnHealth() {
  Vec2 healthPosition;
  do {
    int angle = mRandomAngle(mRandomAngleEngine);
    int distance = mRandomHealth(mRandomHealthEngine);
    Vec2 direction(1, 0);
    direction.rotate(double(angle) * M_PI / 180);
    direction.scale(100 + distance);

    healthPosition = mPlayerShip->position;
    healthPosition.add(direction, 1);

  } while (healthPosition.x < 0 || healthPosition.y < 0 ||
           healthPosition.x > mWordSize.x || healthPosition.y > mWordSize.y);
  addEntity(std::move(std::make_unique<PowerUpHealth>(healthPosition)));
}

void GameStageScreen::onUpdate() {
  if (mLastGameTickUpdate == 0) {
    mLastGameTickUpdate = SDL_GetTicks();
    return;
  }

  Uint32 currentTick = SDL_GetTicks();
  mGameTick += currentTick - mLastGameTickUpdate;
  mLastGameTickUpdate = currentTick;

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

  for (auto &entity : mEntityList)
    entity->onUpdatePhysic();

  if (getTick() - mEnemyLastSpawn >= mEnemySpawnDelay) {
    spawnEnemy();
    mEnemyLastSpawn = getTick();
    mSpawnedCount++;
  }

  if (getTick() - mEnemyLastSpawnMultiplier >= 10000 &&
      mEnemySpawnDelay >= 1000) {
    mEnemySpawnDelay -= 200;
    mEnemyLastSpawnMultiplier = getTick();
  }

  if (getTick() - mHealthLastSpawn >= mHealthSpawnDelay) {
    spawnHealth();
    mHealthLastSpawn = getTick();
  }

  for (std::unique_ptr<GameEntity> &entity : mEntityList) {
    mSAP.move(entity.get());
  }

  bool needUpdateScore = false;
  for (std::vector<std::unique_ptr<GameEntity>>::iterator it =
           mEntityList.begin();
       it != mEntityList.end(); it++) {
    std::unique_ptr<GameEntity> &entity = *it;

    if (entity->mustGone) {
      if (dynamic_cast<Enemy *>(entity.get())) {
        mScore += 100;
        needUpdateScore = true;
      }

      mSAP.remove(entity.get());
      it = mEntityList.erase(it) - 1;
      continue;
    }
  }

  if (needUpdateScore) {
    mScoreLabel.setText(std::to_string(mScore));
    layoutScoreLabel();
  }

  mSAP.update();

  if (mIsFire)
    mPlayerShip->doFire(this);

  for (auto &[entity, collisionSet] : mSAP.getCollisionMap()) {
    for (auto otherEntity : collisionSet) {
      PolygonCollision collision = calculatePolygonCollision(
          entity->boundingBox, otherEntity->boundingBox);
      if (collision.isCollide) {
        if (entity == otherEntity) {
          //          SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "Aneh");
          continue;
        }

        entity->onHit(this, otherEntity);
        otherEntity->onHit(this, entity);

        if (entity->collisionResponse !=
                GameEntity::CollisionResponse::RejectBoth &&
            otherEntity->collisionResponse !=
                GameEntity::CollisionResponse::RejectBoth) {

          Vec2 relativeVelocity{otherEntity->velocity};
          relativeVelocity.substract(entity->velocity);

          entity->position.add(collision.normal, -collision.depth / 2);
          entity->updateBoundingBox();

          otherEntity->position.add(collision.normal, collision.depth / 2);
          otherEntity->updateBoundingBox();

          double e = 0.5;
          double j = -(1.0 + e) * relativeVelocity.dot(collision.normal) /
                     ((1 / entity->mass) + (1 / otherEntity->mass));

          Vec2 impulse{collision.normal};
          impulse.scale(j);

          Vec2 entityExtraVelocity{impulse};
          entityExtraVelocity.scale(-1.0 / entity->mass);
          entity->velocity.add(entityExtraVelocity, 1);

          Vec2 otherEntityExtraVelocity(impulse);
          otherEntityExtraVelocity.scale(1.0 / entity->mass);
          otherEntity->velocity.add(otherEntityExtraVelocity, 1);

          if (dynamic_cast<Meteor *>(entity) != nullptr) {
            mPathFinder.moveObstacle(entity);
          }
          mSAP.move(entity);

          if (dynamic_cast<Meteor *>(otherEntity) != nullptr) {
            mPathFinder.moveObstacle(otherEntity);
          }
          mSAP.move(otherEntity);
        }
      }
    }
  }

  mSAP.update();
  calculateCamera();

  if (mPlayerShip->healthCount <= 0)
    mCallback(Event::GameOver);
}

void GameStageScreen::onDraw(SDL_Renderer *renderer) {

  drawBackground(renderer);

  for (GameEntity *entity :
       mSAP.queryArea(mCameraPosition.x, mCameraPosition.y,
                      mCameraPosition.x + mCameraSize.x,
                      mCameraPosition.y + mCameraSize.y, false)) {
    entity->onDraw(renderer, mCameraPosition);

    for (size_t i = 0; i < entity->boundingBox.size(); i++) {
      Vec2 current = entity->boundingBox[i];
      Vec2 next = entity->boundingBox[(i + 1) % entity->boundingBox.size()];

      current.substract(mCameraPosition);
      next.substract(mCameraPosition);

      SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255);
      SDL_RenderDrawLine(renderer, int(current.x), int(current.y), int(next.x),
                         int(next.y));
    }
  }

  SDL_Rect lifeIcon = {.x = 4, .y = 4};
  SDL_QueryTexture(mPlayerLifeTexture, nullptr, nullptr, &lifeIcon.w,
                   &lifeIcon.h);
  for (int i = 1; i <= mPlayerShip->healthCount; i++) {
    SDL_RenderCopy(renderer, mPlayerLifeTexture, nullptr, &lifeIcon);
    lifeIcon.x += lifeIcon.w + 4;
  }

  mScoreLabel.draw(renderer);
  //  mPathFinder.drawGrid(renderer, mCameraPosition, mCameraSize);
}

void GameStageScreen::onPostDraw() {
  mPathFinder.generateHeatmap(mPlayerShip->position);
}

void GameStageScreen::onSizeChanged(const Vec2 &size) {
  mCameraSize = size;
  layoutScoreLabel();
}
void GameStageScreen::layoutScoreLabel() {
  Vec2 size = mScoreLabel.getLayoutSize();
  mScoreLabel.setCenterPosition({mCameraSize.x - size.x / 2, size.y / 2});
}
