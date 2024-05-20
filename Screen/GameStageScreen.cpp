#include "GameStageScreen.hpp"
#include "../Entity/Enemy.hpp"
#include "../Entity/Meteor.hpp"
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
  std::unique_ptr<Laser> laser =
      std::make_unique<Laser>(position, angle, textureName);
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
GameStageScreen::GameStageScreen() : mRandomEngine(mRandomDevice()) {
  mBackgroundTexture =
      TextureManager::getInstance()->load("Backgrounds/black.png");

  std::string laserSoundPath =
      AssetManager::getInstance()->getAsset("Bonus/sfx_laser1.ogg").string();
  mLaserSound = Mix_LoadWAV(laserSoundPath.c_str());

  std::unique_ptr<PlayerShip> playerShip =
      std::make_unique<PlayerShip>(Vec2(400, 700));
  mPlayerShip = playerShip.get();
  addEntity(std::move(playerShip));

  Map map = Map::fromAsset();

  mWordSize = map.size;

  for (auto& entity : map.entityList) {
    addEntity(std::move(entity));
  }

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

void GameStageScreen::spawnEntity() {
  int Angle = mRandomAngle(mRandomEngine);
  Vec2 direction(1, 0);
  direction.rotate(double(Angle) * M_PI / 180);
  direction.scale(1000);
  Vec2 EnemyPosisition(mPlayerShip->position);
  EnemyPosisition.add(direction, 1);
  addEntity(std::move(std::make_unique<Enemy>(EnemyPosisition)));
}

void GameStageScreen::onUpdate() {
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
  if (SDL_GetTicks() - mLastSpawn >= mSpawnDelay) {
    spawnEntity();
    mLastSpawn = SDL_GetTicks();
    mspawnedCount++;
  }

  if (SDL_GetTicks() - mLastSpawnMultiplier >= 10000 && mSpawnDelay >= 1000) {
    mSpawnDelay -= 200;
    mLastSpawnMultiplier = SDL_GetTicks();
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

  if (mIsFire)
    mPlayerShip->doFire(this);

  for (auto &[entity, collisionSet] : mSAP.getCollisionMap()) {
    for (auto otherEntity : collisionSet) {
      if (isPolygonCollide(entity->boundingBox, otherEntity->boundingBox)) {
        entity->onHit(otherEntity);
      }
    }
  }
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

  // mPathFinder.drawGrid(mRenderer, mCameraPosition, mCameraSize);
}

void GameStageScreen::onPostDraw() {
  mPathFinder.generateHeatmap(mPlayerShip->position);
}

void GameStageScreen::onSizeChanged(const Vec2 &size) { mCameraSize = size; }
