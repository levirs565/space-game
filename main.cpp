#include <SDL.h>
#include <SDL_image.h>
#include <SDL_mixer.h>
#include <SDL_ttf.h>
#include <cmath>
#include <exception>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <queue>
#include <random>
#include <ranges>
#include <set>
#include <unordered_map>
#include <vector>

#include "DataFormat.hpp"
#include "Entity/GameEntity.hpp"
#include "Entity/PlayerShip.hpp"
#include "Entity/Enemy.hpp"
#include "Entity/Laser.hpp"
#include "Entity/Meteor.hpp"
#include "Polygon.hpp"
#include "IGameStage.hpp"
#include "AI/FlowField.hpp"
#include "SAP.hpp"


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

    mWindow = SDL_CreateWindow("Space", SDL_WINDOWPOS_UNDEFINED,
                               SDL_WINDOWPOS_UNDEFINED, 800, 600, 0);

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
    mBackgroundTexture = mTextureLoader->load(
        "/home/levirs565/Unduhan/SpaceShooterRedux/Backgrounds/black.png");

    mLaserSound = Mix_LoadWAV(
        "/home/levirs565/Unduhan/SpaceShooterRedux/Bonus/sfx_laser1.ogg");

    std::unique_ptr<PlayerShip> playerShip =
        std::make_unique<PlayerShip>(mTextureLoader.get(), Vec2(400, 700));
    mPlayerShip = playerShip.get();
    addEntity(std::move(playerShip));

    addEntity(
        std::move(std::make_unique<Enemy>(mTextureLoader.get(), Vec2(100, 0))));
    addEntity(
        std::move(std::make_unique<Enemy>(mTextureLoader.get(), Vec2(300, 0))));
    addEntity(
        std::move(std::make_unique<Enemy>(mTextureLoader.get(), Vec2(800, 0))));

    addEntity(std::move(std::make_unique<Meteor>(
        mTextureLoader.get(), Vec2(100, 500), "Brown_big1")));
    addEntity(std::move(std::make_unique<Meteor>(
        mTextureLoader.get(), Vec2(300, 500), "Brown_big2")));
    addEntity(std::move(std::make_unique<Meteor>(
        mTextureLoader.get(), Vec2(600, 500), "Brown_big3")));
    addEntity(std::move(std::make_unique<Meteor>(
        mTextureLoader.get(), Vec2(615, 1000), "Brown_big3")));
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

  void addEntity(std::unique_ptr<GameEntity> &&entity) {
    GameEntity *ptr = entity.get();
    mEntityList.push_back(std::move(entity));
    mSAP.add(ptr);
  }

  void processKeyDown(const SDL_KeyboardEvent &key) {
    if (key.repeat != 0)
      return;

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
    if (key.repeat != 0)
      return;

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

  void presentScene() { SDL_RenderPresent(mRenderer); }

  void calculateCamera() {
    mCameraPosition.x = SDL_clamp(mPlayerShip->position.x - mCameraSize.x / 2,
                                  0, mWordSize.x - mCameraSize.x);
    mCameraPosition.y = SDL_clamp(mPlayerShip->position.y - mCameraSize.y / 2,
                                  0, mWordSize.y - mCameraSize.y);
  }

  void drawBackground() {
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
        SDL_RenderCopy(mRenderer, mBackgroundTexture, nullptr, &rect);
      }
    }
  }

  void run() {
    while (true) {
      prepareScene();
      processInput();

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
      drawBackground();

      if (mIsFire)
        mPlayerShip->doFire(this);

      for (auto &[entity, collisionSet] : mSAP.getCollisionMap()) {
        for (auto otherEntity : collisionSet) {
          if (isPolygonCollide(entity->boundingBox, otherEntity->boundingBox)) {
            entity->onHit(otherEntity);
          }
        }
      }

      SDL_SetRenderDrawColor(mRenderer, 0, 255, 0, 255);

      for (GameEntity *entity :
           mSAP.queryArea(mCameraPosition.x, mCameraPosition.y,
                          mCameraPosition.x + mCameraSize.x,
                          mCameraPosition.y + mCameraSize.y, false)) {
        entity->onDraw(mRenderer, mCameraPosition);

        for (size_t i = 0; i < entity->boundingBox.size(); i++) {
          Vec2 current = entity->boundingBox[i];
          Vec2 next = entity->boundingBox[(i + 1) % entity->boundingBox.size()];

          current.substract(mCameraPosition);
          next.substract(mCameraPosition);
          SDL_RenderDrawLine(mRenderer, int(current.x), int(current.y),
                             int(next.x), int(next.y));
        }
      }

      // mPathFinder.drawGrid(mRenderer, mCameraPosition, mCameraSize);
      mPathFinder.generateHeatmap(mPlayerShip->position);

      presentScene();
      SDL_Delay(16);
    }
  }

  const Vec2 &getPlayerPosition() override { return mPlayerShip->position; }

  GameEntity *getPlayerEntity() override { return mPlayerShip; }

  void addLaser(const Vec2 &position, double angle) override {
    std::unique_ptr<Laser> laser =
        std::make_unique<Laser>(mTextureLoader.get(), position, angle);
    addEntity(std::move(laser));
    Mix_PlayChannel(1, mLaserSound, 0);
  }

  const Vec2 &getWorldSize() override { return mWordSize; }

  std::vector<std::unique_ptr<GameEntity>> &getEntities() override {
    return mEntityList;
  }

  Vec2 getFlowDirection(const Vec2 &position, const Vec2 &direction) override {
    return mPathFinder.getDirection(position, direction);
  }

  std::vector<Vec2> findNeighbourObstacle(const Vec2 &position) override {
    return mPathFinder.getNeighbourObstacle(position);
  }

  APathFinder *getPathFinder() { return &mPathFinder; }

  SAP *getSAP() { return &mSAP; }

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