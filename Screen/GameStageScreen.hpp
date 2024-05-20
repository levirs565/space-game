#ifndef SPACE_GAMESTAGESCREEN_HPP
#define SPACE_GAMESTAGESCREEN_HPP

#include <SDL_mixer.h>

#include "IScreen.hpp"
#include "../IGameStage.hpp"
#include "../Entity/PlayerShip.hpp"
#include "../AI/FlowField.hpp"
#include "../SAP.hpp"
#include <random>

class GameStageScreen : public IScreen, public IGameStage {
  SDL_Texture *mBackgroundTexture;
  PlayerShip *mPlayerShip;
  std::vector<std::unique_ptr<GameEntity>> mEntityList;
  FlowField mPathFinder;
  Vec2 mCameraSize;
  Vec2 mCameraPosition;
  Vec2 mWordSize{5000, 5000};
  Mix_Chunk *mLaserSound;
  bool mIsUp = false;
  bool mIsLeft = false;
  bool mIsDown = false;
  bool mIsRight = false;
  bool mIsFire = false;
  SAP mSAP;
  std::random_device mRandomDevice;
  std::default_random_engine mRandomEngine;
  std::uniform_int_distribution <int> mRandomAngle;
  Uint32 mLastSpawn = 0;
  Uint32 mSpawnDelay = 5000;
  int mspawnedCount = 0;
  Uint32 mLastSpawnMultiplier = 0;

  void processKeyDown(const SDL_KeyboardEvent &key);
  void processKeyUp(const SDL_KeyboardEvent &key);

  void drawBackground(SDL_Renderer *renderer);
  void calculateCamera();
  void spawnEntity();
public:
  GameStageScreen();

  void onSDLEvent(const SDL_Event &event) override;
  void onSizeChanged(const Vec2 &size) override;
  void onUpdate() override;
  void onDraw(SDL_Renderer *renderer) override;
  void onPostDraw() override;

  void addEntity(std::unique_ptr<GameEntity> &&entity);

  GameEntity *getPlayerEntity() override { return mPlayerShip; }
  void addLaser(const Vec2 &position, double angle, const std::string& textureName) override;
  const Vec2 &getWorldSize() override { return mWordSize; }
  FlowField *getFlowField() override { return &mPathFinder; }
  SAP *getSAP() override { return &mSAP; }
};

#endif // SPACE_GAMESTAGESCREEN_HPP
