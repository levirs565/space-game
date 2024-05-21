#ifndef SPACE_GAMESTAGESCREEN_HPP
#define SPACE_GAMESTAGESCREEN_HPP

#include <SDL_mixer.h>

#include "../AI/FlowField.hpp"
#include "../Entity/PlayerShip.hpp"
#include "../IGameStage.hpp"
#include "../SAP.hpp"
#include "../UI/Label.hpp"
#include "IScreen.hpp"
#include <functional>
#include <random>

class GameStageScreen : public IScreen, public IGameStage {
  SDL_Texture *mBackgroundTexture;
  SDL_Texture *mPlayerLifeTexture;
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
  std::random_device mRandomAngleDevice;
  std::default_random_engine mRandomAngleEngine;
  std::uniform_int_distribution<int> mRandomAngle{0, 360};

  Uint32 mEnemyLastSpawn = 0;
  Uint32 mEnemySpawnDelay = 5000;
  int mSpawnedCount = 0;
  Uint32 mEnemyLastSpawnMultiplier = 0;

  std::random_device mRandomHealthDevice;
  std::default_random_engine mRandomHealthEngine;
  std::uniform_int_distribution<int> mRandomHealth{0, 200};

  Uint32 mHealthLastSpawn = 0;
  Uint32 mHealthSpawnDelay = 15000;

  void processKeyDown(const SDL_KeyboardEvent &key);
  void processKeyUp(const SDL_KeyboardEvent &key);

  void drawBackground(SDL_Renderer *renderer);
  void calculateCamera();
  void spawnEnemy();
  void spawnHealth();

  int mScore = 0;
  Label mScoreLabel{"0"};
  void layoutScoreLabel();

public:
  enum class Event { GameOver };

private:
  std::function<void(Event)> mCallback;

public:
  GameStageScreen(std::function<void(Event)> callback);

  void onSDLEvent(const SDL_Event &event) override;
  void onSizeChanged(const Vec2 &size) override;
  void onUpdate() override;
  void onDraw(SDL_Renderer *renderer) override;
  void onPostDraw() override;

  void addEntity(std::unique_ptr<GameEntity> &&entity);

  GameEntity *getPlayerEntity() override { return mPlayerShip; }
  void addLaser(const Vec2 &position, double angle,
                const std::string &textureName) override;
  const Vec2 &getWorldSize() override { return mWordSize; }
  FlowField *getFlowField() override { return &mPathFinder; }
  SAP *getSAP() override { return &mSAP; }

  int getScore() { return mScore; }
};

#endif // SPACE_GAMESTAGESCREEN_HPP
