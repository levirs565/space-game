#ifndef SPACE_GAMESCREEN_HPP
#define SPACE_GAMESCREEN_HPP

#include <SDL_mixer.h>

#include "IScreen.hpp"
#include "../IGameStage.hpp"
#include "../Entity/PlayerShip.hpp"
#include "../AI/FlowField.hpp"
#include "../SAP.hpp"

class GameScreen : public IScreen, public IGameStage {
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

  void processKeyDown(const SDL_KeyboardEvent &key);
  void processKeyUp(const SDL_KeyboardEvent &key);

  void drawBackground(SDL_Renderer *renderer);
  void calculateCamera();
public:
  GameScreen();

  void onSDLEvent(const SDL_Event &event) override;
  void onSizeChanged(const Vec2 &size) override;
  void onUpdate() override;
  void onDraw(SDL_Renderer *renderer) override;
  void onPostDraw() override;

  void addEntity(std::unique_ptr<GameEntity> &&entity);

  GameEntity *getPlayerEntity() override { return mPlayerShip; }
  void addLaser(const Vec2 &position, double angle) override;
  const Vec2 &getWorldSize() override { return mWordSize; }
  FlowField *getFlowField() override { return &mPathFinder; }
  SAP *getSAP() override { return &mSAP; }
};

#endif // SPACE_GAMESCREEN_HPP
