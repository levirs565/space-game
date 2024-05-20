#ifndef SPACE_GAMESCREEN_HPP
#define SPACE_GAMESCREEN_HPP

#include "GameStageScreen.hpp"
#include "GamePauseScreen.hpp"
#include "GameOverScreen.hpp"

class GameScreen : public IScreen {
public:
  enum class Event {
    Quit
  };
private:
  GameStageScreen mStageScreen;
  GamePauseScreen mPauseScreen;
  GameOverScreen mGameOverScreen;
  bool mIsPause = false;
  bool mIsGameOver = false;
  std::function<void(Event)> mCallback;
public:
  GameScreen(std::function<void(Event)> callback);

  void onSizeChanged(const Vec2& size) override;
  void onSDLEvent(const SDL_Event& event) override;
  void onUpdate() override;
  void onDraw(SDL_Renderer* renderer) override;
  void onPostDraw() override;
};

#endif // SPACE_GAMESCREEN_HPP
