#include "GameScreen.hpp"

void GameScreen::onSizeChanged(const Vec2 &size) {
  mStageScreen.onSizeChanged(size);
  mPauseScreen.onSizeChanged(size);
  mGameOverScreen.onSizeChanged(size);
}
void GameScreen::onSDLEvent(const SDL_Event &event) {
  if (event.type == SDL_KEYDOWN) {
    if (event.key.keysym.scancode == SDL_SCANCODE_ESCAPE) {
      mIsPause = !mIsPause;
      mStageScreen.resetLastUpdateTime();
    }
  }

  if (!mIsPause && !mIsGameOver)
    mStageScreen.onSDLEvent(event);
  else if (mIsGameOver)
    mGameOverScreen.onSDLEvent(event);
  else
    mPauseScreen.onSDLEvent(event);
}
void GameScreen::onUpdate() {
  if (!mIsPause && !mIsGameOver)
    mStageScreen.onUpdate();
  else if (mIsGameOver)
    mGameOverScreen.onUpdate();
  else
    mPauseScreen.onUpdate();
}
void GameScreen::onDraw(SDL_Renderer *renderer) {
  mStageScreen.onDraw(renderer);
  if (mIsPause && !mIsGameOver)
    mPauseScreen.onDraw(renderer);
  else if (mIsGameOver)
    mGameOverScreen.onDraw(renderer);
}
void GameScreen::onPostDraw() {
  if (!mIsPause && !mIsGameOver)
    mStageScreen.onPostDraw();
  else if (mIsGameOver)
    mGameOverScreen.onPostDraw();
  else
    mPauseScreen.onPostDraw();
}

GameScreen::GameScreen(std::function<void(Event)> callback)
    : mCallback(std::move(callback)),
      mPauseScreen([this](GamePauseScreen::Event event) {
        if (event == GamePauseScreen::Event::Resume) {
          mIsPause = false;
          mStageScreen.resetLastUpdateTime();
        } else if (event == GamePauseScreen::Event::Quit)
          mCallback(Event::Quit);
      }),
      mStageScreen([this](GameStageScreen::Event event) {
        if (event == GameStageScreen::Event::GameOver) {
          mGameOverScreen.setScore(mStageScreen.getScore());
          mIsGameOver = true;
        }
      }),
      mGameOverScreen([this](GameOverScreen::Event event) {
        if (event == GameOverScreen::Event::Quit)
          mCallback(Event::Quit);
      }) {}
