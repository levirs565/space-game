#include "GameScreen.hpp"

void GameScreen::onSizeChanged(const Vec2 &size) {
  mStageScreen.onSizeChanged(size);
  mPauseScreen.onSizeChanged(size);
  mGameOverScreen.onSizeChanged(size);
}
void GameScreen::onSDLEvent(const SDL_Event &event) {
  if (event.type == SDL_EVENT_KEY_DOWN) {
    if (event.key.scancode == SDL_SCANCODE_ESCAPE) {
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

GameScreen::GameScreen(GameParams params, MIX_Mixer *mixer, SDL_Window *window,
                       std::function<void(Event)> callback)
    : mWindow(window), mCallback(std::move(callback)),
      mPauseScreen([this](GamePauseScreen::Event event) {
        if (event == GamePauseScreen::Event::Resume) {
          mIsPause = false;
          mStageScreen.resetLastUpdateTime();
        } else if (event == GamePauseScreen::Event::Quit)
          mCallback(Event::Quit);
      }),
      mStageScreen(params, mixer,
                   [this](GameStageScreen::Event event) {
                     if (event == GameStageScreen::Event::GameOver) {
                       mGameOverScreen.setScore(mStageScreen.getScore());
                       mIsGameOver = true;
                     }
                   }),
      mGameOverScreen(mWindow, [this](GameOverScreen::Event event) {
        if (event == GameOverScreen::Event::Quit)
          mCallback(Event::Quit);
      }) {}
